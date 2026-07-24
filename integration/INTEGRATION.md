# UAV_yolo ⇄ NYCU_UAV_offboard 整合與實體驗證流程

這份文件回答三件事：
1. **兩套系統怎麼接**（架構、資料流、誰負責什麼）。
2. **一個能不能 work 的判斷**（我讀過雙方原始碼後的結論）。
3. **一條分階段的實體驗證流程**（從 sim 到實飛，每階段的驗收條件與安全）。

---

## 0. 一句話結論

**可以整合，而且接得很乾淨。** NYCU 的 `global_goto_node` 收到一個 GPS 目標後，
對 PX4 發的正是 `MAV_CMD_DO_REPOSITION` → 進 `AUTO_LOITER`，這跟 UAV_yolo 的導引模型
（旋翼定點跟隨、定翼繞點盤旋）**完全同構**。UAV_yolo 只要把「算出來的目標經緯度」
用 LR24-F 的 `$CMD,seq,GOTO,...` 文字幀送過去即可，其餘（安全 gate、發 DO_REPOSITION、
與 PX4 handshake）沿用 NYCU 已寫好且審查過的節點。

有兩個必須先處理的點（下面詳述）：
- **A. LR24 不回傳姿態/位置** → 地面測地需要另一條 MAVLink(SiK) 電台讀 pose。
- **B. `global_goto_node` 目前硬鎖固定翼** → 四軸要跑整合路徑需套多旋翼 patch
  （見 [`global_goto_multirotor_patch.md`](global_goto_multirotor_patch.md)）；
  或四軸先用 `direct` 後端驗證追蹤本身、整合路徑留給固定翼。

> **但先看 §2 路線 C**：如果目的只是「讓 Pixhawk 收到我們算出的目標」，
> **數傳直接插 Pixhawk 就夠了，完全不需要 companion**——上面 A、B 兩個問題都不存在。
> 四軸首驗建議直接走這條。

---

## 1. 誰負責什麼（分工）

| 子系統 | 跑在哪 | 負責 |
|---|---|---|
| **UAV_yolo**（本專案） | 地面站筆電 | 影像 → YOLO 偵測 → 射線測地 → KF → 導引 → 產生**目標 GPS** |
| **NYCU_UAV_offboard** | 機上 companion（RPi / Jetson Orin） | 收目標 GPS → 安全 gate → PX4 offboard/reposition |
| **PX4 / Pixhawk 6C Mini** | 機上 | 飛控本體、姿態/位置估計、實際致動 |
| **飛行員 + RC/ELRS** | 現場 | 起飛、降落、隨時手動接管（最高權限） |

> **companion 是 RPi 還是 Jetson Orin？** 你口頭說 RPi，但 NYCU 的
> `docs/lr24_communication.md` 寫 Jetson Orin。**這會影響架構選擇**（見 §2 的兩條路）——
> 請先確認實際硬體。功能上兩者都是「跑 ROS 2 的 Linux 伴飛電腦」，接法一致；差別在
> **能不能在機上跑 YOLO**（Orin 有 GPU 可以，RPi 幾乎不行）。

---

## 2. 架構：三條路

### ⭐ 路線 C（最簡，四軸首驗推薦）：**根本不用 companion**

數傳電台直接插 Pixhawk 6C Mini 的 TELEM1，UAV_yolo 用 **一條 MAVLink 幹完三件事**：

```
相機 ──FPV圖傳──→ 採集卡 ──→ ┐
                              ├─ UAV_yolo(地面筆電)
Pixhawk TELEM1 ─SiK數傳─←──→ ┘   ├ 讀 ATTITUDE/GLOBAL_POSITION_INT/GPS_RAW_INT → 測地＋閘門
                                  ├ 送 DO_REPOSITION                        → 目標
                                  └ 送 DO_SET_ROI_LOCATION                  → C-20T 雲台
```

`link.command_backend: direct`（預設值）。PX4 接受任何 GCS 來的 DO_REPOSITION，
**不需要 RPi / Orin、不需要 ROS 2、不需要 uXRCE-DDS Agent、不需要 LR24 協定、
也不需要多旋翼 patch**（那個固定翼 gate 在 NYCU 節點裡，這條路直接繞過）。
而且 **DO_REPOSITION param3 可以帶 loiter 半徑**——LR24 的 GOTO 幀反而做不到。

> **關鍵理解**：既然追蹤本來就跑在地面，companion 在這個架構裡**只是「轉送 + 安全把關」，
> 並不提供任何自主能力**。所以四軸階段拿掉它是完全合理的簡化。

**拿掉 companion 會少掉什麼？** `global_goto_node` 的機上 gate（GPS fix/衛星/eph/epv、
failsafe_flags、land_detected、ACK+setpoint 雙重確認）。其中 **GPS 品質與離地判定已在
UAV_yolo 補上**（`safety.require_gps_quality` / `require_airborne`，資料取自 MAVLink
`GPS_RAW_INT` / `EXTENDED_SYS_STATE`，門檻與該節點一致：fix≥3、衛星≥8、eph≤5m、epv≤8m）。
仍少的是 PX4 `failsafe_flags` 細項與 setpoint 雙重確認——這兩項由 **PX4 自身 failsafe
與飛行員 RC 接管** 兜底。

**什麼時候才真的需要 companion？**
1. 想把追蹤搬到機上（路線 A）——免圖傳、超出影像鏈路距離也能追（需 Orin 級算力）。
2. 想要那層機上 gate 獨立於地面鏈路強制執行。
3. LR24-F（最大 500 mW）射程優於你手上的 SiK 電台時，拿它當指令上行。

---

## 2b. 需要 companion 時：兩條路，先講推薦的

測地（把畫面像素換成 GPS）需要同時有 **影像的像素** 和 **當下的相機姿態
（＝載具姿態 ⊗ 雲台姿態）**。兩者在哪，就決定架構長怎樣。

### 🟢 路線 B（推薦，對應你「追蹤跑地面、RPi 做 offboard」的設想）

YOLO 與測地都在地面筆電。三條獨立鏈路：

```
                 機上                                              地面
  ┌───────────────────────────────┐              ┌────────────────────────────────┐
  │ 相機 ──HDMI→採集卡/FPV發射───────┼──圖傳──────→─┼→ 採集卡 → UAV_yolo(YOLO+測地+KF+導引)│
  │                               │              │        │  算出目標 GPS            │
  │ Pixhawk ─TELEM1→ SiK 電台 ─────┼──MAVLink───→─┼→ SiK → └→ 讀 pose(姿態/位置/雲台)   │
  │        └TELEM2→ companion ──── │              │              │                   │
  │                 (uXRCE-DDS)    │              │       目標 GPS │                   │
  │  global_goto_node ←LR24空端←───┼──LR24 8kbps─←┼── LR24地端 ←──┘ $CMD,seq,GOTO,...   │
  │        │ DO_REPOSITION         │              └────────────────────────────────┘
  │        ▼                       │
  │      PX4 → AUTO_LOITER         │
  └───────────────────────────────┘
```

- **影像**：機上相機 → FPV 圖傳 → 地面採集卡（UAV_yolo `video.source: uvc` 或 `rtsp`）。
- **pose**：Pixhawk TELEM1 → SiK 數傳 → 地面，UAV_yolo 用 **MAVLink** 讀
  `ATTITUDE`/`GLOBAL_POSITION_INT`/雲台回報（這條也順便餵 QGC）。
- **目標指令**：UAV_yolo 經 **LR24-F** 送 `$CMD,seq,GOTO,lat,lon,rel_alt*CS` 給機上
  `global_goto_node`。
- **雲台 ROI**：走 SiK MAVLink 直接下 `DO_SET_ROI_LOCATION` 給 PX4（UAV_yolo 已支援）。

UAV_yolo 設定：`link.command_backend: lr24`。這會啟用 **CompositeLink**——SiK 讀 pose、
LR24 送 GOTO、雲台走 SiK。**為什麼要兩條無線鏈路**：因為 LR24 協定「只有下行指令、
不回傳姿態/位置」（我讀過 `lr24_command_node.cpp` 與 `global_goto_node.cpp` 的
STATUS 回覆，確認沒有 pose 欄位），地面測地拿不到姿態就做不了，所以 pose 一定要靠
另一條 MAVLink 電台。SiK 便宜、UAV_yolo 本來就會講 MAVLink，最省事。

### 🔵 路線 A（若 companion 是 Jetson Orin，其實更乾淨）

整套 UAV_yolo 直接跑在 Orin 上：相機接 Orin、YOLO 在 Orin 跑、pose 從機上 DDS
本地就有（零下行延遲、免 SiK）、導引直接 in-process 呼叫 `/goto_global` service
（免 LR24 那一跳）。地面站只用瀏覽器連上 Orin 監看＋開關導引。這需要幫 UAV_yolo 再寫一個
「ROS2/DDS 後端」（rclpy 訂 PX4 topics + call goto service），是未來最佳解，但工作量較大，
**先不做**；等路線 B 驗證完、確認硬體是 Orin，再考慮。

> **決策**：先照路線 B 走（符合你的設想、且 UAV_yolo 現成能跑）。若之後確定是 Orin
> 且想追求最低延遲，再升級成路線 A。

---

## 3. 兩系統的接點對照（我讀碼確認過的事實）

| 事項 | UAV_yolo 端 | NYCU 端 | 是否相容 |
|---|---|---|---|
| 目標指令 | `send_reposition(lat,lon,alt)` | `$CMD,seq,GOTO,lat,lon,rel_alt` → `DO_REPOSITION` | ✅ 幀已逐位元對拍 `send_lr24_command.build_frame` |
| 控制語意 | 旋翼跟隨點 / 定翼繞點 | DO_REPOSITION → AUTO_LOITER（旋翼定停/定翼繞圈） | ✅ 同構 |
| 高度基準 | 導引輸出相對 home | GOTO=相對 home（限 30~120m）/ GOTO_AMSL=海拔 | ✅ 用 GOTO 送 rel-home |
| 更新率 | 導引 ≤1Hz + deadband | 單筆在途；送→等 ACK/ERR→再送 | ✅ CompositeLink 背景執行緒 coalesce 最新目標 |
| 繞行半徑 | 定翼導引有 radius | **GOTO 幀不帶半徑** | ⚠️ 半徑由 PX4 `NAV_LOITER_RAD` 設；LR24 傳不了（見下） |
| 姿態/位置回地面 | 測地需要 | **STATUS 不含 pose** | ⚠️ 靠 SiK MAVLink 補（路線 B）|
| 載具型別 | 旋翼/定翼皆可 | **硬鎖固定翼** | ⚠️ 四軸要 patch（附件）|
| 接管 | 模式離開白名單即閂鎖停發 | 監控到非 AUTO_LOITER 即 ABORT；RTL/ABORT 可搶占 | ✅ 雙層一致 |
| 失聯 | 數傳逾時停發 | **已接受的 GOTO 不會因 LR24 斷線取消** | ⚠️ 見 §6 安全 |

**繞行半徑注意**：LR24 的 `GOTO` 幀只有 lat/lon/alt，沒有半徑欄位。所以在路線 B 下，
固定翼 standoff 繞行半徑**由 PX4 的 `NAV_LOITER_RAD` 決定**，UAV_yolo UI 的「繞行半徑」
在此模式僅供地面顯示/預測，不會被送出去。若要讓 UAV_yolo 動態改半徑，只能走
`direct`（MAVLink DO_REPOSITION param3）或未來的路線 A。

---

## 4. 實體驗證流程（分階段，前一階段沒過不進下一階段）

沿用 NYCU 既有紀律：**單元 → SITL → 拆槳桌面 → 低風險實飛**。四軸先行。

### Stage 0 — 純軟體（現在就能做，無硬體）
- UAV_yolo：`python -m pytest tests/ -q` → **66 passed**（含測地、KF、導引、安全、
  LR24 協定對拍、端到端模擬）。
- UAV_yolo 模擬飛行：`python run.py --sim` → 儀表板看鎖定→開導引→俯視圖跟上→急停收斂。
- **驗收**：測試全綠；sim 中四軸會跟上目標、定翼會繞點不衝過頭。

### Stage 1 — UAV_yolo ↔ PX4 SITL（direct MAVLink，四軸）
先不接 NYCU，驗證「UAV_yolo 的測地＋導引＋安全」對真 PX4 正確。
```bash
# 1) PX4 四軸 SITL
cd PX4-Autopilot && git checkout v1.17.0 && make px4_sitl gz_x500
# 2) 讓 SITL 開一條 MAVLink 給地面（QGC/UAV_yolo）：SITL 預設 14550 UDP
#    UAV_yolo 設 mavlink.port 指到該 MAVLink（udp:127.0.0.1:14550 需 pymavlink 連法）
#    link.command_backend: direct
python run.py
```
- QGC 手動 arm、起飛、切 Hold。UAV_yolo 用模擬影像或螢幕餵一台車，鎖定→開導引。
- **驗收**：UAV_yolo 發 DO_REPOSITION 後，SITL 四軸飛向目標並定點；未 arm/未離地時
  安全閘門擋住不發；模式切離 Hold → UAV_yolo 閂鎖停發。
- **意義**：證明測地座標合理、導引正確、安全 gate 有效——**與 NYCU 無關的那半條鏈路先確立**。

### Stage 2 — UAV_yolo ↔ LR24 ↔ global_goto_node ↔ PX4 SITL（整合路徑）
這一階段就是「兩系統整合」的核心驗證，全在桌面用 socat 假 LR24 完成。
```bash
# 1) PX4 四軸 SITL + Agent（同上，或 udp）
# 2) 套多旋翼 patch 後啟動 NYCU 節點（見附件 patch）
socat -d -d pty,raw,echo=0,link=/tmp/nycu_lr24_air pty,raw,echo=0,link=/tmp/nycu_lr24_ground
ros2 launch my_offboard_cpp serial_gps_goto.launch.py \
  lr24_port:=/tmp/nycu_lr24_air allow_multirotor:=true \
  min_relative_altitude_m:=10.0 arrival_horizontal_threshold_m:=15.0 max_target_distance_m:=200.0
# 3) UAV_yolo：link.command_backend: lr24、link.lr24_port=/tmp/nycu_lr24_ground
#    mavlink.port 指到 SITL 的 MAVLink（讀 pose）
python run.py
```
- 先用 NYCU 的 `send_lr24_command.py STATUS` 確認節點就緒；再讓 UAV_yolo 鎖定目標、開導引。
- **驗收**：
  - UAV_yolo 儀表板「LR24 指令」顯示 送/ACK 遞增、`ready_for_goto`。
  - 每筆 GOTO 進 `ENROUTE`，SITL 四軸飛向 UAV_yolo 算出的目標。
  - 移動目標：loiter 點隨 UAV_yolo 更新而平移（send→ACK→send 節流，非每幀）。
  - 座標超界/未 arm/未離地 → node 回 ERR，UAV_yolo 顯示未送出原因，飛機不動。
  - QGC 切模式接管 → node ABORTED、UAV_yolo 閂鎖；**兩層安全都生效**。
- **意義**：**這就是「兩系統可以 work」的實證**。全程無實體、可重複、零風險。

### Stage 3 — 拆槳桌面（真 Pixhawk，四軸機架，**拆槳**）
- 真 Pixhawk＋companion＋SiK＋LR24 全部上機，**螺旋槳拆掉**。
- 兩個 `/dev/serial/by-id/` 不互換、連續重開機十次仍收得到 PX4 topics 與 LR24 ACK。
- 未 arm / GPS 無效 / 座標超界 → 任何 GOTO 都不得造成解鎖或致動器輸出。
- 逐一拔 LR24、拔 Pixhawk serial、關 companion → 確認不產生新指令；RC/ELRS 始終可接管。
- **驗收**：所有故障注入都安全；致動器全程無輸出。

### Stage 4 — 低風險實飛（四軸，空曠合法空域、好天氣、目視 + RC 待命）
- 飛行員起飛、爬到安全高度、切 Hold。
- 先確認 UAV_yolo 目標估計穩定（速度合理、不確定度小），**再**開導引，送 fence 內的保守目標。
- 全程保留目視與 RC 接管；先小範圍、確認高度基準與行為後再擴大。
- **驗收**：四軸穩定跟隨地面目標；每一步都可用 RC 立即接管。之後才換固定翼重跑
  Stage 2→4（固定翼不需多旋翼 patch，且能驗證真正的 standoff 繞行）。

---

## 5. 上機前 UAV_yolo 設定速查（路線 B）

`config/local.yaml`（或用 UI 設定頁）：
```yaml
system: { mode: live }
vehicle: { airframe: multirotor }   # 四軸階段；固定翼改 fixedwing
video:   { source: uvc, uvc_name_hint: "採集卡名稱關鍵字" }   # 或 rtsp + rtsp_url
mavlink: { port: COM3, baud: 57600 }        # SiK：讀 pose（＝QGC 那條）
link:
  command_backend: lr24
  lr24_port: COM4            # 地面 LR24-F（與 SiK 不同一條）
  lr24_baud: 115200
  goto_altitude_ref: rel_home
gimbal:  { present: true, control: roi }    # 有 C-20T；ROI 走 SiK MAVLink
```
UI 設定頁也有這些欄位；「指令送出方式」選 `LR24 → 機上 global_goto_node`。

---

## 6. 安全重點（務必讀）

- **兩層接管閂鎖**：UAV_yolo 偵測模式離開白名單即停發並閂鎖；`global_goto_node` 監控到
  非 AUTO_LOITER 即 ABORT。RC/ELRS 永遠是最高權限，先在安全環境演練切換與 kill。
- **失聯語意不同於「取消」**：PX4 一旦接受某個 GOTO，之後 LR24 斷線**不會**取消該目標，
  飛機會繼續飛去/盤旋在最後接受的點。要停就趁鏈路正常送 `RTL`，或飛行員 RC 直接切安全模式。
  **不要等失聯了才想處置。**
- **高度基準**：GOTO 是相對 home（節點限 30~120m，四軸可 patch 放寬）；別跟 AMSL 搞混。
- **地理圍欄**：node 的距離/高度限制與 UAV_yolo 的圍欄都是「應用層」，**不能取代 PX4
  的 geofence / failsafe**。目標點與整個盤旋圓都要在 PX4 geofence 內、預留風與轉彎裕度。
- **繞行半徑**：路線 B 下固定翼半徑由 PX4 `NAV_LOITER_RAD` 決定，UI 數值不會被送出。
- **checksum 強度**：LR24 用 2-hex XOR，偵錯力不如 CRC。實飛務必同時保留 PX4 geofence、
  嚴格距離/高度限制與獨立 RC 接管。

---

## 7. 我實際讀了哪些碼下這些結論（可追溯）

- `NYCU_UAV_offboard/tools/send_lr24_command.py`：幀格式與 checksum（已在 UAV_yolo
  `tests/test_lr24_link.py` 逐位元對拍 `build_frame`／`parse_response_frame`）。
- `NYCU_UAV_offboard/src/global_goto_node.cpp`：確認 `DO_REPOSITION`(param5/6=lat/lon,
  param7=AMSL)、AUTO_LOITER handshake、固定翼硬 gate（line 530）、STATUS 不含 pose、
  完整安全 gate 清單。
- `NYCU_UAV_offboard/docs/{lr24_communication,gps_goto_program}.md`：8kbps 頻寬、
  一次性 GOTO 語意、失聯行為、SITL 流程。
- UAV_yolo `engine.py` / `mavlink_io/telemetry.py`：確認 link 介面（`store` 讀 pose、
  `send_reposition` 送指令），據此做 CompositeLink 無縫替換 MAVLink 後端。
