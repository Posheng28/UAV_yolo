# UAV_yolo — 視覺目標追蹤 × PX4 導引地面站

NYCU UAV 的第二代目標追蹤系統：YOLO 從空拍畫面偵測地面車輛，經**完整射線測地**
（相機內參＋畸變＋雲台/機體三軸姿態）換算 GPS 座標，用 **Kalman 濾波**估計目標
位置與速度，再依載體種類發出對應的 **PX4 導引指令**——旋翼跟隨懸停、固定翼
standoff 繞行。附完整 Web 地面站（儀表板／任務檢查清單／參數設定／相機校正）。

取代第一代 `yolo26/track.py` 的核心理由：

| 問題（第一代） | 解法（本系統） |
|---|---|
| 像素×GSD 線性換算，忽略 roll/pitch（30° 滾轉在 100m 高＝57m 誤差） | 完整射線投影：去畸變→姿態合成→地面求交 |
| 5 幀滑動平均，目標一丟就歸零 | KF 位置+速度聯合估計，盲區用速度外推（coast） |
| 固定翼會「飛到目標頭上」，車急停就衝過頭 | standoff 繞行：指令是「繞目標預測位置轉圈」，不是「飛到目標」 |
| 只拿第一個偵測框，多車亂跳 | 追蹤 ID 鎖定 + 世界座標重鎖定 |
| ArduPilot Copter 模式編號誤用在 PX4/定翼 | PX4 custom_mode 正確解析 + 接管閂鎖 |
| 遙測非阻塞單讀，姿態延遲無上限累積 | 獨立執行緒抽乾 buffer，幀時間戳內插對齊 |
| OBS 虛擬相機中轉 | 採集卡直開（OBS/RTSP/檔案亦可選） |

## 快速開始

```bash
pip install -r requirements.txt

# 無硬體演練（合成影像+合成飛控，全功能可玩）
python run.py --sim

# 實機
python run.py
```

> **要與機上 NYCU_UAV_offboard（RPi/Orin 做 offboard）整合、或想看實體驗證流程？**
> 讀 [`integration/INTEGRATION.md`](integration/INTEGRATION.md)：架構、接點對照、
> Stage 0 sim → 1 SITL 直連 → 2 SITL 經 LR24 整合 → 3 拆槳桌面 → 4 低風險實飛，
> 以及四軸用的 [`integration/global_goto_multirotor_patch.md`](integration/global_goto_multirotor_patch.md)。

開瀏覽器進 `http://127.0.0.1:8600`。**建議第一次先用模擬模式把流程走一遍**：
看著儀表板等 auto 鎖定 → 開導引 → 在「俯視圖」看載具跟上目標 → 切設定頁把載體
換成固定翼、重啟引擎，看 standoff 繞行的行為差異。

## 系統架構

```
影像(採集卡/RTSP) ─▶ YOLO 偵測+ID鎖定 ─▶ 射線測地 ─▶ 目標KF(位置+速度)
                                            ▲               │
數傳(COM) ◀─▶ PX4 遙測(姿態/位置/雲台回報，時間戳內插)        │
    ▲                                                       ▼
    └── DO_REPOSITION(旋翼:跟隨點 / 定翼:繞行圓心+半徑) ◀─ 導引律+安全閘門
    └── DO_SET_ROI_LOCATION(雲台持續指向目標估計位置)
```

模組對應：`vision/`（影像+偵測+校正）、`geometry/`（相機模型+測地）、
`estimation/`（KF）、`guidance.py`（導引律）、`mavlink_io/`（PX4 通訊）、
`safety.py`（閘門）、`engine.py`（狀態機串接）、`simulation.py`（合成閉環）、
`webapp/`（地面站 UI）。

## 硬體接線與 PX4 參數

### 雲台（XF C-20T 三軸）

C-20T 支援 UART（MAVLink）/ S.BUS / PWM。**建議走 MAVLink 接飛控**，讓 PX4
統一管雲台，地面站只發 ROI 座標、飛控用高頻姿態自動指向：

1. 雲台 UART 接飛控任一空閒 TELEM/UART 口（先用原廠 GimbalConfig 軟體確認
   雲台端已設 MAVLink 模式與鮑率）。
2. PX4 參數：`MNT_MODE_IN=4`（MAVLink gimbal protocol v2）、
   `MNT_MODE_OUT=MAVLink`（走 UART）或 `AUX`（走 PWM 三通道）。
3. 對應 UART 的 `MAV_x_CONFIG` 設為該序列埠、協定 gimbal。
4. 驗證：QGC 手動點地圖「指向此處」雲台會轉；本系統儀表板「雲台」欄顯示
   `roi・有回報` 表示收到 `GIMBAL_DEVICE_ATTITUDE_STATUS` 回報（測地會優先用它）。

若雲台只接了 PWM 而無回報，設定頁把「控制方式」改 `pitchyaw`（系統自己算角度
下指令，測地用指令角），或 `none`＋填固定安裝角。

### 影像

相機 HDMI → 採集卡 → 筆電 USB。設定頁選 `HDMI 採集卡`，用「名稱關鍵字」鎖定
裝置（清單顯示在設定頁下方），**不要**再開 OBS 佔用裝置。數位圖傳走網路的話選
RTSP 填網址。

### 數傳

SiK 電台插筆電，設定頁填 COM 埠（裝置管理員查）與鮑率 57600。連上後儀表板
「載具」卡會即時顯示模式/高度/姿態——**高度、姿態、位置全部來自這條數傳的
`GLOBAL_POSITION_INT`/`ATTITUDE`，不需要另外接任何感測器**。

### 定翼專用

`NAV_LOITER_RAD` 設成與 UI「繞行半徑」相同的值（`DO_REPOSITION` 的半徑參數在
部分 PX4 版本有 [已知問題](https://github.com/PX4/PX4-Autopilot/issues/24612)，
兩邊設一致最保險）。

## 操作流程（實機）

1. 過一遍「任務檢查」頁的清單（起飛前把「全部取消勾選」按了重新檢查）。
2. 載具起飛，切 **Hold（AUTO.LOITER）** 模式。
3. 儀表板確認：數傳正常、Home 已取得、目標已鎖定（auto 或點偵測列表）。
4. 確認目標估計穩定（速度合理、不確定度小）→ 按「導引：啟用」。
5. 系統開始以 ≤1Hz 發 `DO_REPOSITION`；「未發送原因」清單隨時告訴你哪個
   閘門擋住了。
6. **飛行員任何時刻撥模式開關即接管**——系統偵測到模式離開 Hold 立即停發並
   閂鎖，切回 Hold 也不會自動恢復，必須在 UI 重新啟用導引。

## 安全設計

- 導引預設關閉；啟用需 UI 明確操作（含確認彈窗）。
- 逐指令檢查：模式白名單、arm 狀態、數傳逾時（2s）、目標估計逾時、
  距 Home 圍欄（預設 500m）、高度上下限（20~120m）、1Hz 速率上限。
- 量測雙重防護：KF innovation gate（統計）+ 30m 硬跳變上限。
- 接管閂鎖：模式切出白名單即鎖定，防止「切回去又自己動起來」。

## 測試

```bash
python -m pytest tests/ -q    # 52 項：幾何/KF/導引/安全/視覺/端到端模擬
```

端到端模擬（`tests/test_sim_e2e.py`）驗證的就是實機要的行為：測地誤差、
盲區 coast、**目標急停後定翼繞停點不過頭**、接管閂鎖、指令節流。

## 已知限制與後續方向

- 平地假設：地面=Home 高度的水平面。丘陵地形需接 DEM（`geolocate.py` 預留了
  `ground_z` 參數）。
- 權重：`weights/best.pt` 不進 git，從訓練機拷貝（沒有時自動退 COCO 預訓練
  `yolo26n.pt`，可先偵測一般車輛）。
- 僅支援 PX4。ArduPilot 需在 `mavlink_io/` 加模式表與 guided 指令轉接。
- RTSP 來源延遲取決於圖傳鏈路，必要時在 `vision/source.py` 加低延遲參數。
