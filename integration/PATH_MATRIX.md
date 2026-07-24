# 路徑總表：每條路徑的準備清單與驗證狀態

> 「這套系統有哪些可走的路、每條路上場前要準備什麼、驗證到什麼程度」的單頁總覽。
> 驗證圖例：✅ 軟體已驗（自動測試）｜🧪 SITL 可驗（照 INTEGRATION.md 階段）｜⚠️ 只能實機驗。
> 最後更新：2026-07-24（路徑總體檢後），測試 113 綠。

## 總體檢結果摘要（2026-07-24）

多代理審查＋人工逐項查證後，確認並修復 12 項（全數有迴歸測試鎖定）：

| 嚴重度 | 問題 | 影響 |
|---|---|---|
| 高 | 設定頁存檔後安全門檻/導引/偵測參數**不會**套用到執行中引擎 | 改嚴圍欄以為生效，實際仍用舊值 → 已改為儲存即熱套用（保留接管閂鎖） |
| 高 | 重新啟用導引後第一筆指令被舊 deadband 吃掉 | 接管後交還控制，目標靜止時一筆都不發、UI 卻全綠 → 已清除舊指令記錄 |
| 高 | 擷取執行緒遇 cv2 例外會靜默死亡 | UI 顯示連線正常但畫面永久凍結 → 已加防護＋自動重連 |
| 高 | UVC 重連「先開新再放舊」 | DirectShow 獨占 → 圖傳中斷後永遠連不回來 → 已改先放後開 |
| 中 | GPS_RAW_INT 的 eph 誤當公分（實為 HDOP×100） | 精度閘門單位錯 → 改用 MAVLink 2 h_acc/v_acc（公尺），無則退 DOP 門檻 |
| 中 | 飛控重開機後訊息串流不會恢復 | SET_MESSAGE_INTERVAL 被洗掉 → 心跳在但姿態斷流 >5s 自動重要求 |
| 中 | home 只收第一筆（PX4 解鎖時會重設 home） | 高度基準偏移 → 高度基準改用最新、NED 原點仍鎖第一筆 |
| 中 | RTL 後排隊中的 GOTO 仍會送出 | 返航後又被導回目標 → RTL 清空佇列 |
| 中 | mavlink.gcs_system_id 是死設定（寫死 255） | 與 QGC 撞 sysid 無法避開 → 已接通 |
| 低 | 手動點選的 ID 永不過期 | 偵測器重啟後 ID 重複可能被劫持 → 60 幀過期 |
| 低 | 校正可混入不同解析度樣本 | 整組校正壞掉 → 尺寸改變即拒收並要求重來 |
| 低 | fourcc 設錯長度會讓啟動整個炸掉／vehicle.firmware 死設定 | 已防護／已改為啟動驗證 |

另外新增三類**契約測試**永久鎖住 UI↔API↔設定檔一致性（斷掉的端點、假的設定 key、
缺失的 DOM id、狀態欄位缺漏——這些都是「靜默失效」型問題，人工看不出來）。

> **誠實聲明**：審查的六個角度中，config 稽核完整跑完（4 項確認全修）；vision/links
> 的發現由人工逐項讀碼查證（確認 9 項、駁回其餘）；engine 與 UI 契約兩路的自動審查
> 因額度中斷，改由人工審查＋契約測試補位（人工審查找到高severity的 deadband bug）。
> 已驗證的就是上表；**未逐字重驗的部分以 113 項自動測試與 SITL 階段把關**。

---

## 1. 指令後端（目標 GPS 怎麼送出去）

### 1a. `direct` — 數傳直插 Pixhawk（四軸首驗，預設）

**準備清單**
- [ ] SiK 數傳一端插 Pixhawk **TELEM1**、一端插筆電；設定頁填 COM 埠、57600
- [ ] 確認 QGC 同時只有一個程式佔 COM 埠（QGC 與 UAV_yolo 擇一，或 QGC 走另一條）
- [ ] 儀表板「載具」卡：模式/高度/姿態即時跳動、Home 已取得
- [ ] `safety.allowed_modes` 含 AUTO.LOITER；圍欄/高度上下限依場地調
- [ ] 飛行員演練：撥模式開關 → 確認 UI 顯示接管閂鎖、停止發送

**驗證狀態**
- ✅ DO_REPOSITION 組包語意（COMMAND_INT frame/param）— 對照 PX4 文件寫成，單元覆蓋
- ✅ 安全閘門全套（模式白名單/arm/離地/GPS品質/圍欄/限速/接管閂鎖）— 12 測試
- ✅ 導引閉環（跟隨、急停收斂、盲區 coast）— sim e2e 8 測試
- 🧪 對真 PX4 的 ACK/行為 — Stage 1（SITL gz_x500）
- ⚠️ 實體電台頻寬/掉包、真 GPS 品質 — Stage 3 拆槳 → Stage 4 實飛

### 1b. `lr24` — 經 LR24-F 給機上 global_goto_node（NYCU 整合，定翼主用）

**準備清單**
- [ ] companion（RPi/Orin）建置：ROS 2 + px4_msgs release/1.17 + Micro-XRCE-DDS Agent v2.4.3
- [ ] Pixhawk TELEM2：`MAV_1_CONFIG=0`、`UXRCE_DDS_CFG=TELEM2`、`SER_TEL2_BAUD=921600`
- [ ] `ros2 topic list | grep /fmu/` 看得到（含 `vehicle_status_v1`）
- [ ] 兩條電台都插筆電：SiK（讀 pose）＋ LR24-F（送 GOTO），設定頁分別填埠
- [ ] `send_lr24_command.py STATUS` 回 `ready_for_goto=true`
- [ ] 四軸要跑這條 → 先套 `global_goto_multirotor_patch.md` 並過 SITL
- [ ] 定翼：PX4 `NAV_LOITER_RAD` 設成 UI 繞行半徑（LR24 幀傳不了半徑）

**驗證狀態**
- ✅ 幀格式/checksum 與 NYCU 官方逐位元對拍 — 14 測試
- ✅ 通道行為（單筆在途、coalesce、ERR 重試、stale 丟棄、RTL）— 單元
- ✅ **引擎×LR24 端到端閉環**（rel-home 高度語意、無半徑欄位、ROI 仍走 MAVLink、接管停發）— 2 e2e
- 🧪 與真 global_goto_node 對接 — Stage 2（socat 假 LR24 + SITL）
- ⚠️ LR24 實體射程/干擾 — 沿用 NYCU 已做過的 link test 紀律

### 1c. `sim` — 模擬（演練/迴歸）

- 準備：`python run.py --sim`，無硬體。
- ✅ 全鏈路合成閉環＝自動測試本體；UI 演練照 help 彈窗流程。

---

## 2. 影像來源

### 2a. `uvc` — Walksnail Avatar VRX → HDMI 採集卡（現行方案）

**準備清單**
- [ ] Mini HDMI（type C）→ HDMI 線；VRX → 採集卡 → 筆電
- [ ] **關 OBS**（別佔裝置、別多一手延遲）
- [ ] 設定頁：來源=uvc、名稱關鍵字鎖採集卡、1920×1080、fourcc=MJPG
- [ ] 按「🎥 測試影像來源」：實際解析度=1920×1080、FPS≥25
- [ ] **整鏈路校正**（相機→VTX→VRX→採集卡收到的畫面跑校正頁，RMS<1px）
- [ ] **實測延遲**（鏡頭拍手機碼錶）填 `latency_ms`（預期 80~200ms，別填官方 22）
- [ ] 圖傳 OSD 關閉或移角落

**驗證狀態**
- ✅ 名稱鎖定/索引 fallback、MJPG、斷線重連、停滯 watchdog — 單元+邏輯
- ✅ 實際幀尺寸≠設定時內參自動對齊 — e2e（防 VRX 硬吐 1080p 的坑）
- ✅ 延遲補償消除測地偏移 — e2e（模擬 300ms 鏈路）
- ⚠️ 你那張採集卡的實際行為（解析度/格式/延遲）— 測試按鈕當場驗

### 2b. `rtsp` — IP 圖傳（未來備援，已生產級）

**準備清單**
- [ ] 相機/圖傳設**固定 IP**；筆電同網段；URL 填設定頁
- [ ] transport 留 auto（UDP 優先自動退 TCP）
- [ ] 按「🎥 測試影像來源」確認通、記下實際解析度
- [ ] **重新校正＋重測延遲**（換影像方案兩件事必做）

**驗證狀態**
- ✅ UDP→TCP 退避順序、開流 5s 限時（OpenCV 寫死 30s 卡死已繞開，實測 30s→3s）、
  停滯 watchdog、連上但無畫面判失敗 — 8 測試 + 瀏覽器實測
- ⚠️ 真實圖傳的串流相容性/延遲 — 接上後用測試按鈕+碼錶驗

### 2c. `file` — VRX SD 卡錄影重播（離線除錯）

- 準備：VRX 插 SD 卡錄影；回來 `source: file` 指到檔案。
- ✅ 循環播放、探測 — 測試覆蓋。用途：不用重飛就重跑追蹤/測地。

### 2d. `obs` — 相容舊流程（不建議）

- 🟡 程式路徑在（名稱鎖定同 uvc），無專屬測試；僅救急用，正式流程勿用。

---

## 3. 雲台路徑（C-20T）

### 3a. `roi` — 飛控自動指向（建議）

**準備清單**
- [ ] C-20T UART 接 Pixhawk 空閒 TELEM；GimbalConfig 確認雲台端 MAVLink 模式
- [ ] PX4：`MNT_MODE_IN=4`、`MNT_MODE_OUT` 依接線、對應 `MAV_x_CONFIG`
- [ ] 地面測試：QGC 地圖「指向此處」雲台會轉
- [ ] 儀表板雲台欄顯示「roi・有回報」（= 收到 GIMBAL_DEVICE_ATTITUDE_STATUS，測地優先用回報）

**驗證狀態**
- ✅ ROI 節流/更新邏輯、回報→測地（yaw_is_earth 兩種慣例）、繞行中目標可見率>70% — e2e
- ⚠️ **C-20T 實際回報的座標慣例**（earth-frame 旗標是否如 spec）— 地面實測必驗：
  雲台指向已知地標，看 UI 測地座標是否合理；不合理就把 attitude_source 切 commanded

### 3b. `pitchyaw` — 地面直接下角度（fallback）

- 準備：同上接線；設定頁 control=pitchyaw。
- ✅ 角度解算幾何 — 單元；🟡 引擎內此路徑無 e2e（幾何同源 roi，風險低）
- 用途：雲台無回報或 ROI 行為異常時的替代。

### 3c. `none` — 固定安裝（無雲台備援）

- 準備：`gimbal.present=false`、`mount_deg` 填實際安裝角（朝下=-90）。
- ✅ 機體姿態合成測地（含滾轉位移手算案例）— 單元；🟡 引擎內無 e2e。
- ⚠️ 注意：無雲台時定翼壓坡度會頻繁丟視野，只適合旋翼低速。

---

## 4. 載體

| | multirotor（先） | fixedwing（後） |
|---|---|---|
| 導引 | 跟隨/正上方，✅ e2e | standoff 繞行、急停不衝頭，✅ e2e |
| direct 後端 | ✅（含半徑參數不需要） | ✅（DO_REPOSITION param3 半徑；⚠️ PX4 部分版本 param3 失效→NAV_LOITER_RAD 兜底） |
| lr24 後端 | 需 multirotor patch（🧪 SITL 驗） | 原生支援，✅ lr24 e2e |
| 切換 | 設定頁載體→重啟引擎，✅ 瀏覽器實測 | 同左 |

---

## 5. 只能實機確認的清單（軟體測不到，上場必查）

1. COM 埠對應與電台實際連通（兩條別插反）
2. C-20T 回報幀慣例（見 3a ⚠️）
3. 採集卡實際輸出（測試按鈕）
4. 圖傳延遲實測值（碼錶法）
5. 整鏈路校正 RMS
6. PX4 收 DO_REPOSITION 的 ACK 與模式切換行為（SITL 先驗過再實機）
7. RC 接管演練（每次任務前）
8. YOLO 權重對實際場地目標的偵測品質（best.pt 從訓練機拷來後試拍）

---

## 6. 已知限制（設計取捨，非 bug）

- 平地假設：地面=Home 高度水平面（丘陵需 DEM，介面已預留 ground_z）
- lr24 模式定翼繞行半徑由 `NAV_LOITER_RAD` 決定（LR24 幀無此欄位）
- PX4 一旦接受 GOTO，鏈路斷線不會取消（要停：趁通時 RTL 或 RC 接管）
- 僅支援 PX4（ArduPilot 需另寫模式表與指令轉接）
