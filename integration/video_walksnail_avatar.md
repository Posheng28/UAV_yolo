# 影像鏈路：Walksnail Avatar（Caddx）+ VRX 接法

## 結論先講

Walksnail Avatar VRX **只有 Mini HDMI 輸出，沒有網路孔、不提供 RTSP**，
所以走 **HDMI → 採集卡 → USB → UAV_yolo（`video.source: uvc`）**。
RTSP 那條路對這台機器不適用。

```
機上：Avatar 相機（裝在 C-20T 雲台）→ Avatar VTX
        │ 1080p60 / H.265，空中段約 22ms
        ▼
地面：Walksnail Avatar VRX ──Mini HDMI──→ HDMI 採集卡 ──USB──→ 筆電 → UAV_yolo
```

## 設定

```yaml
video:
  source: uvc
  uvc_name_hint: "採集卡在裝置清單裡的名稱關鍵字"   # 設定頁下方會列出
  width: 1920          # VRX 走 HDMI 是 1080p60
  height: 1080
  fourcc: MJPG         # 便宜的 USB2 採集卡要靠 MJPG 才吃得下 1080p；USB3 可留空
  latency_ms: 0        # ← 必須實測後填入，見下
```

**不要再經 OBS。** 直接開採集卡少一層轉手（OBS 虛擬相機多 50~100ms 以上），
而且 OBS 可能悄悄縮放/改比例，那會讓相機內參失效。

## 需要的線材

VRX 是 **Mini HDMI（type C）**，採集卡通常是標準 HDMI（type A）——
要一條 **Mini HDMI → HDMI** 線或轉接頭，別買錯成 Micro HDMI。

## ⚠️ 延遲：官方的 22ms 不是你要填的數字

22ms 是**空中段**（相機到眼鏡的 glass-to-glass）。你的鏈路後面還有
**VRX 解碼 → HDMI → 採集卡 → USB → Windows/DirectShow → OpenCV**，
這幾段加起來通常再多 **60~150ms**。所以到程式手上的總延遲多半落在 **80~200ms**。

**一定要實測**，別直接填 22：

1. 手機開碼錶（顯示到 0.01 秒），放在飛行相機前面。
2. 地面站畫面出來後，同時截圖畫面與手機本身。
3. 兩者顯示時間的差 = 總延遲，填進設定頁「圖傳延遲 ms」。
4. 換解析度/畫質模式/採集卡之後要重測。

沒補償的話會有「延遲 × 飛行速度」的**系統性偏移**（20 m/s × 150ms = 3 m），
而且方向固定，KF 濾不掉。

## ⚠️ 校正一定要「整條鏈路一起校」

Avatar 會重新編碼、可能裁切/縮放畫面，所以**不能拿相機單獨校正**。
必須用「實際飛行相機 → VTX → VRX → 採集卡 → 筆電」收到的畫面去跑校正頁。
本專案的校正頁本來就是從即時串流擷取，照著用就是對的。

**下列任一項變動就要重新校正**：圖傳解析度或畫質模式、換相機/鏡頭、
換 VRX 或採集卡、改採集解析度。

> 系統會自動把內參縮放對齊「實際收到的幀尺寸」，所以就算採集卡沒照你要求的
> 解析度輸出也不會算錯；但**鏡頭畸變與裁切只能靠重新校正**。

## 其他實務

- **OSD**：Avatar 支援 Betaflight/INAV canvas OSD，疊字是燒進畫面的。
  接 PX4 通常不會有這層 OSD；若有，把它關掉或縮到角落，免得 YOLO 誤抓。
- **VRX 有 SD 卡槽（最大 256GB）**：飛行時錄一份原始影像，回來可以用
  `video.source: file` + `file_path` 把整段重播進 UAV_yolo，離線重跑追蹤與
  測地做事後分析——**不用再飛一次就能除錯**，很值得每次都錄。
- 天線：VRX 附兩支全向 + 兩片內建指向天線；地面站擺位仍照一般 FPV 原則。
