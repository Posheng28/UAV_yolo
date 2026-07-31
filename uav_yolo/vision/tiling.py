"""切塊推論（tiled inference）：讓小目標重新變得看得見。

為什麼需要
------------------------------------------------
偵測器把整幀縮到網路輸入尺寸（如 1920→960），目標跟著縮一半。空拍時目標本來
就小，縮完更小，低於某個像素數就徹底偵測不到——而且是「完全看不到」，不是
「信心低一點」。

實測（weights/toycar.pt，把訓練影像縮小來模擬飛更高）：

    等效高度   目標寬    整幀推論   切塊推論
      1 m     195 px     100%       52%
      2 m      98 px     100%      100%
      4 m      49 px      64%      100%
      8 m      24 px       0%       88%

切塊把畫面分成數塊各自送進網路，每塊縮放比例小得多，目標相對網路輸入就變大。
8 公尺那格從「完全瞎掉」變成 88%，不需要重新訓練模型。

代價與取捨
------------------------------------------------
推論次數變成 (格數 + 1) 倍。DirectML 上單次約 12ms，3×2 格加整幀＝7 次≈84ms
（約 12Hz）——比整幀的 30Hz 慢，但對「完全偵測不到」是划算的交易。

**整幀那一次不能省**：目標夠大時會被切線切斷（上表 1m 那格掉到 52%），
所以整幀與切塊的結果要合併，取兩者的聯集再去重。

本模組只做幾何與合併，不碰模型，可完整單元測試。
"""

from __future__ import annotations

import math

Box = tuple[float, float, float, float]  # x1, y1, x2, y2


def normalise_mode(value) -> str:
    """把設定值正規化成 off / auto / on。

    🔴 YAML 1.1 會把 `off` / `on` / `yes` / `no` 解析成**布林**，不是字串。
    所以 `tiling: on` 讀進來是 `True`，跟字串 `"on"` 比對永遠不相等——
    切塊不會啟動，而且完全沒有錯誤訊息。設定檔裡雖然已改成加引號，
    但使用者手改或舊設定檔仍可能是布林，所以這裡一律接受兩種形式。
    """
    if isinstance(value, bool):
        return "on" if value else "off"
    text = str(value).strip().lower()
    if text in ("on", "true", "yes", "always"):
        return "on"
    if text in ("auto",):
        return "auto"
    return "off"


def auto_grid(frame_w: int, frame_h: int, input_w: int, input_h: int,
              overlap: float = 0.2) -> tuple[int, int]:
    """算出「塊會被放大而不是縮小」的最小格數。

    ⚠️ 切塊唯一的效果來源是「塊被放大送進網路，目標相對變大」。若塊本身就比
    模型輸入大，前處理會把它**縮小**——切了等於沒切，甚至更糟。

    實測（ONNX 輸入 960x544、目標約 24px、等效 8m 高）：

        格數   塊尺寸      縮放比   偵測率
        3x2   768x648     0.84      10%   ← 塊比輸入大，被縮小
        3x3   768x432     1.25      80%
        4x3   576x432     1.26      90%
        4x4   576x324     1.67     100%

    所以格數不能寫死，要從模型輸入尺寸推。回傳的是「剛好不縮小」的最小格數；
    想更靈敏就再往上加（代價是推論次數等比增加）。
    """
    k = 1.0 + overlap
    nx = max(1, math.ceil(frame_w * k / max(input_w, 1)))
    ny = max(1, math.ceil(frame_h * k / max(input_h, 1)))
    return nx, ny


def tile_rects(width: int, height: int, nx: int, ny: int,
               overlap: float = 0.2) -> list[tuple[int, int, int, int]]:
    """把畫面切成 nx×ny 塊，相鄰塊重疊 overlap 比例。

    重疊是必要的：剛好落在切線上的目標會被兩塊各切一半，兩邊都認不出來。
    回傳 [(x0, y0, w, h)]，保證不超出畫面。
    """
    if nx < 1 or ny < 1:
        raise ValueError("nx/ny 至少為 1")
    if not 0.0 <= overlap < 0.9:
        raise ValueError("overlap 需在 [0, 0.9)")

    tw = min(width, int(round(width / nx * (1 + overlap))))
    th = min(height, int(round(height / ny * (1 + overlap))))
    out = []
    for iy in range(ny):
        for ix in range(nx):
            # 平均分佈左上角，最後一塊靠右/靠下對齊，確保涵蓋整個畫面
            x0 = 0 if nx == 1 else int(round(ix * (width - tw) / (nx - 1)))
            y0 = 0 if ny == 1 else int(round(iy * (height - th) / (ny - 1)))
            out.append((x0, y0, tw, th))
    return out


def offset_boxes(boxes: list[Box], x0: int, y0: int) -> list[Box]:
    """把某一塊裡的座標平移回整幀座標。"""
    return [(b[0] + x0, b[1] + y0, b[2] + x0, b[3] + y0) for b in boxes]


def iou(a: Box, b: Box) -> float:
    ix1, iy1 = max(a[0], b[0]), max(a[1], b[1])
    ix2, iy2 = min(a[2], b[2]), min(a[3], b[3])
    iw, ih = max(0.0, ix2 - ix1), max(0.0, iy2 - iy1)
    inter = iw * ih
    if inter <= 0.0:
        return 0.0
    area_a = max(0.0, a[2] - a[0]) * max(0.0, a[3] - a[1])
    area_b = max(0.0, b[2] - b[0]) * max(0.0, b[3] - b[1])
    union = area_a + area_b - inter
    return inter / union if union > 0 else 0.0


def merge_detections(items: list[tuple[Box, str, float]],
                     iou_threshold: float = 0.45) -> list[tuple[Box, str, float]]:
    """合併整幀與各塊的偵測結果，去掉重複框（同一台車會被多塊各看到一次）。

    以信心排序後貪婪保留：與已保留框重疊超過門檻的就丟棄。同一個物體被
    相鄰兩塊各偵測一次時，重疊會很高，因此會被正確合併成一個。

    去重**跨類別**：COCO 權重常把同一台車在不同塊各判成 car/truck，
    若只在同類別內去重，一台車會留兩個高度重疊的框——追蹤器開兩條
    track，鎖定會在兩個 ID 間跳。空間上高度重疊必是同一物體。
    """
    kept: list[tuple[Box, str, float]] = []
    for box, cls_name, conf in sorted(items, key=lambda it: it[2], reverse=True):
        if any(iou(box, k[0]) > iou_threshold for k in kept):
            continue
        kept.append((box, cls_name, conf))
    return kept


def should_tile(boxes: list[Box], frame_width: int, small_frac: float = 0.06) -> bool:
    """依上一幀的目標大小決定這一幀要不要切塊（auto 模式）。

    目標很大時切塊反而會把它切斷（實測 1m 那格 100%→52%），而且白花數倍算力；
    目標小到整幀推論會漏才值得切。沒有偵測到任何東西時也切——因為
    「什麼都沒看到」正是目標太小的典型症狀。
    """
    if not boxes:
        return True
    widest = max((b[2] - b[0]) for b in boxes)
    return widest / max(frame_width, 1) < small_frac
