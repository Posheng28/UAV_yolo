"""量測影像鏈路端到端延遲（相機→圖傳→採集卡→OpenCV）。

原理（不需要人的反應時間）：
    螢幕上顯示一個大字計時器 → 相機對著螢幕拍 → 畫面繞一圈回到電腦。
    此時「擷取到的那一幀裡顯示的數字」是它被拍下的那一刻，
    而「現在」是它抵達的時刻，兩者相減 = 整條鏈路延遲。
    兩個時間都由同一台電腦的同一個時鐘產生，所以沒有人為誤差。

用法：
    1. 把相機（雲台上那顆、走實際圖傳）對著這個視窗的計時器
    2. python tools/measure_video_latency.py --name "USB Video"
    3. 視窗左邊是計時器、右邊是收到的畫面；讀出兩個數字相減
       （程式也會自動偵測並統計，見下）

自動偵測：
    計時器同時以「二進位光柵」編碼時間（畫面頂端一排黑白格），
    程式在收到的畫面裡找這排格子並解碼，就能自動算出延遲、
    連續取樣後給出中位數與標準差——這是最可靠的讀值。

按 q 結束並列印統計。
"""

from __future__ import annotations

import argparse
import time

import cv2
import numpy as np

# 光柵參數：14 bit 可表示 0~16383 ms（約 16 秒循環），足夠涵蓋任何圖傳延遲
BITS = 14
STRIP_FRAC = 0.14   # 光柵帶佔畫面高度比例——用比例而非固定像素，縮放後才解得出
MARGIN_FRAC = 0.03  # 左右留白比例


def make_pattern_frame(ms: int, w: int = 1280, h: int = 720) -> np.ndarray:
    """產生「計時器＋二進位光柵」畫面。"""
    img = np.full((h, w, 3), 255, np.uint8)

    strip_h = max(int(h * STRIP_FRAC), 8)
    margin = int(w * MARGIN_FRAC)
    cell = (w - 2 * margin) / (BITS + 2)  # 含頭尾兩個定位格

    # 頂端光柵：頭尾各一個黑色定位標記，中間 BITS 個資料格
    x = float(margin)
    cv2.rectangle(img, (int(x), 0), (int(x + cell), strip_h), (0, 0, 0), -1)
    x += cell
    for i in range(BITS):
        bit = (ms >> (BITS - 1 - i)) & 1
        color = (0, 0, 0) if bit else (255, 255, 255)
        cv2.rectangle(img, (int(x), 0), (int(x + cell), strip_h), color, -1)
        x += cell
    cv2.rectangle(img, (int(x), 0), (int(x + cell), strip_h), (0, 0, 0), -1)

    # 人眼可讀的大字計時器（備援：自動解碼失敗時可用肉眼對照截圖）
    text = f"{ms/1000.0:7.3f} s"
    cv2.putText(img, text, (60, h // 2 + 60), cv2.FONT_HERSHEY_SIMPLEX,
                4.5, (0, 0, 0), 10, cv2.LINE_AA)
    cv2.putText(img, "point the camera at this window",
                (60, h - 60), cv2.FONT_HERSHEY_SIMPLEX, 1.1, (90, 90, 90), 2, cv2.LINE_AA)
    return img


def decode_pattern(frame: np.ndarray) -> int | None:
    """從擷取畫面解出光柵編碼的毫秒值。找不到回 None。"""
    h, w = frame.shape[:2]
    # 取畫面頂端一條略高於光柵帶的區域（比例定義，縮放後仍成立）
    strip = frame[: max(int(h * STRIP_FRAC * 0.8), 6), :]
    gray = cv2.cvtColor(strip, cv2.COLOR_BGR2GRAY)
    _, bw = cv2.threshold(gray, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)

    # 取中間一列掃描，找黑白交界
    row = bw[bw.shape[0] // 2, :]
    dark = row < 128
    # 找第一個與最後一個黑區（定位標記）
    idx = np.flatnonzero(dark)
    if idx.size < 10:
        return None
    left, right = int(idx[0]), int(idx[-1])
    span = right - left
    if span < 100:
        return None
    # 標記之間共 BITS+2 格（含兩個定位格）
    cell = span / (BITS + 2)
    value = 0
    for i in range(BITS):
        cx = int(left + cell * (1.5 + i))  # 第 i 個資料格中心
        if not (0 <= cx < row.size):
            return None
        value = (value << 1) | (1 if dark[cx] else 0)
    return value


def open_source(name_hint: str, index: int, width: int, height: int):
    from uav_yolo.vision.source import VideoSource

    src = VideoSource({
        "source": "uvc", "uvc_name_hint": name_hint, "uvc_index": index,
        "width": width, "height": height, "fourcc": "MJPG",
    })
    cap = src._open()
    return cap, src.device_label


def main() -> int:
    ap = argparse.ArgumentParser(description="量測影像鏈路延遲")
    ap.add_argument("--name", default="", help="採集卡名稱關鍵字，例 'USB Video'")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    args = ap.parse_args()

    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

    cap, label = open_source(args.name, args.index, args.width, args.height)
    if cap is None:
        print("!! 無法開啟影像來源；先用設定頁『測試影像來源』確認名稱/索引")
        return 1
    print(f">>> 影像來源：{label}")
    print(">>> 把相機對準『latency timer』視窗，等數值穩定後按 q 結束\n")

    cv2.namedWindow("latency timer", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("latency timer", 1280, 720)
    cv2.namedWindow("captured", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("captured", 800, 450)

    t0 = time.monotonic()
    samples: list[float] = []
    last_report = 0.0

    while True:
        now_ms = int((time.monotonic() - t0) * 1000) % (1 << BITS)
        cv2.imshow("latency timer", make_pattern_frame(now_ms))

        ok, frame = cap.read()
        if ok and frame is not None:
            arrival_ms = int((time.monotonic() - t0) * 1000) % (1 << BITS)
            shot_ms = decode_pattern(frame)
            disp = cv2.resize(frame, (800, 450))
            if shot_ms is not None:
                delay = (arrival_ms - shot_ms) % (1 << BITS)
                if 0 < delay < 3000:  # 合理範圍才收
                    samples.append(delay)
                    cv2.putText(disp, f"{delay} ms", (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 1.6, (0, 220, 0), 3)
            else:
                cv2.putText(disp, "pattern not found", (20, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
            cv2.imshow("captured", disp)

            if samples and time.monotonic() - last_report > 1.0:
                last_report = time.monotonic()
                arr = np.array(samples[-120:])
                print(f"  n={len(samples):4d}  中位數 {np.median(arr):6.0f} ms  "
                      f"標準差 {arr.std():5.0f} ms", end="\r")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    print("\n" + "=" * 60)
    if len(samples) < 10:
        print("樣本太少，無法統計。確認相機真的拍到計時器視窗、光柵沒被遮住。")
        return 1
    arr = np.array(samples)
    med = float(np.median(arr))
    print(f"樣本數 {len(arr)}｜中位數 {med:.0f} ms｜平均 {arr.mean():.0f} ms｜"
          f"標準差 {arr.std():.0f} ms")
    print(f"\n>>> 把設定頁「圖傳延遲 ms」填入：{med:.0f}")
    print("    （中位數比平均可靠：不受偶發掉幀影響）")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
