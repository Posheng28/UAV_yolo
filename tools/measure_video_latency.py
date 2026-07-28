"""量測影像鏈路端到端延遲（螢幕→相機→圖傳→採集卡→OpenCV）。

原理：不靠人的反應時間
-----------------------------------
電腦全螢幕在黑白之間翻轉，並記下「翻轉的那一刻」；相機拍著這個螢幕，
畫面繞一圈回到電腦後，程式偵測擷取畫面裡的亮度何時跟著翻轉，
記下「察覺的那一刻」。兩個時刻都由同一台電腦的同一個時鐘產生，
相減就是整條鏈路的延遲，完全沒有人為誤差。

為什麼用「整片閃爍」而不是條碼
-----------------------------------
早期版本在畫面頂端畫二進位條碼，然後在擷取畫面的頂端解碼。
那假設「螢幕填滿整個擷取畫面」，但實際上相機看到的是螢幕在視野中的
一個矩形，而且 Walksnail VRX 還會在上緣疊 OSD（NO SD／電壓／訊號）。
條碼幾乎永遠不會落在它預期的位置，結果就是一直 pattern not found。
整片亮度翻轉不管螢幕在畫面哪裡、多大、有沒有被 OSD 遮到都成立。

用法
-----------------------------------
    python tools/measure_video_latency.py --name "USB Video"

    1. 會跳出一個黑白閃爍的視窗，把相機對著它（占畫面越大越好）
    2. 程式自動校準亮暗，然後開始取樣，畫面上會顯示每次的延遲
    3. 收滿樣本會印出中位數 → 填進設定頁的「圖傳延遲 ms」
    按 q 可提前結束。

讀值的兩個但書
-----------------------------------
  * 量到的值含「螢幕本身的顯示延遲」（一般 5~15ms），會讓結果略為高估。
  * 解析度受限於採集幀率：30fps 的量化下限約 33ms，所以個位數差異沒有意義。
"""

from __future__ import annotations

import argparse
import random
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

WIN_FLASH = "latency flash  (point the camera here)"
WIN_VIEW = "captured"
ROI_FRAC = 0.30          # 取擷取畫面中央這個比例的區域測亮度
MIN_CONTRAST = 18.0      # 亮暗差低於此值就是沒對準／太暗，不要硬測
SETTLE_S = 0.25          # 翻轉後最少等這麼久才接受偵測，濾掉殘影抖動


def open_source(name_hint: str, index: int, width: int, height: int):
    from uav_yolo.vision.source import VideoSource

    src = VideoSource({
        "source": "uvc", "uvc_name_hint": name_hint, "uvc_index": index,
        "width": width, "height": height, "fourcc": "MJPG",
    })
    return src._open(), src.device_label


def roi_mean(frame: np.ndarray) -> float:
    h, w = frame.shape[:2]
    dy, dx = int(h * (1 - ROI_FRAC) / 2), int(w * (1 - ROI_FRAC) / 2)
    roi = frame[dy:h - dy, dx:w - dx]
    return float(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY).mean())


def draw_roi(frame: np.ndarray) -> np.ndarray:
    h, w = frame.shape[:2]
    dy, dx = int(h * (1 - ROI_FRAC) / 2), int(w * (1 - ROI_FRAC) / 2)
    out = cv2.resize(frame, (800, int(800 * h / w)))
    sy, sx = 800 / w, 800 / w
    cv2.rectangle(out, (int(dx * sx), int(dy * sy)),
                  (int((w - dx) * sx), int((h - dy) * sy)), (0, 200, 255), 2)
    return out


def show_flash(white: bool, headless: bool = False) -> None:
    if headless:
        _FakeCap.flash_state = white
        return
    img = np.full((720, 1280, 3), 255 if white else 0, np.uint8)
    cv2.imshow(WIN_FLASH, img)


class _FakeCap:
    """自我檢驗用：延遲固定的假相機。

    有了它才敢叫人相信量出來的數字——先證明這套邏輯能把「已知的延遲」
    量回來，再拿去量未知的真實鏈路。
    """

    flash_state = False

    def __init__(self, delay_ms: float, fps: float = 30.0):
        self.delay_s = delay_ms / 1000.0
        self.period = 1.0 / fps
        self._history: list[tuple[float, bool]] = []
        self._next = time.monotonic()

    def read(self):
        now = time.monotonic()
        self._history.append((now, _FakeCap.flash_state))
        if now < self._next:               # 模擬幀率上限
            time.sleep(max(0.0, self._next - now))
            now = time.monotonic()
        self._next = now + self.period
        # 這一幀「拍到」的是 delay 秒前的螢幕狀態
        shot_t = now - self.delay_s
        state = False
        for t, s in self._history:
            if t <= shot_t:
                state = s
            else:
                break
        self._history = [h for h in self._history if h[0] > shot_t - 1.0]
        level = 200 if state else 40
        return True, np.full((240, 320, 3), level, np.uint8)

    def release(self):
        pass


def main() -> int:
    ap = argparse.ArgumentParser(description="量測影像鏈路延遲")
    ap.add_argument("--name", default="", help="採集卡名稱關鍵字，例 'USB Video'")
    ap.add_argument("--index", type=int, default=1)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--samples", type=int, default=20, help="要收幾次翻轉")
    ap.add_argument("--selftest", type=float, default=None, metavar="MS",
                    help="不接相機，用已知延遲的假相機驗證量測邏輯本身")
    args = ap.parse_args()

    headless = args.selftest is not None
    if headless:
        print(f">>> 自我檢驗：注入已知延遲 {args.selftest:.0f} ms，看能不能量回來")
        cap, label = _FakeCap(args.selftest), f"fake({args.selftest:.0f}ms)"
    else:
        cap, label = open_source(args.name, args.index, args.width, args.height)
        if cap is None:
            print("!! 無法開啟影像來源；先用設定頁『測試影像來源』確認名稱/索引")
            return 1
    print(f">>> 影像來源：{label}")

    if not headless:
        cv2.namedWindow(WIN_FLASH, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(WIN_FLASH, 900, 520)
        cv2.namedWindow(WIN_VIEW, cv2.WINDOW_NORMAL)

    def wait_q() -> bool:
        return False if headless else (cv2.waitKey(1) & 0xFF == ord("q"))

    def view(frame) -> None:
        if not headless:
            cv2.imshow(WIN_VIEW, draw_roi(frame))

    frame_gaps: list[float] = []
    _last_grab = [0.0]

    def grab():
        ok, f = cap.read()
        if not (ok and f is not None):
            return None, 0.0
        t = time.monotonic()
        if _last_grab[0]:
            frame_gaps.append(t - _last_grab[0])
        _last_grab[0] = t
        return f, t

    # ---------- 校準：量白畫面與黑畫面在擷取端各是多亮 ----------
    print(">>> 校準中：把相機對準閃爍視窗，讓它盡量填滿畫面…")
    levels = {}
    for white in (True, False):
        show_flash(white, headless)
        wait_q()
        t_end = time.monotonic() + 1.5
        vals = []
        while time.monotonic() < t_end:
            f, _ = grab()
            if f is not None:
                vals.append(roi_mean(f))
                view(f)
            if wait_q():
                cap.release(); cv2.destroyAllWindows(); return 1
        levels[white] = float(np.median(vals[len(vals) // 2:])) if vals else 0.0

    hi, lo = levels[True], levels[False]
    contrast = hi - lo
    print(f"    白畫面亮度 {hi:.1f}｜黑畫面亮度 {lo:.1f}｜對比 {contrast:.1f}")
    if contrast < MIN_CONTRAST:
        print(f"!! 對比只有 {contrast:.1f}（需 >{MIN_CONTRAST:.0f}）。")
        print("   把相機拉近讓視窗填滿畫面、關掉房間強光、確認相機真的拍著這個視窗。")
        cap.release(); cv2.destroyAllWindows()
        return 1
    mid = (hi + lo) / 2.0

    # ---------- 量測：隨機時間翻轉，看擷取端何時跨過中線 ----------
    print(f">>> 開始量測（目標 {args.samples} 次翻轉），按 q 可提前結束\n")
    samples: list[float] = []
    white = False
    show_flash(white, headless)
    wait_q()

    while len(samples) < args.samples:
        # 翻轉前隨機停留，避免與擷取幀率同步而產生系統性偏差
        t_hold_end = time.monotonic() + random.uniform(0.4, 0.9)
        while time.monotonic() < t_hold_end:
            f, _ = grab()
            if f is not None:
                view(f)
            if wait_q():
                break

        white = not white
        show_flash(white, headless)
        wait_q()                    # waitKey 才會真的把畫面畫上去
        t_flip = time.monotonic()   # 故意在畫上去之後才記時

        deadline = t_flip + 3.0
        found = False
        while time.monotonic() < deadline:
            f, t_arrive = grab()
            if f is None:
                if wait_q():
                    break
                continue
            m = roi_mean(f)
            if (m > mid) if white else (m < mid):
                delay = (t_arrive - t_flip) * 1000.0
                samples.append(delay)
                arr = np.array(samples)
                print(f"  第 {len(samples):2d} 次：{delay:6.0f} ms"
                      f"   （目前中位數 {np.median(arr):.0f} ms）")
                found = True
                break
            view(f)
            if wait_q():
                break
        if not found:
            print("  （這次沒偵測到翻轉，跳過——相機可能沒對準）")

        if not headless and cv2.getWindowProperty(WIN_FLASH, cv2.WND_PROP_VISIBLE) < 1:
            break

    cap.release()
    if not headless:
        cv2.destroyAllWindows()

    print("\n" + "=" * 62)
    if len(samples) < 5:
        print("樣本太少，無法統計。確認相機真的拍著閃爍視窗、且視窗夠大。")
        return 1
    arr = np.array(samples)
    med = float(np.median(arr))
    print(f"樣本數 {len(arr)}｜中位數 {med:.0f} ms｜平均 {arr.mean():.0f} ms｜"
          f"標準差 {arr.std():.0f} ms｜範圍 {arr.min():.0f}~{arr.max():.0f} ms")

    if headless:
        # 量到的值必然略高於真值：偵測只能發生在「下一幀抵達」的瞬間，
        # 所以會多算平均半個到一個幀週期（30fps ≈ 17~33ms）。
        err = med - args.selftest
        ok = -5.0 <= err <= 40.0
        print(f"\n注入 {args.selftest:.0f} ms → 量到 {med:.0f} ms（差 {err:+.0f} ms）")
        print("差值應落在 0~+33ms：偵測只能發生在下一幀抵達時，必然略為高估。")
        print(f"{'✅ 量測邏輯正確' if ok else '❌ 量測邏輯有問題'}")
        return 0 if ok else 1

    # 偏差修正：偵測只能發生在「下一幀抵達」的瞬間，平均多算半個幀週期。
    # 這個幀週期是量出來的，不是假設的。
    gap_ms = float(np.median(frame_gaps)) * 1000.0 if frame_gaps else 0.0
    suggest = max(0.0, med - gap_ms / 2.0)
    print(f"\n擷取幀間隔中位數 {gap_ms:.0f} ms（約 {1000/gap_ms:.0f} fps）")
    print(f"扣掉半個幀週期的量化偏差 → 建議值 {suggest:.0f} ms")
    print(f"\n>>> 設定頁「圖傳延遲 ms」填：{suggest:.0f}")
    print("    中位數比平均可靠（不受偶發掉幀拉高）。")
    print("    殘留偏差：螢幕本身的顯示延遲（約 5~15ms）仍含在內，屬於高估。")
    print("    不必追求個位數精度——這個量級的誤差在低速飛行下影響很小。")
    print("=" * 62)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
