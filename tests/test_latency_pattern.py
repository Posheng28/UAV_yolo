"""延遲量測工具的正確性（時間編碼版）。

這個工具的可信度直接決定 latency_ms 填得對不對，而填錯就是
「延遲 × 飛行速度」的系統性測地偏移，所以要測。

舊版用畫面頂端的二進位光柵編碼時間，並在**擷取畫面的頂端**解碼——那假設
螢幕填滿整個擷取畫面。實機上不成立：相機看到的是螢幕在視野中的一個矩形，
上緣還被 Walksnail VRX 的 OSD 蓋著，條碼幾乎永遠不在它預期的位置。
現版改成整片亮度翻轉＋中央取樣，這裡測的就是那條路徑。
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from measure_video_latency import ROI_FRAC, _FakeCap, roi_mean  # noqa: E402


def frame_of(level: int, w: int = 320, h: int = 240) -> np.ndarray:
    return np.full((h, w, 3), level, np.uint8)


# ---------------- 亮度取樣 ----------------

def test_roi_mean_tracks_brightness():
    assert roi_mean(frame_of(0)) == pytest.approx(0.0, abs=1.0)
    assert roi_mean(frame_of(255)) == pytest.approx(255.0, abs=1.0)
    assert roi_mean(frame_of(128)) == pytest.approx(128.0, abs=1.0)


def test_roi_ignores_the_edges_of_the_frame():
    """只看中央——四周的 OSD、背景、房間光線都不該影響判讀。

    這是換掉光柵法的核心理由：VRX 會在畫面上緣疊 NO SD／電壓／訊號。
    """
    frame = frame_of(0, 400, 400)
    frame[:60, :] = 255          # 上緣一條亮帶（模擬 OSD）
    frame[-60:, :] = 255         # 下緣一條
    assert roi_mean(frame) == pytest.approx(0.0, abs=1.0), "邊緣的亮帶滲進中央取樣了"


def test_roi_survives_jpeg_and_downscale():
    """圖傳會壓縮、採集卡可能縮放：亮度判讀要撐得住。"""
    frame = frame_of(220, 1920, 1080)
    small = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
    assert ok
    assert roi_mean(cv2.imdecode(buf, cv2.IMREAD_COLOR)) == pytest.approx(220.0, abs=6.0)


def test_roi_fraction_is_a_sane_centre_window():
    assert 0.1 <= ROI_FRAC <= 0.6


# ---------------- 假相機（自我檢驗的基礎） ----------------

def test_fake_cap_actually_delays_the_flash_state():
    """自我檢驗模式的可信度全靠這個假相機真的會延遲。

    它若不延遲，--selftest 就會永遠通過，工具的「已驗證」也就毫無意義。
    """
    cap = _FakeCap(delay_ms=120, fps=1000)

    _FakeCap.flash_state = False
    for _ in range(5):
        cap.read()
    assert roi_mean(cap.read()[1]) < 100, "初始應為暗"

    _FakeCap.flash_state = True
    ok, f = cap.read()
    assert ok and roi_mean(f) < 100, "翻轉後立刻讀到的應該還是舊狀態（延遲存在）"

    import time
    time.sleep(0.2)                      # 等超過注入的 120ms
    assert roi_mean(cap.read()[1]) > 100, "超過延遲時間後應讀到新狀態"


def test_fake_cap_respects_its_frame_rate():
    import time

    cap = _FakeCap(delay_ms=0, fps=50)
    t0 = time.monotonic()
    for _ in range(10):
        cap.read()
    elapsed = time.monotonic() - t0
    assert elapsed > 0.12, f"10 幀 @50fps 至少要 0.18s 左右，實際 {elapsed:.3f}s"
