"""延遲量測光柵的編碼/解碼正確性（含模擬圖傳劣化）。

這個工具的可信度直接決定 latency_ms 填得對不對，
而填錯就是「延遲×飛行速度」的系統性測地偏移，所以要測。
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from measure_video_latency import BITS, decode_pattern, make_pattern_frame  # noqa: E402


@pytest.mark.parametrize("ms", [0, 1, 42, 1000, 5432, 9999, (1 << BITS) - 1])
def test_encode_decode_roundtrip(ms):
    frame = make_pattern_frame(ms)
    assert decode_pattern(frame) == ms


def test_survives_downscale_and_jpeg(tmp_path):
    """圖傳會壓縮＋採集卡可能縮放：解碼要撐得住。"""
    ms = 7321
    frame = make_pattern_frame(ms, 1920, 1080)
    small = cv2.resize(frame, (1280, 720), interpolation=cv2.INTER_AREA)
    ok, buf = cv2.imencode(".jpg", small, [cv2.IMWRITE_JPEG_QUALITY, 60])
    assert ok
    decoded = cv2.imdecode(buf, cv2.IMREAD_COLOR)
    assert decode_pattern(decoded) == ms


def test_survives_blur_and_brightness():
    """對焦不準、曝光偏移下仍要能解。"""
    ms = 4096
    frame = make_pattern_frame(ms)
    blurred = cv2.GaussianBlur(frame, (9, 9), 0)
    darker = np.clip(blurred.astype(np.int16) - 60, 0, 255).astype(np.uint8)
    assert decode_pattern(darker) == ms


def test_returns_none_on_unrelated_image():
    """沒拍到計時器時必須回 None，不能亂給數字（寧可沒讀值也別給錯值）。"""
    noise = np.full((720, 1280, 3), 128, np.uint8)
    assert decode_pattern(noise) is None
