"""影像來源測試：RTSP pipeline 組裝、傳輸退避順序、來源探測。"""

import cv2
import numpy as np
import pytest

from uav_yolo.vision.source import (
    VideoSource,
    build_gst_pipeline,
    has_gstreamer,
    probe_source,
)


# ---------------- RTSP pipeline / 選項 ----------------

def test_gst_pipeline_is_low_latency():
    p = build_gst_pipeline("rtsp://192.168.144.25:8554/main", "udp")
    assert "rtsp://192.168.144.25:8554/main" in p
    assert "latency=0" in p
    assert "protocols=udp" in p
    assert "drop=true" in p and "max-buffers=1" in p and "sync=false" in p
    assert "decodebin" in p  # codec 無關（H.264/H.265 都吃）


def test_gst_pipeline_tcp_variant():
    assert "protocols=tcp" in build_gst_pipeline("rtsp://x/y", "tcp")


def test_gst_pipeline_has_timeout():
    """沒有逾時的話，連不到的位址會讓開流永遠卡住。"""
    p = build_gst_pipeline("rtsp://x/y", "tcp", timeout_s=5.0)
    assert "tcp-timeout=5000000" in p


def test_ffmpeg_options_have_timeout():
    from uav_yolo.vision.source import _ffmpeg_rtsp_options

    opts = _ffmpeg_rtsp_options("udp", timeout_s=5.0)
    assert "stimeout;5000000" in opts   # 舊版 FFmpeg
    assert "timeout;5000000" in opts    # 新版 FFmpeg
    assert "rtsp_transport;udp" in opts


def test_has_gstreamer_returns_bool():
    assert isinstance(has_gstreamer(), bool)


def test_rtsp_auto_transport_tries_udp_then_tcp(monkeypatch):
    """auto 模式必須先試 UDP、失敗才退 TCP。"""
    attempts = []

    def fake_open_rtsp(url, transport, backend, timeout_s=5.0):
        attempts.append(transport)
        return None, "fail"

    monkeypatch.setattr("uav_yolo.vision.source.open_rtsp", fake_open_rtsp)
    src = VideoSource({"source": "rtsp", "rtsp_url": "rtsp://x/y", "rtsp_transport": "auto"})
    assert src._open() is None
    assert attempts == ["udp", "tcp"]


def test_rtsp_explicit_transport_does_not_fall_back(monkeypatch):
    attempts = []

    def fake_open_rtsp(url, transport, backend, timeout_s=5.0):
        attempts.append(transport)
        return None, "fail"

    monkeypatch.setattr("uav_yolo.vision.source.open_rtsp", fake_open_rtsp)
    src = VideoSource({"source": "rtsp", "rtsp_url": "rtsp://x/y", "rtsp_transport": "tcp"})
    src._open()
    assert attempts == ["tcp"]


def test_rtsp_rejects_stream_that_opens_but_yields_no_frame(monkeypatch):
    """能連上但取不到畫面，不能當成成功——否則引擎會空轉。"""
    class DeadCap:
        def read(self):
            return False, None
        def release(self):
            pass

    monkeypatch.setattr(
        "uav_yolo.vision.source.open_rtsp",
        lambda url, transport, backend, timeout_s=5.0: (DeadCap(), f"ffmpeg/{transport}"),
    )
    src = VideoSource({"source": "rtsp", "rtsp_url": "rtsp://x/y", "rtsp_transport": "udp"})
    assert src._open() is None
    assert "取得畫面" in (src.error or "")


# ---------------- 開串流限時（OpenCV 內建 30s 逾時擋不住） ----------------

def test_open_with_timeout_returns_quickly_on_hang():
    """OpenCV FFMPEG 後端遇到連不到的位址會卡 30 秒，必須自己在外面限時。"""
    import time as _t

    from uav_yolo.vision.source import _open_with_timeout

    t0 = _t.monotonic()
    result = _open_with_timeout(lambda: (_t.sleep(5), "late")[1], timeout_s=0.3)
    elapsed = _t.monotonic() - t0
    assert result is None
    assert elapsed < 1.5, f"逾時保護失效，等了 {elapsed:.1f}s"


def test_open_with_timeout_passes_through_fast_result():
    from uav_yolo.vision.source import _open_with_timeout

    sentinel = object()
    assert _open_with_timeout(lambda: sentinel, timeout_s=2.0) is sentinel


def test_open_with_timeout_releases_late_capture():
    """逾時後才冒出來的 cap 仍要被釋放，不能洩漏。"""
    import time as _t

    from uav_yolo.vision.source import _open_with_timeout

    released = []

    class LateCap:
        def release(self):
            released.append(True)

    def slow():
        _t.sleep(0.4)
        return LateCap()

    assert _open_with_timeout(slow, timeout_s=0.1) is None
    _t.sleep(0.8)  # 等背景回收
    assert released == [True]


# ---------------- 來源探測（起飛前測試按鈕） ----------------

@pytest.fixture
def sample_video(tmp_path):
    path = tmp_path / "clip.avi"
    writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"MJPG"), 20.0, (320, 240))
    assert writer.isOpened()
    for i in range(30):
        frame = np.full((240, 320, 3), i * 5 % 255, np.uint8)
        writer.write(frame)
    writer.release()
    return path


def test_probe_source_reports_real_resolution(sample_video):
    res = probe_source({"source": "file", "file_path": str(sample_video)}, sample_frames=6)
    assert res["ok"] is True
    assert (res["width"], res["height"]) == (320, 240)
    assert res["frames"] >= 1
    assert isinstance(res["gstreamer"], bool)


def test_probe_source_reports_failure_clearly():
    res = probe_source({"source": "file", "file_path": "does_not_exist.mp4"}, sample_frames=2)
    assert res["ok"] is False
    assert res["error"]
    assert res["mode"] == "file"
