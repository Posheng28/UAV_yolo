"""影像來源必須能從「啟動當下開不了」的狀態自動康復。

實戰情境：按「重啟引擎」時前一個 capture 還沒釋放裝置、或圖傳/採集卡還沒供電，
開啟會失敗。舊行為是直接放棄且不啟動重連執行緒 → 引擎永久失明、
測試按鈕卻能開得起來（因為那時裝置已空出來），非常難排查。
"""

import threading
import time

import numpy as np
import pytest

from uav_yolo.vision.source import VideoSource, list_video_devices


class FlakyCap:
    """前 N 次開啟失敗，之後成功。"""

    def __init__(self):
        self.released = False

    def read(self):
        return True, np.full((16, 16, 3), 7, np.uint8)

    def release(self):
        self.released = True

    def isOpened(self):
        return True


def test_start_launches_retry_thread_even_if_open_fails(monkeypatch):
    src = VideoSource({"source": "uvc", "uvc_index": 1})
    monkeypatch.setattr(src, "_open", lambda: None)

    ok = src.start()
    try:
        assert ok is False                      # 誠實回報當下沒開起來
        assert src._thread is not None and src._thread.is_alive(), \
            "開啟失敗就不啟動重連執行緒 → 永久失明"
        assert src.error and "重試" in src.error
    finally:
        src.stop()


def test_recovers_when_device_becomes_available(monkeypatch):
    """裝置稍後空出來，必須自動接上並開始出幀。"""
    src = VideoSource({"source": "uvc", "uvc_index": 1})
    state = {"fail_until": time.monotonic() + 0.6}

    def flaky_open():
        if time.monotonic() < state["fail_until"]:
            return None
        return FlakyCap()

    monkeypatch.setattr(src, "_open", flaky_open)
    src.start()
    try:
        deadline = time.monotonic() + 6.0
        got = None
        while time.monotonic() < deadline:
            frame, _t = src.get_frame()
            if frame is not None:
                got = frame
                break
            time.sleep(0.05)
        assert got is not None, "裝置空出來後仍未自動恢復"
        assert src.connected is True
    finally:
        src.stop()


def test_stop_is_clean_after_failed_start(monkeypatch):
    src = VideoSource({"source": "uvc", "uvc_index": 1})
    monkeypatch.setattr(src, "_open", lambda: None)
    src.start()
    src.stop()
    assert not src._thread.is_alive()
    assert src.connected is False


def test_list_video_devices_works_off_main_thread():
    """pygrabber 走 COM：FastAPI 端點跑在執行緒池，未 CoInitialize 會靜默回空清單。"""
    main = list_video_devices()
    result = {}

    def worker():
        result["v"] = list_video_devices()

    t = threading.Thread(target=worker)
    t.start()
    t.join(timeout=10)

    assert "v" in result, "列舉裝置在子執行緒卡住"
    # 主執行緒抓得到裝置時，子執行緒也必須抓得到（不可因 COM 未初始化而變空）
    if main:
        assert result["v"], "子執行緒列舉裝置回空清單（COM 未初始化）"
