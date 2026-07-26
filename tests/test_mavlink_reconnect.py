"""數傳必須能從「啟動當下埠打不開」自動康復——與影像同一類的韌性需求。

實戰情境：接 QGC 改參數、拔插電台時，埠會短暫消失或被占用。舊行為是開一次
失敗就永久放棄、要手動「重啟引擎」；現在背景執行緒持續重試，埠回來就自動接上。
"""

import time

import pytest

from uav_yolo.mavlink_io.telemetry import MavlinkConnection


class FakeConn:
    """假 pymavlink 連線：只回一則心跳後就沒了。"""

    def __init__(self):
        self.closed = False

        class _Mav:
            total_bytes_received = 40

        self.mav = _Mav()
        self._sent = False

    def recv_match(self, blocking=True, timeout=0.2):
        time.sleep(0.02)
        return None

    def close(self):
        self.closed = True


def test_start_launches_thread_even_if_open_fails(monkeypatch):
    conn = MavlinkConnection(port="COM_NOPE", baud=115200)
    monkeypatch.setattr(conn, "_try_open", lambda: False)

    ok = conn.start()
    try:
        assert ok is False                       # 誠實回報當下沒開起來
        assert conn._thread is not None and conn._thread.is_alive(), \
            "開埠失敗就不啟動執行緒 → 數傳永久死掉"
    finally:
        conn.stop()


def test_reconnects_when_port_becomes_available(monkeypatch):
    conn = MavlinkConnection(port="COMx", baud=115200)
    state = {"ok_after": time.monotonic() + 0.4}

    def flaky_open():
        if time.monotonic() < state["ok_after"]:
            conn._conn = None
            return False
        conn._conn = FakeConn()
        return True

    monkeypatch.setattr(conn, "_try_open", flaky_open)
    conn.start()
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            if conn._conn is not None:
                break
            time.sleep(0.05)
        assert conn._conn is not None, "埠回來後仍未自動重連"
    finally:
        conn.stop()


def test_stop_is_clean_after_failed_start(monkeypatch):
    conn = MavlinkConnection(port="COM_NOPE", baud=115200)
    monkeypatch.setattr(conn, "_try_open", lambda: False)
    conn.start()
    conn.stop()
    assert not conn._thread.is_alive()
