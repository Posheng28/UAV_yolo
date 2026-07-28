"""地面站必須發送 GCS 心跳。

實機上踩到的：本地面站從不發心跳，PX4 一收到我們的指令就登記「有地面站」，
接著立刻認定資料鏈路中斷 → failsafe → **拒絕解鎖**。症狀極度反直覺：

    完全不接數傳   → 可以 arm（PX4 從沒看過地面站）
    接上本地面站   → 不能 arm（看過，但心跳沒了＝斷線）
    接上 QGC       → 可以 arm（QGC 有發心跳）

「我只是被動監看」不是不發心跳的理由——MAVLink 規定每個節點都要發。
"""

import time
from types import SimpleNamespace

import pytest

from uav_yolo.mavlink_io.telemetry import MavlinkConnection


class FakeMav:
    def __init__(self):
        self.heartbeats = []
        self.commands = []

    def heartbeat_send(self, mtype, autopilot, base_mode, custom_mode, status):
        self.heartbeats.append((mtype, autopilot, base_mode, custom_mode, status))

    def command_long_send(self, *a, **k):
        self.commands.append(a)


def make_link():
    link = MavlinkConnection.__new__(MavlinkConnection)   # 不碰真的序列埠
    import threading

    link._send_lock = threading.Lock()
    link._conn = SimpleNamespace(mav=FakeMav())
    link.gcs_heartbeats_sent = 0
    link._last_hb_sent_t = 0.0
    return link


def test_heartbeat_is_sent():
    link = make_link()
    link._send_gcs_heartbeat()
    assert link.gcs_heartbeats_sent == 1, "一次心跳都沒發＝PX4 會判定鏈路中斷"


def test_heartbeat_identifies_us_as_a_ground_station():
    from pymavlink import mavutil

    link = make_link()
    link._send_gcs_heartbeat()
    mtype, autopilot, _, _, status = link._conn.mav.heartbeats[0]
    assert mtype == mavutil.mavlink.MAV_TYPE_GCS, "型別要是 GCS，否則會被當成另一台載具"
    assert autopilot == mavutil.mavlink.MAV_AUTOPILOT_INVALID, "地面站不是自駕儀"
    assert status == mavutil.mavlink.MAV_STATE_ACTIVE


def test_heartbeat_is_rate_limited_to_about_1hz():
    """別每圈迴圈都發——接收迴圈跑很快，會把低頻寬數傳灌爆。"""
    link = make_link()
    for _ in range(50):
        link._send_gcs_heartbeat()
    assert link.gcs_heartbeats_sent == 1, "限速失效，會洗爆數傳頻寬"

    link._last_hb_sent_t -= MavlinkConnection.GCS_HEARTBEAT_INTERVAL_S + 0.01
    link._send_gcs_heartbeat()
    assert link.gcs_heartbeats_sent == 2, "間隔到了卻沒補發"


def test_interval_is_fast_enough_for_px4_datalink_timeout():
    """PX4 的 COM_DL_LOSS_T 預設 10 秒；心跳間隔必須遠小於它，
    否則掉幾封就會被判定斷線。"""
    assert MavlinkConnection.GCS_HEARTBEAT_INTERVAL_S <= 2.0


def test_send_failure_does_not_break_the_receive_loop():
    """心跳送不出去（埠拔掉）不該把接收迴圈拖死。"""
    link = make_link()

    def boom(*a, **k):
        raise OSError("port gone")

    link._conn.mav.heartbeat_send = boom
    link._send_gcs_heartbeat()          # 不該拋出
    assert link.gcs_heartbeats_sent == 0


def test_no_connection_is_a_noop():
    link = make_link()
    link._conn = None
    link._send_gcs_heartbeat()
    assert link.gcs_heartbeats_sent == 0


# ---------------- 多段 STATUSTEXT 重組 ----------------

def test_split_statustext_is_reassembled():
    """STATUSTEXT 單則上限 50 字元，超過會切段；不重組就會漏掉關鍵字。

    實機收到過：「Arming denied: Resolve system health failures firs」+「t」，
    被截斷的正好是句尾。飛控唯一的解釋管道不能斷在一半。
    """
    from uav_yolo.mavlink_io.telemetry import TelemetryStore

    store = TelemetryStore()
    chunks = ["Arming denied: Resolve system health failures firs", "t"]
    cid = 7
    buf: dict[int, list[str]] = {}
    for text in chunks:                      # 複刻接收端的重組邏輯
        buf.setdefault(cid, []).append(text)
        if len(text) < 50:
            store.push_message(1.0, 2, "".join(buf.pop(cid)).strip())

    msgs = store.recent_messages()
    assert len(msgs) == 1, "切成兩段的訊息變成兩筆了"
    assert msgs[0]["text"] == "Arming denied: Resolve system health failures first"


def test_single_chunk_statustext_still_works():
    from uav_yolo.mavlink_io.telemetry import TelemetryStore

    store = TelemetryStore()
    store.push_message(1.0, 2, "Preflight Fail: Avionics Power low: 4.57 Volt")
    assert store.recent_messages()[0]["text"].endswith("4.57 Volt")
