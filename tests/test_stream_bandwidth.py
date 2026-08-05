"""關掉用不到的串流，把數傳頻寬讓給姿態與位置。

實測 LR24 @115200：不關的話要求的串流只拿得到約 37%（ATTITUDE 要 10Hz 只來
3.7Hz），因為 PX4 預設一直在送 ATTITUDE_QUATERNION / VFR_HUD / RC_CHANNELS
等我們一項也不用的東西。關掉後 ATTITUDE 3.7→7.9 Hz、GLOBAL_POSITION_INT
1.4→3.9 Hz。

姿態頻率直接決定測地品質：要拿它內插出「影像拍攝當下」的姿態，
270ms 一筆太粗，飛機轉動時誤差會直接進到地面座標。
"""

import threading
from types import SimpleNamespace

import pytest

from uav_yolo.mavlink_io.telemetry import MavlinkConnection, TelemetryStore

MAV_CMD_SET_MESSAGE_INTERVAL = 511


class FakeMav:
    def __init__(self):
        self.cmds = []

    def command_long_send(self, sysid, comp, cmd, conf, p1, p2, p3, p4, p5, p6, p7):
        self.cmds.append((cmd, p1, p2))


def make_link(stream_rates=None):
    link = MavlinkConnection.__new__(MavlinkConnection)
    link._send_lock = threading.Lock()
    link._conn = SimpleNamespace(mav=FakeMav())
    link._target_sys = 1
    link._target_comp = 1
    link.stream_rates = stream_rates or {"ATTITUDE": 10, "GLOBAL_POSITION_INT": 5}
    # 真實的 MavlinkConnection 一定有 store：_request_intervals 之後會順便去問
    # WATCHED_PARAMS（EKF2_HGT_REF 等），需要它來記已經拿到的值。
    link.store = TelemetryStore()
    return link


def test_unused_streams_are_disabled_on_connect():
    link = make_link()
    link._request_intervals()
    disabled = {int(p1) for cmd, p1, p2 in link._conn.mav.cmds
                if cmd == MAV_CMD_SET_MESSAGE_INTERVAL and p2 == -1}
    assert disabled == set(MavlinkConnection.UNUSED_MSG_IDS), (
        "沒把用不到的串流關掉，姿態頻率會被它們吃掉一半以上"
    )


def test_interval_minus_one_means_stop():
    """MAV_CMD_SET_MESSAGE_INTERVAL 的約定：-1 = 停送。傳 0 是『用預設頻率』，反效果。"""
    link = make_link()
    link._request_intervals()
    for cmd, p1, p2 in link._conn.mav.cmds:
        if int(p1) in MavlinkConnection.UNUSED_MSG_IDS:
            assert p2 == -1, f"訊息 {int(p1)} 用 {p2} 關閉，應為 -1"


def test_needed_streams_are_still_requested():
    """關無用串流不能誤傷要用的。"""
    link = make_link({"ATTITUDE": 10, "GLOBAL_POSITION_INT": 5, "GPS_RAW_INT": 2})
    link._request_intervals()
    requested = {int(p1): p2 for cmd, p1, p2 in link._conn.mav.cmds if p2 > 0}
    assert 30 in requested, "ATTITUDE 沒被要求"
    assert 33 in requested, "GLOBAL_POSITION_INT 沒被要求"
    assert requested[30] == pytest.approx(1e6 / 10), "ATTITUDE 間隔換算錯誤（應為微秒）"


def test_disabled_and_requested_sets_do_not_overlap():
    """同一則訊息不能既關掉又要求——那是設定互相打架，結果看運氣。"""
    link = make_link({"ATTITUDE": 10, "GLOBAL_POSITION_INT": 5, "GPS_RAW_INT": 2,
                      "EXTENDED_SYS_STATE": 1, "GIMBAL_DEVICE_ATTITUDE_STATUS": 5})
    link._request_intervals()
    disabled = {int(p1) for cmd, p1, p2 in link._conn.mav.cmds if p2 == -1}
    requested = {int(p1) for cmd, p1, p2 in link._conn.mav.cmds if p2 > 0}
    assert not (disabled & requested), f"這些訊息被同時關閉與要求：{disabled & requested}"


def test_local_position_ned_is_disabled_but_global_is_not():
    """LOCAL_POSITION_NED(32) 我們不用；GLOBAL_POSITION_INT(33) 是核心，別搞混。"""
    assert 32 in MavlinkConnection.UNUSED_MSG_IDS
    assert 33 not in MavlinkConnection.UNUSED_MSG_IDS
