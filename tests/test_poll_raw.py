"""poll_raw：讀回驗證的時序防護。

讀回 setpoint 對帳時，「送出指令之前」收到的 setpoint 是舊狀態——
拿它對帳會得到假陽性（看起來一致，其實 PX4 根本還沒處理你的指令）。
newer_than 就是擋這個的。
"""

from types import SimpleNamespace

from uav_yolo.mavlink_io.telemetry import MavlinkConnection


def make_link():
    link = MavlinkConnection.__new__(MavlinkConnection)
    link.raw_last = {}
    return link


def test_returns_latest_message():
    link = make_link()
    msg = SimpleNamespace(alt=120.0)
    link.raw_last["POSITION_TARGET_GLOBAL_INT"] = (msg, 100.0)
    assert link.poll_raw("POSITION_TARGET_GLOBAL_INT") is msg


def test_stale_message_is_rejected_by_newer_than():
    """送出時刻之後才收到的才算數——舊 setpoint 對帳是假陽性。"""
    link = make_link()
    link.raw_last["POSITION_TARGET_GLOBAL_INT"] = (SimpleNamespace(), 100.0)
    assert link.poll_raw("POSITION_TARGET_GLOBAL_INT", newer_than=100.5) is None
    assert link.poll_raw("POSITION_TARGET_GLOBAL_INT", newer_than=99.5) is not None


def test_unknown_type_returns_none():
    assert make_link().poll_raw("NOPE") is None
