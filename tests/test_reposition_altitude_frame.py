"""導引指令的高度必須用「相對 home」座標框送，不能自己算 AMSL。

實機量到的落差：飛控的 home 高度來自 GPS（134.5m AMSL），但它當下的高度估計
來自氣壓計（-0.5m AMSL）——同一台飛機、停在 home 旁邊，兩者差 135 公尺。

若照 `home_alt_amsl + alt_rel_m` 算成 AMSL 送出去，飛控會拿這個數字跟自己
（差 135m 的）高度估計相比：**操作員設定的 4m 會變成叫它爬 139m。**
用 MAV_FRAME_GLOBAL_RELATIVE_ALT_INT 讓飛控自己換算就沒有這個問題。
真機實測：帶 CHANGE_MODE 旗標時該座標框回 ACCEPTED。
"""

import threading
from types import SimpleNamespace

import pytest

from uav_yolo.mavlink_io.telemetry import MavlinkConnection

MAV_FRAME_GLOBAL_RELATIVE_ALT_INT = 6
MAV_FRAME_GLOBAL_INT = 5
MAV_CMD_DO_REPOSITION = 192


class FakeMav:
    def __init__(self):
        self.sent = []

    def command_int_send(self, sysid, comp, frame, command, cur, autoc,
                         p1, p2, p3, p4, x, y, z):
        self.sent.append(SimpleNamespace(frame=frame, command=command, p1=p1, p2=p2,
                                         p3=p3, p4=p4, x=x, y=y, z=z))


def make_link():
    link = MavlinkConnection.__new__(MavlinkConnection)
    link._send_lock = threading.Lock()
    link._conn = SimpleNamespace(mav=FakeMav())
    link._target_sys = 1
    link._target_comp = 1
    return link


def test_altitude_is_sent_relative_to_home_not_amsl():
    link = make_link()
    # 若照舊法算 AMSL 會是 134.5+4=138.5，那個數字絕不能出現在指令裡
    link.send_reposition(24.786, 120.997, alt_amsl=138.5, alt_rel_m=4.0)

    cmd = link._conn.mav.sent[0]
    assert cmd.command == MAV_CMD_DO_REPOSITION
    assert cmd.frame == MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, (
        "用 AMSL 座標框時，home 與目前高度基準不一致會讓 4m 變成 139m"
    )
    assert cmd.z == pytest.approx(4.0), f"送出的高度應為相對 home 的 4m，實際 {cmd.z}"
    assert cmd.z != pytest.approx(138.5)


def test_change_mode_flag_is_set():
    """實測：沒有 CHANGE_MODE 旗標時 PX4 回 UNSUPPORTED，帶了才 ACCEPTED。"""
    link = make_link()
    link.send_reposition(24.786, 120.997, alt_amsl=138.5, alt_rel_m=4.0)
    assert link._conn.mav.sent[0].p2 == pytest.approx(1.0)


def test_position_is_sent_as_degrees_times_1e7():
    link = make_link()
    link.send_reposition(24.7860275, 120.9971365, alt_amsl=0.0, alt_rel_m=10.0)
    cmd = link._conn.mav.sent[0]
    assert cmd.x == 247860275
    assert cmd.y == 1209971365


def test_missing_relative_altitude_is_refused_loudly():
    """沒有相對高度就無法安全下令——寧可拋錯，也不要默默退回會爬錯高度的 AMSL。"""
    link = make_link()
    with pytest.raises(ValueError):
        link.send_reposition(24.786, 120.997, alt_amsl=138.5, alt_rel_m=None)


def test_fixedwing_orbit_parameters_survive_the_change():
    link = make_link()
    link.send_reposition(24.786, 120.997, alt_amsl=0.0, alt_rel_m=80.0,
                         loiter_radius_m=150.0, loiter_ccw=True)
    cmd = link._conn.mav.sent[0]
    assert cmd.p3 == pytest.approx(150.0)
    assert cmd.p4 == pytest.approx(1.0)
    assert cmd.z == pytest.approx(80.0)
