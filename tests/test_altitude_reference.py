"""導引指令的高度基準防護。

背景，兩件都是實機踩到的：

1. **DO_REPOSITION 的 param7 一律被 PX4 當成 AMSL**，不看座標框。
   v1.17.0 navigator_main.cpp:
       rep->current.alt = PX4_ISFINITE(cmd.param7) ? cmd.param7 : ...
   所以「改用相對座標框送相對高度」是行不通的——飛控會把 4.0 解讀成
   海拔 4 公尺，在地面高程 113m 的場地等於往地下 109m。
   （我確實這樣改過，而且飛控回 ACCEPTED。ACCEPTED 只代表指令被收下。）

2. 既然只能送 AMSL，就必須確定 `home_alt_amsl` 與飛控當下的高度是同一個基準。
   EKF2_HGT_REF 指向未安裝的測距儀時兩者差了 135m，此時「跟隨高度 4m」
   會變成叫飛機爬 139m。飛控自己有對帳的依據：
       (alt_amsl - home_alt_amsl) 應該等於 rel_alt
   對不上就是基準壞了，任何高度指令都不可信。
"""

import threading
from types import SimpleNamespace

import pytest

from uav_yolo.config import Config
from uav_yolo.mavlink_io.telemetry import MavlinkConnection
from uav_yolo.simulation import build_sim_engine

MAV_FRAME_GLOBAL_INT = 5


class FakeMav:
    def __init__(self):
        self.sent = []

    def command_int_send(self, sysid, comp, frame, command, cur, autoc,
                         p1, p2, p3, p4, x, y, z):
        self.sent.append(SimpleNamespace(frame=frame, command=command, z=z, p2=p2))


def make_link():
    link = MavlinkConnection.__new__(MavlinkConnection)
    link._send_lock = threading.Lock()
    link._conn = SimpleNamespace(mav=FakeMav())
    link._target_sys = 1
    link._target_comp = 1
    return link


# ---------------- 送出去的必須是 AMSL ----------------

def test_reposition_sends_amsl_in_the_global_frame():
    """PX4 不看座標框，param7 就是 AMSL。送相對高度會讓飛機往地下飛。"""
    link = make_link()
    link.send_reposition(24.786, 120.997, alt_amsl=117.0, alt_rel_m=4.0)

    cmd = link._conn.mav.sent[0]
    assert cmd.frame == MAV_FRAME_GLOBAL_INT
    assert cmd.z == pytest.approx(117.0), (
        f"送出 {cmd.z}；若送成相對高度 4.0，PX4 會解讀為海拔 4m ＝ 往地下飛"
    )
    assert cmd.z != pytest.approx(4.0)


def test_change_mode_flag_still_set():
    """實測：沒有 CHANGE_MODE 旗標時 PX4 回 UNSUPPORTED。"""
    link = make_link()
    link.send_reposition(24.786, 120.997, alt_amsl=117.0, alt_rel_m=4.0)
    assert link._conn.mav.sent[0].p2 == pytest.approx(1.0)


# ---------------- 高度基準對帳 ----------------

def make_engine(tmp_path):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return build_sim_engine(cfg, realtime=False)


def pos(alt_amsl, rel_alt):
    return SimpleNamespace(lat=24.786, lon=120.997, alt_amsl=alt_amsl,
                           rel_alt=rel_alt, t=0.0, vn=0.0, ve=0.0)


def test_consistent_reference_passes(tmp_path):
    engine = make_engine(tmp_path)
    engine.link.store.home = SimpleNamespace(lat=24.786, lon=120.997, alt_amsl=113.0)
    assert engine._altitude_reference_sane(pos(117.0, 4.0)) is None


def test_broken_reference_is_caught(tmp_path):
    """實機數值：home 113m（GPS 基準）、EKF 高度 -0.45m（氣壓計基準）。"""
    engine = make_engine(tmp_path)
    engine.link.store.home = SimpleNamespace(lat=24.786, lon=120.997, alt_amsl=113.0)
    reason = engine._altitude_reference_sane(pos(-0.45, -113.45 + 0.0))
    # rel_alt 若與 (alt_amsl - home) 一致就不算壞；這裡刻意讓它不一致
    reason = engine._altitude_reference_sane(pos(-0.45, 4.0))
    assert reason and "高度基準不一致" in reason
    assert "EKF2_HGT_REF" in reason, "要指出該去查哪個參數，否則操作員無從下手"


def test_small_disagreement_is_tolerated(tmp_path):
    """氣壓雜訊/取樣時間差造成的幾公尺落差不該擋住任務。"""
    engine = make_engine(tmp_path)
    engine.link.store.home = SimpleNamespace(lat=24.786, lon=120.997, alt_amsl=113.0)
    assert engine._altitude_reference_sane(pos(117.0, 6.0)) is None


def test_no_home_or_no_position_is_not_an_error(tmp_path):
    """還沒拿到 home 時不該報基準錯誤——那由別的閘門負責。"""
    engine = make_engine(tmp_path)
    engine.link.store.home = None
    assert engine._altitude_reference_sane(pos(117.0, 4.0)) is None
    engine.link.store.home = SimpleNamespace(lat=24.786, lon=120.997, alt_amsl=113.0)
    assert engine._altitude_reference_sane(None) is None
