"""導引指令的地速上限跟著目標速度走。

為什麼不交給飛控預設：低空做幾公尺的小修正時，PX4 會用 MPC_XY_CRUISE 衝到
約 3m/s、傾角約 17 度。**機身一傾斜就把相機視軸甩開，而相機幾何正是目標座標
的來源**——所以這是精度問題，不是舒適度問題。

為什麼不寫死一個慢速度：那樣就追不上會動的車。指令速度跟著 KF 估到的目標
速度走，靜止時慢、跑起來才放行。
"""

from types import SimpleNamespace

import numpy as np
import pytest

from uav_yolo.guidance import MultirotorGuidance, build_guidance


def est(speed: float, pos=(0.0, 0.0)):
    """夠用的假估計器：導引只讀 predict_ahead / speed / vel_ne。"""
    v = np.array([speed, 0.0])
    return SimpleNamespace(
        speed=speed, vel_ne=v,
        pos_ne=np.array(pos, dtype=float),
        predict_ahead=lambda s: np.array(pos, dtype=float) + v * s,
    )


def test_stationary_target_gets_a_gentle_speed():
    g = MultirotorGuidance(follow_alt_m=4.0, standoff_m=0.0, max_speed_ms=5.0)
    cmd = g.compute(est(0.0))
    assert cmd.speed_ms == pytest.approx(1.0), (
        "靜止目標仍全速衝的話，傾角會把相機甩開"
    )


def test_moving_target_gets_enough_speed_to_keep_up():
    g = MultirotorGuidance(follow_alt_m=4.0, standoff_m=0.0, max_speed_ms=8.0)
    cmd = g.compute(est(3.0))
    assert cmd.speed_ms == pytest.approx(3.0 * 1.5 + 1.0)
    assert cmd.speed_ms > 3.0, "指令速度不能低於目標速度，否則永遠追不上"


def test_speed_is_capped():
    g = MultirotorGuidance(follow_alt_m=4.0, standoff_m=0.0, max_speed_ms=5.0)
    assert g.compute(est(20.0)).speed_ms == pytest.approx(5.0)


def test_zero_max_speed_defers_to_the_flight_controller():
    """0 = 不干預，讓 PX4 用自己的 MPC_XY_CRUISE。"""
    g = MultirotorGuidance(follow_alt_m=4.0, standoff_m=0.0, max_speed_ms=0.0)
    assert g.compute(est(2.0)).speed_ms is None


def test_fixedwing_never_gets_a_speed_limit():
    """固定翼降速會失速。這個機制只適用旋翼。"""
    g = build_guidance("fixedwing", {"fixedwing": {"orbit_radius_m": 150.0, "alt_m": 80.0}})
    assert g.compute(est(5.0)).speed_ms is None


def test_config_wires_the_limit_through():
    g = build_guidance("multirotor", {"multirotor": {"follow_alt_m": 4.0, "standoff_m": 0.0,
                                                    "max_speed_ms": 2.0}})
    assert g.compute(est(10.0)).speed_ms == pytest.approx(2.0)


def test_speed_never_drops_below_the_floor():
    """上限設得比下限還低時，仍要能動——否則飛機原地不動卻沒有任何說明。"""
    g = MultirotorGuidance(follow_alt_m=4.0, standoff_m=0.0, max_speed_ms=0.3)
    assert g.compute(est(0.0)).speed_ms == pytest.approx(0.3)
