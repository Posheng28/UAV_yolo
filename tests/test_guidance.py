"""導引律測試。"""

import numpy as np
import pytest

from uav_yolo.estimation import TargetEstimator
from uav_yolo.guidance import FixedWingGuidance, MultirotorGuidance, build_guidance


def make_est(pos, vel):
    est = TargetEstimator()
    est.update(0.0, np.array(pos, dtype=float))
    est.x = np.array([pos[0], pos[1], vel[0], vel[1]], dtype=float)
    return est


def test_multirotor_follows_behind_moving_target():
    est = make_est([100.0, 0.0], [10.0, 0.0])  # 往北跑 10 m/s
    g = MultirotorGuidance(follow_alt_m=40.0, standoff_m=15.0)
    cmd = g.compute(est)
    # 預測點 = 100+10（前置1秒）；退距 15m 在正南 → 95
    assert cmd.point_ne[0] == pytest.approx(110.0 - 15.0, abs=1e-6)
    assert cmd.point_ne[1] == pytest.approx(0.0, abs=1e-6)
    assert cmd.alt_rel_m == 40.0
    assert cmd.loiter_radius_m is None


def test_multirotor_overhead_mode():
    est = make_est([50.0, -20.0], [0.0, 0.0])
    g = MultirotorGuidance(follow_alt_m=30.0, standoff_m=0.0)
    cmd = g.compute(est)
    assert np.allclose(cmd.point_ne, [50.0, -20.0], atol=1e-6)


def test_multirotor_keeps_last_bearing_when_target_stops():
    g = MultirotorGuidance(follow_alt_m=40.0, standoff_m=15.0)
    g.compute(make_est([0.0, 0.0], [0.0, 12.0]))   # 目標往東跑 → 退距在西側
    cmd = g.compute(make_est([30.0, 60.0], [0.0, 0.0]))  # 急停後：沿用西側方位
    assert cmd.point_ne[0] == pytest.approx(30.0, abs=1e-6)
    assert cmd.point_ne[1] == pytest.approx(60.0 - 15.0, abs=1e-6)


def test_fixedwing_orbit_center_uses_lead_prediction():
    est = make_est([0.0, 0.0], [8.0, 6.0])  # 10 m/s 斜向
    g = FixedWingGuidance(orbit_radius_m=150.0, orbit_dir="ccw", lead_time_s=3.0, alt_m=80.0)
    cmd = g.compute(est)
    assert np.allclose(cmd.point_ne, [24.0, 18.0], atol=1e-6)  # pos + vel*3s
    assert cmd.loiter_radius_m == 150.0
    assert cmd.loiter_ccw is True
    assert cmd.alt_rel_m == 80.0


def test_fixedwing_stopped_target_orbit_centers_on_stop_point():
    """急停案例：圓心貼住停點，飛機繞圈而不是衝過頭。"""
    est = make_est([500.0, 200.0], [0.0, 0.0])
    g = FixedWingGuidance(orbit_radius_m=120.0, orbit_dir="cw", lead_time_s=3.0, alt_m=80.0)
    cmd = g.compute(est)
    assert np.allclose(cmd.point_ne, [500.0, 200.0], atol=1e-6)


def test_build_guidance_switches_airframe():
    cfg = {
        "multirotor": {"follow_alt_m": 35.0, "standoff_m": 10.0},
        "fixedwing": {"orbit_radius_m": 200.0, "orbit_dir": "cw", "lead_time_s": 2.0, "alt_m": 90.0},
    }
    assert isinstance(build_guidance("multirotor", cfg), MultirotorGuidance)
    fw = build_guidance("fixedwing", cfg)
    assert isinstance(fw, FixedWingGuidance)
    assert fw.orbit_radius_m == 200.0
