"""KF 估計器測試：速度收斂、急停、離群拒絕、盲區外推。"""

import numpy as np
import pytest

from uav_yolo.estimation import TargetEstimator

RNG = np.random.default_rng(42)


def feed_track(est, t0, duration, dt, pos_fn, noise=3.0):
    """依軌跡函式餵量測，回傳最後時間。"""
    t = t0
    while t < t0 + duration:
        z = pos_fn(t) + RNG.normal(0, noise, 2)
        est.update(t, z)
        t += dt
    return t


def test_velocity_convergence_straight_line():
    """等速 10 m/s 直線：3 秒內速度估計收斂到 ±1.5 m/s。"""
    est = TargetEstimator(accel_std=3.0, meas_std=4.0)
    vel = np.array([8.0, -6.0])  # 速度 10 m/s
    feed_track(est, 0.0, 5.0, 0.1, lambda t: vel * t)
    assert est.speed == pytest.approx(10.0, abs=1.5)
    assert np.allclose(est.vel_ne, vel, atol=1.5)


def test_sudden_stop_velocity_decays():
    """急停場景：目標 10 m/s 跑 5 秒後急停，估計速度需在 3 秒內降到 <2 m/s。

    這是使用者點名的固定翼痛點——舊系統無速度概念，急停後只會停在舊資料。
    """
    est = TargetEstimator(accel_std=3.0, meas_std=4.0)
    vel = np.array([10.0, 0.0])
    stop_pos = vel * 5.0

    t = feed_track(est, 0.0, 5.0, 0.1, lambda t: vel * t)
    assert est.speed > 7.0  # 急停前有明確速度

    feed_track(est, t, 3.0, 0.1, lambda _t: stop_pos)
    assert est.speed < 2.0                         # 速度估計已歸零附近
    assert np.allclose(est.pos_ne, stop_pos, atol=3.0)  # 位置貼住停點


def test_outlier_rejected_then_recovers():
    est = TargetEstimator(accel_std=3.0, meas_std=4.0, max_jump_m=30.0)
    feed_track(est, 0.0, 3.0, 0.1, lambda t: np.array([5.0 * t, 0.0]))
    pos_before = est.pos_ne.copy()

    ok, reason = est.update(3.05, pos_before + np.array([200.0, 0.0]))  # 誤鎖別台車的爆量測
    assert not ok
    assert "gated" in reason
    assert np.allclose(est.pos_ne, pos_before, atol=1.0)  # 估計不被拉走

    ok, _ = est.update(3.1, np.array([5.0 * 3.1, 0.0]))
    assert ok


def test_coast_extrapolates_and_uncertainty_grows():
    """盲區（固定翼繞到看不到目標）：位置照速度外推、不確定度上升。"""
    est = TargetEstimator(accel_std=2.0, meas_std=4.0)
    vel = np.array([0.0, 12.0])
    t = feed_track(est, 0.0, 5.0, 0.1, lambda t: vel * t)

    std_before = est.pos_std
    est.predict_to(t + 4.0)  # 4 秒沒看到目標
    expected = vel * (t + 4.0)
    assert np.allclose(est.pos_ne, expected, atol=6.0)
    assert est.pos_std > std_before  # 不確定度隨盲區時間增長
    assert est.time_since_update(t + 4.0) == pytest.approx(4.0, abs=0.2)


def test_predict_ahead_for_orbit_center():
    est = TargetEstimator()
    est.update(0.0, np.array([0.0, 0.0]))
    for i in range(1, 40):
        est.update(i * 0.1, np.array([6.0 * i * 0.1, 0.0]))
    lead = est.predict_ahead(3.0)
    # 往前 3 秒 ≈ 目前位置 + 18m（等速 6 m/s）
    assert lead[0] - est.pos_ne[0] == pytest.approx(3.0 * est.vel_ne[0], abs=1e-9)


def test_reset():
    est = TargetEstimator()
    est.update(0.0, np.array([1.0, 2.0]))
    assert est.initialized
    est.reset()
    assert not est.initialized
