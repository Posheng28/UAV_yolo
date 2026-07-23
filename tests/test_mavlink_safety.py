"""PX4 模式解析、遙測內插、安全閘門測試（無硬體）。"""

import math

import numpy as np
import pytest

from uav_yolo.mavlink_io.px4_modes import encode_custom_mode, mode_string
from uav_yolo.mavlink_io.telemetry import (
    AttitudeSample,
    Heartbeat,
    PositionSample,
    TelemetryStore,
)
from uav_yolo.safety import SafetyGates


# ---------- PX4 模式 ----------

def test_mode_string_px4_encoding():
    assert mode_string(encode_custom_mode(4, 3)) == "AUTO.LOITER"
    assert mode_string(encode_custom_mode(4, 4)) == "AUTO.MISSION"
    assert mode_string(encode_custom_mode(3)) == "POSCTL"
    assert mode_string(encode_custom_mode(1)) == "MANUAL"
    assert mode_string(encode_custom_mode(6)) == "OFFBOARD"
    # 舊系統用 ArduPilot Copter 的 4=GUIDED 判斷——PX4 的 4 是 AUTO 主模式，證明不能混用
    assert "AUTO" in mode_string(encode_custom_mode(4, 2))


# ---------- 遙測內插 ----------

def test_attitude_interpolation_with_yaw_wrap():
    store = TelemetryStore()
    store.push_attitude(AttitudeSample(t=10.0, roll=0.0, pitch=0.0, yaw=math.radians(179.0)))
    store.push_attitude(AttitudeSample(t=11.0, roll=0.2, pitch=-0.1, yaw=math.radians(-179.0)))
    mid = store.attitude_at(10.5)
    # 179° → -179° 的最短路徑經過 180°，不是 0°
    assert abs(abs(math.degrees(mid.yaw)) - 180.0) < 1e-6
    assert mid.roll == pytest.approx(0.1, abs=1e-9)


def test_position_interpolation():
    store = TelemetryStore()
    store.push_position(PositionSample(t=0.0, lat=24.0, lon=120.0, rel_alt=50.0, alt_amsl=150.0, vn=10.0, ve=0.0))
    store.push_position(PositionSample(t=2.0, lat=24.0002, lon=120.0, rel_alt=54.0, alt_amsl=154.0, vn=10.0, ve=0.0))
    mid = store.position_at(1.0)
    assert mid.lat == pytest.approx(24.0001, abs=1e-9)
    assert mid.rel_alt == pytest.approx(52.0, abs=1e-9)
    # 超出範圍取端點
    assert store.position_at(-5.0).rel_alt == 50.0
    assert store.position_at(99.0).rel_alt == 54.0


def test_link_alive_timeout():
    store = TelemetryStore()
    assert not store.link_alive(now=0.0, timeout_s=2.0)
    store.set_heartbeat(Heartbeat(t=100.0, mode="AUTO.LOITER", armed=True))
    assert store.link_alive(now=101.0, timeout_s=2.0)
    assert not store.link_alive(now=103.5, timeout_s=2.0)


# ---------- 安全閘門 ----------

def make_gates():
    cfg = {
        "link_timeout_s": 2.0,
        "allowed_modes": ["AUTO.LOITER"],
        "max_cmd_distance_m": 500.0,
        "min_cmd_alt_m": 20.0,
        "max_cmd_alt_m": 120.0,
    }
    return SafetyGates(cfg, rate_hz=1.0)


def all_pass_kwargs():
    return dict(
        guidance_enabled=True,
        mode="AUTO.LOITER",
        armed=True,
        link_ok=True,
        est_initialized=True,
        est_age_s=0.1,
        coast_timeout_s=8.0,
        cmd_point_ne=np.array([100.0, 50.0]),
    )


def test_gates_all_pass():
    gates = make_gates()
    report = gates.evaluate(0.0, **all_pass_kwargs())
    assert report.ok
    assert report.blocked == []


def test_pilot_override_latch_and_reset():
    """接管閂鎖：切出允許模式即鎖定，切回也不自動恢復，重新啟用才解。"""
    gates = make_gates()
    gates.observe_mode("AUTO.LOITER", guidance_enabled=True)
    gates.observe_mode("POSCTL", guidance_enabled=True)  # 飛行員撥桿接管
    assert gates.pilot_override_latched

    gates.observe_mode("AUTO.LOITER", guidance_enabled=True)  # 切回來
    assert gates.pilot_override_latched  # 仍閂鎖

    report = gates.evaluate(0.0, **all_pass_kwargs())
    assert not report.ok
    assert any("接管" in r for r in report.blocked)

    gates.reset_override()
    assert gates.evaluate(0.0, **all_pass_kwargs()).ok


def test_gate_reasons():
    gates = make_gates()
    kwargs = all_pass_kwargs()
    kwargs.update(link_ok=False, armed=False, mode="POSCTL")
    report = gates.evaluate(0.0, **kwargs)
    assert not report.ok
    joined = "，".join(report.blocked)
    assert "數傳" in joined and "arm" in joined and "POSCTL" in joined


def test_geofence_blocks_far_command():
    gates = make_gates()
    kwargs = all_pass_kwargs()
    kwargs["cmd_point_ne"] = np.array([600.0, 0.0])
    report = gates.evaluate(0.0, **kwargs)
    assert not report.ok
    assert any("圍欄" in r for r in report.blocked)


def test_estimator_stale_blocks():
    gates = make_gates()
    kwargs = all_pass_kwargs()
    kwargs.update(est_age_s=10.0)
    report = gates.evaluate(0.0, **kwargs)
    assert not report.ok
    assert any("逾時" in r for r in report.blocked)


def test_rate_limit():
    gates = make_gates()
    assert gates.evaluate(0.0, **all_pass_kwargs()).ok
    gates.mark_sent(0.0)
    assert not gates.evaluate(0.5, **all_pass_kwargs()).ok  # 1Hz 內
    assert gates.evaluate(1.1, **all_pass_kwargs()).ok


def test_alt_clamp():
    gates = make_gates()
    assert gates.clamp_alt(5.0) == 20.0
    assert gates.clamp_alt(300.0) == 120.0
    assert gates.clamp_alt(60.0) == 60.0
