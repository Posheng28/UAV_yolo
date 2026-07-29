"""自穩雲台鎖定天底時的相機朝向。

情境：XF C-20T（＝CADDX GM3）這類三軸自穩雲台，切到 LOOK-DOWN 模式後
**不論機身姿態都保持垂直朝下**，但它**不回報自己的角度**（ArduPilot 驅動的
原始碼註解：「gimbal does not provide attitude so simply return targets」，
PX4 更是連驅動都沒有）。

這個組合原本無解：
  - attitude_source=feedback/auto → 等不到回報 → 測地停擺
  - gimbal.present=false          → 套用機身 roll/pitch → **重複計算**，
                                    因為那正是雲台已經穩定掉的部分

fixed_earth 解決它：pitch/roll 直接用「相對地面」的固定值，yaw 用機身航向
（C-20T 所有已記載模式的偏航都跟隨機身）。相機朝向完全已知，不需要回報。
"""

import math

import numpy as np
import pytest

from uav_yolo.config import Config
from uav_yolo.geometry import geolocate_pixel
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path, **gimbal):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")},
                "gimbal": gimbal})
    return build_sim_engine(cfg, realtime=False)


def att(roll_deg=0.0, pitch_deg=0.0, yaw_deg=0.0):
    from types import SimpleNamespace
    return SimpleNamespace(t=0.0, roll=math.radians(roll_deg),
                           pitch=math.radians(pitch_deg), yaw=math.radians(yaw_deg))


def nadir_ray(R):
    """相機光軸在世界系的方向（相機 +Z 為光軸）。"""
    return R @ np.array([0.0, 0.0, 1.0])


def test_camera_stays_nadir_regardless_of_airframe_tilt(tmp_path):
    """機身怎麼傾，相機光軸都必須維持垂直朝下。

    這正是「不論飛機什麼姿態都直照地面」的定義。
    """
    engine = make_engine(tmp_path, present=True, control="none",
                         attitude_source="fixed_earth",
                         mount_deg={"roll": 0.0, "pitch": -90.0, "yaw": 0.0})
    for roll, pitch in ((0, 0), (20, 0), (0, -25), (15, 18), (-30, 12)):
        R = engine._camera_rotation(att(roll, pitch, 0.0), 0.0)
        d = nadir_ray(R)
        assert d[2] == pytest.approx(1.0, abs=1e-9), (
            f"機身 roll={roll} pitch={pitch} 時光軸不是垂直向下：{d}"
        )


def test_body_mount_would_have_tilted_with_the_airframe(tmp_path):
    """對照組：把同一個雲台當成固定安裝，光軸會跟著機身歪掉。

    這就是設錯時的誤差來源，也是為什麼需要 fixed_earth。
    """
    engine = make_engine(tmp_path, present=False,
                         mount_deg={"roll": 0.0, "pitch": -90.0, "yaw": 0.0})
    R = engine._camera_rotation(att(roll_deg=20.0), 0.0)
    d = nadir_ray(R)
    assert d[2] < 0.95, "固定安裝模式下光軸應隨機身傾斜，這個對照組沒有成立"


def test_error_from_getting_it_wrong_is_invisible_in_hover(tmp_path):
    """量化「懸停測不出來」：同樣的設定錯誤，懸停幾公分、平移數公尺。"""
    wrong = make_engine(tmp_path, present=False,
                        mount_deg={"roll": 0.0, "pitch": -90.0, "yaw": 0.0})
    cam = wrong.camera_model
    vehicle = np.array([0.0, 0.0, -15.0])          # 15 公尺高
    u, v = cam.width / 2.0, cam.height / 2.0

    truth = geolocate_pixel(
        u, v, cam, wrong._camera_rotation(att(0, 0, 0), 0.0), vehicle)
    hover = geolocate_pixel(
        u, v, cam, wrong._camera_rotation(att(2, 0, 0), 0.0), vehicle)
    moving = geolocate_pixel(
        u, v, cam, wrong._camera_rotation(att(15, 0, 0), 0.0), vehicle)

    hover_err = float(np.linalg.norm(hover[:2] - truth[:2]))
    move_err = float(np.linalg.norm(moving[:2] - truth[:2]))
    assert hover_err < 0.6, f"懸停誤差 {hover_err:.2f}m（應該小到測不出來）"
    assert move_err > 3.0, f"平移誤差 {move_err:.2f}m（應該大到不能忽略）"


def test_yaw_follows_the_airframe(tmp_path):
    """C-20T 所有已記載模式的偏航都跟隨機身，所以相機偏航＝機身航向＋安裝偏移。"""
    engine = make_engine(tmp_path, present=True, control="none",
                         attitude_source="fixed_earth",
                         mount_deg={"roll": 0.0, "pitch": -80.0, "yaw": 0.0})
    d0 = nadir_ray(engine._camera_rotation(att(yaw_deg=0.0), 0.0))
    d90 = nadir_ray(engine._camera_rotation(att(yaw_deg=90.0), 0.0))
    # pitch 不是正下方時光軸有水平分量，該分量必須跟著機身航向轉
    ang = math.degrees(math.atan2(d90[1], d90[0]) - math.atan2(d0[1], d0[0]))
    assert abs(((ang + 180) % 360) - 180 - 90) < 1e-6 or abs(ang - 90) < 1e-6


def test_mount_yaw_offset_is_applied(tmp_path):
    """雲台若不是正對機頭裝，mount_deg.yaw 是相對機頭的偏移。"""
    a = make_engine(tmp_path, present=True, control="none",
                    attitude_source="fixed_earth",
                    mount_deg={"roll": 0.0, "pitch": -80.0, "yaw": 0.0})
    b = make_engine(tmp_path, present=True, control="none",
                    attitude_source="fixed_earth",
                    mount_deg={"roll": 0.0, "pitch": -80.0, "yaw": 90.0})
    da = nadir_ray(a._camera_rotation(att(yaw_deg=0.0), 0.0))
    db = nadir_ray(b._camera_rotation(att(yaw_deg=0.0), 0.0))
    assert not np.allclose(da, db), "安裝偏移沒有生效"


def test_needs_airframe_yaw(tmp_path):
    """沒有機身航向就算不出相機偏航——必須回 None 而不是猜一個。"""
    engine = make_engine(tmp_path, present=True, control="none",
                         attitude_source="fixed_earth",
                         mount_deg={"roll": 0.0, "pitch": -90.0, "yaw": 0.0})
    assert engine._camera_rotation(None, 0.0) is None
