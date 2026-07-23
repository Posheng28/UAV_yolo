"""幾何核心測試：手算已知案例 + 投影/反投影 roundtrip。"""

import math

import numpy as np
import pytest

from uav_yolo.geometry import (
    CameraModel,
    GeoRef,
    camera_rotation_body_mount,
    camera_rotation_gimbal_earth,
    euler_zyx_to_R,
    geolocate_pixel,
    intersect_ground,
)


@pytest.fixture
def cam():
    # 水平 FOV 90 度、1280x720 → fx = 640
    return CameraModel.from_hfov(90.0, 1280, 720)


def test_fov_intrinsics(cam):
    assert cam.K[0, 0] == pytest.approx(640.0)
    assert cam.hfov_deg == pytest.approx(90.0)
    ray = cam.pixel_to_ray(640.0, 360.0)  # 畫面正中心
    assert np.allclose(ray, [0, 0, 1], atol=1e-9)


def test_gimbal_straight_down_center_pixel(cam):
    """雲台垂直朝下、機在 100m：畫面中心 = 機正下方。"""
    R_wc = camera_rotation_gimbal_earth(gimbal_yaw=0.0, gimbal_pitch=math.radians(-90.0))
    vehicle = np.array([0.0, 0.0, -100.0])  # NED，100m AGL
    hit = geolocate_pixel(640.0, 360.0, cam, R_wc, vehicle)
    assert hit is not None
    assert np.allclose(hit, [0.0, 0.0, 0.0], atol=1e-6)


def test_gimbal_down_right_edge_pixel(cam):
    """朝下、yaw=0：畫面右緣（fx=640 → 45 度斜角）= 正東 100m。"""
    R_wc = camera_rotation_gimbal_earth(0.0, math.radians(-90.0))
    vehicle = np.array([0.0, 0.0, -100.0])
    hit = geolocate_pixel(640.0 + 640.0, 360.0, cam, R_wc, vehicle)
    assert hit is not None
    assert hit[0] == pytest.approx(0.0, abs=1e-6)   # 北向不變
    assert hit[1] == pytest.approx(100.0, abs=1e-6)  # 東 100m


def test_gimbal_yaw_rotates_image_axes(cam):
    """yaw=90（鏡頭朝東）時，畫面右緣應落在正南。"""
    R_wc = camera_rotation_gimbal_earth(math.radians(90.0), math.radians(-90.0))
    vehicle = np.array([0.0, 0.0, -100.0])
    hit = geolocate_pixel(640.0 + 640.0, 360.0, cam, R_wc, vehicle)
    assert hit is not None
    assert hit[0] == pytest.approx(-100.0, abs=1e-6)  # 南 100m
    assert hit[1] == pytest.approx(0.0, abs=1e-6)


def test_gimbal_45deg_pitch_geometry(cam):
    """雲台 pitch -45、機在 100m：畫面中心落在前方 100m 地面。"""
    R_wc = camera_rotation_gimbal_earth(0.0, math.radians(-45.0))
    vehicle = np.array([0.0, 0.0, -100.0])
    hit = geolocate_pixel(640.0, 360.0, cam, R_wc, vehicle)
    assert hit is not None
    assert hit[0] == pytest.approx(100.0, abs=1e-6)  # 北 100m
    assert hit[1] == pytest.approx(0.0, abs=1e-6)


def test_body_mount_roll_shifts_ground_point(cam):
    """固定安裝朝下 + 機體右滾 30 度：中心點橫移 alt*tan(30)，方向為西（NED 慣例）。

    這正是舊系統忽略 roll/pitch 造成的誤差量級：100m 高、30 度滾轉 = 57.7m。
    """
    R_wb = euler_zyx_to_R(0.0, 0.0, math.radians(30.0))
    R_wc = camera_rotation_body_mount(R_wb, 0.0, math.radians(-90.0), 0.0)
    vehicle = np.array([0.0, 0.0, -100.0])
    hit = geolocate_pixel(640.0, 360.0, cam, R_wc, vehicle)
    assert hit is not None
    expected_shift = 100.0 * math.tan(math.radians(30.0))
    assert hit[0] == pytest.approx(0.0, abs=1e-6)
    assert hit[1] == pytest.approx(-expected_shift, rel=1e-6)


def test_projection_roundtrip(cam):
    """世界點 → 投影成像素 → 測地還原：應回到同一點（模擬模式的基礎）。"""
    R_wc = camera_rotation_gimbal_earth(math.radians(30.0), math.radians(-60.0))
    vehicle = np.array([50.0, -20.0, -80.0])
    target_world = np.array([120.0, 35.0, 0.0])

    point_cam = R_wc.T @ (target_world - vehicle)
    pixel = cam.project(point_cam)
    assert pixel is not None
    u, v = pixel

    hit = geolocate_pixel(u, v, cam, R_wc, vehicle)
    assert hit is not None
    assert np.allclose(hit, target_world, atol=1e-4)


def test_roundtrip_with_distortion():
    """帶畸變係數的模型也要能 roundtrip（去畸變路徑驗證）。"""
    K = np.array([[600.0, 0, 640.0], [0, 600.0, 360.0], [0, 0, 1.0]])
    dist = np.array([-0.25, 0.08, 0.001, -0.001, 0.0])  # 廣角桶狀畸變的典型量級
    cam = CameraModel(K, dist, 1280, 720)
    R_wc = camera_rotation_gimbal_earth(0.0, math.radians(-70.0))
    vehicle = np.array([0.0, 0.0, -60.0])
    target_world = np.array([15.0, 8.0, 0.0])

    point_cam = R_wc.T @ (target_world - vehicle)
    pixel = cam.project(point_cam)
    assert pixel is not None
    hit = geolocate_pixel(pixel[0], pixel[1], cam, R_wc, vehicle)
    assert hit is not None
    assert np.allclose(hit, target_world, atol=1e-3)


def test_intersect_ground_rejects_sky_and_grazing():
    p = np.array([0.0, 0.0, -100.0])
    assert intersect_ground(p, np.array([0.0, 0.0, -1.0])) is None      # 朝天
    assert intersect_ground(p, np.array([1.0, 0.0, 1e-9])) is None      # 近水平掠射
    assert intersect_ground(np.array([0.0, 0.0, 5.0]), np.array([0.0, 0.0, 1.0])) is None  # 在地下


def test_georef_roundtrip():
    ref = GeoRef(24.569890, 120.841928)
    ned = ref.lla_to_ned(24.571000, 120.843000, 55.0)
    lat, lon, alt = ref.ned_to_lla(ned)
    assert lat == pytest.approx(24.571000, abs=1e-9)
    assert lon == pytest.approx(120.843000, abs=1e-9)
    assert alt == pytest.approx(55.0, abs=1e-9)
    # 量級 sanity：緯度差 0.00111 度 ≈ 123.6m
    assert ned[0] == pytest.approx(0.00111 * 111320.0, rel=1e-3)


def test_scaled_intrinsics(cam):
    half = cam.scaled_to(640, 360)
    assert half.K[0, 0] == pytest.approx(cam.K[0, 0] / 2)
    assert half.K[0, 2] == pytest.approx(cam.K[0, 2] / 2)
    ray_full = cam.pixel_to_ray(960.0, 540.0)
    ray_half = half.pixel_to_ray(480.0, 270.0)
    assert np.allclose(ray_full, ray_half, atol=1e-9)
