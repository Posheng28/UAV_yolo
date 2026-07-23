"""座標系定義與旋轉合成。

慣例（全專案統一）：
    世界系  : 局地 NED（x=北, y=東, z=下），原點 = home 點地面。
    機體系  : FRD（x=前, y=右, z=下）。
    尤拉角  : 航太 ZYX 順序，R = Rz(yaw) @ Ry(pitch) @ Rx(roll)，
              R 把「機體系向量」轉成「世界系向量」。
    相機系  : 光學慣例（+Z 沿光軸向前, +X 影像右, +Y 影像下）。
    雲台角  : pitch 0 = 對水平線，-90 = 垂直朝下；yaw 為地理航向（北=0，東=+90）。
              三軸自穩雲台 roll 恆為 0。
"""

from __future__ import annotations

import math

import numpy as np

# 相機光學系 → 載架 FRD 系的固定對齊：
#   載架前(x) = 相機光軸(z)、載架右(y) = 影像右(x)、載架下(z) = 影像下(y)
CAM_TO_CARRIER = np.array(
    [
        [0.0, 0.0, 1.0],
        [1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
    ]
)


def wrap_pi(angle: float) -> float:
    """把角度包回 (-pi, pi]。"""
    return math.atan2(math.sin(angle), math.cos(angle))


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])


def euler_zyx_to_R(yaw: float, pitch: float, roll: float) -> np.ndarray:
    """ZYX 尤拉角（弧度）→ 旋轉矩陣（子系向量 → 母系向量）。"""
    return _rot_z(yaw) @ _rot_y(pitch) @ _rot_x(roll)


def quat_wxyz_to_euler_zyx(w: float, x: float, y: float, z: float) -> tuple[float, float, float]:
    """MAVLink 四元數（w,x,y,z）→ (roll, pitch, yaw) 弧度，航太 ZYX 慣例。"""
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return roll, pitch, yaw


def camera_rotation_gimbal_earth(gimbal_yaw: float, gimbal_pitch: float, gimbal_roll: float = 0.0) -> np.ndarray:
    """三軸自穩雲台：由地理系雲台角直接得到 R（相機系 → 世界 NED）。

    自穩雲台把機體 roll/pitch 隔離掉，相機姿態只取決於雲台角本身，
    所以這條路徑「不需要」機體姿態——這是雲台對測地精度的最大貢獻。
    """
    return euler_zyx_to_R(gimbal_yaw, gimbal_pitch, gimbal_roll) @ CAM_TO_CARRIER


def camera_rotation_body_mount(
    R_world_body: np.ndarray,
    mount_yaw: float,
    mount_pitch: float,
    mount_roll: float,
) -> np.ndarray:
    """固定安裝相機：R（相機系 → 世界）= 機體姿態 ∘ 安裝角。

    無雲台時使用；此時機體 roll/pitch 會直接影響視線方向，
    三軸姿態一個都不能少（舊系統只用 yaw 的錯誤在此修正）。
    """
    return R_world_body @ euler_zyx_to_R(mount_yaw, mount_pitch, mount_roll) @ CAM_TO_CARRIER
