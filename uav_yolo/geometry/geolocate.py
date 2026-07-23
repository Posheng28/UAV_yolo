"""射線測地：像素 → 世界座標（NED / 經緯度）。

取代舊系統的「像素偏移 × GSD」線性近似。完整流程：
    像素 → 去畸變 → 相機系視線 → (雲台/機體姿態) → 世界系視線 → 與地面求交。
平地假設：地面 = home 高度的水平面（NED z=0）。
"""

from __future__ import annotations

import math

import numpy as np

M_PER_DEG_LAT = 111320.0
MAX_SLANT_RANGE_M = 10000.0  # 視線幾乎水平時的發散保護


class GeoRef:
    """局地 NED ↔ WGS84 經緯度（等距圓柱近似，數公里內誤差可忽略）。"""

    def __init__(self, lat0: float, lon0: float):
        self.lat0 = float(lat0)
        self.lon0 = float(lon0)
        self._m_per_deg_lon = M_PER_DEG_LAT * math.cos(math.radians(lat0))

    def lla_to_ned(self, lat: float, lon: float, rel_alt: float) -> np.ndarray:
        n = (lat - self.lat0) * M_PER_DEG_LAT
        e = (lon - self.lon0) * self._m_per_deg_lon
        return np.array([n, e, -rel_alt])

    def ned_to_lla(self, ned: np.ndarray) -> tuple[float, float, float]:
        lat = self.lat0 + ned[0] / M_PER_DEG_LAT
        lon = self.lon0 + ned[1] / self._m_per_deg_lon
        return lat, lon, -float(ned[2])


def intersect_ground(p_ned: np.ndarray, dir_world: np.ndarray, ground_z: float = 0.0) -> np.ndarray | None:
    """射線與水平地面求交。射線朝上或交點過遠（近水平掠射）回 None。"""
    dz = float(dir_world[2])
    if dz <= 1e-6:
        return None
    s = (ground_z - float(p_ned[2])) / dz
    if s <= 0.0 or s > MAX_SLANT_RANGE_M:
        return None
    return np.asarray(p_ned, dtype=np.float64) + s * np.asarray(dir_world, dtype=np.float64)


def geolocate_pixel(
    u: float,
    v: float,
    camera_model,
    R_world_cam: np.ndarray,
    vehicle_ned: np.ndarray,
    ground_z: float = 0.0,
) -> np.ndarray | None:
    """像素 → 地面 NED 座標；不可解（朝天/掠射）回 None。"""
    ray_cam = camera_model.pixel_to_ray(u, v)
    dir_world = R_world_cam @ ray_cam
    return intersect_ground(vehicle_ned, dir_world, ground_z)
