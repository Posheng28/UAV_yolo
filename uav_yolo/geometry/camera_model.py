"""相機模型：內參 + 畸變。像素 ↔ 光線的橋樑。

來源兩種：
    1. 棋盤格校正檔（UI 校正頁產生，含 K 與畸變係數）— 準確，建議。
    2. 只給水平 FOV 的近似（無畸變）— 沒校正前的過渡方案。
"""

from __future__ import annotations

import math
from pathlib import Path

import cv2
import numpy as np
import yaml


class CameraModel:
    def __init__(self, K: np.ndarray, dist: np.ndarray, width: int, height: int, source: str = "unknown"):
        self.K = np.asarray(K, dtype=np.float64).reshape(3, 3)
        self.dist = np.asarray(dist, dtype=np.float64).reshape(-1)
        self.width = int(width)
        self.height = int(height)
        self.source = source  # "calibrated" | "fov_approx"

    # ---------- 建構 ----------

    @classmethod
    def from_hfov(cls, hfov_deg: float, width: int, height: int) -> "CameraModel":
        fx = width / (2.0 * math.tan(math.radians(hfov_deg) / 2.0))
        K = np.array([[fx, 0, width / 2.0], [0, fx, height / 2.0], [0, 0, 1.0]])
        return cls(K, np.zeros(5), width, height, source="fov_approx")

    @classmethod
    def from_file(cls, path: str | Path) -> "CameraModel":
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)
        return cls(
            np.array(data["K"], dtype=np.float64),
            np.array(data["dist"], dtype=np.float64),
            data["width"],
            data["height"],
            source="calibrated",
        )

    @classmethod
    def load(cls, intrinsics_file: str | Path, fallback_hfov_deg: float, width: int, height: int) -> "CameraModel":
        """有校正檔用校正檔（解析度不合時等比縮放），否則 FOV 近似。"""
        path = Path(intrinsics_file)
        if path.exists():
            model = cls.from_file(path)
            if model.width != width or model.height != height:
                model = model.scaled_to(width, height)
            return model
        return cls.from_hfov(fallback_hfov_deg, width, height)

    def scaled_to(self, width: int, height: int) -> "CameraModel":
        sx = width / self.width
        sy = height / self.height
        K = self.K.copy()
        K[0, :] *= sx
        K[1, :] *= sy
        return CameraModel(K, self.dist, width, height, source=self.source)

    def save(self, path: str | Path, rms: float | None = None) -> None:
        data = {
            "K": self.K.tolist(),
            "dist": self.dist.tolist(),
            "width": self.width,
            "height": self.height,
        }
        if rms is not None:
            data["calibration_rms_px"] = float(rms)
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            yaml.safe_dump(data, f, sort_keys=False)

    # ---------- 幾何 ----------

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray:
        """像素座標 → 相機系單位視線向量（先去畸變）。"""
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        norm = cv2.undistortPoints(pts, self.K, self.dist)  # -> 歸一化平面 (x', y')
        x, y = norm[0, 0]
        ray = np.array([x, y, 1.0])
        return ray / np.linalg.norm(ray)

    def project(self, point_cam: np.ndarray) -> tuple[float, float] | None:
        """相機系 3D 點 → 像素座標（含畸變；模擬模式用）。點在鏡頭後方回 None。"""
        point_cam = np.asarray(point_cam, dtype=np.float64)
        if point_cam[2] <= 1e-6:
            return None
        pts, _ = cv2.projectPoints(
            point_cam.reshape(1, 1, 3),
            np.zeros(3),
            np.zeros(3),
            self.K,
            self.dist,
        )
        u, v = pts[0, 0]
        return float(u), float(v)

    @property
    def hfov_deg(self) -> float:
        return math.degrees(2.0 * math.atan(self.width / (2.0 * self.K[0, 0])))
