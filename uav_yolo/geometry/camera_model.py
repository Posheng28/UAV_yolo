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

    @property
    def max_invertible_r(self) -> float:
        """畸變徑向映射還單調（＝去畸變還有意義）的最大歸一化半徑。

        🔴 Brown-Conrady 的 r_dist = r(1 + k1 r² + k2 r⁴ + k3 r⁶) 在 k1 為負且夠
        大時會在某個 r 折返。超過折返點之後，同一個 r_dist 對應多個 r，
        `cv2.undistortPoints` 的迭代解**不會報錯**，只會回傳垃圾。

        本專案實測（config/camera_intrinsics.yaml，k1=-0.349）：折返發生在
        r_undist=1.3611、最大可表示 r_dist=0.8795，而畫面四角 r=1.11~1.22
        ——**畫面 20.2% 的區域落在不可逆區**。沿 v=540 掃 u 得到的離軸角
        45.09°→58.91°→51.47° 非單調（實體鏡頭不可能）；2.8m 天底下
        1.10% 的像素測地到 8m 外，最差達數十公里。標定樣本沒有涵蓋四角時
        就會這樣，而 RMS 0.59px 看起來完全正常。
        """
        cached = getattr(self, "_max_inv_r", None)
        if cached is not None:
            return cached
        d = [float(v) for v in self.dist] + [0.0] * 8
        k1, k2, k3 = d[0], d[1], d[4]
        # 有理模型（CALIB_RATIONAL_MODEL，8 個係數）多了分母 k4/k5/k6，
        # 正是為了讓廣角鏡頭的徑向映射能一路單調到畫面四角。
        k4, k5, k6 = (d[5], d[6], d[7]) if len(self.dist) >= 8 else (0.0, 0.0, 0.0)
        r = np.linspace(0.0, 3.0, 30001)
        num = 1.0 + k1 * r**2 + k2 * r**4 + k3 * r**6
        den = 1.0 + k4 * r**2 + k5 * r**4 + k6 * r**6
        with np.errstate(divide="ignore", invalid="ignore"):
            rd = r * num / den

        # 🔴 要找的是**第一個**轉折點，不是全域最大值。
        # 有理模型的分母會過零（本專案實測 k4..k6 = 0.707/-0.383/-0.231），
        # 極點附近 rd 會衝到數百；用 argmax 會抓到那個極點，算出
        # max_invertible_r = 208，於是「畫面邊緣不可逆就拒收」的防護
        # 完全失效——沒有任何像素的半徑會超過 208。
        # 單調遞增在哪裡結束，可逆範圍就在哪裡結束，之後的一切都不可信。
        finite = np.isfinite(rd)
        sign_flip = np.signbit(den[:-1]) != np.signbit(den[1:])   # 分母過零＝極點
        decreasing = np.diff(rd) <= 0
        bad = decreasing | sign_flip | ~finite[:-1] | ~finite[1:]
        idx = int(np.argmax(bad)) if bad.any() else -1
        if idx <= 0:
            value = float("inf")          # 全段單調：沒有折返問題
        else:
            value = float(rd[idx])
        self._max_inv_r = value
        return value

    def unusable_pixel_fraction(self, step: int = 8) -> float:
        """畫面中落在畸變不可逆區、因而拿不到視線的像素比例（實測，非估算）。

        別用「1-(r_max/r_corner)²」那種圓盤近似：畫面是 16:9 的矩形，四角
        只佔一小塊，圓盤公式會把 19.7% 誇大成 47%，讀起來像半個畫面廢掉。
        """
        cached = getattr(self, "_unusable_frac", None)
        if cached is not None:
            return cached
        limit = self.max_invertible_r
        if not math.isfinite(limit):
            self._unusable_frac = 0.0
            return 0.0
        us = np.arange(0, self.width, step)
        vs = np.arange(0, self.height, step)
        x = (us - self.K[0, 2]) / self.K[0, 0]
        y = (vs - self.K[1, 2]) / self.K[1, 1]
        r = np.hypot(x[None, :], y[:, None])
        self._unusable_frac = float((r > limit).mean())
        return self._unusable_frac

    def corner_radius(self) -> float:
        """畫面四角的歸一化半徑（拿來和 max_invertible_r 比對）。"""
        xs = [(0 - self.K[0, 2]) / self.K[0, 0], (self.width - 1 - self.K[0, 2]) / self.K[0, 0]]
        ys = [(0 - self.K[1, 2]) / self.K[1, 1], (self.height - 1 - self.K[1, 2]) / self.K[1, 1]]
        return float(max(math.hypot(x, y) for x in xs for y in ys))

    def pixel_to_ray(self, u: float, v: float) -> np.ndarray | None:
        """像素座標 → 相機系單位視線向量（先去畸變）。

        落在畸變模型不可逆區的像素回 None——去畸變在那裡會靜默回傳垃圾，
        而測地把垃圾當成真實視線，會算出離飛機數公里的「目標」。
        寧可當作這一幀沒看到（走 COAST），也不要餵一個假座標給導引。
        """
        x_n = (float(u) - self.K[0, 2]) / self.K[0, 0]
        y_n = (float(v) - self.K[1, 2]) / self.K[1, 1]
        if math.hypot(x_n, y_n) > self.max_invertible_r:
            return None
        pts = np.array([[[float(u), float(v)]]], dtype=np.float64)
        norm = cv2.undistortPoints(pts, self.K, self.dist)  # -> 歸一化平面 (x', y')
        x, y = norm[0, 0]
        if not (math.isfinite(x) and math.isfinite(y)):
            return None
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

    @property
    def vfov_deg(self) -> float:
        """垂直視角。天底跟隨時「短邊」才是決定目標多快出框的那一邊，
        所以控制迴路的時間尺度要看它，不是看水平視角。"""
        return math.degrees(2.0 * math.atan(self.height / (2.0 * self.K[1, 1])))
