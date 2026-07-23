"""地面目標狀態估計：NE 平面等速模型 Kalman 濾波。

解決舊系統兩個根本問題：
    1. 「5 幀滑動平均、丟失即歸零」→ 改為位置+速度聯合估計，
       丟失時用速度外推（coast），繞行盲區、目標急停都有記憶。
    2. 「跳變 >25m 一律拒發」→ 改為 innovation gating（依當前不確定度
       自適應的統計閘門），不會在誤差暫時放大時把正常量測全擋掉。
"""

from __future__ import annotations

import math

import numpy as np


class TargetEstimator:
    """狀態 x = [n, e, vn, ve]，量測 z = [n, e]。時間一律用單調秒。"""

    def __init__(
        self,
        accel_std: float = 3.0,
        meas_std: float = 8.0,
        gate_sigma: float = 4.0,
        max_jump_m: float = 30.0,
    ):
        self.accel_std = float(accel_std)
        self.meas_std = float(meas_std)
        self.gate_sigma = float(gate_sigma)
        self.max_jump_m = float(max_jump_m)
        self.reset()

    # ---------- 生命週期 ----------

    def reset(self) -> None:
        self.x: np.ndarray | None = None
        self.P: np.ndarray | None = None
        self.t: float | None = None
        self.last_update_t: float | None = None
        self.rejected_streak = 0

    @property
    def initialized(self) -> bool:
        return self.x is not None

    def time_since_update(self, now: float) -> float:
        if self.last_update_t is None:
            return math.inf
        return now - self.last_update_t

    # ---------- 濾波 ----------

    def predict_to(self, t: float) -> None:
        if self.x is None or self.t is None:
            return
        dt = t - self.t
        if dt <= 0.0:
            return
        F = np.array(
            [
                [1, 0, dt, 0],
                [0, 1, 0, dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float64,
        )
        q = self.accel_std**2
        d4, d3, d2 = dt**4 / 4.0, dt**3 / 2.0, dt**2
        Q = q * np.array(
            [
                [d4, 0, d3, 0],
                [0, d4, 0, d3],
                [d3, 0, d2, 0],
                [0, d3, 0, d2],
            ],
            dtype=np.float64,
        )
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.t = t

    def update(self, t: float, z_ne: np.ndarray) -> tuple[bool, str]:
        """餵入一筆測地量測。回傳 (是否採納, 原因)。"""
        z = np.asarray(z_ne, dtype=np.float64).reshape(2)

        if self.x is None:
            self.x = np.array([z[0], z[1], 0.0, 0.0])
            big_vel = 15.0**2  # 初始速度未知：給大變異數讓前幾筆量測快速定出速度
            self.P = np.diag([self.meas_std**2, self.meas_std**2, big_vel, big_vel])
            self.t = t
            self.last_update_t = t
            return True, "init"

        self.predict_to(t)

        H = np.array([[1, 0, 0, 0], [0, 1, 0, 0]], dtype=np.float64)
        R = np.eye(2) * self.meas_std**2
        y = z - H @ self.x
        S = H @ self.P @ H.T + R

        # 統計閘門 + 硬跳變上限（雙保險，硬上限承襲原系統規格）
        nis = float(y @ np.linalg.solve(S, y))
        jump = float(np.linalg.norm(y))
        if math.sqrt(max(nis, 0.0)) > self.gate_sigma or jump > self.max_jump_m:
            self.rejected_streak += 1
            return False, f"gated(nis={math.sqrt(nis):.1f}sigma, jump={jump:.1f}m)"

        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_update_t = t
        self.rejected_streak = 0
        return True, "ok"

    # ---------- 查詢 ----------

    @property
    def pos_ne(self) -> np.ndarray:
        return self.x[:2].copy()

    @property
    def vel_ne(self) -> np.ndarray:
        return self.x[2:].copy()

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.x[2:]))

    @property
    def pos_std(self) -> float:
        """位置不確定度（m，兩軸幾何平均）。"""
        return float(math.sqrt(max(self.P[0, 0] + self.P[1, 1], 0.0) / 2.0))

    def predict_ahead(self, lead_s: float) -> np.ndarray:
        """目前估計往前外推 lead_s 秒的位置（導引用，不動內部狀態）。"""
        return self.x[:2] + self.x[2:] * lead_s
