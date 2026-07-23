"""棋盤格相機校正（UI 校正頁的後端）。

流程：對著棋盤格從不同角度/距離擷取 15~25 張 → calibrateCamera →
存成 config/camera_intrinsics.yaml，測地立即改用真實內參＋畸變係數。
"""

from __future__ import annotations

import threading

import cv2
import numpy as np

from ..geometry.camera_model import CameraModel


class CalibrationSession:
    def __init__(self, board_cols: int = 9, board_rows: int = 6, square_mm: float = 25.0):
        """board_cols/rows = 內角點數（10x7 格的板子 → 9x6）。"""
        self.board_size = (int(board_cols), int(board_rows))
        self.square_mm = float(square_mm)
        self._lock = threading.Lock()
        self.obj_points: list[np.ndarray] = []
        self.img_points: list[np.ndarray] = []
        self.image_size: tuple[int, int] | None = None  # (w, h)
        self.last_found = False
        self.result: dict | None = None

        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0 : self.board_size[0], 0 : self.board_size[1]].T.reshape(-1, 2)
        self._objp = objp * (self.square_mm / 1000.0)

    @property
    def count(self) -> int:
        with self._lock:
            return len(self.img_points)

    def find_corners(self, frame) -> np.ndarray | None:
        """找角點（供預覽疊加；不入庫）。"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray,
            self.board_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK,
        )
        self.last_found = bool(found)
        if not found:
            return None
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        return corners

    def capture(self, frame) -> bool:
        """本幀找得到棋盤就收錄一張，回傳是否成功。"""
        corners = self.find_corners(frame)
        if corners is None:
            return False
        with self._lock:
            h, w = frame.shape[:2]
            self.image_size = (w, h)
            self.obj_points.append(self._objp.copy())
            self.img_points.append(corners)
        return True

    def compute(self) -> dict:
        """執行校正，回傳 {rms, camera_model, hfov_deg}；樣本不足丟 ValueError。"""
        with self._lock:
            if len(self.img_points) < 8:
                raise ValueError(f"樣本不足：目前 {len(self.img_points)} 張，至少 8 張（建議 15+）")
            rms, K, dist, _rvecs, _tvecs = cv2.calibrateCamera(
                self.obj_points, self.img_points, self.image_size, None, None
            )
            model = CameraModel(K, dist.reshape(-1), self.image_size[0], self.image_size[1], source="calibrated")
            self.result = {"rms": float(rms), "camera_model": model, "hfov_deg": model.hfov_deg}
            return self.result

    def save(self, path: str) -> None:
        if not self.result:
            raise ValueError("尚未計算校正結果")
        self.result["camera_model"].save(path, rms=self.result["rms"])

    def reset(self) -> None:
        with self._lock:
            self.obj_points.clear()
            self.img_points.clear()
            self.result = None
