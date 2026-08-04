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
        # 純量鏡像，讀取時不必拿鎖（見 count 的說明）
        self.n_views = 0
        self.computing = False

        objp = np.zeros((self.board_size[0] * self.board_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0 : self.board_size[0], 0 : self.board_size[1]].T.reshape(-1, 2)
        self._objp = objp * (self.square_mm / 1000.0)

    @property
    def count(self) -> int:
        """🔴 刻意不拿鎖。

        compute() 會握著 self._lock 跑 cv2.calibrateCamera，而那個函式的耗時
        是超線性的——實測 9×6 棋盤、1920×1080：10 張 0.3s、25 張 1.5s、
        50 張 20.5s、**100 張 211s**。如果 count 也要拿同一把鎖，計算期間
        `/api/calib/status` 會整個卡住，校正頁看起來就像「按了沒反應」。
        int 的讀寫在 CPython 是原子的，讀鏡像值安全。
        """
        return self.n_views

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
        """本幀找得到棋盤就收錄一張，回傳是否成功。

        解析度改變（換來源/換設定）時混入不同尺寸的樣本會讓校正整組錯掉，
        直接拒收；新增樣本也會作廢先前算好的結果（必須重算）。
        """
        corners = self.find_corners(frame)
        if corners is None:
            return False
        with self._lock:
            h, w = frame.shape[:2]
            if self.image_size is not None and self.image_size != (w, h):
                self.last_found = False
                raise ValueError(
                    f"影像尺寸改變（{self.image_size[0]}x{self.image_size[1]} → {w}x{h}）；"
                    "請按「開始/重來」重新收集"
                )
            self.image_size = (w, h)
            self.obj_points.append(self._objp.copy())
            self.img_points.append(corners)
            self.n_views = len(self.img_points)
            self.result = None  # 樣本已變，舊結果作廢
        return True

    # 超過這個張數，計算時間開始失控而精度早已飽和（實測見 count 的說明）
    RECOMMENDED_MAX_VIEWS = 30

    def estimated_compute_s(self) -> float:
        """粗估這次計算要跑多久（實測資料擬合，用來給操作員一個預期）。"""
        n = max(self.n_views, 1)
        return 0.3 * (n / 10.0) ** 3.2

    def compute(self) -> dict:
        """執行校正，回傳 {rms, camera_model, hfov_deg}；樣本不足丟 ValueError。"""
        with self._lock:
            if len(self.img_points) < 8:
                raise ValueError(f"樣本不足：目前 {len(self.img_points)} 張，至少 8 張（建議 15+）")
            self.computing = True
            try:
                # 🔴 先用標準 5 參數模型；若它在畫面四角之前就折返（＝去畸變在
                # 邊緣不可逆、測地會算出離譜座標），改用有理模型重算。
                #
                # 這不是「拍不夠多」能解決的：本專案的 FPV 鏡頭 HFOV 91°，
                # 桶狀畸變強到標準模型的徑向多項式 r(1+k1r²+k2r⁴+k3r⁶) 在
                # r≈0.88 就折返，而畫面四角在 r≈1.23。實測用 100 張重拍，
                # 折返半徑只從 0.8795 動到 0.8861——模型本身的極限。
                # 有理模型多了分母 (1+k4r²+k5r⁴+k6r⁶)，正是為廣角設計的。
                rms, K, dist, _r, _t = cv2.calibrateCamera(
                    self.obj_points, self.img_points, self.image_size, None, None
                )
                model = self._as_model(K, dist)
                used = "standard"
                if model.corner_radius() > model.max_invertible_r:
                    rms_r, K_r, dist_r, _r2, _t2 = cv2.calibrateCamera(
                        self.obj_points, self.img_points, self.image_size, None, None,
                        flags=cv2.CALIB_RATIONAL_MODEL,
                    )
                    model_r = self._as_model(K_r, dist_r)
                    # 只有真的把可逆範圍撐到四角以外才採用，否則留原本的
                    if model_r.corner_radius() <= model_r.max_invertible_r:
                        rms, model, used = rms_r, model_r, "rational"
            finally:
                self.computing = False
            self.result = {
                "rms": float(rms),
                "camera_model": model,
                "hfov_deg": model.hfov_deg,
                "model": used,
                "invertible_to_corners": model.corner_radius() <= model.max_invertible_r,
            }
            return self.result

    def _as_model(self, K, dist) -> CameraModel:
        return CameraModel(K, np.asarray(dist).reshape(-1),
                           self.image_size[0], self.image_size[1], source="calibrated")

    def save(self, path: str) -> None:
        if not self.result:
            raise ValueError("尚未計算校正結果")
        self.result["camera_model"].save(path, rms=self.result["rms"])

    def reset(self) -> None:
        with self._lock:
            self.obj_points.clear()
            self.img_points.clear()
            self.result = None
