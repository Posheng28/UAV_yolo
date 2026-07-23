"""視覺層測試：鎖定狀態機（純邏輯）＋合成影像校正全流程。"""

import cv2
import numpy as np
import pytest

from uav_yolo.vision.calibration import CalibrationSession
from uav_yolo.vision.detector import Detection, TargetLock, build_class_filter


def det(tid, area_scale=1.0, cls="Car"):
    size = 100.0 * area_scale
    return Detection(track_id=tid, cls_name=cls, conf=0.8, bbox=(0.0, 0.0, size, size))


# ---------- 類別過濾 ----------

def test_class_filter_case_insensitive():
    names = {0: "car", 1: "person", 2: "truck"}
    assert build_class_filter(names, ["Car", "Truck"]) == {0, 2}


def test_class_filter_no_match_returns_none():
    assert build_class_filter({0: "cat"}, ["Car"]) is None
    assert build_class_filter({0: "car"}, []) is None


# ---------- 鎖定狀態機 ----------

def test_auto_lock_needs_consecutive_frames():
    lock = TargetLock(mode="auto", min_lock_frames=3)
    assert lock.update([det(7)]) is None
    assert lock.update([det(7)]) is None
    got = lock.update([det(7)])  # 第 3 幀達標：鎖定且當幀立即回傳
    assert got is not None and got.track_id == 7
    assert lock.locked_id == 7
    assert lock.update([det(7)]).track_id == 7


def test_auto_lock_prefers_largest_and_resets_on_switch():
    lock = TargetLock(mode="auto", min_lock_frames=3)
    lock.update([det(1, 1.0), det(2, 2.0)])  # 2 比較大 → 候選 2
    lock.update([det(1, 3.0), det(2, 1.0)])  # 換 1 大 → 候選歸零重數
    lock.update([det(1, 3.0)])
    assert not lock.locked
    lock.update([det(1, 3.0)])
    assert lock.locked_id == 1


def test_locked_follows_id_not_size():
    """鎖定後就算出現更大的框也不換目標（修掉舊系統多車亂跳）。"""
    lock = TargetLock(mode="auto", min_lock_frames=1)
    lock.update([det(5)])
    got = lock.update([det(5, 1.0), det(9, 10.0)])
    assert got.track_id == 5
    assert lock.locked_id == 5


def test_locked_target_missing_returns_none_keeps_lock():
    lock = TargetLock(mode="auto", min_lock_frames=1)
    lock.update([det(5)])
    assert lock.update([]) is None       # 本幀沒看到
    assert lock.locked_id == 5           # 但鎖定保持（coast 由引擎層決定何時放棄）


def test_manual_mode_waits_for_click():
    lock = TargetLock(mode="manual", min_lock_frames=1)
    for _ in range(10):
        assert lock.update([det(3), det(4)]) is None
    assert not lock.locked
    lock.request_manual_lock(4)
    got = lock.update([det(3), det(4)])
    assert got.track_id == 4


def test_manual_relock_in_auto_mode():
    lock = TargetLock(mode="auto", min_lock_frames=1)
    lock.update([det(1)])
    assert lock.locked_id == 1
    lock.request_manual_lock(2)  # UI 點另一台車
    got = lock.update([det(1), det(2)])
    assert got.track_id == 2


def test_ground_pixel_is_bottom_center():
    d = Detection(track_id=1, cls_name="Car", conf=0.9, bbox=(100.0, 50.0, 200.0, 150.0))
    assert d.ground_pixel == (150.0, 150.0)


# ---------- 合成影像校正 ----------

def render_board(K, rvec, tvec, board_size, square_m, image_size):
    """用已知內參/外參把棋盤投影畫成影像（黑白格）。"""
    w, h = image_size
    img = np.full((h, w), 160, np.uint8)
    cols, rows = board_size  # 內角點數
    for gy in range(rows + 1):
        for gx in range(cols + 1):
            # 每格四角（物座標，含外圍一圈）
            corners_obj = np.array(
                [
                    [(gx - 1) * square_m, (gy - 1) * square_m, 0],
                    [gx * square_m, (gy - 1) * square_m, 0],
                    [gx * square_m, gy * square_m, 0],
                    [(gx - 1) * square_m, gy * square_m, 0],
                ],
                np.float32,
            )
            pts, _ = cv2.projectPoints(corners_obj, rvec, tvec, K, None)
            color = 0 if (gx + gy) % 2 == 0 else 255
            cv2.fillConvexPoly(img, pts.reshape(-1, 2).astype(np.int32), color)
    return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)


def test_calibration_recovers_known_intrinsics():
    """合成 12 個視角 → 校正應還原出已知焦距（誤差 <2%）。"""
    true_K = np.array([[800.0, 0, 640.0], [0, 800.0, 360.0], [0, 0, 1.0]])
    board = (9, 6)
    square = 0.025
    sess = CalibrationSession(board_cols=9, board_rows=6, square_mm=25.0)

    poses = [
        ((0.0, 0.0, 0.0), (-0.11, -0.07, 0.45)),
        ((0.2, 0.0, 0.0), (-0.10, -0.08, 0.50)),
        ((-0.2, 0.1, 0.0), (-0.12, -0.06, 0.48)),
        ((0.0, 0.25, 0.1), (-0.09, -0.07, 0.55)),
        ((0.15, -0.2, 0.0), (-0.13, -0.05, 0.42)),
        ((-0.15, -0.15, 0.2), (-0.10, -0.09, 0.52)),
        ((0.3, 0.1, -0.1), (-0.08, -0.07, 0.60)),
        ((0.0, -0.3, 0.0), (-0.12, -0.08, 0.47)),
        ((-0.25, 0.2, 0.1), (-0.11, -0.04, 0.58)),
        ((0.1, 0.15, 0.3), (-0.09, -0.06, 0.50)),
        ((-0.1, -0.1, -0.2), (-0.14, -0.07, 0.44)),
        ((0.25, -0.05, 0.15), (-0.10, -0.10, 0.53)),
    ]
    for rvec, tvec in poses:
        frame = render_board(true_K, np.array(rvec), np.array(tvec), board, square, (1280, 720))
        sess.capture(frame)

    assert sess.count >= 10  # 至少大部分視角成功找到角點
    result = sess.compute()
    assert result["rms"] < 1.0
    K = result["camera_model"].K
    assert K[0, 0] == pytest.approx(800.0, rel=0.02)
    assert K[1, 1] == pytest.approx(800.0, rel=0.02)
    assert K[0, 2] == pytest.approx(640.0, rel=0.03)


def test_calibration_insufficient_samples_raises():
    sess = CalibrationSession()
    with pytest.raises(ValueError):
        sess.compute()
