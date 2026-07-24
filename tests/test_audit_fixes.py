"""路徑總體檢確認後修正的迴歸鎖定：GPS 單位、pending 逾期、校正尺寸、RTL 清佇列。"""

import numpy as np
import pytest

from uav_yolo.links import Lr24CommandChannel
from uav_yolo.mavlink_io.telemetry import GpsSample, LandedSample
from uav_yolo.safety import SafetyGates
from uav_yolo.vision.calibration import CalibrationSession
from uav_yolo.vision.detector import Detection, TargetLock


def make_gates(**over):
    cfg = {"allowed_modes": ["AUTO.LOITER"], **over}
    return SafetyGates(cfg, rate_hz=1.0)


def base_kwargs(gps):
    return dict(
        guidance_enabled=True, mode="AUTO.LOITER", armed=True, link_ok=True,
        est_initialized=True, est_age_s=0.1, coast_timeout_s=8.0,
        cmd_point_ne=np.array([50.0, 0.0]),
        gps=gps, landed=LandedSample(0.0, landed_state=2),
    )


# ---------------- GPS 精度單位（eph 是 HDOP×100 不是公分） ----------------

def test_gps_gate_prefers_meter_accuracy_from_h_acc():
    gates = make_gates()
    good = GpsSample(0.0, 3, 12, eph_m=1.2, epv_m=2.0, hdop=9.9, vdop=9.9)
    # h_acc 可用時以公尺精度為準——即使 DOP 爛也不看
    assert gates.evaluate(0.0, **base_kwargs(good)).ok

    bad = GpsSample(0.0, 3, 12, eph_m=7.0, epv_m=2.0, hdop=0.5, vdop=0.5)
    report = gates.evaluate(0.0, **base_kwargs(bad))
    assert not report.ok
    assert any("水平誤差" in b for b in report.blocked)


def test_gps_gate_falls_back_to_dop_when_no_h_acc():
    gates = make_gates()
    # 老韌體：無 h_acc（inf），只有 DOP
    ok_dop = GpsSample(0.0, 3, 12, hdop=1.0, vdop=1.5)
    assert gates.evaluate(0.0, **base_kwargs(ok_dop)).ok

    bad_dop = GpsSample(0.0, 3, 12, hdop=4.0, vdop=1.5)
    report = gates.evaluate(0.0, **base_kwargs(bad_dop))
    assert not report.ok
    assert any("HDOP" in b for b in report.blocked)


def test_gps_gate_blocks_when_no_accuracy_info_at_all():
    gates = make_gates()
    unknown = GpsSample(0.0, 3, 12)  # 無 h_acc 也無 DOP
    report = gates.evaluate(0.0, **base_kwargs(unknown))
    assert not report.ok
    assert any("精度" in b for b in report.blocked)


# ---------------- pending_manual_id 逾期 ----------------

def det(tid):
    return Detection(track_id=tid, cls_name="Car", conf=0.8, bbox=(0, 0, 50, 50))


def test_stale_manual_request_expires_and_cannot_hijack_later():
    lock = TargetLock(mode="auto", min_lock_frames=2)
    lock.request_manual_lock(99)  # 點了一個之後才會出現的 ID

    for _ in range(TargetLock.PENDING_EXPIRE_FRAMES):
        lock.update([det(1)])
    assert lock.pending_manual_id is None, "過期的手動請求未被清除"
    assert lock.locked_id == 1  # auto 鎖定不被殭屍請求卡住

    # 之後 99 出現也不會被舊請求劫持
    got = lock.update([det(1), det(99)])
    assert got.track_id == 1


def test_manual_mode_request_also_expires():
    lock = TargetLock(mode="manual", min_lock_frames=1)
    lock.request_manual_lock(7)
    for _ in range(TargetLock.PENDING_EXPIRE_FRAMES + 1):
        lock.update([det(3)])
    assert lock.pending_manual_id is None


# ---------------- 校正尺寸一致性 ----------------

def test_calibration_rejects_mixed_resolution(tmp_path):
    import cv2
    from test_vision import render_board

    true_K = np.array([[800.0, 0, 640.0], [0, 800.0, 360.0], [0, 0, 1.0]])
    sess = CalibrationSession(9, 6, 25.0)
    frame = render_board(true_K, np.array([0.0, 0.0, 0.0]), np.array([-0.11, -0.07, 0.45]),
                         (9, 6), 0.025, (1280, 720))
    assert sess.capture(frame)

    smaller = cv2.resize(frame, (640, 360))
    with pytest.raises(ValueError, match="尺寸"):
        sess.capture(smaller)
    assert sess.count == 1  # 錯的沒被收進去


# ---------------- RTL 清空排隊中的 GOTO ----------------

def test_rtl_clears_pending_goto():
    ch = Lr24CommandChannel(transport=None, status_interval_s=999)
    ch.goto(24.5, 120.8, 60.0)
    assert ch._pending_goto is not None
    ch.request_rtl()
    assert ch._pending_goto is None, "RTL 後殘留的 GOTO 會把飛機又導回目標"
    assert ch._emergency == "RTL"
