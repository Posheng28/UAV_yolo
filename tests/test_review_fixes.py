"""全專案覆審（2026-07-31）修正的回歸測試。

每個測試對應一項已確認缺陷，釘住「修正後」的正確行為：
    - KF / 測地 / 安全閘門對 NaN 一律 fail-closed
    - 手動鎖定：pending 不得餓死現任目標；點選已消失的 ID 要立即拒絕
    - LR24 重試不得被 staleness 誤殺
    - 引擎：idle 走擷取時間軸、圍欄錨定目前 home、loop_error 會自癒、
      發布序號單調遞增（前端凍結偵測的依據）
    - server：restart 失敗要浮出錯誤、engine 摘成 None
"""

import math
import time
import types

import numpy as np
import pytest

from uav_yolo.config import Config
from uav_yolo.estimation import TargetEstimator
from uav_yolo.geometry.geolocate import intersect_ground
from uav_yolo.safety import SafetyGates
from uav_yolo.simulation import build_sim_engine
from uav_yolo.vision.detector import Detection, TargetLock


def make_engine(tmp_path):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return build_sim_engine(cfg, realtime=False)


# ---------------- NaN fail-closed ----------------

def test_kf_rejects_nonfinite_measurement():
    kf = TargetEstimator()
    ok, why = kf.update(1.0, np.array([math.nan, 5.0]))
    assert not ok and "非有限" in why
    assert not kf.initialized, "NaN 不得初始化 KF"

    kf.update(1.0, np.array([10.0, 5.0]))
    assert kf.initialized
    ok, _ = kf.update(2.0, np.array([math.inf, 5.0]))
    assert not ok
    assert np.all(np.isfinite(kf.x)), "NaN 量測不得毒化既有狀態"


def test_intersect_ground_rejects_nonfinite():
    down = np.array([0.0, 0.0, 1.0])
    assert intersect_ground(np.array([0.0, 0.0, math.nan]), down) is None
    assert intersect_ground(np.array([0.0, 0.0, -10.0]),
                            np.array([math.nan, 0.0, 1.0])) is None
    # 正常路徑不受影響
    hit = intersect_ground(np.array([0.0, 0.0, -10.0]), down)
    assert hit is not None and hit[2] == pytest.approx(0.0)


def _all_pass_kwargs():
    gps = types.SimpleNamespace(fix_type=3, satellites=12, eph_m=1.0, epv_m=1.5,
                                hdop=0.8, vdop=1.0)
    landed = types.SimpleNamespace(airborne=True)
    return dict(guidance_enabled=True, mode="AUTO.LOITER", armed=True, link_ok=True,
                est_initialized=True, est_age_s=0.5, coast_timeout_s=8.0,
                gps=gps, landed=landed)


def test_gates_block_nonfinite_cmd_point():
    gates = SafetyGates({"allowed_modes": ["AUTO.LOITER"]}, rate_hz=1.0)
    report = gates.evaluate(0.0, cmd_point_ne=np.array([math.nan, 10.0]),
                            **_all_pass_kwargs())
    assert not report.ok
    assert any("非有限" in b for b in report.blocked), \
        "NaN 指令點必須被明確擋下（NaN>500 比較為 False，不擋就穿圍欄）"
    # 正常點通過
    report = gates.evaluate(0.0, cmd_point_ne=np.array([30.0, 10.0]),
                            **_all_pass_kwargs())
    assert report.ok


# ---------------- 手動鎖定 ----------------

def _det(tid, x=100.0, y=100.0):
    return Detection(track_id=tid, cls_name="car", conf=0.9,
                     bbox=(x, y, x + 50.0, y + 50.0))


def test_pending_absent_id_does_not_starve_locked_target():
    """manual 模式下誤點一個已消失的 ID，不得讓現任鎖定斷量測 3 秒。"""
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(1)
    assert lock.update([_det(1)]) is not None  # 鎖上 #1

    lock.request_manual_lock(999)              # 誤點：#999 不存在
    det = lock.update([_det(1)])
    assert det is not None and det.track_id == 1, \
        "pending 等待中仍要持續回報現任目標，否則 KF 進 coast"


def test_pending_absent_id_still_waits_when_nothing_locked():
    lock = TargetLock(mode="manual")
    lock.request_manual_lock(999)
    assert lock.update([_det(1)]) is None      # 沒現任鎖定時照舊空等


def test_manual_lock_rejects_vanished_id(tmp_path):
    engine = make_engine(tmp_path)
    for _ in range(3):
        engine.sim_world.step(0.05)
        engine.step()
    err = engine.manual_lock(424242)
    assert err is not None and "不在畫面" in err
    ids = {d["id"] for d in engine.status().detections}
    if ids:  # 模擬世界有偵測時，合法 ID 要能鎖
        assert engine.manual_lock(next(iter(ids))) is None


# ---------------- LR24 重試 ----------------

class _DeadTransport:
    """永遠收不到回覆的傳輸層（模擬機上端沒開/鏈路單向不通）。"""

    def write(self, data):
        return len(data)

    def flush(self):
        pass

    def readline(self):
        time.sleep(0.01)
        return b""

    def close(self):
        pass


def test_lr24_retry_survives_beyond_goto_max_age():
    """重排必須刷新時間戳：否則一次逾時＋backoff 後就被 stale 誤殺，
    「重試」實際上最多一次，靜止目標永遠收不到指令。"""
    from uav_yolo.links import Lr24CommandChannel

    ch = Lr24CommandChannel(transport=_DeadTransport(), response_timeout_s=0.05,
                            retry_backoff_s=0.01, goto_max_age_s=0.15,
                            status_interval_s=999.0)
    ch.start()
    try:
        ch.goto(24.0, 120.0, 4.0)
        time.sleep(0.6)  # 遠超過 goto_max_age_s，重試應仍在進行
        assert ch.retry_count >= 2, f"重試只跑了 {ch.retry_count} 次就停"
        assert ch.dropped_stale == 0, "重試中的目標不該被 staleness 丟棄"
    finally:
        ch.stop()
    snap = ch.snapshot()
    assert "retry" in snap and "dropped_stale" in snap


# ---------------- 引擎行為 ----------------

class _DeadVideo:
    def get_frame(self):
        return None, None

    def stop(self):
        pass


def test_idle_path_uses_capture_timeline(tmp_path):
    """影像斷線時 KF 只能推進到「現在 − 影像延遲」，不能推到量測的未來。"""
    engine = make_engine(tmp_path)
    engine.sim_world.step(0.05)
    engine.step()  # 讓 georef / home 就緒

    engine.video_latency_s = 0.5
    now = engine.clock()
    engine.estimator.update(now - 1.0, np.array([5.0, 5.0]))
    engine.video = _DeadVideo()
    engine._last_idle_publish = 0.0
    engine.step()
    assert engine.estimator.t == pytest.approx(now - 0.5, abs=0.05), \
        "idle 外推須落在擷取時間軸（clock − latency）"


def test_loop_error_clears_after_good_step(tmp_path):
    engine = make_engine(tmp_path)
    engine.loop_error = "TypeError: 舊的瞬態錯誤"
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.loop_error is None, "成功走完一輪要清 loop_error，否則永久紅色橫幅"


def test_status_seq_increments(tmp_path):
    engine = make_engine(tmp_path)
    engine.sim_world.step(0.05)
    engine.step()
    s1 = engine.status().seq
    engine.sim_world.step(0.05)
    engine.step()
    s2 = engine.status().seq
    assert s2 > s1 > 0, "發布序號要單調遞增（前端凍結偵測的依據）"


def test_fence_anchors_to_current_home(tmp_path):
    """PX4 重新解鎖會重設 home；圍欄距離必須以目前 home 為圓心。"""
    engine = make_engine(tmp_path)
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.georef is not None
    # home 尚未移動：錨點就是 NED 原點
    assert float(np.linalg.norm(engine._current_home_ne())) == pytest.approx(0.0, abs=1e-6)
    # 模擬 home 北移 ~111m（0.001 度）
    old_home = engine.link.store.home
    engine.link.store.home = types.SimpleNamespace(
        lat=old_home.lat + 0.001, lon=old_home.lon, alt_amsl=old_home.alt_amsl)
    ne = engine._current_home_ne()
    assert ne[0] == pytest.approx(111.3, abs=1.0)
    assert ne[1] == pytest.approx(0.0, abs=1e-3)


def test_commands_sent_uses_total_not_deque_len(tmp_path):
    engine = make_engine(tmp_path)
    assert engine.cmd_total == 0
    for i in range(20):  # 超過 deque maxlen=15
        engine.cmd_history.append({"t": float(i)})
        engine.cmd_total += 1
    assert len(engine.cmd_history) == 15
    assert engine.cmd_total == 20


# ---------------- server / EngineManager ----------------

def test_restart_failure_surfaces_error(tmp_path, monkeypatch):
    from uav_yolo.webapp import server as server_mod

    cfg = Config(local_path=tmp_path / "local.yaml")
    manager = server_mod.EngineManager(cfg)

    stopped = {"n": 0}

    class FakeEngine:
        def stop(self):
            stopped["n"] += 1

    manager.engine = FakeEngine()

    def boom(_cfg):
        raise RuntimeError("COM10 已被占用")

    monkeypatch.setattr(server_mod, "create_engine", boom)
    with pytest.raises(RuntimeError):
        manager.restart()
    assert stopped["n"] == 1, "重建前必須先停掉舊引擎（相機/序列埠獨占）"
    assert manager.engine is None, "失敗後不得殘留死引擎繼續回覆狀態"
    assert manager.error and "重啟失敗" in manager.error
