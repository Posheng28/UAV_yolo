"""任務錄影與重鎖門檻——2026-08-04 實飛回饋所引出的兩項。

實飛觀察（data/missions/mission_20260804_102026.jsonl）：
    · 3.1m 高、HFOV 91° → 可視地面僅約 6.3×3.6m，車子 1m/s 移動 3 秒就出框；
      偵測只在 54% 的 snap 存在，狀態每 0.5 秒 TRACK↔COAST 跳動。
    · 期間 dets 常為 2（只有一台車），代表有草地誤判同時存在。
    · 舊的重鎖門檻固定下限 15m，比整個視野還寬 2.5 倍 → 誤判可被接手成目標。
    · JSONL 記得到座標，但記不到「那個框是車還是草」——所以要錄影。
"""

import numpy as np
import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path, **over):
    cfg = Config(local_path=tmp_path / "local.yaml")
    base = {
        "system": {"mode": "sim", "mission_log_dir": str(tmp_path / "m")},
        "vehicle": {"airframe": "multirotor"},
        "video": {"width": 640, "height": 360},
        "detector": {"lock_mode": "auto", "min_lock_frames": 6},
        "sim": {"patrol": False},
        "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")},
    }
    for k, v in over.items():
        base.setdefault(k, {}).update(v) if isinstance(v, dict) else base.update({k: v})
    cfg.update(base)
    return build_sim_engine(cfg, realtime=False)


def crank(engine, seconds, dt=0.05):
    for _ in range(int(seconds / dt)):
        engine.sim_world.step(dt)
        engine.step()


def test_guidance_start_records_a_playable_video(tmp_path):
    """導引一開就要開始錄，關掉就收檔，而且檔案要真的能讀。"""
    import cv2

    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    crank(engine, 3.0)
    assert engine._mission_video is not None, "導引開了卻沒有在錄"
    path = engine._mission_video_path
    engine.set_guidance_enabled(False)
    assert engine._mission_video is None, "關導引後沒有收檔"

    assert path is not None and path.exists(), "錄影檔沒有產生"
    assert path.suffix == ".mp4" and path.stem == engine_stem(path), "檔名要與 JSONL 同名"
    assert path.stat().st_size > 1000, f"錄影檔太小（{path.stat().st_size} bytes），可能沒寫進去"
    cap = cv2.VideoCapture(str(path))
    try:
        assert cap.isOpened(), "錄影檔打不開"
        ok, frame = cap.read()
        assert ok and frame is not None, "錄影檔讀不出畫面"
    finally:
        cap.release()


def engine_stem(path):
    return path.stem


def test_recording_can_be_disabled(tmp_path):
    engine = make_engine(tmp_path, system={"mission_video": False})
    engine.set_guidance_enabled(True)
    crank(engine, 1.0)
    try:
        assert engine._mission_video is None, "設定關掉了還是在錄"
    finally:
        engine.set_guidance_enabled(False)


def test_deadband_never_exceeds_what_the_camera_can_see(tmp_path):
    """🔴 指令重發門檻不能大於可視範圍，否則控制迴路失去意義。

    實飛數據：deadband 設 3.0m，而 3.1m 高度下短邊視野全長只有 3.6m。
    車子幾乎要走完整個畫面，指令才更新一次（實測間隔 2.4 秒 vs 穿越畫面
    3.6 秒）——飛機永遠在追 3 秒前的位置，結果繞著目標打轉、半徑約 5m、
    週期約 10 秒，收斂不進去。
    """
    engine = make_engine(tmp_path)
    engine.reposition_deadband_m = 3.0

    import math

    def at(alt):
        return engine._effective_deadband(type("P", (), {"rel_alt": alt})())

    def half_view(alt):
        return math.tan(math.radians(engine.camera_model.vfov_deg / 2.0)) * alt

    low = at(3.1)
    # 不變量與鏡頭無關：門檻必須明顯小於「短邊半視野」，指令才來得及在
    # 目標離開畫面前更新。綁死絕對數字會隨鏡頭改變而失效。
    assert low < half_view(3.1), (
        f"3.1m 高時 deadband {low:.2f}m 不小於半視野 {half_view(3.1):.2f}m")
    assert low < 3.0, "低空時必須比設定值收得更緊"
    assert low >= 0.5, "太小也沒意義（發送本來就有 1Hz 限速）"
    assert at(6.0) > low, "門檻要隨高度放寬"
    assert at(40.0) == pytest.approx(3.0), "高空時應回到設定值，不該被壓小"
    # 沒有高度資訊時退回設定值，不要自作聰明
    assert engine._effective_deadband(None) == pytest.approx(3.0)


def test_reacquire_gate_scales_with_altitude(tmp_path):
    """🔴 重鎖門檻必須跟著高度縮放。

    舊值固定下限 15m，但天底相機在 3m 高只看得到約 6m 的地面——等於畫面裡
    任何一個偵測（包括草地誤判）都能被當成走失的目標接手，實飛就是這樣亂飛。
    """
    engine = make_engine(tmp_path)
    crank(engine, 8.0)
    assert engine.estimator.initialized, "測試前提：KF 要先起來"

    from uav_yolo.vision.detector import Detection

    def try_at(alt: float, offset_m: float) -> bool:
        """在高度 alt、距離預測點 offset_m 處放一個偵測，看會不會被接手。"""
        engine.lock.locked_id = None
        engine.lock._last_box = None          # 排除影像空間重綁，只驗世界座標這條
        target = engine.estimator.pos_ne + np.array([offset_m, 0.0])
        pos = type("P", (), {"rel_alt": alt, "lat": 24.0, "lon": 120.0,
                             "alt_amsl": 100.0, "vn": 0.0, "ve": 0.0})()
        det = Detection(track_id=9999, cls_name="car", conf=0.9, bbox=(10, 10, 40, 40))
        orig = engine._geolocate
        engine._geolocate = lambda d, p, a, t: target.copy()
        try:
            engine._try_reacquire([det], pos, None, engine.clock())
        finally:
            engine._geolocate = orig
        return engine.lock.locked_id == 9999

    # 3m 高：可視地面約 6m。舊版門檻固定 15m ⇒ 10m 外的草地誤判也會被接手。
    assert try_at(3.0, 2.0) is True, "近距離的合理重鎖被擋掉了，等於再也接不回目標"
    assert try_at(3.0, 10.0) is False, (
        "3m 高時 10m 外的偵測仍被接手——那比整個視野還遠，只可能是誤判")

    # 高空：門檻不該被高度壓死，仍由 KF 不確定度決定
    assert try_at(60.0, 2.0) is True, "高空的正常重鎖不該被擋"


# ---------------- 「已鎖定」與「算得出座標」是兩件事 ----------------

def test_locked_but_no_fix_says_why_instead_of_claiming_not_locked(tmp_path):
    """🔴 實機回報：畫面上紅框寫 LOCKED、清單寫「已鎖定」，閘門卻說
    「尚未鎖定目標」——兩邊互相矛盾，操作員只能困惑。

    真正的原因是載具 rel_alt = -3m（飛機在 home 高度平面底下），往下的
    射線永遠碰不到那個平面，測地每幀無解，所以 KF 從未初始化。而閘門那句
    話看的是「KF 有沒有初始化」，不是「有沒有鎖定」。
    """
    from types import SimpleNamespace

    from uav_yolo.safety import SafetyGates
    from uav_yolo.vision.detector import Detection

    engine = make_engine(tmp_path)
    crank(engine, 8.0)
    assert engine.lock.locked, "測試前提：要先鎖上"

    det = Detection(track_id=10, cls_name="Car", conf=0.72, bbox=(400, 250, 460, 300))
    att = engine.link.store.attitude_at(engine.clock())
    pos = SimpleNamespace(lat=24.7852, lon=120.9965, rel_alt=-3.0,
                          alt_amsl=97.0, vn=0.0, ve=0.0)
    assert engine._geolocate(det, pos, att, engine.clock()) is None
    note = engine.last_meas_note
    assert "高度" in note and "-3" in note, f"沒講出是高度的問題：{note}"

    # 閘門要分辨「沒鎖定」與「鎖定了但算不出座標」
    gates = SafetyGates({"allowed_modes": ["AUTO.LOITER"]}, rate_hz=1.0)
    common = dict(guidance_enabled=True, mode="AUTO.LOITER", armed=True, link_ok=True,
                  est_age_s=0.0, coast_timeout_s=8.0, cmd_point_ne=None,
                  require_gps_quality=False)
    gates.require_gps_quality = False
    gates.require_airborne = False

    locked_msg = gates.evaluate(0.0, est_initialized=False, target_locked=True,
                                meas_note=note, **{k: v for k, v in common.items()
                                                   if k != "require_gps_quality"}).blocked
    unlocked_msg = gates.evaluate(0.0, est_initialized=False, target_locked=False,
                                  meas_note="", **{k: v for k, v in common.items()
                                                   if k != "require_gps_quality"}).blocked
    assert any("已鎖定" in g and "算不出" in g for g in locked_msg), locked_msg
    assert any("高度" in g for g in locked_msg), "閘門要把測地失敗的原因帶出來"
    assert any("尚未鎖定" in g for g in unlocked_msg), unlocked_msg
    assert not any("已鎖定" in g for g in unlocked_msg), "沒鎖定時不該說已鎖定"


# ---------------- 測地的離地高度來源（光流/測距儀） ----------------

def test_rangefinder_height_rescues_a_broken_home_reference(tmp_path):
    """🔴 2026-08-04 實機：裝光流後 rel_alt 回報 -2.3m（飛機明明在飛），
    測地整趟無解、零指令。測距儀量的是離地真實高度，不受 home 基準影響。
    """
    from types import SimpleNamespace

    from uav_yolo.mavlink_io.telemetry import DistanceSample

    engine = make_engine(tmp_path)
    crank(engine, 6.0)
    att = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
    pos = SimpleNamespace(lat=24.7852, lon=120.9965, rel_alt=-2.3,
                          alt_amsl=97.0, vn=0.0, ve=0.0)

    # 沒有測距儀：只能用壞掉的 rel_alt
    engine.link.store.distance = None
    alt, src = engine._height_agl(pos, att)
    assert src == "home" and alt == pytest.approx(-2.3)

    # 有測距儀：拿到真實離地高度，測地就活了
    engine.link.store.distance = DistanceSample(
        t=engine.clock(), current_m=2.8, min_m=0.1, max_m=12.0, orientation=25)
    alt, src = engine._height_agl(pos, att)
    assert src == "rangefinder" and alt == pytest.approx(2.8, abs=0.01)


def test_rangefinder_is_rejected_when_out_of_range_stale_or_tilted(tmp_path):
    """測距儀不是無條件可信：超量程、過期、機身傾斜都要退回 rel_alt。"""
    from types import SimpleNamespace

    from uav_yolo.mavlink_io.telemetry import DistanceSample

    engine = make_engine(tmp_path)
    crank(engine, 6.0)
    level = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
    pos = SimpleNamespace(lat=24.7852, lon=120.9965, rel_alt=5.0,
                          alt_amsl=100.0, vn=0.0, ve=0.0)
    now = engine.clock()

    # 超出量程（讀數頂到 max）
    engine.link.store.distance = DistanceSample(now, 12.0, 0.1, 12.0, 25)
    assert engine._height_agl(pos, level)[1] == "home"

    # 資料過期
    engine.link.store.distance = DistanceSample(now - 30.0, 2.8, 0.1, 12.0, 25)
    assert engine._height_agl(pos, level)[1] == "home"

    # 機身傾斜 30 度：量到的是斜距，不是垂直高度
    import math as _m
    tilted = SimpleNamespace(roll=_m.radians(30.0), pitch=0.0, yaw=0.0)
    engine.link.store.distance = DistanceSample(now, 2.8, 0.1, 12.0, 25)
    assert engine._height_agl(pos, tilted)[1] == "home"

    # 不是朝下的感測器（例如前向避障）
    engine.link.store.distance = DistanceSample(now, 2.8, 0.1, 12.0, orientation=0)
    assert engine._height_agl(pos, level)[1] == "home"


def test_height_source_can_be_forced_to_home(tmp_path):
    """設定成 home 就一律用 rel_alt，不要自作聰明。"""
    from types import SimpleNamespace

    from uav_yolo.mavlink_io.telemetry import DistanceSample

    engine = make_engine(tmp_path)
    engine.cfg.override({"camera": {"height_source": "home"}})
    crank(engine, 6.0)
    engine.link.store.distance = DistanceSample(engine.clock(), 2.8, 0.1, 12.0, 25)
    pos = SimpleNamespace(lat=24.0, lon=120.0, rel_alt=5.0, alt_amsl=100.0, vn=0.0, ve=0.0)
    att = SimpleNamespace(roll=0.0, pitch=0.0, yaw=0.0)
    assert engine._height_agl(pos, att) == (5.0, "home")


def test_recording_stops_when_nothing_is_happening(tmp_path):
    """🔴 導引忘了關時，影片不該繼續錄地面。

    實測後果：一個 181.9MB 的檔案，內容是 7.5 分鐘的靜止地面，而該趟真正
    的飛行只有 190 秒；一個下午 13 趟共 414MB，JSONL 才 1.5MB。
    """
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    crank(engine, 2.0)
    assert engine._mission_video is not None, "測試前提：有事發生時要在錄"

    frame = np.zeros((360, 640, 3), np.uint8)
    written = []
    engine._mission_video = type("W", (), {"write": lambda s, f: written.append(1)})()

    # 有事發生（已解鎖）→ 要寫
    engine.link.store.heartbeat.armed = True
    engine._mission_video_next_t = 0.0
    engine._mission_video_write(frame, 100.0)
    assert written, "解鎖中卻沒在錄"

    # 沒事發生（未解鎖 + 沒有目標估計）→ 不寫
    written.clear()
    engine.link.store.heartbeat.armed = False
    engine.estimator.reset()
    engine._mission_video_next_t = 0.0
    engine._mission_video_write(frame, 200.0)
    assert not written, (
        "未解鎖且無目標時仍在寫入影片——導引忘了關就會錄出好幾百 MB 的地面")

    engine._mission_video = None
    engine.set_guidance_enabled(False)
