"""端到端模擬閉環測試：影像→偵測→測地→KF→導引→(模擬)飛控→動力學。

驗證使用者點名的核心場景：
    1. 追蹤精度：測地+KF 的位置誤差有界、速度估計收斂。
    2. 盲區：偵測 dropout 期間進 COAST、KF 外推撐住、恢復後續追。
    3. 急停：目標急停後導引點收斂到停點附近（定翼=繞著停點轉，不過頭）。
    4. 安全：指令速率限制、模式閘門。
"""

import numpy as np
import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine

DT = 0.05


def make_cfg(tmp_path, airframe: str) -> Config:
    cfg = Config()
    # 測試用 local.yaml 寫到 tmp，避免弄髒真的設定
    cfg._local_path = tmp_path / "local.yaml"
    cfg.update(
        {
            "system": {"mode": "sim"},
            "vehicle": {"airframe": airframe},
            "video": {"width": 960, "height": 540},
            "detector": {"lock_mode": "auto", "min_lock_frames": 6},
        }
    )
    return cfg


def crank(engine, world, seconds: float):
    steps = int(seconds / DT)
    for _ in range(steps):
        world.step(DT)
        engine.step()


@pytest.fixture
def mc(tmp_path):
    cfg = make_cfg(tmp_path, "multirotor")
    engine = build_sim_engine(cfg, realtime=False)
    engine.set_guidance_enabled(True)
    return engine, engine.sim_world


@pytest.fixture
def fw(tmp_path):
    cfg = make_cfg(tmp_path, "fixedwing")
    engine = build_sim_engine(cfg, realtime=False)
    engine.set_guidance_enabled(True)
    return engine, engine.sim_world


# ---------------- 旋翼 ----------------

def test_mc_locks_and_tracks_accurately(mc):
    engine, world = mc
    crank(engine, world, 10.0)

    assert engine.state == "TRACK"
    assert engine.lock.locked

    err = np.linalg.norm(engine.estimator.pos_ne - world.target_pos)
    assert err < 12.0, f"測地+KF 誤差 {err:.1f}m 過大"
    assert engine.estimator.speed == pytest.approx(world.target_speed, abs=2.5)


def test_mc_coast_through_dropout_then_recover(mc):
    engine, world = mc
    crank(engine, world, 13.5)  # dropout 預設 12~15s
    assert world.in_dropout()
    assert engine.state == "COAST"
    # 盲區中 KF 外推：誤差仍有界
    err = np.linalg.norm(engine.estimator.pos_ne - world.target_pos)
    assert err < 25.0

    crank(engine, world, 4.0)  # 過了 dropout
    assert engine.state == "TRACK"
    err = np.linalg.norm(engine.estimator.pos_ne - world.target_pos)
    assert err < 12.0


def test_mc_converges_after_sudden_stop(mc):
    engine, world = mc
    crank(engine, world, 33.0)  # 25s 急停 + 8s 收斂

    assert engine.estimator.speed < 2.0, "急停後速度估計未歸零"
    stop_err = np.linalg.norm(engine.estimator.pos_ne - world.target_pos)
    assert stop_err < 10.0

    # 導引點 = 停點 + standoff（15m）附近
    assert world.repositions, "從未發出導引指令"
    last = world.repositions[-1]
    cmd_err = np.linalg.norm(np.array([last["n"], last["e"]]) - world.target_pos)
    assert cmd_err < 15.0 + 10.0

    # 載具最終停在指令點附近懸停
    veh_err = np.linalg.norm(world.veh_pos - np.array([last["n"], last["e"]]))
    assert veh_err < 8.0


def test_mc_command_rate_limited_and_roi_sent(mc):
    engine, world = mc
    crank(engine, world, 20.0)

    ts = [r["t"] for r in world.repositions]
    assert len(ts) >= 3
    intervals = np.diff(ts)
    assert intervals.min() >= 0.95, f"指令間隔 {intervals.min():.2f}s 違反 1Hz 限制"

    assert world.rois, "雲台 ROI 從未更新"
    # ROI 指向目標估計位置（非載具位置）
    last_roi = world.rois[-1]
    roi_err = np.linalg.norm(np.array([last_roi["n"], last_roi["e"]]) - world.target_pos)
    assert roi_err < 15.0


def test_mc_pilot_override_stops_commands(mc):
    engine, world = mc
    crank(engine, world, 10.0)
    n_before = len(world.repositions)
    assert n_before > 0

    # 飛行員撥桿接管（模式切出 AUTO.LOITER）
    from uav_yolo.mavlink_io.telemetry import Heartbeat

    orig_push = world._push_telemetry

    def override_push():
        orig_push()
        world.store.set_heartbeat(Heartbeat(world.sim_t, mode="POSCTL", armed=True))

    world._push_telemetry = override_push
    crank(engine, world, 5.0)
    assert engine.gates.pilot_override_latched
    n_after_override = len(world.repositions)

    # 切回 AUTO.LOITER：閂鎖仍在、不恢復發送
    world._push_telemetry = orig_push
    crank(engine, world, 3.0)
    assert len(world.repositions) == n_after_override
    assert any("接管" in b for b in engine.gate_report_blocked)

    # UI 重新啟用 → 恢復
    engine.set_guidance_enabled(True)
    crank(engine, world, 3.0)
    assert len(world.repositions) > n_after_override


# ---------------- 固定翼 ----------------

def test_fw_standoff_orbit_never_overflies(fw):
    engine, world = fw
    crank(engine, world, 55.0)  # 接近 + 進軌道 + 目標 25s 急停後繞停點

    assert engine.state in ("TRACK", "COAST")
    assert world.repositions
    last = world.repositions[-1]
    assert last["radius"] == pytest.approx(150.0), "定翼指令未帶繞行半徑"

    # 急停後：指令圓心收斂到停點附近
    cmd_err = np.linalg.norm(np.array([last["n"], last["e"]]) - world.target_pos)
    assert cmd_err < 20.0

    # 最後 15 秒：載具距目標 ≈ 軌道半徑（150m），絕不衝到目標頭上
    dists = []
    for _ in range(int(15.0 / DT)):
        world.step(DT)
        engine.step()
        dists.append(float(np.linalg.norm(world.veh_pos - world.target_pos)))
    dists = np.array(dists)
    assert dists.min() > 60.0, f"定翼衝得離目標太近（{dists.min():.0f}m），standoff 失效"
    assert 100.0 < dists.mean() < 210.0, f"平均距離 {dists.mean():.0f}m 偏離軌道半徑"


def test_video_latency_compensation_removes_geolocation_bias(tmp_path):
    """圖傳延遲未補償會造成系統性測地偏移；設定 latency_ms 後應顯著改善。

    做法：讓影像來源回報「舊」的時間戳（模擬 300ms 鏈路延遲），
    比較 latency_ms=0 與 latency_ms=300 兩者的測地誤差。
    """
    LAG = 0.3

    def run(latency_ms):
        cfg = make_cfg(tmp_path / f"c{latency_ms}", "multirotor")
        cfg.update({"video": {"latency_ms": latency_ms}})
        engine = build_sim_engine(cfg, realtime=False)
        world = engine.sim_world

        # 包裝影像來源：畫面內容是「現在」，但時間戳謊報成 now（即實際延遲 LAG）
        real_get = engine.video.get_frame
        def lagged():
            frame, t = real_get()
            return frame, (None if t is None else t + LAG)
        engine.video.get_frame = lagged

        # 載具持續移動，延遲才會轉成位置誤差
        world.veh_vel = np.array([12.0, 0.0])
        errs = []
        for _ in range(int(12.0 / DT)):
            world.step(DT)
            world.veh_pos = world.veh_pos + np.array([12.0, 0.0]) * DT
            engine.step()
            if engine.estimator.initialized:
                errs.append(float(np.linalg.norm(engine.estimator.pos_ne - world.target_pos)))
        return float(np.mean(errs[-40:])) if len(errs) >= 40 else float("inf")

    err_uncompensated = run(0)
    err_compensated = run(int(LAG * 1000))

    assert err_compensated < err_uncompensated, (
        f"補償後誤差沒有改善：{err_compensated:.1f}m vs {err_uncompensated:.1f}m")


def test_fw_gimbal_keeps_target_in_view_while_orbiting(fw):
    engine, world = fw
    crank(engine, world, 40.0)  # 已進軌道

    # 繞行中最近 10 秒的偵測命中率：雲台 ROI 指向下有目標的幀應佔多數
    seen = 0
    total = int(10.0 / DT)
    for _ in range(total):
        world.step(DT)
        engine.step()
        if engine.status().detections:
            seen += 1
    assert seen / total > 0.7, f"繞行中目標可見率僅 {seen/total:.0%}（雲台指向失效）"
