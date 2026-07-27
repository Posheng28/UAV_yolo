"""設定熱套用：操作員在設定頁改門檻後必須「當場生效」，不能等重啟引擎。

這條路徑的落差是會出事的：把圍欄從 500m 改成 200m 按下儲存，
若引擎仍抱著建構當下的 500m，指令會照舊飛到 500m 外。
"""

import numpy as np
import pytest
from fastapi.testclient import TestClient

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine
from uav_yolo.webapp.server import create_app


def make_cfg(tmp_path) -> Config:
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return cfg


# ---------------- 引擎層熱套用 ----------------

def test_apply_live_config_updates_geofence(tmp_path):
    cfg = make_cfg(tmp_path)
    engine = build_sim_engine(cfg, realtime=False)
    assert engine.gates.max_cmd_distance_m == 500.0

    cfg.update({"safety": {"max_cmd_distance_m": 200.0, "max_cmd_alt_m": 80.0}})
    applied = engine.apply_live_config()

    assert "safety" in applied
    assert engine.gates.max_cmd_distance_m == 200.0
    assert engine.gates.clamp_alt(300.0) == 80.0


def test_apply_live_config_preserves_pilot_override_latch(tmp_path):
    """熱套用不能把接管閂鎖洗掉——否則改個參數就等於偷偷重新啟用導引。"""
    cfg = make_cfg(tmp_path)
    engine = build_sim_engine(cfg, realtime=False)
    engine.gates.observe_mode("AUTO.LOITER", guidance_enabled=True)
    engine.gates.observe_mode("POSCTL", guidance_enabled=True)
    assert engine.gates.pilot_override_latched

    cfg.update({"safety": {"max_cmd_distance_m": 300.0}})
    engine.apply_live_config()

    assert engine.gates.pilot_override_latched, "熱套用洗掉了接管閂鎖"
    assert engine.gates.max_cmd_distance_m == 300.0


def test_apply_live_config_updates_guidance_and_detector_and_latency(tmp_path):
    cfg = make_cfg(tmp_path)
    engine = build_sim_engine(cfg, realtime=False)

    cfg.update({
        "guidance": {"multirotor": {"follow_alt_m": 55.0, "standoff_m": 25.0}},
        "detector": {"conf": 0.8, "min_lock_frames": 3},
        "video": {"latency_ms": 250},
        "estimator": {"coast_timeout_s": 12.0},
    })
    engine.apply_live_config()

    assert engine.guidance.follow_alt_m == 55.0
    assert engine.guidance.standoff_m == 25.0
    assert engine.lock.min_lock_frames == 3
    assert engine.video_latency_s == pytest.approx(0.25)
    assert engine.coast_timeout_s == 12.0


def test_live_geofence_actually_blocks_commands(tmp_path):
    """端到端：改嚴圍欄後，原本會過的指令點必須被擋下。"""
    cfg = make_cfg(tmp_path)
    engine = build_sim_engine(cfg, realtime=False)

    far_point = np.array([300.0, 0.0])
    kw = dict(
        guidance_enabled=True, mode="AUTO.LOITER", armed=True, link_ok=True,
        est_initialized=True, est_age_s=0.1, coast_timeout_s=8.0,
        cmd_point_ne=far_point,
        gps=engine.link.store.gps, landed=engine.link.store.landed,
    )
    # sim 世界推一步，確保 gps/landed 遙測已就緒
    engine.sim_world.step(0.05)
    kw["gps"] = engine.link.store.gps
    kw["landed"] = engine.link.store.landed

    assert engine.gates.evaluate(0.0, **kw).ok  # 500m 圍欄下 300m 可過

    cfg.update({"safety": {"max_cmd_distance_m": 200.0}})
    engine.apply_live_config()

    report = engine.gates.evaluate(10.0, **kw)
    assert not report.ok
    assert any("圍欄" in b for b in report.blocked)


# ---------------- webapp 端點 ----------------

def test_put_config_applies_live_and_reports(tmp_path, monkeypatch):
    monkeypatch.setattr("uav_yolo.config.LOCAL_PATH", tmp_path / "local.yaml")
    cfg = make_cfg(tmp_path)
    app = create_app(cfg)

    with TestClient(app) as client:
        res = client.put("/api/config", json={"safety": {"max_cmd_distance_m": 150.0}})
        assert res.status_code == 200
        body = res.json()
        assert body["ok"] is True
        assert "safety" in body.get("applied", [])
        # 執行中的引擎當場吃到新值
        assert app.state.manager.engine.gates.max_cmd_distance_m == 150.0


def test_status_endpoint_shape(tmp_path):
    cfg = make_cfg(tmp_path)
    app = create_app(cfg)
    with TestClient(app) as client:
        st = client.get("/api/status").json()
        assert st["ready"] is True
        for key in ("state", "vehicle", "target", "gates", "video", "mavlink", "gimbal"):
            assert key in st, f"狀態缺欄位 {key}（UI 會讀）"


def test_checklist_and_guidance_endpoints(tmp_path):
    cfg = make_cfg(tmp_path)
    app = create_app(cfg)
    with TestClient(app) as client:
        cl = client.get("/api/checklist").json()
        assert cl["total"] > 0
        assert client.post("/api/guidance", json={"enabled": True}).json()["ok"]
        assert app.state.manager.engine.guidance_enabled is True
        assert client.post("/api/unlock").json()["ok"]


def test_unsupported_firmware_is_rejected(tmp_path):
    """vehicle.firmware 曾是無人讀取的死設定，會讓人誤以為能切 ArduPilot。"""
    cfg = make_cfg(tmp_path)
    cfg.update({"vehicle": {"firmware": "ardupilot"}})
    with pytest.raises(ValueError, match="px4"):
        build_sim_engine(cfg, realtime=False)
