"""鏈路自檢測試：每條鏈路的通/不通判定要正確，且自檢絕不可發送指令。"""

import numpy as np
import pytest
from fastapi.testclient import TestClient

from uav_yolo.config import Config
from uav_yolo.selfcheck import run_selfcheck
from uav_yolo.simulation import build_sim_engine
from uav_yolo.webapp.server import create_app

DT = 0.05


def make_engine(tmp_path, **over):
    cfg = Config()
    cfg._local_path = tmp_path / "local.yaml"
    patch = {"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
             "sim": {"patrol": False}}
    patch.update(over)
    cfg.update(patch)
    engine = build_sim_engine(cfg, realtime=False)
    return engine, engine.sim_world, cfg


def crank(engine, world, seconds):
    for _ in range(int(seconds / DT)):
        world.step(DT)
        engine.step()


def by_key(result):
    return {c["key"]: c for c in result["checks"]}


def test_selfcheck_covers_every_link(tmp_path):
    engine, world, cfg = make_engine(tmp_path)
    crank(engine, world, 6.0)
    res = run_selfcheck(engine, cfg)

    expected = {"video", "camera", "latency", "detector", "telemetry",
                "gps", "home", "gimbal", "command", "pipeline"}
    assert set(by_key(res)) == expected, "自檢漏掉鏈路"
    for c in res["checks"]:
        assert c["status"] in ("pass", "warn", "fail", "skip")
        assert c["detail"], f"{c['key']} 沒有說明實測到什麼"


def test_healthy_sim_passes_core_links(tmp_path):
    engine, world, cfg = make_engine(tmp_path, video={"latency_ms": 120,
                                                      "width": 640, "height": 360})
    crank(engine, world, 8.0)
    res = run_selfcheck(engine, cfg)
    k = by_key(res)

    for key in ("telemetry", "gps", "home", "gimbal", "latency", "pipeline"):
        assert k[key]["status"] == "pass", f"{key} 應通過但為 {k[key]['status']}：{k[key]['detail']}"
    assert res["counts"]["fail"] == 0


def test_missing_telemetry_is_reported_as_fail(tmp_path):
    """沒有心跳＝數傳不通，必須是 fail 並給出排除方向。"""
    engine, world, cfg = make_engine(tmp_path)
    engine.link.store.heartbeat = None
    res = run_selfcheck(engine, cfg)
    k = by_key(res)

    assert k["telemetry"]["status"] == "fail"
    assert k["telemetry"]["fix"], "不通卻沒告訴使用者怎麼修"
    assert res["verdict"] == "fail"


def test_bad_gps_is_reported_as_fail(tmp_path):
    from uav_yolo.mavlink_io.telemetry import GpsSample

    engine, world, cfg = make_engine(tmp_path)
    crank(engine, world, 2.0)
    engine.link.store.set_gps(GpsSample(0.0, fix_type=1, satellites=3, hdop=9.0))
    res = run_selfcheck(engine, cfg)
    k = by_key(res)

    assert k["gps"]["status"] == "fail"
    assert "衛星" in k["gps"]["detail"] or "fix" in k["gps"]["detail"]


def test_uncalibrated_camera_warns_not_fails(tmp_path):
    """未校正可以飛（精度差），是 warn 不是 fail。"""
    engine, world, cfg = make_engine(tmp_path)
    crank(engine, world, 2.0)
    res = run_selfcheck(engine, cfg)
    k = by_key(res)
    assert k["camera"]["status"] in ("warn", "pass")
    if k["camera"]["status"] == "warn":
        assert "校正" in k["camera"]["fix"]


def test_zero_latency_warns(tmp_path):
    engine, world, cfg = make_engine(tmp_path, video={"latency_ms": 0,
                                                      "width": 640, "height": 360})
    res = run_selfcheck(engine, cfg)
    assert by_key(res)["latency"]["status"] == "warn"


def test_gimbal_skipped_when_absent(tmp_path):
    engine, world, cfg = make_engine(tmp_path, gimbal={"present": False})
    res = run_selfcheck(engine, cfg)
    assert by_key(res)["gimbal"]["status"] == "skip"


def test_selfcheck_never_sends_commands(tmp_path):
    """自檢是唯讀動作：跑完不可產生任何導引指令。"""
    engine, world, cfg = make_engine(tmp_path)
    crank(engine, world, 8.0)
    n_before = len(world.repositions)
    rois_before = len(world.rois)

    for _ in range(3):
        run_selfcheck(engine, cfg)

    assert len(world.repositions) == n_before, "自檢竟然發出了導引指令"
    assert len(world.rois) == rois_before


def test_selfcheck_endpoint(tmp_path):
    cfg = Config()
    cfg._local_path = tmp_path / "local.yaml"
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360}})
    app = create_app(cfg)
    with TestClient(app) as client:
        res = client.post("/api/selfcheck")
        assert res.status_code == 200
        body = res.json()
        assert body["verdict"] in ("pass", "warn", "fail")
        assert len(body["checks"]) == 10
        assert body["summary"]
