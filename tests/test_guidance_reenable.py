"""重新啟用導引／換目標後，第一筆指令不可被舊 deadband 吃掉。

實戰情境：飛行員接管把飛機飛到別處 → 操作員按重新啟用 → 目標沒怎麼動，
若拿接管前的舊指令點比 deadband，就會「閘門全綠但一筆都不發」，飛機不會回到目標。
"""

import numpy as np
import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine

DT = 0.05


def make_engine(tmp_path, airframe="multirotor"):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({
        "system": {"mode": "sim"},
        "vehicle": {"airframe": airframe},
        "video": {"width": 640, "height": 360},
        "sim": {"patrol": False},  # 這些測試依賴「目標會停」，關掉巡邏
        "camera": {"intrinsics_file": str(tmp_path / "no_such_intrinsics.yaml")},
    })
    engine = build_sim_engine(cfg, realtime=False)
    return engine, engine.sim_world


def crank(engine, world, seconds):
    for _ in range(int(seconds / DT)):
        world.step(DT)
        engine.step()


def test_reenable_clears_deadband_so_first_command_is_sent(tmp_path):
    engine, world = make_engine(tmp_path)
    engine.set_guidance_enabled(True)

    # 目標急停後跑一段，讓指令穩定下來（後續移動會小於 deadband）
    crank(engine, world, 32.0)
    assert world.repositions, "初始未發出任何指令"
    assert engine.last_cmd is not None

    # 飛行員接管 → 停發
    from uav_yolo.mavlink_io.telemetry import Heartbeat

    orig_push = world._push_telemetry

    def override_push():
        orig_push()
        world.store.set_heartbeat(Heartbeat(world.sim_t, mode="POSCTL", armed=True))

    world._push_telemetry = override_push
    crank(engine, world, 3.0)
    assert engine.gates.pilot_override_latched
    n_during_override = len(world.repositions)

    # 接管期間飛行員把機體飛走
    world.veh_pos = world.veh_pos + np.array([120.0, 60.0])

    # 交還控制權：目標仍靜止（移動遠小於 deadband）
    world._push_telemetry = orig_push
    engine.set_guidance_enabled(True)
    assert engine.last_cmd is None, "重新啟用未清除舊指令記錄"

    crank(engine, world, 4.0)
    assert len(world.repositions) > n_during_override, \
        "重新啟用後沒有發出任何指令（被舊 deadband 擋掉）"


def test_unlock_clears_last_command(tmp_path):
    engine, world = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    crank(engine, world, 12.0)
    assert engine.last_cmd is not None

    engine.unlock()
    assert engine.last_cmd is None
    assert engine.state == "SEARCH"


def test_deadband_suppression_is_reported_to_operator(tmp_path):
    """節流時 UI 不能只顯示「指令發送中」，要說明為何沒在發。"""
    engine, world = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    crank(engine, world, 34.0)  # 目標 25s 急停，之後幾乎不動 → 進入 deadband 抑制

    st = engine.status()
    assert st.guidance_enabled
    assert not st.gates, f"不該有閘門阻擋：{st.gates}"
    assert st.guidance_note, "deadband 抑制時沒有向操作員說明原因"
    assert "門檻" in st.guidance_note
