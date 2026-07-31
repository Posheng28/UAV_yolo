"""飛行員接管後的恢復流程。

機制鏈（實機已驗證各環節）：
    導引啟用中，飛行員半舵 0.5 秒
        → PX4 自動切 Position（MAN_OVERRIDE_SPD）
        → 模式離開 allowed_moded → SafetyGates 閂鎖 → 停止發送
    恢復 = 切回 Hold + 在 UI 按「恢復導引」
        → set_guidance_enabled(True)（已啟用時再啟用）
        → 解除閂鎖 + 清 deadband 基準 + **不換任務記錄檔**

「不換記錄檔」是刻意的：接管-恢復是同一趟任務的一段插曲，
覆盤時要在同一份時間軸上看到它。
"""

import json

import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim",
                           "mission_log_dir": str(tmp_path / "m")},
                "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "guidance": {"enabled": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return build_sim_engine(cfg, realtime=False)


def latch(engine):
    """模擬接管：導引開著、模式從 Hold 被踢到 Position。"""
    engine.gates.observe_mode("AUTO.LOITER", guidance_enabled=True)
    engine.gates.observe_mode("POSCTL", guidance_enabled=True)
    assert engine.gates.pilot_override_latched


def test_stick_input_latches_and_blocks(tmp_path):
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    latch(engine)
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().latched is True, "UI 靠這個欄位顯示恢復按鈕"
    assert any("接管" in g for g in engine.gate_report_blocked), (
        f"閘門沒有講明是接管擋的：{engine.gate_report_blocked}"
    )


def test_reenable_clears_the_latch(tmp_path):
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    latch(engine)
    engine.set_guidance_enabled(True)        # UI 恢復按鈕做的事
    assert engine.gates.pilot_override_latched is False
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().latched is False


def test_recovery_does_not_rotate_the_mission_file(tmp_path):
    """接管-恢復是同一趟任務，記錄不能被切成兩份。"""
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    latch(engine)
    engine.set_guidance_enabled(True)
    files = list((tmp_path / "m").glob("*.jsonl"))
    assert len(files) == 1, f"恢復導引不該開新記錄檔，實得 {len(files)} 份"


def test_recovery_resets_the_deadband_baseline(tmp_path):
    """接管期間飛行員可能把機體飛到別處；恢復後第一筆指令
    不能被「接管前的舊指令點」的 deadband 吃掉。"""
    import numpy as np

    from uav_yolo.engine import LastCommand

    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    engine.last_cmd = LastCommand(t=0.0, lat=24.786, lon=120.997, alt_rel=4.0,
                                  point_ne=np.array([2.0, 2.0]), label="follow",
                                  radius=None)
    latch(engine)
    engine.set_guidance_enabled(True)
    assert engine.last_cmd is None, "deadband 基準沒清，恢復後可能一筆都不發"


def test_latch_survives_mode_switch_back_without_ui_action(tmp_path):
    """只切回 Hold、不按 UI → 閂鎖必須維持。
    這是刻意設計：防止系統在飛行員不知情時自己搶回控制。"""
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    latch(engine)
    engine.gates.observe_mode("AUTO.LOITER", guidance_enabled=True)  # 切回 Hold
    assert engine.gates.pilot_override_latched is True, (
        "切回 Hold 就自動恢復的話，等於系統偷偷搶回控制權"
    )
