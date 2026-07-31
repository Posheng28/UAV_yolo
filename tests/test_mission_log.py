"""任務記錄：導引開＝開檔、導引關＝收尾，中間逐行記錄。

需求來源：操作員「鎖定了但飛機沒動」，而指令歷史放記憶體 server 一重啟就沒了。
覆盤需要的是「這趟到底發了什麼、飛控回了什麼、閘門什麼時候在擋」——
所以每次啟用導引就開一份 JSONL，逐行 flush（當機也不掉已寫的資料）。
"""

import json

import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim",
                           "mission_log_dir": str(tmp_path / "missions")},
                "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "guidance": {"enabled": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return build_sim_engine(cfg, realtime=False)


def crank(engine, seconds=3.0, dt=0.05):
    t = 0.0
    while t < seconds:
        engine.sim_world.step(dt)
        engine.step()
        t += dt


def read_events(path):
    return [json.loads(line) for line in path.read_text(encoding="utf-8").splitlines()]


def test_enabling_guidance_opens_a_mission_file(tmp_path):
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    files = list((tmp_path / "missions").glob("mission_*.jsonl"))
    assert len(files) == 1, "啟用導引沒有建立任務記錄檔"
    events = read_events(files[0])
    assert events[0]["event"] == "guidance_on"
    assert "follow_alt_m" in events[0], "開頭要記下當時的導引設定，覆盤才知道那趟用什麼參數"


def test_commands_and_snapshots_are_recorded(tmp_path):
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    crank(engine, 4.0)                      # 模擬會鎖定並發出指令

    files = list((tmp_path / "missions").glob("mission_*.jsonl"))
    events = read_events(files[0])
    kinds = {e["event"] for e in events}
    assert "command" in kinds, "發出的指令沒有進記錄"
    assert "snap" in kinds, "沒有週期性狀態快照，覆盤看不出閘門何時在擋"

    cmd = next(e for e in events if e["event"] == "command")
    for key in ("lat", "lon", "alt_rel", "n", "e"):
        assert key in cmd

    snap = next(e for e in events if e["event"] == "snap")
    for key in ("state", "mode", "gates", "link_ok"):
        assert key in snap


def test_disabling_guidance_closes_with_a_summary(tmp_path):
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    crank(engine, 2.0)
    engine.set_guidance_enabled(False)

    files = list((tmp_path / "missions").glob("mission_*.jsonl"))
    events = read_events(files[0])
    assert events[-1]["event"] == "guidance_off"
    assert "commands_sent" in events[-1]
    assert engine.status().mission_log is None, "關導引後 UI 不該還顯示記錄中"


def test_each_mission_gets_its_own_file(tmp_path):
    """使用者明確要求：每次任務一份記錄檔。"""
    import time as _t

    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    engine.set_guidance_enabled(False)
    _t.sleep(1.1)                            # 檔名以秒為單位，隔一秒確保不同名
    engine.set_guidance_enabled(True)
    engine.set_guidance_enabled(False)
    files = list((tmp_path / "missions").glob("mission_*.jsonl"))
    assert len(files) == 2, f"兩次任務應有兩份檔案，實得 {len(files)}"


def test_engine_stop_closes_the_log(tmp_path):
    """引擎停止（重啟/關站）也要正常收尾，不能留半開的檔案。"""
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    engine.stop()
    files = list((tmp_path / "missions").glob("mission_*.jsonl"))
    events = read_events(files[0])
    assert events[-1]["event"] == "guidance_off"
    assert events[-1]["reason"] == "引擎停止"


def test_status_exposes_active_mission_file(tmp_path):
    engine = make_engine(tmp_path)
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().mission_log is None
    engine.set_guidance_enabled(True)
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().mission_log, "導引啟用中 UI 要看得到記錄檔名"


def test_repeated_enable_does_not_rotate_the_file(tmp_path):
    """已啟用時再按一次啟用（UI 重送）不該開新檔。"""
    engine = make_engine(tmp_path)
    engine.set_guidance_enabled(True)
    engine.set_guidance_enabled(True)
    files = list((tmp_path / "missions").glob("mission_*.jsonl"))
    assert len(files) == 1
