"""影像斷線時，導引與狀態機必須繼續運作。

實機任務（mission_20260731_190445）踩到的：操作員在 COAST（KF 估計還活著）
時啟用導引，影像恰好在那一刻停格 6.5 秒。導引評估原本只在「收到影像幀」時
執行，於是：

    +0.0s ~ +6.1s   萬事俱備（armed、AUTO.LOITER、link OK、估計存活）
                    但 _run_guidance 一次都沒跑——一筆指令都沒發
                    閘門顯示的還是停格前的過期理由「導引開關未開啟」
    +6.9s           影像恢復，coast 已過期 → LOST → 自動解鎖
    之後 10 分鐘    再也沒鎖回來

影像斷線的瞬間，KF 外推正是系統該拿出來用的東西——那正是 COAST 存在的意義。
"""

from types import SimpleNamespace

import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


class DeadVideo:
    """影像源斷線：永遠沒有新幀。"""

    def get_frame(self):
        return None, None

    def stop(self):
        pass


def make_engine(tmp_path):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"},
                "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "guidance": {"enabled": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return build_sim_engine(cfg, realtime=False)


def crank(engine, seconds, dt=0.05, idle=False):
    t = 0.0
    while t < seconds:
        engine.sim_world.step(dt)
        engine._last_idle_publish = 0.0     # 讓 idle 路徑每步都執行
        engine.step()
        t += dt


def locked_engine(tmp_path):
    """跑到鎖定、估計初始化為止。"""
    engine = make_engine(tmp_path)
    crank(engine, 3.0)
    assert engine.estimator.initialized, "前置失敗：模擬沒有鎖定"
    return engine


def test_gate_report_stays_fresh_without_video(tmp_path):
    """影像斷線時閘門清單要繼續更新，不能停在過期理由。

    實機就是這樣誤導覆盤的：明明導引已開，快照卻連續 13 份寫著
    「導引開關未開啟」——那是停格前的殘影。
    """
    engine = locked_engine(tmp_path)
    engine.video = DeadVideo()
    engine.set_guidance_enabled(True)
    crank(engine, 1.0)
    assert not any("導引開關未開啟" in g for g in engine.gate_report_blocked), (
        f"影像斷線後閘門清單停在過期理由：{engine.gate_report_blocked}"
    )


def test_commands_flow_during_video_dropout(tmp_path):
    """COAST 期間就算沒有影像，也要能用 KF 預測位置發指令。"""
    engine = locked_engine(tmp_path)
    engine.video = DeadVideo()
    engine.set_guidance_enabled(True)
    crank(engine, 2.0)
    assert engine.last_cmd is not None, (
        "影像斷線就完全不發指令——COAST 的存在意義就是這時候頂上"
    )


def test_state_machine_advances_to_lost_without_video(tmp_path):
    """coast 過期的判定不能停擺，否則 UI 永遠顯示「盲區外推」。"""
    engine = locked_engine(tmp_path)
    engine.video = DeadVideo()
    crank(engine, engine.coast_timeout_s + 2.0)
    assert engine.state in ("LOST", "SEARCH"), (
        f"影像斷線後狀態機卡死在 {engine.state}"
    )


def test_video_state_is_recorded_in_mission_snapshots(tmp_path):
    """快照要能分辨「相機沒看到目標」與「影像根本沒進來」。"""
    import json

    engine = make_engine(tmp_path)
    engine.cfg.update({"system": {"mission_log_dir": str(tmp_path / "m")}})
    engine.set_guidance_enabled(True)
    crank(engine, 1.5)
    files = list((tmp_path / "m").glob("*.jsonl"))
    events = [json.loads(l) for l in files[0].read_text(encoding="utf-8").splitlines()]
    snap = next(e for e in events if e["event"] == "snap")
    assert "video" in snap, "快照沒有影像狀態，覆盤分不出兩種完全不同的故障"