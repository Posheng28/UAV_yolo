"""導引高度被安全下限夾掉時，必須講出來。

實際發生：操作員在設定頁把「跟隨高度」填 4m，但 safety.min_cmd_alt_m 是 20m，
clamp_alt 靜默把指令改成 20m。畫面上寫 4、飛機飛 20——夾制本身是對的
（防止自動導引把飛機帶到太低的高度），但不講就是災難。
"""

import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path, **patch):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}, **patch})
    return build_sim_engine(cfg, realtime=False), cfg


def test_no_warning_when_altitude_is_within_limits(tmp_path):
    engine, _ = make_engine(tmp_path, guidance={"multirotor": {"follow_alt_m": 40.0}},
                            safety={"min_cmd_alt_m": 20.0, "max_cmd_alt_m": 120.0})
    assert engine.alt_clamp_warning() is None
    assert engine.status().alt_clamp_note is None


def test_warns_when_requested_altitude_is_below_the_floor(tmp_path):
    engine, _ = make_engine(tmp_path, guidance={"multirotor": {"follow_alt_m": 4.0}},
                            safety={"multirotor": {"min_cmd_alt_m": 20.0},
                                    "max_cmd_alt_m": 120.0})
    note = engine.alt_clamp_warning()
    assert note, "填 4m 卻被夾成 20m，必須警告"
    assert "4" in note and "20" in note, f"警告要講清楚哪個值變成哪個值：{note}"
    assert engine.status().alt_clamp_note == note, "狀態要帶著它，UI 才看得到"


def test_warning_is_visible_before_the_first_frame_is_processed(tmp_path):
    """還沒有影像時就要看得到——那正是操作員在地面調參數的時刻。

    警告來自設定而非量測，若只在 _publish_status 裡塞，影像斷線時它會消失。
    """
    engine, _ = make_engine(tmp_path, guidance={"multirotor": {"follow_alt_m": 4.0}},
                            safety={"multirotor": {"min_cmd_alt_m": 20.0}})
    assert engine.status().alt_clamp_note, "一幀都還沒跑就該看到警告"


def test_warns_when_requested_altitude_is_above_the_ceiling(tmp_path):
    engine, _ = make_engine(tmp_path, guidance={"multirotor": {"follow_alt_m": 200.0}},
                            safety={"min_cmd_alt_m": 20.0, "max_cmd_alt_m": 120.0})
    note = engine.alt_clamp_warning()
    assert note and "120" in note


def test_saving_the_setting_reports_the_clamp_immediately(tmp_path):
    """存檔當下就要講——等飛到天上才發現就太遲了。"""
    engine, cfg = make_engine(tmp_path, guidance={"multirotor": {"follow_alt_m": 40.0}},
                              safety={"multirotor": {"min_cmd_alt_m": 20.0}})
    assert not any("⚠" in a for a in engine.apply_live_config())

    cfg.update({"guidance": {"multirotor": {"follow_alt_m": 4.0}}})
    applied = engine.apply_live_config()
    assert any("⚠" in a and "夾" in a for a in applied), f"存檔沒有回報夾制：{applied}"


def test_fixedwing_altitude_is_checked_too(tmp_path):
    engine, _ = make_engine(tmp_path, vehicle={"airframe": "fixedwing"},
                            guidance={"fixedwing": {"alt_m": 5.0}},
                            safety={"min_cmd_alt_m": 20.0})
    assert engine.alt_clamp_warning(), "固定翼的繞行高度也會被夾，一樣要講"


def test_the_command_actually_uses_the_clamped_altitude(tmp_path):
    """確認警告講的是真的：實際送出的高度就是夾制後的值。"""
    engine, _ = make_engine(tmp_path, guidance={"multirotor": {"follow_alt_m": 4.0}},
                            safety={"multirotor": {"min_cmd_alt_m": 20.0}})
    assert engine.gates.clamp_alt(4.0) == pytest.approx(20.0)


# ---------------- 高度下限依載體分開 ----------------

def test_multirotor_uses_its_own_altitude_floor(tmp_path):
    """旋翼吃 safety.multirotor.min_cmd_alt_m，不是共用值。

    兩種載體共用一個高度下限，實務上一定會演變成「為了旋翼調鬆、
    忘了調回來就飛固定翼」。
    """
    engine, _ = make_engine(tmp_path, vehicle={"airframe": "multirotor"},
                            safety={"min_cmd_alt_m": 20.0,
                                    "multirotor": {"min_cmd_alt_m": 3.0},
                                    "fixedwing": {"min_cmd_alt_m": 20.0}})
    assert engine.gates.min_cmd_alt_m == pytest.approx(3.0)
    assert engine.gates.clamp_alt(4.0) == pytest.approx(4.0), "旋翼 4m 不該被夾"


def test_fixedwing_keeps_the_higher_floor(tmp_path):
    engine, _ = make_engine(tmp_path, vehicle={"airframe": "fixedwing"},
                            safety={"min_cmd_alt_m": 20.0,
                                    "multirotor": {"min_cmd_alt_m": 3.0},
                                    "fixedwing": {"min_cmd_alt_m": 20.0}})
    assert engine.gates.min_cmd_alt_m == pytest.approx(20.0)
    assert engine.gates.clamp_alt(4.0) == pytest.approx(20.0), "固定翼 4m 必須被擋上去"


def test_airframe_subsections_do_not_leak_into_safety_gates(tmp_path):
    """`multirotor`/`fixedwing` 是子區段，不能被當成門檻欄位塞進 SafetyGates。"""
    engine, _ = make_engine(tmp_path, vehicle={"airframe": "multirotor"},
                            safety={"multirotor": {"min_cmd_alt_m": 3.0}})
    assert not hasattr(engine.gates, "multirotor")
    assert not hasattr(engine.gates, "fixedwing")


def test_shared_values_fill_in_what_the_airframe_section_does_not_override(tmp_path):
    """載體子區段只覆蓋它列出的欄位，其餘一律沿用共用值。

    否則加一個載體專屬高度下限，就會把圍欄、逾時等所有共用門檻一起弄丟。
    """
    engine, _ = make_engine(tmp_path, vehicle={"airframe": "multirotor"},
                            safety={"max_cmd_distance_m": 250.0, "max_cmd_alt_m": 90.0,
                                    "link_timeout_s": 3.0,
                                    "multirotor": {"min_cmd_alt_m": 3.0}})
    assert engine.gates.min_cmd_alt_m == pytest.approx(3.0)     # 來自載體區段
    assert engine.gates.max_cmd_alt_m == pytest.approx(90.0)    # 來自共用
    assert engine.gates.max_cmd_distance_m == pytest.approx(250.0)
    assert engine.gates.link_timeout_s == pytest.approx(3.0)


def test_live_config_reapplies_the_airframe_floor(tmp_path):
    engine, cfg = make_engine(tmp_path, vehicle={"airframe": "multirotor"},
                              safety={"multirotor": {"min_cmd_alt_m": 3.0}})
    assert engine.gates.min_cmd_alt_m == pytest.approx(3.0)

    cfg.update({"safety": {"multirotor": {"min_cmd_alt_m": 8.0}}})
    engine.apply_live_config()
    assert engine.gates.min_cmd_alt_m == pytest.approx(8.0), "熱套用沒吃到載體專屬值"
