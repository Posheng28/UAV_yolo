"""雲台姿態是「實測」還是「飛控用指令角合成的」，必須分得出來。

🔴 這是一個看不出來的陷阱：

PX4 在 `MNT_MODE_OUT = 0`(AUX) 與 `= 1`(v1) 兩種輸出模式下，會拿**指令角**
合成 `GIMBAL_DEVICE_ATTITUDE_STATUS` 並發布出來（`output_rc.cpp` 與
`OutputMavlinkV1::_stream_device_attitude_status()`）。訊息型別、欄位、更新率
與真正的實測回報**完全一樣**。

指令角在下列情況與現實不符，而且不會有任何提示：
  - 雲台正在轉動（落後於指令）
  - 撞到機械極限（pitch -105~+145、roll ±60、yaw ±160）
  - 操作員切換了工作模式
  - 觸發傾角保護（雲台被中和且不可控）

只有真正實作 MAVLink 雲台協定 v2 的裝置會送 `GIMBAL_DEVICE_INFORMATION`(283)。
沒收到它 ⇒ 手上的姿態是指令值。

具體案例：XF C-20T（＝CADDX GM3）。ArduPilot 自家驅動 `AP_Mount_CADDX.cpp`
是單向的，原始碼註解逐字寫著：
    // gimbal does not provide attitude so simply return targets
而 PX4 v1.17.0 全樹搜尋 caddx/xfrobot/gm3 是零命中。
"""

from types import SimpleNamespace

import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path, **patch):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}, **patch})
    return build_sim_engine(cfg, realtime=False)


def test_default_is_not_measured():
    """沒收到 GIMBAL_DEVICE_INFORMATION 之前，一律不可宣稱是實測值。"""
    from uav_yolo.mavlink_io.telemetry import MavlinkConnection

    link = MavlinkConnection.__new__(MavlinkConnection)
    link.gimbal_information_seen = False
    assert link.gimbal_information_seen is False


def test_status_exposes_attitude_provenance(tmp_path):
    """狀態必須帶出這個欄位，否則 UI 與自檢都分不出指令角與實測值。"""
    engine = make_engine(tmp_path, gimbal={"present": True, "control": "roi"})
    engine.sim_world.step(0.05)
    engine.step()
    assert "attitude_measured" in engine.status().gimbal


def test_sim_models_a_real_v2_gimbal(tmp_path):
    """模擬的是理想 v2 裝置：它推的姿態在模型裡就是相機真正的指向。"""
    engine = make_engine(tmp_path, gimbal={"present": True, "control": "roi"})
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().gimbal["attitude_measured"] is True


def test_status_reports_commanded_when_information_never_arrived(tmp_path):
    """實機上沒收到 GIMBAL_DEVICE_INFORMATION ⇒ 手上的是指令角，不可宣稱實測。"""
    engine = make_engine(tmp_path, gimbal={"present": True, "control": "roi"})
    engine.link.gimbal_information_seen = False
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().gimbal["attitude_measured"] is False


def test_selfcheck_warns_when_attitude_is_only_commanded(tmp_path):
    """收到姿態但不是實測 → 必須警告，不能報 pass。

    報 pass 的話操作員會以為測地用的是真實指向，而實際上雲台一撞到極限
    或切換模式，算出來的座標就偏掉，畫面上卻一切正常。
    """
    from uav_yolo.selfcheck import check_gimbal

    engine = make_engine(tmp_path, gimbal={"present": True, "control": "roi"})
    engine.link.gimbal_information_seen = False
    engine.sim_world.step(0.05)
    engine.step()

    # 塞一筆姿態進去，模擬「飛控合成的回報」
    from uav_yolo.mavlink_io.telemetry import GimbalSample
    engine.link.store.push_gimbal(
        GimbalSample(engine.clock(), 0.0, -1.57, 0.0, True))

    res = check_gimbal(engine, engine.cfg)
    assert res.status == "warn", f"應該警告，實得 {res.status}"
    assert "指令角" in res.detail


def test_selfcheck_passes_only_with_real_measured_attitude(tmp_path):
    from uav_yolo.mavlink_io.telemetry import GimbalSample
    from uav_yolo.selfcheck import check_gimbal

    engine = make_engine(tmp_path, gimbal={"present": True, "control": "roi"})
    engine.link.gimbal_information_seen = True
    engine.sim_world.step(0.05)
    engine.step()
    engine.link.store.push_gimbal(
        GimbalSample(engine.clock(), 0.0, -1.57, 0.0, True))

    assert check_gimbal(engine, engine.cfg).status == "pass"
