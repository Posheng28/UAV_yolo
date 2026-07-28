"""偵測器爆炸時，引擎不能靜默假裝「畫面裡沒車」。

最惡劣的失效模式：推論丟例外 → 迴圈執行緒死掉 → /api/status 繼續回上一份
快照 → 儀表板顯示一切正常，實際上早就停止追蹤。換權重、換推論後端
（ONNX/DirectML）、模型檔損壞都會走到這條路徑。
"""

import pytest

from uav_yolo.config import Config
from uav_yolo.simulation import build_sim_engine


def make_engine(tmp_path):
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}, "video": {"width": 640, "height": 360},
                "sim": {"patrol": False},
                "camera": {"intrinsics_file": str(tmp_path / "no_intr.yaml")}})
    return build_sim_engine(cfg, realtime=False)


def test_detector_exception_is_reported_not_swallowed(tmp_path):
    engine = make_engine(tmp_path)
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().detector_error is None

    class Boom:
        def detect(self, frame, t=None):
            raise RuntimeError("shape mismatch: expected [1,3,384,640]")

    engine.detector = Boom()
    engine.sim_world.step(0.05)
    assert engine.step() is True, "偵測失敗不該讓整個 step 停擺"

    err = engine.status().detector_error
    assert err and "shape mismatch" in err, f"偵測例外沒有進到狀態：{err!r}"
    assert engine.status().detections == []


def test_detector_error_clears_once_detection_recovers(tmp_path):
    """錯誤要會消失，否則修好了 UI 還掛著紅字，下次真的壞掉沒人信。"""
    engine = make_engine(tmp_path)

    class Flaky:
        def __init__(self):
            self.fail = True

        def detect(self, frame, t=None):
            if self.fail:
                raise RuntimeError("boom")
            return []

    engine.detector = Flaky()
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().detector_error is not None

    engine.detector.fail = False
    engine.sim_world.step(0.05)
    engine.step()
    assert engine.status().detector_error is None


def test_telemetry_position_still_shown_when_video_is_down(tmp_path):
    """採集卡沒插時，儀表板仍要顯示飛控回報的 GPS 位置。

    實機遇到：影像來源斷線 → 閒置狀態發布把位置寫成 None → 畫面顯示
    「無 GPS 位置」，但飛控其實 20 幾顆星定位良好。這種假象會讓操作員
    往完全錯的方向查（以為 GPS 掛了，實際上只是相機沒插）。
    """
    engine = make_engine(tmp_path)
    engine.sim_world.step(0.05)
    engine.step()                                  # 先跑一幀，讓遙測有位置
    assert engine.status().vehicle["has_fix"] is True

    class DeadVideo:
        def get_frame(self):
            return None, 0.0

        def stop(self):
            pass

    engine.video = DeadVideo()
    engine._last_idle_publish = 0.0
    assert engine.step() is False                  # 沒有新影像
    v = engine.status().vehicle
    assert v["has_fix"] is True, "影像斷線就把 GPS 位置報成沒有，會誤導排查方向"
    assert v["lat"] is not None and v["lon"] is not None


def test_step_exception_does_not_kill_the_loop_thread(tmp_path):
    """_run 必須擋住 step 的例外——執行緒死了 UI 是看不出來的。"""
    engine = make_engine(tmp_path)
    calls = {"n": 0}

    def exploding_step():
        calls["n"] += 1
        raise ValueError("engine blew up")

    engine.step = exploding_step
    engine._stop.clear()

    # 直接跑 _run 幾圈：沒有 try/except 的話第一圈就會把例外拋出來
    import threading

    t = threading.Thread(target=engine._run, daemon=True)
    t.start()
    for _ in range(200):
        if calls["n"] >= 3:
            break
        import time as _t
        _t.sleep(0.01)
    engine._stop.set()
    t.join(timeout=2.0)

    assert calls["n"] >= 3, "迴圈在第一個例外就死了"
    assert engine.loop_error and "engine blew up" in engine.loop_error
