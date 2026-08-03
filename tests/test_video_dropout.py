"""圖傳斷線的成因調查（2026-08-03）所確立的行為。

背景：使用者回報「跑任務的時候圖傳有時候會斷線」。鑑識結論是——
    · 觸發源在機器端（Windows Kernel-PnP 事件證實採集卡 MacroSilicon MS2109
      當天從 USB 匯流排 surprise-removed 1540 次，基線是每天 1~7 次），
    · 但**我們的程式把每一次幾十毫秒的打嗝放大成 3.7~8.8 秒的凍結畫面**：
      舊版只要 cap.read() 失敗一次，就 sleep(0.5) 再把整個 DirectShow graph
      拆掉重建（重建成本實測 0.83~5.9 秒）。
    · 而 RF 失鎖是另一種長相：VRX 仍持續輸出 HDMI，read() 一路成功，
      舊版完全偵測不到（connected 維持 True、fps 正常）。

這些測試釘住三件事：瞬時失敗要能撐過去、真的斷了才重連、畫面停格要看得見。
"""

import time

import numpy as np
import pytest

from uav_yolo.vision.source import (
    FREEZE_ALERT_S,
    READ_FAIL_TOLERANCE,
    VideoSource,
)


class ScriptedCap:
    """依腳本回傳成功/失敗的假擷取裝置。"""

    def __init__(self, script=None, frame=None):
        # script: 依序取用的 bool（True=讀成功）；用完之後一律成功
        self.script = list(script or [])
        self.reads = 0
        self.released = False
        self._frame = frame

    def read(self):
        self.reads += 1
        ok = self.script.pop(0) if self.script else True
        if not ok:
            return False, None
        if self._frame is not None:
            return True, self._frame
        # 每次都給不同內容（且維持明亮），避免被停格/全黑偵測誤判
        return True, np.full((32, 32, 3), 100 + self.reads % 50, np.uint8)

    def release(self):
        self.released = True

    def isOpened(self):
        return True

    def set(self, *a):
        return True

    def get(self, *a):
        return 0


def _run_source(monkeypatch, caps, wait_s=1.0, cfg=None):
    """用一串預先準備好的假裝置跑 VideoSource，回傳 (src, opens)。"""
    src = VideoSource(cfg or {"source": "uvc", "uvc_index": 1})
    opens = {"n": 0}

    def fake_open():
        opens["n"] += 1
        return caps.pop(0) if caps else ScriptedCap()

    monkeypatch.setattr(src, "_open", fake_open)
    src.start()
    time.sleep(wait_s)
    return src, opens


def test_single_failed_read_does_not_tear_down_the_device(monkeypatch):
    """核心修正：一次瞬時讀取失敗不得觸發重連。

    這正是把 USB 打嗝放大成數秒黑畫面的那條路徑。
    """
    cap = ScriptedCap(script=[True, True, False, True, True])
    src, opens = _run_source(monkeypatch, [cap], wait_s=0.6)
    try:
        assert opens["n"] == 1, f"單次讀取失敗竟然重開了裝置（{opens['n']} 次）"
        assert cap.released is False, "裝置被 release＝畫面會黑掉數秒"
        assert src.connected is True, "瞬時失敗期間不該把自己標成未連線"
        assert src.read_fail_total >= 1, "失敗要被計數（沒計數就查不出鏈路多髒）"
    finally:
        src.stop()


def test_hiccup_is_recorded_as_an_event(monkeypatch):
    """撐過去的瞬斷也要留痕，否則修好後就再也不知道鏈路有多不乾淨。"""
    src, _ = _run_source(monkeypatch, [ScriptedCap(script=[True, False, False, True])],
                         wait_s=0.6)
    try:
        kinds = [e["kind"] for e in src.drain_events()]
        assert "video_hiccup" in kinds, f"瞬斷沒有留下事件：{kinds}"
    finally:
        src.stop()


def test_persistent_failure_still_reconnects(monkeypatch):
    """容忍不是縱容：連續失敗超過預算，還是要重連。"""
    dead = ScriptedCap(script=[False] * (READ_FAIL_TOLERANCE + 40))
    src, opens = _run_source(monkeypatch, [dead, ScriptedCap()], wait_s=1.2)
    try:
        assert opens["n"] >= 2, "連續讀取失敗必須升級成重連"
        kinds = [e["kind"] for e in src.drain_events()]
        assert "video_lost" in kinds and "video_restored" in kinds, \
            f"斷線/復原事件不完整：{kinds}"
    finally:
        src.stop()


def test_frozen_picture_is_detected_although_reads_succeed(monkeypatch):
    """RF 失鎖的長相：read() 一路成功，但畫面逐位元不變。

    舊版對這種情況完全沉默——connected=True、fps 正常、看門狗不跳，
    而凍住的畫面會被當成新量測餵進 KF。
    """
    still = np.full((32, 32, 3), 200, np.uint8)
    src, opens = _run_source(monkeypatch, [ScriptedCap(frame=still)],
                             wait_s=FREEZE_ALERT_S + 1.0)
    try:
        assert src.connected is True, "停格不是斷線，不該重連"
        assert opens["n"] == 1
        assert src.frozen is True, "畫面完全不動卻沒被判定停格"
        assert src.error and "停格" in src.error
        assert any(e["kind"] == "video_freeze" for e in src.drain_events())
    finally:
        src.stop()


def test_blank_picture_flags_suspected_link_loss(monkeypatch):
    """幾乎全黑＝典型的「無訊號」畫面，是 RF 失鎖的可觀察簽名。"""
    dark = np.zeros((32, 32, 3), np.uint8)
    src, _ = _run_source(monkeypatch, [ScriptedCap(frame=dark)],
                         wait_s=FREEZE_ALERT_S + 1.0)
    try:
        assert src.blank is True
        assert src.mean_luma is not None and src.mean_luma < 8.0
        kinds = [e["kind"] for e in src.drain_events()]
        assert "video_blank" in kinds, f"全黑畫面沒有留下事件：{kinds}"
    finally:
        src.stop()


def test_live_picture_is_not_flagged(monkeypatch):
    """反向保護：畫面正常變動時不得誤報停格/全黑（誤報會製造警報疲勞）。"""
    src, _ = _run_source(monkeypatch, [ScriptedCap()], wait_s=FREEZE_ALERT_S + 0.8)
    try:
        assert src.frozen is False and src.blank is False
        assert src.error is None
        assert src.frames_total > 0
    finally:
        src.stop()


def test_frozen_picture_must_not_command_the_aircraft(tmp_path):
    """🔴 本專案最重要的一條安全測試。

    畫面停格但擷取仍在吐幀（＝圖傳失鎖、VRX 仍輸出 HDMI 的長相）時，天底鎖定
    的相機讓固定像素永遠對應到「飛機下方固定偏移」的地面點：飛機一動，目標就
    跟著動，形成追不到的胡蘿蔔。修正前實測（模擬）：12 秒內目標估計漂 158m、
    KF 認定靜止的車以 15.2m/s 逃逸、**所有安全閘門全程通過**、又發出 12 筆
    指令把飛機一路帶走。

    修正後要求：停格期間一筆指令都不准發，並且畫面恢復後不必重啟就能續飛。
    """
    from uav_yolo.config import Config
    from uav_yolo.simulation import build_sim_engine

    dt = 0.05
    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({
        "system": {"mode": "sim"},
        "vehicle": {"airframe": "multirotor"},
        "video": {"width": 960, "height": 540},
        "detector": {"lock_mode": "auto", "min_lock_frames": 6},
        "sim": {"patrol": False},
        "camera": {"intrinsics_file": str(tmp_path / "no_intrinsics.yaml")},
    })
    engine = build_sim_engine(cfg, realtime=False)
    world = engine.sim_world
    engine.set_guidance_enabled(True)

    def crank(seconds):
        for _ in range(int(seconds / dt)):
            world.step(dt)
            engine.step()

    crank(10.0)
    assert engine.state == "TRACK" and engine.lock.locked, "基線沒鎖上，測試前提不成立"
    base_cmds = engine.cmd_total
    assert base_cmds > 0, "正常情況下本來就該發得出指令"

    # ---- 凍結：內容固定、時間戳前進、車子還在畫面裡 ----
    video = engine.video
    frozen_frame = video.get_frame()[0]
    frozen_dets = list(video.last_detections)
    assert frozen_dets, "凍結當下畫面裡沒有目標，測試無效"
    holder = {"t": engine.clock()}

    def frozen_get_frame():
        holder["t"] += dt
        return frozen_frame.copy(), holder["t"]

    original_get_frame = video.get_frame
    video.get_frame = frozen_get_frame
    video.last_detections = frozen_dets
    video.frozen = True          # 真實 VideoSource 在畫面 1.5s 沒變化後會立起來

    crank(12.0)
    assert engine.cmd_total == base_cmds, (
        f"停格期間竟然還發了 {engine.cmd_total - base_cmds} 筆指令——"
        "這會把飛機帶著幻影目標一路飛走")
    assert any("停格" in g for g in engine.gate_report_blocked), \
        f"閘門沒有講出停格這個原因：{engine.gate_report_blocked}"

    # ---- 恢復：不必重啟引擎就能續飛 ----
    video.get_frame = original_get_frame
    video.frozen = False
    crank(10.0)
    assert engine.state == "TRACK" and engine.lock.locked, "畫面恢復後沒有自己回到追蹤"
    assert engine.cmd_total > base_cmds, "畫面恢復後指令沒有恢復發送"


def test_video_test_endpoint_does_not_open_a_second_handle(monkeypatch, tmp_path):
    """/api/video/test 不得對引擎正在用的 DirectShow 裝置開第二個 handle。

    實測 3/3 都造成傷害（畫面凍 6.2 秒、cv2.error、一次 0xC0000005 打死行程），
    而這正是操作員發現圖傳怪怪時第一個會按的按鈕。
    """
    from fastapi.testclient import TestClient

    from uav_yolo.config import Config
    from uav_yolo.vision import source as source_mod
    from uav_yolo.webapp.server import create_app

    called = {"probe": 0}
    monkeypatch.setattr(source_mod, "probe_source",
                        lambda *a, **k: called.__setitem__("probe", called["probe"] + 1) or {"ok": True})

    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.override({"system": {"mode": "sim"}})
    app = create_app(cfg)
    with TestClient(app) as client:
        engine = app.state.manager.engine
        # 模擬引擎正握著一台 uvc 裝置
        engine.video.cfg = {"source": "uvc", "uvc_name_hint": "USB Video", "uvc_index": 1}
        engine.video.mode = "uvc"
        engine.video.connected = True
        engine.video.device_label = "USB Video"
        r = client.post("/api/video/test",
                        json={"source": "uvc", "uvc_name_hint": "USB Video", "uvc_index": 1})
        assert r.status_code == 200
        body = r.json()
        assert called["probe"] == 0, "同一台獨占裝置竟然又被開了一次"
        assert body["ok"] is True and "不重複開啟" in (body.get("note") or "")

        # 不同裝置就該照常實際測試
        r2 = client.post("/api/video/test",
                         json={"source": "uvc", "uvc_name_hint": "OtherCard", "uvc_index": 3})
        assert r2.status_code == 200
        assert called["probe"] == 1, "不同裝置應該走真正的 probe_source"
