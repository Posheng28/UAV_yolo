"""主引擎：影像 → 偵測鎖定 → 測地 → KF → 導引 → MAVLink，全鏈路狀態機。

狀態機：
    SEARCH  未鎖定目標（等 auto 連續幀達標或 UI 點選）
    TRACK   鎖定且本幀有量測
    COAST   鎖定但暫時看不到（定翼繞行盲區/遮蔽）→ KF 速度外推撐住
    LOST    coast 逾時 → 解鎖回 SEARCH，保留最後位置供顯示

安全原則：導引指令預設不發（guidance_enabled=False），
UI 明確開啟後仍逐項過 SafetyGates 才會出手。
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass, field

import cv2
import numpy as np

from .config import (
    DEFAULT_INTRINSICS_FILE,
    DEFAULT_WEIGHTS_FILE,
    PROJECT_ROOT,
    Config,
)
from .estimation import TargetEstimator
from .geometry import (
    CameraModel,
    GeoRef,
    camera_rotation_body_mount,
    camera_rotation_gimbal_earth,
    geolocate_pixel,
    euler_zyx_to_R,
)
from .guidance import build_guidance
from .safety import SafetyGates
from .vision.detector import Detection


@dataclass
class LastCommand:
    t: float
    lat: float
    lon: float
    alt_rel: float
    point_ne: np.ndarray
    label: str
    radius: float | None = None


def _tile_grid(det_cfg: dict) -> tuple[int, int] | None:
    """切塊格數；0 或未設 = auto（由模型輸入尺寸推算，見 tiling.auto_grid）。

    寫死格數很危險：塊若比模型輸入大就會被縮小，切了反而更糟
    （實測 3x2 在 960x544 輸入下偵測率 10%，改 auto 推出的 3x3 是 80%）。
    """
    cols = int(det_cfg.get("tile_cols", 0) or 0)
    rows = int(det_cfg.get("tile_rows", 0) or 0)
    return (cols, rows) if cols > 0 and rows > 0 else None


@dataclass
class EngineStatus:
    """給 Web UI 的快照（engine.status() 序列化）。"""

    state: str = "SEARCH"
    sim: bool = False
    airframe: str = "multirotor"
    video: dict = field(default_factory=dict)
    vehicle: dict = field(default_factory=dict)
    target: dict = field(default_factory=dict)
    gates: list = field(default_factory=list)
    detections: list = field(default_factory=list)
    guidance_enabled: bool = False
    guidance_note: str = ""
    last_command: dict | None = None
    camera: dict = field(default_factory=dict)
    mavlink: dict = field(default_factory=dict)
    gimbal: dict = field(default_factory=dict)
    loop_hz: float = 0.0
    detector_error: str | None = None  # 推論丟例外＝畫面看似沒車，必須讓操作員看到
    loop_error: str | None = None      # 迴圈本體例外（比偵測失敗更嚴重）
    alt_clamp_note: str | None = None  # 設定的導引高度被安全上下限夾掉時的說明
    commands: list = field(default_factory=list)  # 最近發出的導引指令（新→舊，含 ACK）
    mission_log: str | None = None     # 任務記錄檔名（導引啟用中才有）
    latched: bool = False              # 飛行員接管閂鎖：UI 要顯示專用的恢復按鈕
    seq: int = 0                       # 發布序號：前端偵測「引擎凍結但 HTTP 還活著」用


class TrackerEngine:
    def __init__(self, cfg: Config, *, video, detector, link, clock=time.monotonic):
        """video/detector/link 可注入（live 或 sim），clock 供模擬時間。"""
        self.cfg = cfg
        self.video = video
        self.detector = detector
        self.link = link
        self.clock = clock

        self.airframe = cfg.get("vehicle.airframe", "multirotor")
        self.sim_mode = cfg.get("system.mode") == "sim"

        firmware = cfg.get("vehicle.firmware", "px4")
        if firmware != "px4":
            # 別讓設定看起來像個支援的開關卻靜默無效：模式表/AUTO.LOITER 白名單/
            # DO_REPOSITION 語意全是 PX4 專屬的。
            raise ValueError(
                f"vehicle.firmware={firmware!r} 不支援；本系統目前僅支援 px4"
            )

        vid = cfg.section("video")
        self.camera_model = CameraModel.load(
            PROJECT_ROOT / (cfg.get("camera.intrinsics_file") or DEFAULT_INTRINSICS_FILE),
            cfg.get("camera.fallback_hfov_deg", 120.0),
            int(vid.get("width", 1280)),
            int(vid.get("height", 720)),
        )

        est_cfg = cfg.section("estimator")
        self.estimator = TargetEstimator(
            accel_std=est_cfg.get("accel_std", 3.0),
            meas_std=est_cfg.get("meas_std", 8.0),
            gate_sigma=est_cfg.get("gate_sigma", 4.0),
            max_jump_m=cfg.get("safety.max_meas_jump_m", 30.0),
        )
        self.coast_timeout_s = float(est_cfg.get("coast_timeout_s", 8.0))

        self.guidance = build_guidance(self.airframe, cfg.section("guidance"))
        self.gates = SafetyGates(self._safety_cfg(), cfg.get("guidance.rate_hz", 1.0))
        self.guidance_enabled = bool(cfg.get("guidance.enabled", False))
        deadband_key = "fixedwing" if self.airframe == "fixedwing" else "multirotor"
        self.reposition_deadband_m = float(
            cfg.get(f"guidance.{deadband_key}.reposition_deadband_m", 3.0)
        )
        self.cmd_refresh_s = float(cfg.get("guidance.cmd_refresh_s", 20.0))

        from .vision.detector import TargetLock

        det_cfg = cfg.section("detector")
        self.lock = TargetLock(
            mode=det_cfg.get("lock_mode", "auto"),
            min_lock_frames=int(det_cfg.get("min_lock_frames", 6)),
        )

        gim = cfg.section("gimbal")
        self.gimbal_present = bool(gim.get("present", True))
        self.gimbal_control = gim.get("control", "roi") if self.gimbal_present else "none"
        self.gimbal_stabilized = bool(gim.get("stabilized", True))
        self.gimbal_attitude_source = gim.get("attitude_source", "auto")
        mount = gim.get("mount_deg", {})
        self.mount_rad = (
            math.radians(mount.get("yaw", 0.0)),
            math.radians(mount.get("pitch", -90.0)),
            math.radians(mount.get("roll", 0.0)),
        )
        self.roi_update_min_m = float(gim.get("roi_update_min_m", 2.0))

        self.state = "SEARCH"
        self.georef: GeoRef | None = None
        self.home_alt_amsl: float | None = None
        self.last_cmd: LastCommand | None = None
        self.last_roi_ne: np.ndarray | None = None
        self.last_roi_t: float | None = None
        self.last_gimbal_cmd: tuple[float, float] | None = None  # (pitch_rad, yaw_rad)
        self.last_meas_note = ""
        # 從空集合起始（而非 None）：引擎還沒處理過任何幀時，任何手動鎖定
        # 都該被拒絕——否則驗證被 getattr 預設值跳過，亂點 ID 也回 ok。
        self._last_track_ids: set[int] = set()
        self.detector_error: str | None = None
        self.loop_error: str | None = None
        self.cmd_history: deque = deque(maxlen=15)   # 最近發出的導引指令（UI 顯示）
        self.cmd_total = 0                            # 本任務累計（deque 會截頂，不能拿 len 當計數）
        self._mission_lock = threading.Lock()         # 開/關/寫跨 HTTP 與迴圈執行緒
        self._video_events: deque = deque(maxlen=50)  # 最近的影像事件（UI 顯示）
        # 任務錄影（導引開＝開錄、導引關＝收檔）
        self._mission_video = None
        self._mission_video_wh = (0, 0)
        self._mission_video_size: tuple[int, int] | None = None   # 實際來源尺寸
        self._mission_video_path = None
        self._mission_video_next_t = 0.0
        self._mission_video_dt = 1.0 / 15.0
        self.last_known_lla: tuple[float, float] | None = None
        self.gate_report_blocked: list[str] = []
        self.guidance_note = ""
        self.vehicle_path: deque[tuple[float, float]] = deque(maxlen=600)
        self.target_path: deque[tuple[float, float]] = deque(maxlen=600)

        self._jpeg: bytes | None = None
        self._raw_frame = None
        self._jpeg_lock = threading.Lock()
        self._status_lock = threading.Lock()
        self._status = EngineStatus()
        self._last_frame_t: float | None = None
        self._last_capture_t: float | None = None
        self.video_latency_s = float(cfg.get("video.latency_ms", 0.0)) / 1000.0
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._loop_times: deque[float] = deque(maxlen=30)

    # ------------------------------------------------ 生命週期

    def start(self) -> None:
        self.video.start()
        if hasattr(self.link, "start"):
            self.link.start()
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, daemon=True, name="engine")
        self._thread.start()

    def stop(self) -> None:
        # 先停迴圈執行緒、再收任務記錄：反過來的話 join 前迴圈可能還在跑
        # 半輪，對已關閉的檔繼續寫 snap/command——那些事件全數落空。
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        self._mission_close("引擎停止")
        self.video.stop()
        if hasattr(self.link, "stop"):
            self.link.stop()

    def _run(self) -> None:
        while not self._stop.is_set():
            try:
                progressed = self.step()
            except Exception as exc:
                # 迴圈執行緒死掉是最惡劣的失效模式：/api/status 仍回上一份快照，
                # 儀表板顯示一切正常，實際上早就停止追蹤了。寧可記錄並續跑。
                self.loop_error = f"{type(exc).__name__}: {exc}"
                progressed = False
            time.sleep(0.005 if progressed else 0.02)

    # ------------------------------------------------ UI 操作

    def apply_live_config(self) -> list[str]:
        """把「不需重建硬體連線」的設定熱套用到執行中的引擎，回傳已套用的項目。

        沒有這個的話，操作員在設定頁把圍欄改嚴、按儲存，UI 沒標 ⟳ 讓他以為
        生效了，實際上引擎仍抱著建構當下的舊門檻——這是會出事的落差。
        影像來源/載體/雲台接線/MAVLink 埠等仍需重啟引擎（UI 標 ⟳）。
        """
        cfg = self.cfg
        applied: list[str] = []

        # 安全門檻：就地更新，保留接管閂鎖
        self.gates.update_limits(self._safety_cfg(), cfg.get("guidance.rate_hz", 1.0))
        applied.append("safety")

        # 導引參數（載體種類仍需重啟，這裡只換同載體的數值）。
        # 運行狀態要搬過去：standoff 的方位閂鎖（_last_bearing）若被重設回
        # 預設值（正南），存一個無關設定就會讓靜止目標的跟隨點瞬間跳 ~21m，
        # 飛機無預警繞目標重新定位。gates 用 in-place update 就是同一個理由。
        old_guidance = self.guidance
        self.guidance = build_guidance(self.airframe, cfg.section("guidance"))
        if hasattr(old_guidance, "_last_bearing") and hasattr(self.guidance, "_last_bearing"):
            self.guidance._last_bearing = old_guidance._last_bearing
        deadband_key = "fixedwing" if self.airframe == "fixedwing" else "multirotor"
        self.reposition_deadband_m = float(
            cfg.get(f"guidance.{deadband_key}.reposition_deadband_m", 3.0)
        )
        self.cmd_refresh_s = float(cfg.get("guidance.cmd_refresh_s", 20.0))
        applied.append("guidance")
        # 存檔當下就要講：填 4m 卻被夾成 20m，等飛上去才發現就太遲了
        alt_warn = self.alt_clamp_warning()
        if alt_warn:
            applied.append(f"⚠ {alt_warn}")

        # 偵測器閾值與鎖定行為
        det_cfg = cfg.section("detector")
        if hasattr(self.detector, "conf"):
            self.detector.conf = float(det_cfg.get("conf", 0.55))
        # imgsz 只是 predict 的參數、不必重載模型，但漏掉它會讓設定頁改了沒反應
        # （UI 又沒標 ⟳）＝靜默失效。而它直接決定迴圈速率：CPU 推論下
        # 1280 需 304ms/幀（3.3Hz）、640 只要 167ms（6Hz）。
        if hasattr(self.detector, "tiling"):
            self.detector.tiling = det_cfg.get("tiling", "off")
            self.detector.tile_grid = _tile_grid(det_cfg)
            self.detector.tile_overlap = float(det_cfg.get("tile_overlap", 0.2))
        if hasattr(self.detector, "set_imgsz"):
            # ONNX 是靜態尺寸，套不上時要把原因回報到 UI，不能默默忽略
            reason = self.detector.set_imgsz(int(det_cfg.get("imgsz", 640)))
            if reason:
                applied.append(f"（imgsz 未套用：{reason}）")
        elif hasattr(self.detector, "imgsz"):
            self.detector.imgsz = int(det_cfg.get("imgsz", 640))
        self.lock.mode = det_cfg.get("lock_mode", "auto")
        self.lock.min_lock_frames = max(1, int(det_cfg.get("min_lock_frames", 6)))
        applied.append("detector")

        # 估計器與影像延遲補償
        est_cfg = cfg.section("estimator")
        self.estimator.accel_std = float(est_cfg.get("accel_std", 3.0))
        self.estimator.meas_std = float(est_cfg.get("meas_std", 8.0))
        self.estimator.gate_sigma = float(est_cfg.get("gate_sigma", 4.0))
        self.estimator.max_jump_m = float(cfg.get("safety.max_meas_jump_m", 30.0))
        self.coast_timeout_s = float(est_cfg.get("coast_timeout_s", 8.0))
        self.video_latency_s = float(cfg.get("video.latency_ms", 0.0)) / 1000.0
        applied.append("estimator/latency")

        return applied

    def set_guidance_enabled(self, enabled: bool) -> None:
        was = self.guidance_enabled
        self.guidance_enabled = bool(enabled)
        if enabled:
            self.gates.reset_override()  # 重新啟用 = 飛行員把控制權交回來
            # 清掉上一輪的指令記錄：否則 _run_guidance 的 deadband 會拿「接管前」
            # 的舊指令點來比，目標若沒怎麼移動就整個不發——閘門全綠但飛機不動，
            # 而飛行員接管期間可能已經把機體飛到別處了。
            self.last_cmd = None
            if not was:
                self._mission_open()
        elif was:
            self._mission_close("操作員關閉導引")

    # ---------------- 任務記錄（導引開＝開檔、導引關＝收尾） ----------------
    #
    # 指令歷史放記憶體的話 server 一重啟就沒了；而「這趟到底發了什麼、
    # 飛控回了什麼、閘門什麼時候擋」正是飛完要覆盤的東西。每次啟用導引
    # 就開一份 JSONL，逐行 flush（當機也不掉資料），關導引寫入摘要收尾。

    def _mission_dir(self) -> "Path":
        from pathlib import Path

        d = Path(self.cfg.get("system.mission_log_dir", "data/missions"))
        d.mkdir(parents=True, exist_ok=True)
        return d

    # 開/關/寫都可能同時來自 HTTP 執行緒（導引開關）與迴圈執行緒（snap/
    # command）：不加鎖的話「關檔」與「寫入」交錯會對已關閉的檔案物件寫入，
    # 或半行 JSON 撕裂。鎖只包 I/O，粒度極小，不影響迴圈速率。

    def _mission_video_open(self, stem: str) -> None:
        """任務錄影：把「操作員看到的疊圖畫面」逐幀寫成影片。

        為什麼錄疊圖而不是原始畫面：覆盤時最想知道的是「當下到底鎖到了什麼」，
        而框線、鎖定標記、狀態文字全在疊圖上。JSONL 記得到座標與閘門，
        但記不到「那個框是車還是草」——2026-08-04 實飛就卡在這一點。
        """
        if not bool(self.cfg.get("system.mission_video", True)):
            return
        try:
            scale = float(self.cfg.get("system.mission_video_scale", 0.5))
            fps = float(self.cfg.get("system.mission_video_fps", 15.0))
            w, h = self._mission_video_size or (1280, 720)
            w, h = max(2, int(w * scale) // 2 * 2), max(2, int(h * scale) // 2 * 2)
            path = self._mission_dir() / f"{stem}.mp4"
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), fps, (w, h))
            if not writer.isOpened():
                writer.release()
                self.loop_error = f"任務錄影開檔失敗（codec 不支援？）：{path.name}"
                return
            self._mission_video = writer
            self._mission_video_wh = (w, h)
            self._mission_video_path = path
            self._mission_video_next_t = 0.0
            self._mission_video_dt = 1.0 / max(fps, 1.0)
        except Exception as exc:
            self._mission_video = None
            self.loop_error = f"任務錄影開檔失敗：{exc}"

    def _mission_video_write(self, frame, now: float) -> None:
        w = self._mission_video
        if w is None or frame is None:
            return
        if now < self._mission_video_next_t:
            return                      # 依設定的 fps 抽幀，別把磁碟寫爆
        self._mission_video_next_t = now + self._mission_video_dt
        try:
            tw, th = self._mission_video_wh
            if (frame.shape[1], frame.shape[0]) != (tw, th):
                frame = cv2.resize(frame, (tw, th), interpolation=cv2.INTER_AREA)
            w.write(frame)
        except Exception:
            pass                        # 錄影失敗不能擋任務

    def _mission_video_close(self) -> None:
        w, self._mission_video = self._mission_video, None
        if w is not None:
            try:
                w.release()
            except Exception:
                pass

    def _mission_open(self) -> None:
        try:
            with self._mission_lock:
                name = time.strftime("mission_%Y%m%d_%H%M%S.jsonl")
                self._mission_path = self._mission_dir() / name
                self._mission_fh = open(self._mission_path, "a", encoding="utf-8")
                self._mission_snap_t = 0.0
                self.cmd_total = 0
            self._mission_video_open(self._mission_path.stem)
            self._mission_write("guidance_on", {
                "airframe": self.airframe,
                "follow_alt_m": getattr(self.guidance, "follow_alt_m",
                                        getattr(self.guidance, "alt_m", None)),
                "standoff_m": getattr(self.guidance, "standoff_m", None),
                "max_speed_ms": getattr(self.guidance, "max_speed_ms", None),
                "weights": getattr(self.detector, "weights_path", None),
                "tiling": getattr(self.detector, "tiling", None),
            })
        except Exception as exc:      # 記錄失敗不能擋任務，但要看得到
            self.loop_error = f"任務記錄開檔失敗：{exc}"
            with self._mission_lock:
                self._mission_fh = None
                self._mission_path = None

    def _mission_close(self, reason: str) -> None:
        if getattr(self, "_mission_fh", None) is None:
            return
        self._mission_write("guidance_off", {
            "reason": reason,
            # cmd_history 是 maxlen=15 的 deque，len() 封頂後永遠 15；
            # 覆盤要的是本任務真正發了幾筆
            "commands_sent": self.cmd_total,
            "pilot_override_latched": self.gates.pilot_override_latched,
        })
        self._mission_video_close()
        with self._mission_lock:
            try:
                if self._mission_fh is not None:
                    self._mission_fh.close()
            except Exception:
                pass
            self._mission_fh = None
            self._mission_path = None

    def _mission_write(self, event: str, payload: dict) -> None:
        import json

        try:
            rec = {"event": event, "t": round(self.clock(), 3),
                   "wall": time.strftime("%H:%M:%S"), **payload}
            line = json.dumps(rec, ensure_ascii=False) + "\n"
            with self._mission_lock:
                if self._mission_fh is None:
                    return
                self._mission_fh.write(line)
                self._mission_fh.flush()   # 逐行落盤：當機/斷電也不掉已寫的
        except Exception:
            pass

    def manual_lock(self, track_id: int) -> str | None:
        """UI 點選鎖定。成功回 None，拒絕回原因字串（HTTP 409 顯示給操作員）。"""
        track_id = int(track_id)
        # 點選的 ID 必須在「最近一幀」的偵測裡：畫面重繪/延遲會讓操作員點到
        # 已消失的框，默默收下會白等 3 秒 pending 過期，UI 看起來像沒反應。
        if track_id not in self._last_track_ids and track_id != self.lock.locked_id:
            return f"目標 #{track_id} 已不在畫面中，請重新點選"
        # 換到不同目標時，估計器必須跟著重置——否則 KF 還抱著舊車的軌跡，
        # 新車的每一筆量測都被 30m 跳變閘擋掉：UI 顯示已鎖新車，導引卻繼續
        # 朝舊車外推位置發指令最長 8 秒，然後 LOST 默默撤銷操作員的選擇。
        if self.lock.locked_id is not None and track_id != self.lock.locked_id:
            self.estimator.reset()
            self.last_cmd = None
        self.lock.request_manual_lock(track_id)
        return None

    def unlock(self) -> None:
        self.lock.unlock()
        self.estimator.reset()
        self.state = "SEARCH"
        self.last_cmd = None  # 換目標後第一筆指令不該被舊 deadband 吃掉

    # ------------------------------------------------ 主循環單步

    def step(self) -> bool:
        frame, frame_t = self.video.get_frame()
        if frame is None or frame_t == self._last_frame_t:
            # 沒有新影像時仍要發布狀態（限流 2Hz）：否則影像一斷，儀表板的
            # 數傳/GPS/Home 全部停在舊值或空白，看起來像「什麼都斷了」，
            # 會把排查方向整個帶偏——實際上遙測可能好好的。
            now_wall = time.monotonic()
            if now_wall - getattr(self, "_last_idle_publish", 0.0) >= 0.5:
                self._last_idle_publish = now_wall
                try:
                    # 位置要照樣從遙測取，不能傳 None：否則採集卡沒插時儀表板會顯示
                    # 「無 GPS 位置」，但飛控其實定位得好好的——那正是這段程式要
                    # 避免的誤導，只是先前漏了位置這一項。
                    clock_now = self.clock()
                    # 🔴 導引與狀態機不能掛在「有沒有影像幀」上。實機任務踩到：
                    # 操作員在 COAST（估計還活著）時啟用導引，影像恰好在那一刻
                    # 停格 6.5 秒——導引評估只在收到幀時執行，於是唯一可發射的
                    # 窗口整個被吃掉，coast 過期自動解鎖，事後閘門顯示的還是
                    # 停格前的過期理由。影像斷線時：KF 外推正是該頂上的東西，
                    # 狀態機要照走（COAST→LOST 判定）、導引要照發（用預測位置）。
                    #
                    # ⚠ 時間軸：幀路徑跑在「擷取時間軸」（capture_t = 到達 − 延遲），
                    # idle 若用牆鐘會把 KF 推到量測的未來——影像恢復後的頭
                    # ~latency 秒，每筆量測 predict_to 的 dt≤0 靜默 no-op、
                    # 以錯誤時刻融合，把延遲補償要消除的滯後又加回來。
                    idle_t = clock_now - self.video_latency_s
                    # home 高度與 NED 基準的刷新不能只在幀路徑做：影像斷線期間
                    # PX4 可能重設 home（重新解鎖），指令高度會帶著舊基準偏置
                    store_idle = self.link.store
                    if store_idle.home is not None:
                        self.home_alt_amsl = store_idle.home.alt_amsl
                    self.estimator.predict_to(idle_t)
                    self._update_state_machine(idle_t, False)
                    self._run_guidance(clock_now)
                    self._publish_status(
                        clock_now, [], self.link.store.position_at(clock_now))
                    # 這條分支也要自清 loop_error。之前只在有幀的路徑清，
                    # 而採集卡拔掉時**只有**這條在跑——一次瞬態例外就會永久
                    # 掛著紅字，把底下真正的影像/數傳錯誤全遮住。
                    self.loop_error = None
                except Exception as exc:
                    self.loop_error = f"{type(exc).__name__}: {exc}"
            return False
        self._last_frame_t = frame_t
        now = self.clock()

        # 採集卡/圖傳不保證會照設定的解析度輸出（例：要求 720p 但 VRX 給 1080p）。
        # 內參必須對齊「實際幀」尺寸，否則焦距差幾倍、測地整個歪掉。
        fh, fw = frame.shape[:2]
        if (fw, fh) != (self.camera_model.width, self.camera_model.height):
            self.camera_model = self.camera_model.scaled_to(fw, fh)

        store = self.link.store
        if self.georef is None and store.home is not None:
            self.georef = GeoRef(store.home.lat, store.home.lon)  # NED 原點鎖第一筆，保持連續
        if store.home is not None:
            self.home_alt_amsl = store.home.alt_amsl  # 高度基準用最新（PX4 解鎖時會重設 home）

        try:
            detections = self.detector.detect(frame, frame_t)
            self.detector_error = None
        except Exception as exc:
            # 換權重/換推論後端最容易在這裡炸（形狀不符、模型檔壞、驅動問題）。
            # 靜默回空清單會讓操作員以為「畫面裡沒車」，必須讓 UI 看得到。
            self.detector_error = f"{type(exc).__name__}: {exc}"
            detections = []
        # 給 manual_lock 驗證「點選的 ID 目前真的在畫面上」用
        self._last_track_ids = {d.track_id for d in detections}
        locked_det = self.lock.update(detections)

        # 影像鏈路有固定延遲（RTSP/數位圖傳可達數百 ms）：這一幀「拍的是」
        # capture_t 當下的世界，而不是它送達的時刻。整條感知鏈統一用 capture_t，
        # 才會拿到當時的姿態/位置。20 m/s 下 300ms 未補償 = 6m 測地誤差。
        capture_t = frame_t - self.video_latency_s
        self._last_capture_t = capture_t

        pos = store.position_at(capture_t)
        att = store.attitude_at(capture_t)

        self.estimator.predict_to(capture_t)

        # 🔴 畫面停格時絕對不能餵量測。實測（模擬，2026-08-03）：畫面凍住但
        # 擷取仍在吐幀（＝VRX 失鎖仍輸出 HDMI 的長相）時，天底鎖定的相機讓
        # 固定像素永遠對應到「飛機下方固定偏移」的地面點——飛機朝它飛，點就
        # 跟著跑。12 秒內目標估計漂 158m、KF 認為靜止的車以 15.2m/s 在跑、
        # 期間所有安全閘門全數通過、又發出 12 筆指令。追不到的胡蘿蔔會把
        # 飛機一路帶到圍欄邊。停格 → 當作本幀沒有量測，自然走 COAST→LOST。
        frozen = bool(getattr(self.video, "frozen", False))
        measured = False
        if frozen:
            self.last_meas_note = "影像停格，本幀量測作廢"
        elif locked_det is not None and pos is not None and self.georef is not None:
            hit_ne = self._geolocate(locked_det, pos, att, capture_t)
            if hit_ne is not None:
                ok, note = self.estimator.update(capture_t, hit_ne)
                self.last_meas_note = note
                measured = ok

        # 未鎖定但 KF 還活著 → 世界座標重鎖定（丟失後目標重新出現）。
        # 停格畫面同樣不能用來重鎖：那會把飛機自己的移動當成目標的移動。
        if (
            not frozen
            and locked_det is None
            and self.estimator.initialized
            and pos is not None
            and self.georef is not None
            and detections
        ):
            self._try_reacquire(detections, pos, att, capture_t)

        self._update_state_machine(capture_t, measured)

        if pos is not None:
            ned = self.georef.lla_to_ned(pos.lat, pos.lon, pos.rel_alt) if self.georef else None
            if ned is not None:
                self.vehicle_path.append((float(ned[0]), float(ned[1])))
        if self.estimator.initialized:
            p = self.estimator.pos_ne
            self.target_path.append((float(p[0]), float(p[1])))
            if self.georef is not None:
                lat, lon, _ = self.georef.ned_to_lla(np.array([p[0], p[1], 0.0]))
                self.last_known_lla = (lat, lon)

        self._run_guidance(now)
        self._run_gimbal(now, pos)

        with self._jpeg_lock:
            self._raw_frame = frame.copy()  # 校正頁要用未疊圖的原始幀
        self._mission_video_size = (frame.shape[1], frame.shape[0])
        self._render_overlay(frame, detections, locked_det, pos)
        self._mission_video_write(frame, now)   # 疊圖後才寫，覆盤看得到鎖到什麼
        self._publish_status(now, detections, pos)
        self._loop_times.append(now)
        # 成功走完一輪就清 loop_error（對照 detector_error 的做法）：
        # 否則一次瞬態例外＝永久紅色橫幅，之後真正的 video/mavlink 錯誤
        # 全被它遮蔽（app.js 的錯誤鏈把 loop_error 排第一）——警報疲勞。
        self.loop_error = None
        return True

    # ------------------------------------------------ 測地

    def _camera_rotation(self, att, frame_t):
        """依設定/可用資料決定 R(相機→世界)。回 None 表示資訊不足。"""
        store = self.link.store

        # 自穩雲台鎖定在「相對地面固定的角度」（如天底鎖定 LOOK-DOWN）：
        # 相機朝向完全已知，不需要任何回報，也不該套用機身 roll/pitch
        # ——那正是雲台已經穩定掉的部分，再套一次就是重複計算。
        # 只有偏航跟著機身轉（C-20T 所有已記載的模式都是如此），所以 yaw
        # 用機身航向加上安裝偏移。
        if self.gimbal_attitude_source == "fixed_earth":
            if att is None:
                self.last_meas_note = "無機體航向（fixed_earth 需要 yaw）"
                return None
            yaw_offset, pitch, roll = self.mount_rad   # mount_rad 是 (yaw, pitch, roll)
            return camera_rotation_gimbal_earth(att.yaw + yaw_offset, pitch, roll)

        if self.gimbal_present and self.gimbal_control != "none":
            gs = store.gimbal_at(frame_t) if self.gimbal_attitude_source in ("auto", "feedback") else None
            if gs is not None:
                yaw = gs.yaw
                if not gs.yaw_is_earth:
                    if att is None:
                        return None
                    yaw = att.yaw + gs.yaw
                roll = 0.0 if self.gimbal_stabilized else gs.roll
                return camera_rotation_gimbal_earth(yaw, gs.pitch, roll)
            if self.gimbal_attitude_source in ("auto", "commanded") and self.last_gimbal_cmd is not None:
                pitch, yaw = self.last_gimbal_cmd
                return camera_rotation_gimbal_earth(yaw, pitch, 0.0)
            return None
        if att is None:
            return None
        R_wb = euler_zyx_to_R(att.yaw, att.pitch, att.roll)
        return camera_rotation_body_mount(R_wb, *self.mount_rad)

    def _geolocate(self, det: Detection, pos, att, frame_t) -> np.ndarray | None:
        R_wc = self._camera_rotation(att, frame_t)
        if R_wc is None:
            self.last_meas_note = "無姿態資訊（雲台/機體）"
            return None
        # 🔴 相對高度 ≤0 時測地在幾何上就無解：地面被定義成「home 高度的水平面」
        # （NED z=0），飛機若在那個平面底下，往下的射線永遠不會再碰到它，
        # intersect_ground 每幀都回 None。實機遇過 rel_alt = -3m（home 取在
        # 較高處或高度基準漂移），症狀是「框是紅的、清單顯示已鎖定，但一筆
        # 座標都算不出來」——不講清楚的話完全看不出是高度的問題。
        alt = float(getattr(pos, "rel_alt", 0.0) or 0.0)
        if alt <= 0.2:
            self.last_meas_note = (
                f"載具相對高度 {alt:.1f}m（≤0）：射線打不到地面，測地無解。"
                "請確認 home 高度基準，或先起飛到 home 之上")
            return None
        vehicle_ned = self.georef.lla_to_ned(pos.lat, pos.lon, pos.rel_alt)
        u, v = det.ground_pixel
        hit = geolocate_pixel(u, v, self.camera_model, R_wc, vehicle_ned)
        if hit is None:
            self.last_meas_note = "視線未交地（朝天/掠射，或像素落在畸變不可逆區）"
            return None
        # 🔴 斜距合理性：天底鎖定相機看得到的地面，最遠就是 alt·tan(HFOV/2)，
        # 加上姿態傾斜也不會超過幾倍高度。超過就是測地壞了（畸變模型在邊緣
        # 失效、姿態錯、或高度錯），實測壞標定會算出離飛機數公里的「目標」。
        # geolocate.py 的 MAX_SLANT_RANGE_M 是 10km，2.8m 高度下等於沒有把關。
        slant = float(np.linalg.norm(hit - vehicle_ned))
        limit = self.SLANT_RANGE_ALT_RATIO * max(float(pos.rel_alt), 1.0)
        if slant > limit:
            self.last_meas_note = (
                f"測地距離 {slant:.0f}m 超過高度的 {self.SLANT_RANGE_ALT_RATIO:.0f} 倍"
                f"（{limit:.0f}m），判定為測地失效並丟棄")
            return None
        return hit[:2]

    def _try_reacquire(self, detections, pos, att, frame_t) -> None:
        # 🔴 重鎖門檻必須跟著高度縮放。原本固定 15m 下限，但天底相機在
        # 3m 高只看得到約 6.3×3.6m 的地面——15m 的門檻比「整個畫面」還寬
        # 2.5 倍，等於畫面裡**任何一個**偵測都能被當成走失的目標接手。
        # 2026-08-04 實飛就是這樣：草地誤判被接手成目標，飛機跟著亂飛。
        # 上限取 2×高度（涵蓋整個視野仍有餘裕），下限 4m 供高空使用。
        alt = float(getattr(pos, "rel_alt", 0.0) or 0.0)
        gate = min(max(4.0, 3.0 * self.estimator.pos_std), 2.0 * max(alt, 1.0))
        pred = self.estimator.pos_ne
        best_id, best_d = None, gate
        for det in detections:
            hit = self._geolocate(det, pos, att, frame_t)
            if hit is None:
                continue
            d = float(np.linalg.norm(hit - pred))
            if d < best_d:
                best_id, best_d = det.track_id, d
        if best_id is not None:
            self.lock.locked_id = best_id
            # 同步影像記憶：不同步的話 _last_box 還停在失鎖前的位置、
            # _miss_age 累積讓影像重綁閘門張到 ~22 倍目標半徑——新 ID 下一幀
            # 閃爍時，會用「陳舊位置＋全開閘門」綁上恰好路過的另一台車。
            for det in detections:
                if det.track_id == best_id:
                    self.lock._remember(det)
                    break

    # ------------------------------------------------ 狀態機

    def _update_state_machine(self, frame_t: float, measured: bool) -> None:
        if not self.lock.locked and not self.estimator.initialized:
            self.state = "SEARCH"
            return
        if not self.estimator.initialized:
            # 已鎖定但還沒有第一筆有效測地（遙測/姿態未齊）：維持搜尋顯示、不解鎖
            self.state = "SEARCH"
            return
        age = self.estimator.time_since_update(frame_t)
        if measured or age <= 0.5:
            self.state = "TRACK"
        elif age <= self.coast_timeout_s:
            self.state = "COAST"
        else:
            self.state = "LOST"
            self.lock.unlock()
            self.estimator.reset()

    # ------------------------------------------------ 導引 + 雲台

    DEADBAND_VIEW_FRACTION = 0.5   # deadband 最多吃掉「短邊半視野」的幾成

    def _effective_deadband(self, pos) -> float:
        """指令重發門檻，但不得大於相機看得到的範圍。

        🔴 2026-08-04 實飛：設定的 deadband 是 3.0m，而 3.1m 高度下短邊視野
        全長只有 3.6m（半邊 1.79m）。等於「車子幾乎走完整個畫面，指令才更新
        一次」——實測指令間隔 2.4 秒、車子穿越畫面 3.6 秒，飛機永遠在追 3 秒前
        的位置，結果是繞著目標打轉、半徑約 5m、週期約 10 秒，收斂不進去。
        deadband 的用途是抑制抖動，不該大到讓控制迴路失去意義，所以用
        高度換算出的可視範圍把它夾住；高空時設定值本來就較小，不受影響。
        """
        base = self.reposition_deadband_m
        alt = float(getattr(pos, "rel_alt", 0.0) or 0.0)
        if alt <= 0.0:
            return base
        try:
            half_v = math.tan(math.radians(self.camera_model.vfov_deg / 2.0)) * alt
        except Exception:
            return base
        # 下限 0.5m：再小也沒意義（發送本來就有 1Hz 限速），只會徒增指令量
        return max(0.5, min(base, self.DEADBAND_VIEW_FRACTION * half_v))

    def _current_home_ne(self) -> np.ndarray:
        """目前 home 在（鎖定於第一筆 home 的）NED 座標系裡的位置。

        NED 原點不隨 home 重設而動（動了 KF 會崩），但圍欄距離必須以
        「目前的 home」為圓心量。home 沒重設過時這裡就是 (0,0)，行為不變。
        """
        home = self.link.store.home
        if home is None or self.georef is None:
            return np.zeros(2)
        ne = self.georef.lla_to_ned(home.lat, home.lon, 0.0)
        return np.array([ne[0], ne[1]])

    def _safety_cfg(self) -> dict:
        """安全門檻：`safety.<載體>` 子區段覆蓋 `safety` 的共用值。

        高度限制不可能兩種載體共用一個數字：固定翼低於安全高度就是墜機，
        旋翼在 4m 懸停卻是再正常不過的測試。共用一個值的結果是操作員把它
        調鬆到適合旋翼，然後忘了換回來就飛固定翼。
        """
        merged = {k: v for k, v in self.cfg.section("safety").items()
                  if k not in ("multirotor", "fixedwing")}
        merged.update(self.cfg.section("safety").get(self.airframe) or {})
        return merged

    ALT_REFERENCE_TOLERANCE_M = 10.0
    # 天底鎖定相機的地面涵蓋 ≈ alt·tan(HFOV/2)，90° HFOV 下斜距最多約 1.4×alt。
    # 取 6 倍留足姿態傾斜與高度誤差的餘裕，同時仍能擋掉「數公里外的目標」。
    SLANT_RANGE_ALT_RATIO = 6.0

    def _altitude_reference_sane(self, pos) -> str | None:
        """home 高度與飛控自己的高度估計是不是同一個基準？不是就回原因字串。

        DO_REPOSITION 的 param7 被 PX4 當成 AMSL，而我們送的是
        `home_alt_amsl + 設定的相對高度`。這只有在「home 高度」與「飛控當下的
        AMSL 高度」出自同一個基準時才成立。

        實機踩過：EKF2_HGT_REF 指向沒安裝的測距儀，於是 home 是 GPS 基準
        （113~140m）、EKF 高度是氣壓計基準（約 0m），兩者差 135m。此時
        「跟隨高度 4m」算出來的指令會叫飛機爬 139m。

        飛控自己有一致的答案可以對帳：`rel_alt` 就是它認為的「相對 home 高度」，
        所以 (alt_amsl − home_alt_amsl) 應該等於 rel_alt。差太多就是基準不一致，
        這時**任何**高度指令都不可信，寧可不發。
        """
        home = self.link.store.home
        if home is None:
            return None
        if pos is None:
            # 🔴 fail-closed。這是 DO_REPOSITION param7（PX4 一律當 AMSL）唯一
            # 的把關，而位置停更的那段時間**指令照樣在發**（KF coast 最長 8 秒）。
            # 查不了就不能放行：實機踩過 EKF2_HGT_REF 指向未安裝的測距儀，
            # 基準差 135m，「跟隨 4m」會變成叫飛機爬 139m。
            return "位置資料過期，無法驗證高度基準（AMSL），暫不發送指令"
        implied = float(pos.alt_amsl) - float(home.alt_amsl)
        gap = abs(implied - float(pos.rel_alt))
        if gap <= self.ALT_REFERENCE_TOLERANCE_M:
            return None
        return (
            f"高度基準不一致：home {home.alt_amsl:.0f}m 與目前高度 {pos.alt_amsl:.0f}m "
            f"相減得 {implied:+.0f}m，但飛控回報相對高度 {pos.rel_alt:+.0f}m（差 {gap:.0f}m）"
            "——檢查 EKF2_HGT_REF 是否指向未安裝的感測器"
        )

    def alt_clamp_warning(self) -> str | None:
        """導引高度會被安全上下限夾掉時的警告字串（沒被夾則回 None）。

        操作員在設定頁填「跟隨高度 4m」，安全下限卻是 20m —— 送出去的是 20m，
        飛機會爬到操作員沒預期的高度。夾制本身是對的（防止自動飛太低），
        但**默默夾掉不講**就是災難：畫面上寫 4，飛機飛 20。
        """
        requested = getattr(self.guidance, "follow_alt_m", None)
        if requested is None:
            requested = getattr(self.guidance, "alt_m", None)
        if requested is None:
            return None
        requested = float(requested)
        clamped = self.gates.clamp_alt(requested)
        if abs(clamped - requested) < 1e-6:
            return None
        return (
            f"導引高度 {requested:.0f}m 會被安全限制夾到 {clamped:.0f}m"
            f"（下限 {self.gates.min_cmd_alt_m:.0f}m／上限 {self.gates.max_cmd_alt_m:.0f}m）"
            "——要真的飛這個高度，請一併調整安全高度下限"
        )

    def _run_guidance(self, now: float) -> None:
        store = self.link.store
        hb = store.heartbeat
        # 🔴 心跳快照永遠不會被清空，只會停止更新。模式若照舊採用一份凍結的
        # 心跳，飛行員接管的偵測（observe_mode 比對模式變化）就完全失明：
        # 心跳停在 AUTO.LOITER，飛手早已切走，閂鎖不觸發、指令繼續發。
        # 過期就當作「沒有心跳」，讓既有的「尚未收到心跳」路徑接手。
        hb_fresh = bool(hb) and (now - hb.t) <= self.gates.link_timeout_s
        mode = hb.mode if hb_fresh else None
        armed = bool(hb.armed) if hb_fresh else False
        link_ok = store.link_alive(now, self.gates.link_timeout_s)
        # lr24 後端是「遙測、指令兩條實體鏈路」：store.link_alive 只證明遙測
        # （SiK）活著，指令通道（LR24 序列埠）拔線/寫入失敗時 connected=False
        # ——不併進 link_ok 的話閘門全綠、指令卻全數落入黑洞。
        cmd_ch = getattr(self.link, "command", None)
        if cmd_ch is not None and not getattr(cmd_ch, "connected", True):
            link_ok = False

        self.gates.observe_mode(mode, self.guidance_enabled)

        cmd = None
        if self.estimator.initialized:
            cmd = self.guidance.compute(self.estimator)
            cmd.alt_rel_m = self.gates.clamp_alt(cmd.alt_rel_m)

        report = self.gates.evaluate(
            now,
            guidance_enabled=self.guidance_enabled,
            mode=mode,
            armed=armed,
            link_ok=link_ok,
            est_initialized=self.estimator.initialized,
            # 年齡的評估時刻不能凍結在最後一幀：影像斷線時 _last_capture_t
            # 不再前進，量測年齡會永遠顯示 0.1 秒、逾時閘永遠不跳。
            # 取「最後擷取時刻」與「現在（換算到擷取時間軸）」較新者。
            est_age_s=self.estimator.time_since_update(
                max(self._last_capture_t or -math.inf, now - self.video_latency_s)),
            coast_timeout_s=self.coast_timeout_s,
            # 圍欄要量「離目前 home」的距離：NED 原點鎖在第一筆 home（刻意，
            # 換原點會讓 KF 崩），但 PX4 重新解鎖會重設 home——換電池移動幾百
            # 公尺後，圍欄若還錨在舊原點，離飛手 800m 的點會被當 500m 放行。
            cmd_point_ne=(cmd.point_ne - self._current_home_ne()) if cmd else None,
            gps=getattr(store, "gps", None),
            landed=getattr(store, "landed", None),
            video_frozen=bool(getattr(self.video, "frozen", False)),
            target_locked=bool(self.lock.locked),
            meas_note=self.last_meas_note,
        )
        # 高度基準不一致時，送出去的 AMSL 高度會錯得離譜（實測可差 135m）。
        # 這比任何安全門檻都優先——寧可完全不發，也不要發一個高度是錯的指令。
        # 查核要用「拿得到的最新位置」：影像斷線時 _last_capture_t 會凍在最後
        # 一幀，拿舊時刻去查等於永遠在驗一筆陳年樣本。
        pos_for_check = store.position_at(max(self._last_capture_t or -math.inf,
                                              now - self.video_latency_s))
        alt_ref_problem = self._altitude_reference_sane(pos_for_check)
        if alt_ref_problem:
            report.blocked.append(alt_ref_problem)
            report.ok = False

        self.gate_report_blocked = report.blocked
        if report.throttled and not report.blocked:
            # 中性說明，不進紅色阻擋清單
            self.guidance_note = f"限速中（每秒最多 {1.0 / self.gates.min_interval_s:.0f} 筆），等下一個發送時槽"
        if cmd is None or not report.ok or self.georef is None:
            return

        deadband = self._effective_deadband(pos_for_check)
        if self.last_cmd is not None:
            moved = float(np.linalg.norm(cmd.point_ne - self.last_cmd.point_ne))
            # deadband 內仍要定期重發（keepalive）：engine 的「已發送」只代表
            # 交給了後端——LR24 走非同步通道，逾時/被拒/過期丟棄後若目標靜止，
            # deadband 會讓這筆指令永遠不再送出（飛機停在原地、閘門卻全綠）。
            # 直連 MAVLink 重發同一點是冪等的，代價只是每 cmd_refresh_s 一幀。
            if moved < deadband and (now - self.last_cmd.t) < self.cmd_refresh_s:
                # 正常節流，但要讓操作員看得出「現在沒在發」的原因，
                # 否則閘門全綠卻不動會被誤判成系統掛了。
                self.guidance_note = (
                    f"目標僅移動 {moved:.1f}m（< {deadband:.1f}m 門檻），維持現有指令"
                )
                return
        self.guidance_note = ""

        lat, lon, _ = self.georef.ned_to_lla(np.array([cmd.point_ne[0], cmd.point_ne[1], 0.0]))
        alt_amsl = (self.home_alt_amsl or 0.0) + cmd.alt_rel_m
        # 🔴 限速閘門要在「送出之前」上膛。原本 mark_sent 排在 send 之後，
        # send 拋例外（例如 COM 埠被拔）就永遠不上膛，同一筆指令會以迴圈頻率
        # 重送——實測 8 秒內 149 次嘗試 vs 上限 8 次（18.6 倍），把半雙工鏈路
        # 灌爆。送不出去是要重試沒錯，但必須照 guidance.rate_hz 的節奏重試。
        self.gates.mark_sent(now)
        try:
            self.link.send_reposition(  # speed_ms 為 None 時各後端自行退回飛控預設
                lat, lon, alt_amsl,
                alt_rel_m=cmd.alt_rel_m,   # LR24 GOTO 用相對 home 高度；MAVLink 直連忽略
                loiter_radius_m=cmd.loiter_radius_m,
                loiter_ccw=cmd.loiter_ccw,
                speed_ms=cmd.speed_ms,
            )
        except Exception as exc:
            # 送不出去必須看得見：靜默失敗會讓操作員盯著「指令發送中」的綠字，
            # 而飛機其實什麼都沒收到。
            self.loop_error = f"指令送出失敗：{type(exc).__name__}: {exc}"
            self.guidance_note = "指令送出失敗（見錯誤列），下一個發送時槽重試"
            return
        self.last_cmd = LastCommand(
            t=now, lat=lat, lon=lon, alt_rel=cmd.alt_rel_m,
            point_ne=cmd.point_ne.copy(), label=cmd.label, radius=cmd.loiter_radius_m,
        )
        # 指令記錄：操作員必須看得到「到底發了什麼、飛控回了什麼」。
        # 只留 last_cmd 的話，「鎖定了卻沒動」會被誤判成系統壞掉，
        # 實際上可能只是導引沒開、閘門在擋、或飛控拒收。
        rec = {
            "t": now,
            "wall": time.strftime("%H:%M:%S"),
            "n": round(float(cmd.point_ne[0]), 1),
            "e": round(float(cmd.point_ne[1]), 1),
            "lat": round(lat, 7), "lon": round(lon, 7),
            "alt_rel": round(float(cmd.alt_rel_m), 1),
            "speed": None if cmd.speed_ms is None else round(float(cmd.speed_ms), 1),
            "label": cmd.label,
            "ack": None,   # 由 _publish_status 用 COMMAND_ACK 回填
        }
        self.cmd_history.append(rec)
        self.cmd_total += 1
        self._mission_write("command", {k: v for k, v in rec.items() if k != "ack"})

    def _run_gimbal(self, now: float, pos) -> None:
        if not self.gimbal_present or self.gimbal_control == "none":
            return
        if not self.estimator.initialized or self.georef is None:
            return
        target_ne = self.estimator.pos_ne

        if self.gimbal_control == "roi":
            if self.last_roi_ne is not None and self.last_roi_t is not None:
                moved = float(np.linalg.norm(target_ne - self.last_roi_ne))
                if moved < self.roi_update_min_m and now - self.last_roi_t < 2.0:
                    return
            lat, lon, _ = self.georef.ned_to_lla(np.array([target_ne[0], target_ne[1], 0.0]))
            self.link.send_roi_location(lat, lon, self.home_alt_amsl or 0.0)
            self.last_roi_ne = target_ne.copy()
            self.last_roi_t = now

        elif self.gimbal_control == "pitchyaw" and pos is not None:
            vehicle_ned = self.georef.lla_to_ned(pos.lat, pos.lon, pos.rel_alt)
            delta = np.array([target_ne[0], target_ne[1], 0.0]) - vehicle_ned
            horiz = float(np.hypot(delta[0], delta[1]))
            yaw = math.atan2(delta[1], delta[0])
            pitch = -math.atan2(-float(vehicle_ned[2]), max(horiz, 1e-6))
            if self.last_gimbal_cmd is None or (
                abs(pitch - self.last_gimbal_cmd[0]) > math.radians(2)
                or abs(yaw - self.last_gimbal_cmd[1]) > math.radians(2)
            ):
                self.link.send_gimbal_pitchyaw(math.degrees(pitch), math.degrees(yaw))
                self.last_gimbal_cmd = (pitch, yaw)

    # ------------------------------------------------ 顯示與狀態

    def _render_overlay(self, frame, detections, locked_det, pos) -> None:
        state_colors = {
            "SEARCH": (180, 180, 180),
            "TRACK": (80, 200, 80),
            "COAST": (60, 170, 255),
            "LOST": (70, 70, 230),
        }
        # 配色（BGR）：偵測到=綠框、已鎖定=紅框。灰框在雜亂地面上看不見，故不用。
        GREEN, RED = (80, 220, 80), (60, 60, 235)
        # 線寬隨畫面解析度縮放，1080p 才不會細到看不見
        scale = max(frame.shape[1] / 960.0, 1.0)
        thin, thick = max(int(2 * scale), 2), max(int(4 * scale), 3)
        font_sc = 0.6 * scale

        for det in detections:
            x1, y1, x2, y2 = map(int, det.bbox)
            is_locked = locked_det is not None and det.track_id == locked_det.track_id
            color = RED if is_locked else GREEN
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, thick if is_locked else thin)

            # ID 標籤：加深色底條，雜亂背景上也讀得到
            label = f"#{det.track_id} {det.cls_name} {det.conf:.2f}"
            if is_locked:
                label += " LOCKED"
            (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, font_sc, thin)
            ly = max(y1 - int(8 * scale), th + int(6 * scale))
            cv2.rectangle(frame, (x1, ly - th - int(6 * scale)),
                          (x1 + tw + int(8 * scale), ly + int(4 * scale)), color, -1)
            cv2.putText(frame, label, (x1 + int(4 * scale), ly),
                        cv2.FONT_HERSHEY_SIMPLEX, font_sc, (20, 20, 20), thin, cv2.LINE_AA)

            if is_locked:
                u, v = map(int, det.ground_pixel)
                cv2.circle(frame, (u, v), max(int(6 * scale), 5), RED, -1)
                cv2.circle(frame, (u, v), max(int(6 * scale), 5), (255, 255, 255), thin)

        color = state_colors.get(self.state, (200, 200, 200))
        cv2.putText(frame, f"STATE: {self.state}", (16, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if self.estimator.initialized and self.last_known_lla:
            lat, lon = self.last_known_lla
            cv2.putText(
                frame,
                f"TGT {lat:.6f}, {lon:.6f}  v={self.estimator.speed:.1f}m/s",
                (16, 58), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2,
            )
        if pos is not None:
            cv2.putText(
                frame, f"UAV alt {pos.rel_alt:.0f}m", (16, 84),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 120), 2,
            )

        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
        if ok:
            with self._jpeg_lock:
                self._jpeg = buf.tobytes()

    def jpeg_frame(self) -> bytes | None:
        with self._jpeg_lock:
            return self._jpeg

    def raw_frame(self):
        with self._jpeg_lock:
            return None if self._raw_frame is None else self._raw_frame.copy()

    def _drain_video_events(self) -> None:
        """把擷取執行緒累積的影像事件收進 UI 快取、任務記錄與常駐黑盒子。

        常駐檔（data/missions/video_events.jsonl）是刻意的：任務記錄只在導引
        開啟時存在，但圖傳最常在起飛前與盤旋等待時斷——那些全都落在任務記錄
        之外。事件本身很稀疏（一次任務通常個位數筆），檔案成長可忽略。
        """
        drain = getattr(self.video, "drain_events", None)
        if drain is None:
            return
        try:
            events = drain()
        except Exception:
            return
        if not events:
            return
        import json

        for ev in events:
            self._video_events.append(ev)
            self._mission_write(ev.get("kind", "video_event"),
                                {k: v for k, v in ev.items() if k not in ("kind", "t", "wall")})
        try:
            path = self._mission_dir() / "video_events.jsonl"
            with open(path, "a", encoding="utf-8") as fh:
                for ev in events:
                    fh.write(json.dumps({"date": time.strftime("%Y-%m-%d"), **ev},
                                        ensure_ascii=False) + "\n")
        except Exception:
            pass   # 黑盒子寫不進去不能擋任務

    def _publish_status(self, now: float, detections, pos) -> None:
        self._drain_video_events()
        store = self.link.store
        hb = store.heartbeat
        loop_hz = 0.0
        if len(self._loop_times) >= 2:
            span = self._loop_times[-1] - self._loop_times[0]
            if span > 0:
                loop_hz = (len(self._loop_times) - 1) / span

        # meas_note 要放在 initialized 分支**外面**：最需要看到「量測為什麼
        # 失敗」的時刻，正是估計器還沒起來的時候（框是紅的、清單顯示已鎖定，
        # 卻一筆座標都算不出來）。原本只在 initialized 時才附上，等於永遠看不到。
        target: dict = {"initialized": self.estimator.initialized,
                        "locked": bool(self.lock.locked),
                        "meas_note": self.last_meas_note}
        if self.estimator.initialized:
            age = self.estimator.time_since_update(self._last_capture_t or now)
            p = self.estimator.pos_ne
            target.update(
                n=float(p[0]), e=float(p[1]),
                speed=self.estimator.speed,
                heading_deg=math.degrees(math.atan2(self.estimator.vel_ne[1], self.estimator.vel_ne[0])),
                age_s=age, pos_std=self.estimator.pos_std,
                meas_note=self.last_meas_note,
            )
            if self.last_known_lla:
                target["lat"], target["lon"] = self.last_known_lla

        status = EngineStatus(
            state=self.state,
            sim=self.sim_mode,
            airframe=self.airframe,
            guidance_enabled=self.guidance_enabled,
            guidance_note=self.guidance_note,
            video={
                "connected": getattr(self.video, "connected", True),
                "fps": round(getattr(self.video, "fps", 0.0), 1),
                "device": getattr(self.video, "device_label", ""),
                "error": getattr(self.video, "error", None),
                # 幀齡是「畫面凍住」與「擷取斷線」的分界：connected 仍為 True
                # 但幀齡一直長 = 停格（RF 失鎖的典型長相），UI 要分開講。
                # 用引擎自己的時鐘，不是牆鐘：模擬模式跑虛擬時鐘（從 0 起算），
                # 混用會算出十萬秒的幀齡，讓每次模擬演練都掛著停格警告。
                "frame_age_s": (None if self._last_frame_t is None
                                else round(max(0.0, self.clock() - self._last_frame_t), 1)),
                "health": (self.video.snapshot()
                           if hasattr(self.video, "snapshot") else None),
                "events": list(self._video_events)[-8:],
            },
            vehicle={
                "has_fix": pos is not None,
                "lat": getattr(pos, "lat", None),
                "lon": getattr(pos, "lon", None),
                "rel_alt": getattr(pos, "rel_alt", None),
                "mode": hb.mode if hb else None,
                "armed": hb.armed if hb else False,
                "link_ok": store.link_alive(now, self.gates.link_timeout_s),
                "home_set": store.home is not None,
                # 飛控自己講的話。「Arming denied: ...」只會出現在這裡，
                # 沒有它操作員只能面對「就是不能 arm」而毫無線索。
                "messages": store.recent_messages() if hasattr(store, "recent_messages") else [],
            },
            target=target,
            gates=list(self.gate_report_blocked),
            detections=[
                {
                    "id": d.track_id, "cls": d.cls_name, "conf": round(d.conf, 2),
                    "bbox": [round(v, 1) for v in d.bbox],
                    "locked": self.lock.locked_id == d.track_id,
                }
                for d in detections
            ],
            last_command=(
                {
                    "label": self.last_cmd.label,
                    "lat": self.last_cmd.lat, "lon": self.last_cmd.lon,
                    "alt_rel": self.last_cmd.alt_rel,
                    "radius": self.last_cmd.radius,
                    "age_s": round(now - self.last_cmd.t, 1),
                    "n": float(self.last_cmd.point_ne[0]),
                    "e": float(self.last_cmd.point_ne[1]),
                }
                if self.last_cmd
                else None
            ),
            camera={
                "source": self.camera_model.source,
                "hfov_deg": round(self.camera_model.hfov_deg, 1),
            },
            mavlink={
                "error": getattr(self.link, "error", None),
                "backend": "lr24" if hasattr(self.link, "command") else "direct",
                "lr24": self.link.command.snapshot() if hasattr(self.link, "command") else None,
            },
            gimbal={
                "present": self.gimbal_present,
                "control": self.gimbal_control,
                "has_feedback": store.gimbal_at(self._last_capture_t or now) is not None,
                # 🔴 收到姿態 ≠ 那是實測的。PX4 在 MNT_MODE_OUT=0/1 會拿指令角
                # 合成同一則訊息發出來，長得一模一樣。真 v2 裝置才送
                # GIMBAL_DEVICE_INFORMATION，沒有它就是指令值。
                "attitude_measured": bool(getattr(self.link, "gimbal_information_seen", False)
                                          or getattr(getattr(self.link, "telemetry", None),
                                                     "gimbal_information_seen", False)),
            },
            loop_hz=round(loop_hz, 1),
            detector_error=self.detector_error,
            loop_error=self.loop_error,
            alt_clamp_note=self.alt_clamp_warning(),
        )
        # 回填 COMMAND_ACK：DO_REPOSITION=192。ACK 晚於發送抵達，所以每次發布
        # 狀態時都對「ACK 時間之前最近的一筆」補上結果。
        ack = getattr(self.link, "last_ack", {}).get(192) or getattr(
            getattr(self.link, "telemetry", None), "last_ack", {}).get(192)
        if ack is not None:
            names = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED",
                     3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
            result, ack_t = ack
            for rec in reversed(self.cmd_history):
                if rec["t"] <= ack_t:
                    verdict = names.get(int(result), str(result))
                    # 🔴 ACK 也要進任務記錄。原本只回填記憶體裡的 cmd_history，
                    # 而 command 事件是在「送出當下」寫的、那時 ACK 還沒回來，
                    # 所以覆盤時**永遠看不到飛控收了沒**——而「指令有沒有被
                    # 接受」正是事後最想知道的一件事（拒收與上行斷掉的畫面
                    # 一模一樣：指令列表照長、ACK 欄全空）。
                    if rec.get("ack") != verdict:
                        self._mission_write("command_ack", {
                            "cmd_t": round(rec["t"], 3), "result": verdict,
                            "label": rec.get("label"),
                        })
                    rec["ack"] = verdict
                    break
        status.commands = [dict(r, ago=round(now - r["t"], 1))
                           for r in reversed(self.cmd_history)]
        status.mission_log = (str(self._mission_path.name)
                              if getattr(self, "_mission_path", None) else None)

        # 任務記錄：每 0.5 秒一筆完整狀態快照（含閘門），覆盤時才知道
        # 「那 8 秒為什麼沒發指令」是誰擋的。
        #
        # 但閒置時要放慢：實例（2026-08-04）操作員飛完忘了關導引，記錄以 2Hz
        # 續寫 160 分鐘 → 25190 筆、17MB，而真正在飛的只有前 63 秒。覆盤時
        # 那些閒置 snap 還會把偵測率稀釋成 0%，看起來像整場都沒偵測到。
        # 「有事發生」= 鎖定中 or 已解鎖 or 影像正常，此時維持 0.5s 全解析度。
        interesting = (self.estimator.initialized
                       or bool(getattr(store.heartbeat, "armed", False))
                       or bool(getattr(self.video, "connected", False)))
        snap_dt = 0.5 if interesting else 5.0
        if getattr(self, "_mission_fh", None) is not None and now - self._mission_snap_t >= snap_dt:
            self._mission_snap_t = now
            self._mission_write("snap", {
                "state": self.state,
                "mode": status.vehicle.get("mode"),
                "armed": status.vehicle.get("armed"),
                "link_ok": status.vehicle.get("link_ok"),
                "veh": [status.vehicle.get("lat"), status.vehicle.get("lon"),
                        status.vehicle.get("rel_alt")],
                "tgt": ([round(float(self.estimator.pos_ne[0]), 1),
                         round(float(self.estimator.pos_ne[1]), 1),
                         round(float(self.estimator.speed), 1)]
                        if self.estimator.initialized else None),
                "dets": len(status.detections),
                # 影像狀態必須進快照：上次覆盤時分不出「相機沒看到車」和
                # 「影像根本沒進來」，兩者的處置完全不同。
                # 欄位序：連線、fps、幀齡s、累計幀數、重開次數、平均亮度。
                # 判讀：connected=False 或重開次數在爬 → 擷取層斷（USB/裝置）；
                #      connected=True 但累計幀數不動、幀齡上升 → 畫面凍住；
                #      幀數正常爬但亮度 <8 → 圖傳失鎖（RF），採集端其實正常。
                "video": [bool(status.video.get("connected")),
                          round(float(status.video.get("fps") or 0.0), 1),
                          status.video.get("frame_age_s"),
                          (status.video.get("health") or {}).get("frames_total"),
                          (status.video.get("health") or {}).get("reopen_total"),
                          (status.video.get("health") or {}).get("mean_luma")],
                "gates": list(self.gate_report_blocked),
                "note": self.guidance_note or None,
                "latched": self.gates.pilot_override_latched,
            })
        status.vehicle["path"] = list(self.vehicle_path)[-200:]
        status.target["path"] = list(self.target_path)[-200:]
        # 發布序號：/api/status 永遠回 200，引擎迴圈卡死時前端只會看到
        # 「數字都不動」——多數欄位本來就常常不動，分不出來。序號凍結
        # >2s 是唯一可靠的凍結訊號（前端據此掛紅色橫幅）。
        self._status_seq = getattr(self, "_status_seq", 0) + 1
        status.seq = self._status_seq
        with self._status_lock:
            self._status = status

    def status(self) -> EngineStatus:
        with self._status_lock:
            status = self._status
        # 高度夾制警告來自設定、不是量測，所以不該等到跑完一幀才出現：
        # 影像還沒進來（或斷線）時操作員最需要看到「你填的高度不會生效」。
        status.alt_clamp_note = self.alt_clamp_warning()
        # 同理：任務記錄狀態要即時，關導引的瞬間 UI 就該把「記錄中」拿掉，
        # 不能等下一幀影像進來才更新
        status.mission_log = (str(self._mission_path.name)
                              if getattr(self, "_mission_path", None) else None)
        status.latched = self.gates.pilot_override_latched
        # loop_error 也要即時：例外若發生在 _publish_status **之前**，快照就永遠
        # 停在出事前那一份（顯示「✓ 全部閘門通過，指令發送中」），錯誤本身反而
        # 傳不出去——最需要它的時候看不到，正是最糟的組合。
        status.loop_error = self.loop_error
        return status


def create_engine(cfg: Config) -> TrackerEngine:
    """依 system.mode 建 live 或 sim 引擎。"""
    if cfg.get("system.mode") == "sim":
        from .simulation import build_sim_engine

        return build_sim_engine(cfg)

    from .mavlink_io import MavlinkConnection
    from .vision import Detector, VideoSource

    det_cfg = cfg.section("detector")
    weights = det_cfg.get("weights") or DEFAULT_WEIGHTS_FILE
    weights_path = PROJECT_ROOT / weights
    detector = Detector(
        weights=str(weights_path if weights_path.exists() else weights),
        conf=det_cfg.get("conf", 0.55),
        imgsz=det_cfg.get("imgsz", 640),
        class_names=det_cfg.get("class_names", []),
        tiling=det_cfg.get("tiling", "off"),
        tile_grid=_tile_grid(det_cfg),
        tile_overlap=float(det_cfg.get("tile_overlap", 0.2)),
    )
    video = VideoSource(cfg.section("video"))
    telemetry = MavlinkConnection(
        port=cfg.get("mavlink.port", "COM3"),
        baud=int(cfg.get("mavlink.baud", 57600)),
        stream_rates=cfg.get("mavlink.stream_rates", {}),
        # 與 QGC 同網時可改這個避開 sysid 撞號（QGC 預設也是 255）
        source_system=int(cfg.get("mavlink.gcs_system_id", 255)),
    )

    backend = cfg.get("link.command_backend", "direct")
    if backend == "lr24":
        # 整合 NYCU：pose 走 MAVLink(SiK)、目標 GOTO 走 LR24 給 companion 的 global_goto_node
        from .links import CompositeLink, Lr24CommandChannel

        alt_ref = cfg.get("link.goto_altitude_ref", "rel_home")
        channel = Lr24CommandChannel(
            port=cfg.get("link.lr24_port", "COM4"),
            baud=int(cfg.get("link.lr24_baud", 115200)),
            alt_ref=alt_ref,
            response_timeout_s=float(cfg.get("link.lr24_response_timeout_s", 8.0)),
        )
        link = CompositeLink(telemetry, channel, goto_alt_ref=alt_ref)
    else:
        link = telemetry  # direct：同一條 MAVLink 既讀遙測又發 DO_REPOSITION

    return TrackerEngine(cfg, video=video, detector=detector, link=link)
