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
        self.detector_error: str | None = None
        self.loop_error: str | None = None
        self.cmd_history: deque = deque(maxlen=15)   # 最近發出的導引指令（UI 顯示）
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
        self._mission_close("引擎停止")
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=3.0)
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

        # 導引參數（載體種類仍需重啟，這裡只換同載體的數值）
        self.guidance = build_guidance(self.airframe, cfg.section("guidance"))
        deadband_key = "fixedwing" if self.airframe == "fixedwing" else "multirotor"
        self.reposition_deadband_m = float(
            cfg.get(f"guidance.{deadband_key}.reposition_deadband_m", 3.0)
        )
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
        self.lock.min_lock_frames = int(det_cfg.get("min_lock_frames", 6))
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

    def _mission_open(self) -> None:
        import json
        from pathlib import Path

        try:
            name = time.strftime("mission_%Y%m%d_%H%M%S.jsonl")
            self._mission_path = self._mission_dir() / name
            self._mission_fh = open(self._mission_path, "a", encoding="utf-8")
            self._mission_snap_t = 0.0
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
            self._mission_fh = None
            self._mission_path = None

    def _mission_close(self, reason: str) -> None:
        if getattr(self, "_mission_fh", None) is None:
            return
        self._mission_write("guidance_off", {
            "reason": reason,
            "commands_sent": len(self.cmd_history),
            "pilot_override_latched": self.gates.pilot_override_latched,
        })
        try:
            self._mission_fh.close()
        except Exception:
            pass
        self._mission_fh = None
        self._mission_path = None

    def _mission_write(self, event: str, payload: dict) -> None:
        if getattr(self, "_mission_fh", None) is None:
            return
        import json

        try:
            rec = {"event": event, "t": round(self.clock(), 3),
                   "wall": time.strftime("%H:%M:%S"), **payload}
            self._mission_fh.write(json.dumps(rec, ensure_ascii=False) + "\n")
            self._mission_fh.flush()   # 逐行落盤：當機/斷電也不掉已寫的
        except Exception:
            pass

    def manual_lock(self, track_id: int) -> None:
        self.lock.request_manual_lock(track_id)

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
                    self._publish_status(
                        clock_now, [], self.link.store.position_at(clock_now))
                except Exception:
                    pass
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
        locked_det = self.lock.update(detections)

        # 影像鏈路有固定延遲（RTSP/數位圖傳可達數百 ms）：這一幀「拍的是」
        # capture_t 當下的世界，而不是它送達的時刻。整條感知鏈統一用 capture_t，
        # 才會拿到當時的姿態/位置。20 m/s 下 300ms 未補償 = 6m 測地誤差。
        capture_t = frame_t - self.video_latency_s
        self._last_capture_t = capture_t

        pos = store.position_at(capture_t)
        att = store.attitude_at(capture_t)

        self.estimator.predict_to(capture_t)

        measured = False
        if locked_det is not None and pos is not None and self.georef is not None:
            hit_ne = self._geolocate(locked_det, pos, att, capture_t)
            if hit_ne is not None:
                ok, note = self.estimator.update(capture_t, hit_ne)
                self.last_meas_note = note
                measured = ok

        # 未鎖定但 KF 還活著 → 世界座標重鎖定（丟失後目標重新出現）
        if (
            locked_det is None
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
        self._render_overlay(frame, detections, locked_det, pos)
        self._publish_status(now, detections, pos)
        self._loop_times.append(now)
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
        vehicle_ned = self.georef.lla_to_ned(pos.lat, pos.lon, pos.rel_alt)
        u, v = det.ground_pixel
        hit = geolocate_pixel(u, v, self.camera_model, R_wc, vehicle_ned)
        if hit is None:
            self.last_meas_note = "視線未交地（朝天/掠射）"
            return None
        return hit[:2]

    def _try_reacquire(self, detections, pos, att, frame_t) -> None:
        gate = max(15.0, 3.0 * self.estimator.pos_std)
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
        if home is None or pos is None:
            return None
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
        mode = hb.mode if hb else None
        armed = bool(hb.armed) if hb else False
        link_ok = store.link_alive(now, self.gates.link_timeout_s)

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
            est_age_s=self.estimator.time_since_update(
                now if self._last_capture_t is None else self._last_capture_t),
            coast_timeout_s=self.coast_timeout_s,
            cmd_point_ne=cmd.point_ne if cmd else None,
            gps=getattr(store, "gps", None),
            landed=getattr(store, "landed", None),
        )
        # 高度基準不一致時，送出去的 AMSL 高度會錯得離譜（實測可差 135m）。
        # 這比任何安全門檻都優先——寧可完全不發，也不要發一個高度是錯的指令。
        alt_ref_problem = self._altitude_reference_sane(
            self.link.store.position_at(self._last_capture_t or now))
        if alt_ref_problem:
            report.blocked.append(alt_ref_problem)
            report.ok = False

        self.gate_report_blocked = report.blocked
        if report.throttled and not report.blocked:
            # 中性說明，不進紅色阻擋清單
            self.guidance_note = f"限速中（每秒最多 {1.0 / self.gates.min_interval_s:.0f} 筆），等下一個發送時槽"
        if cmd is None or not report.ok or self.georef is None:
            return

        if self.last_cmd is not None:
            moved = float(np.linalg.norm(cmd.point_ne - self.last_cmd.point_ne))
            if moved < self.reposition_deadband_m:
                # 正常節流，但要讓操作員看得出「現在沒在發」的原因，
                # 否則閘門全綠卻不動會被誤判成系統掛了。
                self.guidance_note = (
                    f"目標僅移動 {moved:.1f}m（< {self.reposition_deadband_m:.0f}m 門檻），維持現有指令"
                )
                return
        self.guidance_note = ""

        lat, lon, _ = self.georef.ned_to_lla(np.array([cmd.point_ne[0], cmd.point_ne[1], 0.0]))
        alt_amsl = (self.home_alt_amsl or 0.0) + cmd.alt_rel_m
        self.link.send_reposition(  # speed_ms 為 None 時各後端自行退回飛控預設
            lat, lon, alt_amsl,
            alt_rel_m=cmd.alt_rel_m,   # LR24 GOTO 用相對 home 高度；MAVLink 直連忽略
            loiter_radius_m=cmd.loiter_radius_m,
            loiter_ccw=cmd.loiter_ccw,
            speed_ms=cmd.speed_ms,
        )
        self.gates.mark_sent(now)
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

    def _publish_status(self, now: float, detections, pos) -> None:
        store = self.link.store
        hb = store.heartbeat
        loop_hz = 0.0
        if len(self._loop_times) >= 2:
            span = self._loop_times[-1] - self._loop_times[0]
            if span > 0:
                loop_hz = (len(self._loop_times) - 1) / span

        target: dict = {"initialized": self.estimator.initialized}
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
                    rec["ack"] = names.get(int(result), str(result))
                    break
        status.commands = [dict(r, ago=round(now - r["t"], 1))
                           for r in reversed(self.cmd_history)]
        status.mission_log = (str(self._mission_path.name)
                              if getattr(self, "_mission_path", None) else None)

        # 任務記錄：每 0.5 秒一筆完整狀態快照（含閘門），覆盤時才知道
        # 「那 8 秒為什麼沒發指令」是誰擋的
        if getattr(self, "_mission_fh", None) is not None and now - self._mission_snap_t >= 0.5:
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
                "gates": list(self.gate_report_blocked),
                "note": self.guidance_note or None,
                "latched": self.gates.pilot_override_latched,
            })
        status.vehicle["path"] = list(self.vehicle_path)[-200:]
        status.target["path"] = list(self.target_path)[-200:]
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
