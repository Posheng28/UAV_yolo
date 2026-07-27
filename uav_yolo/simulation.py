"""模擬模式：合成載具動力學 + 合成地面目標 + 合成影像/偵測，閉環驗證全系統。

用途：
    1. 無硬體端到端測試（pytest 斷言測地精度、急停收斂、盲區 coast）。
    2. UI 演練：隊員任務前熟悉操作流程（鎖定、開導引、接管演練）。

時間採「模擬時鐘」：world.clock 傳給引擎，測試可用 world.step(dt) 手搖快轉；
UI 即時模式則由 SimLink.start() 開執行緒以真實時間步進。
"""

from __future__ import annotations

import math
import threading
import time

import cv2
import numpy as np

from .config import DEFAULT_INTRINSICS_FILE, PROJECT_ROOT, Config
from .geometry import CameraModel, GeoRef, camera_rotation_gimbal_earth
from .geometry.frames import wrap_pi
from .mavlink_io.telemetry import (
    AttitudeSample,
    GimbalSample,
    GpsSample,
    Heartbeat,
    Home,
    LandedSample,
    PositionSample,
    TelemetryStore,
)
from .vision.detector import Detection

G = 9.81


class SimWorld:
    """合成世界：地面車 + 載具（旋翼/定翼）+ 自穩雲台。"""

    def __init__(self, cfg: Config):
        sim = cfg.section("sim")
        home = sim.get("home", {})
        self.georef = GeoRef(home.get("lat", 24.5699), home.get("lon", 120.8419))
        self.home_alt_amsl = 100.0
        self.airframe = cfg.get("vehicle.airframe", "multirotor")

        self.sim_t = 0.0
        self.store = TelemetryStore(history_s=10.0)
        self.store.set_home(Home(self.georef.lat0, self.georef.lon0, self.home_alt_amsl))

        # 地面目標：從 home 附近出發直線行駛，stop_after_s 後急停
        self.target_speed = float(sim.get("target_speed_ms", 10.0))
        self.stop_after_s = float(sim.get("stop_after_s", 25.0))
        # patrol=True：驅→停→轉向→再驅，永不永久停止（讓 UI 示範有動態可看）；
        # 測試需要「停了就穩定停住觀察收斂」，故 e2e 一律設 patrol:false。
        self.patrol = bool(sim.get("patrol", False))
        self.patrol_hold_s = float(sim.get("patrol_hold_s", 6.0))  # 每段結尾停多久
        self.patrol_leg_s = float(sim.get("patrol_leg_s", 18.0))   # 每段行駛多久
        self._base_heading = math.radians(35.0)
        self.target_pos = np.array([20.0, 10.0])
        self.target_vel = self.target_speed * np.array(
            [math.cos(self._base_heading), math.sin(self._base_heading)]
        )
        self.dropouts = [tuple(w) for w in sim.get("detection_dropout", [])]
        self.pixel_noise = float(sim.get("pixel_noise_px", 2.0))
        self._rng = np.random.default_rng(7)

        # 載具初始狀態
        if self.airframe == "fixedwing":
            self.veh_pos = np.array([-250.0, -80.0])
            self.veh_alt = 80.0
            self.veh_heading = math.radians(45.0)
            self.airspeed = 18.0
            self.turn_rate_max = 0.4
        else:
            self.veh_pos = np.array([0.0, 0.0])
            self.veh_alt = 40.0
            self.veh_heading = 0.0
            self.mc_speed_max = 12.0
        self.veh_roll = 0.0
        self.veh_vel = np.array([0.0, 0.0])

        # 指令狀態（SimLink 寫入）
        self.cmd_point: np.ndarray | None = None
        self.cmd_alt_rel: float | None = None
        self.cmd_radius: float | None = None
        self.cmd_ccw = False
        self.roi_point: np.ndarray | None = None

        # 雲台（自穩、地理系角度）；無 ROI 時預設垂直朝下
        self.gimbal_yaw = 0.0
        self.gimbal_pitch = math.radians(-90.0)

        self.repositions: list[dict] = []
        self.rois: list[dict] = []

        self._push_telemetry()

    # ---- 時鐘 ----

    def clock(self) -> float:
        return self.sim_t

    # ---- 動力學 ----

    def step(self, dt: float = 0.05) -> None:
        self.sim_t += dt

        if self.patrol:
            # 巡邏：每段 = 行駛 patrol_leg_s + 停 patrol_hold_s，之後轉 130° 續行（近似三角巡邏、留在畫面內）
            period = self.patrol_leg_s + self.patrol_hold_s
            leg = int(self.sim_t // period)
            phase = self.sim_t % period
            if phase < self.patrol_leg_s:
                heading = self._base_heading + leg * math.radians(130.0)
                self.target_vel = self.target_speed * np.array(
                    [math.cos(heading), math.sin(heading)]
                )
            else:
                self.target_vel = np.array([0.0, 0.0])  # 段末短停（示範急停收斂）
        elif self.sim_t >= self.stop_after_s:
            # 急停邏輯（使用者點名的固定翼痛點場景；測試用）
            self.target_vel = np.array([0.0, 0.0])
        self.target_pos = self.target_pos + self.target_vel * dt

        if self.airframe == "fixedwing":
            self._step_fixedwing(dt)
        else:
            self._step_multirotor(dt)

        # 自穩雲台指向 ROI（模擬飛控 DO_SET_ROI_LOCATION 的行為）
        if self.roi_point is not None:
            delta_n = self.roi_point[0] - self.veh_pos[0]
            delta_e = self.roi_point[1] - self.veh_pos[1]
            horiz = math.hypot(delta_n, delta_e)
            self.gimbal_yaw = math.atan2(delta_e, delta_n)
            self.gimbal_pitch = -math.atan2(self.veh_alt, max(horiz, 1e-6))

        self._push_telemetry()

    def _step_multirotor(self, dt: float) -> None:
        if self.cmd_point is not None:
            error = self.cmd_point - self.veh_pos
            vel = error * 0.8
            speed = float(np.linalg.norm(vel))
            if speed > self.mc_speed_max:
                vel = vel / speed * self.mc_speed_max
            self.veh_vel = vel
            self.veh_pos = self.veh_pos + vel * dt
            if self.cmd_alt_rel is not None:
                self.veh_alt += np.clip(self.cmd_alt_rel - self.veh_alt, -3.0 * dt, 3.0 * dt)
            if speed > 1.0:
                self.veh_heading = math.atan2(vel[1], vel[0])
        else:
            self.veh_vel = np.array([0.0, 0.0])
        self.veh_roll = 0.0

    def _step_fixedwing(self, dt: float) -> None:
        """定翼：等速飛行 + 轉彎率限制，向指令圓軌道收斂（簡化 loiter 導引）。"""
        center = self.cmd_point if self.cmd_point is not None else np.array([0.0, 0.0])
        radius = self.cmd_radius if self.cmd_radius else 150.0

        delta = center - self.veh_pos
        d = float(np.linalg.norm(delta))
        bearing_to_center = math.atan2(delta[1], delta[0])
        c = float(np.clip((d - radius) / radius, -1.0, 1.0)) * (math.pi / 2) * 0.9
        if self.cmd_ccw:
            desired = bearing_to_center + math.pi / 2 - c
        else:
            desired = bearing_to_center - math.pi / 2 + c

        err = wrap_pi(desired - self.veh_heading)
        turn = float(np.clip(err * 1.5, -self.turn_rate_max, self.turn_rate_max))
        self.veh_heading = wrap_pi(self.veh_heading + turn * dt)
        self.veh_roll = math.atan2(self.airspeed * turn, G)  # 協調轉彎坡度（顯示/真實感用）

        self.veh_vel = self.airspeed * np.array([math.cos(self.veh_heading), math.sin(self.veh_heading)])
        self.veh_pos = self.veh_pos + self.veh_vel * dt
        if self.cmd_alt_rel is not None:
            self.veh_alt += np.clip(self.cmd_alt_rel - self.veh_alt, -2.0 * dt, 2.0 * dt)

    def _push_telemetry(self) -> None:
        t = self.sim_t
        lat, lon, _ = self.georef.ned_to_lla(np.array([self.veh_pos[0], self.veh_pos[1], 0.0]))
        self.store.push_attitude(AttitudeSample(t, self.veh_roll, 0.0, self.veh_heading))
        self.store.push_position(
            PositionSample(
                t=t, lat=lat, lon=lon,
                rel_alt=self.veh_alt, alt_amsl=self.home_alt_amsl + self.veh_alt,
                vn=float(self.veh_vel[0]), ve=float(self.veh_vel[1]),
            )
        )
        self.store.push_gimbal(GimbalSample(t, 0.0, self.gimbal_pitch, self.gimbal_yaw, yaw_is_earth=True))
        self.store.set_heartbeat(Heartbeat(t, mode="AUTO.LOITER", armed=True))
        # 模擬良好 GPS 與已離地，讓安全閘門在 sim 中可通過
        self.store.set_gps(
            GpsSample(t, fix_type=3, satellites=14, eph_m=0.8, epv_m=1.2, hdop=0.7, vdop=1.0)
        )
        self.store.set_landed(LandedSample(t, landed_state=2))  # 2 = IN_AIR

    # ---- 查詢（渲染/測試用） ----

    def in_dropout(self) -> bool:
        return any(a <= self.sim_t <= b for a, b in self.dropouts)

    def vehicle_ned(self) -> np.ndarray:
        return np.array([self.veh_pos[0], self.veh_pos[1], -self.veh_alt])


class SimLink:
    """鴨子型別等同 MavlinkConnection：store + 指令介面 + 可選即時執行緒。"""

    def __init__(self, world: SimWorld, realtime: bool = True):
        self.world = world
        self.store = world.store
        self.error: str | None = None
        self._realtime = realtime
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

    def start(self) -> None:
        if not self._realtime:
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="sim-world")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)

    def _loop(self) -> None:
        while not self._stop.is_set():
            self.world.step(0.05)
            time.sleep(0.05)

    # ---- 指令介面（與 MavlinkConnection 同簽名） ----

    def send_reposition(self, lat, lon, alt_amsl, alt_rel_m=None, loiter_radius_m=None, loiter_ccw=False) -> None:
        ned = self.world.georef.lla_to_ned(lat, lon, 0.0)
        self.world.cmd_point = np.array([ned[0], ned[1]])
        self.world.cmd_alt_rel = alt_rel_m if alt_rel_m is not None else alt_amsl - self.world.home_alt_amsl
        self.world.cmd_radius = loiter_radius_m
        self.world.cmd_ccw = loiter_ccw
        self.world.repositions.append(
            {"t": self.world.sim_t, "n": float(ned[0]), "e": float(ned[1]),
             "alt_rel": self.world.cmd_alt_rel, "radius": loiter_radius_m}
        )

    def send_roi_location(self, lat, lon, alt_amsl) -> None:
        ned = self.world.georef.lla_to_ned(lat, lon, 0.0)
        self.world.roi_point = np.array([ned[0], ned[1]])
        self.world.rois.append({"t": self.world.sim_t, "n": float(ned[0]), "e": float(ned[1])})

    def clear_roi(self) -> None:
        self.world.roi_point = None

    def send_gimbal_pitchyaw(self, pitch_deg, yaw_deg_earth) -> None:
        self.world.gimbal_pitch = math.radians(pitch_deg)
        self.world.gimbal_yaw = math.radians(yaw_deg_earth)
        self.world.roi_point = None


class SimVideoSource:
    """渲染合成畫面 + 生成該幀的偵測（供 SimDetector 回傳）。"""

    CAR_LEN, CAR_WID = 4.6, 2.0

    def __init__(self, world: SimWorld, cfg: Config):
        self.world = world
        vid = cfg.section("video")
        self.width = int(vid.get("width", 1280))
        self.height = int(vid.get("height", 720))
        self.camera_model = CameraModel.load(
            PROJECT_ROOT / (cfg.get("camera.intrinsics_file") or DEFAULT_INTRINSICS_FILE),
            cfg.get("camera.fallback_hfov_deg", 120.0),
            self.width,
            self.height,
        )
        self.connected = True
        self.fps = 20.0
        self.error = None
        self.device_label = "SIM 合成影像"
        self.last_detections: list[Detection] = []
        self._last_render_t: float | None = None
        self._frame = None

    def start(self) -> bool:
        return True

    def stop(self) -> None:
        pass

    def get_frame(self):
        t = self.world.sim_t
        if t != self._last_render_t:
            self._frame = self._render()
            self._last_render_t = t
        return (self._frame.copy() if self._frame is not None else None), t

    def _render(self):
        w = self.world
        frame = np.full((self.height, self.width, 3), (58, 74, 62), np.uint8)  # 土綠地面

        R_wc = camera_rotation_gimbal_earth(w.gimbal_yaw, w.gimbal_pitch, 0.0)
        R_cw = R_wc.T
        veh = w.vehicle_ned()

        def project(ned_point: np.ndarray):
            p_cam = R_cw @ (ned_point - veh)
            return self.camera_model.project(p_cam)

        # 參考格線（每 40m 一點，畫小十字，提供地面移動感）
        base_n = int(w.veh_pos[0] // 40) * 40
        base_e = int(w.veh_pos[1] // 40) * 40
        for gn in range(base_n - 200, base_n + 201, 40):
            for ge in range(base_e - 200, base_e + 201, 40):
                px = project(np.array([gn, ge, 0.0]))
                if px and 0 <= px[0] < self.width and 0 <= px[1] < self.height:
                    x, y = int(px[0]), int(px[1])
                    cv2.line(frame, (x - 4, y), (x + 4, y), (80, 100, 84), 1)
                    cv2.line(frame, (x, y - 4), (x, y + 4), (80, 100, 84), 1)

        # 目標車（依行進方向旋轉的矩形）
        heading = math.atan2(w.target_vel[1], w.target_vel[0]) if np.linalg.norm(w.target_vel) > 0.5 else 0.0
        cn, sn = math.cos(heading), math.sin(heading)
        half_l, half_w = self.CAR_LEN / 2, self.CAR_WID / 2
        corners_px = []
        for dl, dw in [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]:
            corner = np.array(
                [w.target_pos[0] + dl * cn - dw * sn, w.target_pos[1] + dl * sn + dw * cn, 0.0]
            )
            px = project(corner)
            if px is None:
                corners_px = []
                break
            corners_px.append(px)

        self.last_detections = []
        if corners_px:
            pts = np.array(corners_px, np.float32)
            cv2.fillConvexPoly(frame, pts.astype(np.int32), (40, 40, 170))  # 紅車
            xs, ys = pts[:, 0], pts[:, 1]
            x1, y1, x2, y2 = float(xs.min()), float(ys.min()), float(xs.max()), float(ys.max())
            on_screen = x2 > 0 and y2 > 0 and x1 < self.width and y1 < self.height
            if on_screen and not w.in_dropout():
                noise = w._rng.normal(0, w.pixel_noise, 4)
                self.last_detections = [
                    Detection(
                        track_id=1,
                        cls_name="Car",
                        conf=0.92,
                        bbox=(x1 + noise[0], y1 + noise[1], x2 + noise[2], y2 + noise[3]),
                    )
                ]

        cv2.putText(frame, f"SIM t={w.sim_t:5.1f}s", (self.width - 200, 28),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (220, 220, 220), 1)
        return frame


class SimDetector:
    """回傳 SimVideoSource 渲染時算好的偵測（跳過 YOLO，行為介面一致）。"""

    def __init__(self, video: SimVideoSource):
        self.video = video

    def detect(self, frame, t: float | None = None) -> list[Detection]:
        return list(self.video.last_detections)


def build_sim_engine(cfg: Config, realtime: bool = True):
    from .engine import TrackerEngine

    world = SimWorld(cfg)
    video = SimVideoSource(world, cfg)
    detector = SimDetector(video)
    link = SimLink(world, realtime=realtime)
    engine = TrackerEngine(cfg, video=video, detector=detector, link=link, clock=world.clock)
    engine.sim_world = world  # 測試/UI 觀察用
    return engine
