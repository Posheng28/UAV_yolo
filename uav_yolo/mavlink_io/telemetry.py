"""PX4 遙測接收與指令發送。

修正舊系統的接收路徑問題：
    舊：每幀對每種訊息 recv_match 一次（非阻塞）→ 取到的是 buffer 裡「最舊」
        的一則；飛控發送率 > 主迴圈率時，姿態延遲會無上限累積。
    新：獨立執行緒阻塞收訊、把 buffer 抽乾，樣本帶單調時間戳進歷史環形緩衝，
        影像處理時用「幀的時間戳」內插取當下姿態/位置——時間對齊。
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass

from ..geometry.frames import quat_wxyz_to_euler_zyx, wrap_pi
from .px4_modes import mode_string

# MAVLink 訊息 id（SET_MESSAGE_INTERVAL 用）
MSG_ID_ATTITUDE = 30
MSG_ID_GLOBAL_POSITION_INT = 33
MSG_ID_GPS_RAW_INT = 24
MSG_ID_EXTENDED_SYS_STATE = 245
MSG_ID_HOME_POSITION = 242
MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS = 285

GIMBAL_DEVICE_FLAGS_YAW_LOCK = 16


@dataclass
class AttitudeSample:
    t: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class PositionSample:
    t: float
    lat: float
    lon: float
    rel_alt: float
    alt_amsl: float
    vn: float
    ve: float


@dataclass
class GimbalSample:
    t: float
    roll: float
    pitch: float
    yaw: float
    yaw_is_earth: bool


@dataclass
class Heartbeat:
    t: float
    mode: str
    armed: bool


@dataclass
class GpsSample:
    """GPS_RAW_INT 品質（對應 global_goto_node 的 GPS gate）。

    注意單位：eph_m/epv_m 是「公尺精度」，來自 MAVLink 2 的 h_acc/v_acc（mm）；
    GPS_RAW_INT 的 eph/epv 欄位其實是 **HDOP/VDOP×100（無因次）**，不是公分——
    老韌體沒有 h_acc 時退回用 DOP 門檻判斷（hdop/vdop 欄位）。
    """
    t: float
    fix_type: int
    satellites: int
    eph_m: float = float("inf")   # h_acc 不可用時為 inf
    epv_m: float = float("inf")
    hdop: float | None = None
    vdop: float | None = None


@dataclass
class LandedSample:
    """EXTENDED_SYS_STATE.landed_state：1=ON_GROUND 2=IN_AIR 3=TAKEOFF 4=LANDING。"""
    t: float
    landed_state: int

    @property
    def airborne(self) -> bool:
        return self.landed_state in (2, 3, 4)


@dataclass
class Home:
    lat: float
    lon: float
    alt_amsl: float


def _interp_angle(a1: float, a2: float, frac: float) -> float:
    return wrap_pi(a1 + wrap_pi(a2 - a1) * frac)


def _interp_samples(buf: deque, t: float, fields: list[str], angle_fields: set[str], cls):
    """時間戳內插；超出範圍取端點。buf 需依時間遞增。"""
    if not buf:
        return None
    if t <= buf[0].t:
        return buf[0]
    if t >= buf[-1].t:
        return buf[-1]
    # 線性掃描（緩衝只有數十筆，夠快）
    for i in range(len(buf) - 1):
        s1, s2 = buf[i], buf[i + 1]
        if s1.t <= t <= s2.t:
            span = s2.t - s1.t
            frac = 0.0 if span <= 0 else (t - s1.t) / span
            values = {"t": t}
            for f in fields:
                v1, v2 = getattr(s1, f), getattr(s2, f)
                if f in angle_fields:
                    values[f] = _interp_angle(v1, v2, frac)
                else:
                    values[f] = v1 + (v2 - v1) * frac
            for extra in vars(s1):
                if extra not in values:
                    values[extra] = getattr(s1, extra)
            return cls(**values)
    return buf[-1]


class TelemetryStore:
    """執行緒安全的遙測歷史，支援任意時間點內插查詢。"""

    def __init__(self, history_s: float = 5.0):
        self.history_s = history_s
        self._lock = threading.Lock()
        self._att: deque[AttitudeSample] = deque()
        self._pos: deque[PositionSample] = deque()
        self._gimbal: deque[GimbalSample] = deque()
        self.heartbeat: Heartbeat | None = None
        self.home: Home | None = None
        self.gps: GpsSample | None = None
        self.landed: LandedSample | None = None
        self.last_msg_t: float | None = None
        # 飛控自己講的話（STATUSTEXT）。「為什麼拒絕 arm」只會從這裡出來，
        # 不解析等於把飛控唯一的解釋管道丟掉，操作員只能面對「就是不能 arm」。
        self.messages: deque[tuple[float, int, str]] = deque(maxlen=40)

    def _trim(self, buf: deque, now: float) -> None:
        while buf and now - buf[0].t > self.history_s:
            buf.popleft()

    # ---- 寫入 ----

    def push_message(self, t: float, severity: int, text: str) -> None:
        """收下飛控的 STATUSTEXT；同一句連續重複只留最新一筆（PX4 會狂洗同一則）。"""
        with self._lock:
            if self.messages and self.messages[-1][2] == text:
                self.messages[-1] = (t, severity, text)
            else:
                self.messages.append((t, severity, text))
            self.last_msg_t = t

    def recent_messages(self, limit: int = 8) -> list[dict]:
        with self._lock:
            items = list(self.messages)[-limit:]
        return [{"t": t, "severity": sev, "text": txt} for t, sev, txt in reversed(items)]

    def push_attitude(self, s: AttitudeSample) -> None:
        with self._lock:
            self._att.append(s)
            self._trim(self._att, s.t)
            self.last_msg_t = s.t

    def push_position(self, s: PositionSample) -> None:
        with self._lock:
            self._pos.append(s)
            self._trim(self._pos, s.t)
            self.last_msg_t = s.t

    def push_gimbal(self, s: GimbalSample) -> None:
        with self._lock:
            self._gimbal.append(s)
            self._trim(self._gimbal, s.t)
            self.last_msg_t = s.t

    def set_heartbeat(self, hb: Heartbeat) -> None:
        with self._lock:
            self.heartbeat = hb
            self.last_msg_t = hb.t

    def set_home(self, home: Home) -> None:
        # 最新為準：PX4 會在解鎖時重設 home，rel_alt 的基準跟著變；
        # 抱著第一筆舊 home 會讓 alt_amsl 換算差掉 home 位移量。
        # （測地的 NED 原點由引擎的 georef 鎖定第一筆，不受此更新影響。）
        with self._lock:
            self.home = home

    def set_gps(self, gps: GpsSample) -> None:
        with self._lock:
            self.gps = gps
            self.last_msg_t = gps.t

    def set_landed(self, landed: LandedSample) -> None:
        with self._lock:
            self.landed = landed
            self.last_msg_t = landed.t

    # ---- 查詢 ----

    def attitude_at(self, t: float) -> AttitudeSample | None:
        with self._lock:
            return _interp_samples(self._att, t, ["roll", "pitch", "yaw"], {"roll", "pitch", "yaw"}, AttitudeSample)

    def position_at(self, t: float) -> PositionSample | None:
        with self._lock:
            return _interp_samples(
                self._pos, t, ["lat", "lon", "rel_alt", "alt_amsl", "vn", "ve"], set(), PositionSample
            )

    def gimbal_at(self, t: float) -> GimbalSample | None:
        with self._lock:
            if not self._gimbal:
                return None
            s = _interp_samples(
                self._gimbal, t, ["roll", "pitch", "yaw"], {"roll", "pitch", "yaw"}, GimbalSample
            )
            return s

    def link_alive(self, now: float, timeout_s: float) -> bool:
        with self._lock:
            return self.last_msg_t is not None and (now - self.last_msg_t) <= timeout_s


class MavlinkConnection:
    """數傳電台上的 PX4 連線：接收執行緒 + 指令發送（皆執行緒安全）。"""

    def __init__(
        self,
        port: str,
        baud: int,
        stream_rates: dict[str, int] | None = None,
        source_system: int = 255,
    ):
        self.port = port
        self.baud = baud
        self.stream_rates = stream_rates or {}
        self.source_system = int(source_system)
        self.store = TelemetryStore()
        self.last_ack: dict[int, tuple[int, float]] = {}  # command -> (result, t)
        self._conn = None
        self._send_lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._target_sys = 1
        self._target_comp = 1
        self._intervals_requested = False
        self._last_att_t: float | None = None
        self._last_pos_t: float | None = None
        self._hb_count = 0
        self._last_interval_req_t = 0.0
        self.error: str | None = None
        # 診斷計數：分辨「完全沒資料（電台/接線/供電）」vs「有資料但解不出（鮑率不合）」
        self.raw_bytes = 0
        self.msgs_parsed = 0
        self.heartbeats_seen = 0
        self.gcs_heartbeats_sent = 0   # 我們發出去的；0 代表 PX4 會判定鏈路中斷
        self._last_hb_sent_t = 0.0
        self.hb_wrong_comp = 0  # 收到心跳但來自非自駕儀元件（雲台/相機）

    # ---- 生命週期 ----

    def _try_open(self) -> bool:
        """嘗試開埠。成功回 True。失敗設 error 回 False（不丟例外）。"""
        from pymavlink import mavutil  # 延遲載入：無硬體的測試不需要

        try:
            self._conn = mavutil.mavlink_connection(
                self.port, baud=self.baud,
                source_system=self.source_system, source_component=190,
            )
            return True
        except Exception as exc:  # 埠不存在/被占用（接 QGC、拔插電台時常見）
            self._conn = None
            self.error = f"開啟 {self.port} 失敗（{exc}）；埠一回來會自動重連"
            return False

    def start(self) -> bool:
        """啟動接收。**即使一開始開不了埠也會啟動執行緒持續重試**。

        以前開埠失敗就 return False 且不啟動執行緒 → 埠一旦在啟動當下不在
        （接 QGC、拔插電台、重新列舉），數傳就永久死掉，要手動「重啟引擎」。
        現在改成背景執行緒持續重試，埠回來就自動接上。
        """
        self._stop.clear()
        opened = self._try_open()
        self._intervals_requested = False
        self._thread = threading.Thread(target=self._recv_loop, daemon=True, name="mavlink-recv")
        self._thread.start()
        return opened

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._conn:
            try:
                self._conn.close()
            except Exception:
                pass

    # ---- 接收 ----

    def _recv_loop(self) -> None:
        from pymavlink import mavutil

        while not self._stop.is_set():
            if self._conn is None:
                # 尚未開埠或斷線：每 1.5 秒重試，埠回來就接上（不用手動重啟引擎）
                if self._stop.wait(1.5):
                    break
                self._try_open()
                continue
            self._send_gcs_heartbeat()
            try:
                msg = self._conn.recv_match(blocking=True, timeout=0.2)
            except Exception as exc:
                # 讀取拋例外通常是埠被拔掉/斷線 → 關掉重來，交給上面重連
                self.error = f"MAVLink 接收中斷（{exc}）；重連中"
                try:
                    self._conn.close()
                except Exception:
                    pass
                self._conn = None
                continue
            # 原始位元組總量（pymavlink 累計）：即使解不出任何封包也會增加，
            # 用來判斷「有沒有東西進來」——鮑率不合時 bytes 漲、封包=0。
            self.raw_bytes = getattr(getattr(self._conn, "mav", None), "total_bytes_received", self.raw_bytes)
            if msg is None:
                continue
            self.msgs_parsed += 1
            now = time.monotonic()
            mtype = msg.get_type()

            if mtype == "HEARTBEAT":
                self.heartbeats_seen += 1
                # 只認自駕儀元件的心跳（雲台/相機也會發心跳，別混入）
                if msg.get_srcComponent() != 1:
                    self.hb_wrong_comp += 1
                    continue
                self._target_sys = msg.get_srcSystem()
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                self.store.set_heartbeat(Heartbeat(t=now, mode=mode_string(msg.custom_mode), armed=armed))
                # 要求串流。SET_MESSAGE_INTERVAL 是一次性上行指令，在 LR24 這種無線
                # 上行掉一封就沒了；而且不同訊息（姿態 vs 位置）要分開要求，任何一條
                # 沒進來都要重發。因此：①開頭前 15 秒每次心跳都重發（提高送達率）
                # ②之後只要「姿態或位置」任一條斷流 >5s 就重發。
                self._hb_count += 1
                att_stale = self._last_att_t is None or (now - self._last_att_t) > 5.0
                pos_stale = self._last_pos_t is None or (now - self._last_pos_t) > 5.0
                warmup = self._hb_count <= 15  # 開頭多發幾次，扛掉上行掉包
                due = (now - self._last_interval_req_t) > 3.0
                if not self._intervals_requested or warmup or ((att_stale or pos_stale) and due):
                    self._request_intervals()
                    self._intervals_requested = True
                    self._last_interval_req_t = now

            elif mtype == "ATTITUDE":
                self._last_att_t = now
                self.store.push_attitude(AttitudeSample(now, msg.roll, msg.pitch, msg.yaw))

            elif mtype == "GLOBAL_POSITION_INT":
                if msg.lat == 0 and msg.lon == 0:
                    continue
                self._last_pos_t = now
                self.store.push_position(
                    PositionSample(
                        t=now,
                        lat=msg.lat / 1e7,
                        lon=msg.lon / 1e7,
                        rel_alt=msg.relative_alt / 1000.0,
                        alt_amsl=msg.alt / 1000.0,
                        vn=msg.vx / 100.0,
                        ve=msg.vy / 100.0,
                    )
                )

            elif mtype == "HOME_POSITION":
                self.store.set_home(Home(lat=msg.latitude / 1e7, lon=msg.longitude / 1e7, alt_amsl=msg.altitude / 1000.0))

            elif mtype == "GIMBAL_DEVICE_ATTITUDE_STATUS":
                w, x, y, z = msg.q
                roll, pitch, yaw = quat_wxyz_to_euler_zyx(w, x, y, z)
                yaw_is_earth = bool(msg.flags & GIMBAL_DEVICE_FLAGS_YAW_LOCK)
                self.store.push_gimbal(GimbalSample(now, roll, pitch, yaw, yaw_is_earth))

            elif mtype == "GPS_RAW_INT":
                # 公尺精度來自 MAVLink 2 擴充欄位 h_acc/v_acc（mm）；
                # eph/epv 是 HDOP/VDOP×100（無因次），別當距離用。
                # 0 與 UINT32_MAX(4294967295) 都代表「未知」——沒定位時是這個哨兵值。
                UINT32_MAX = 4294967295
                h_acc = getattr(msg, "h_acc", 0) or 0
                v_acc = getattr(msg, "v_acc", 0) or 0
                h_known = 0 < h_acc < UINT32_MAX
                v_known = 0 < v_acc < UINT32_MAX
                self.store.set_gps(
                    GpsSample(
                        t=now,
                        fix_type=int(msg.fix_type),
                        satellites=int(msg.satellites_visible),
                        eph_m=(h_acc / 1000.0) if h_known else float("inf"),
                        epv_m=(v_acc / 1000.0) if v_known else float("inf"),
                        hdop=None if msg.eph in (0, 65535) else msg.eph / 100.0,
                        vdop=None if msg.epv in (0, 65535) else msg.epv / 100.0,
                    )
                )

            elif mtype == "EXTENDED_SYS_STATE":
                self.store.set_landed(LandedSample(now, int(msg.landed_state)))

            elif mtype == "COMMAND_ACK":
                self.last_ack[msg.command] = (msg.result, now)

            elif mtype == "STATUSTEXT":
                # 飛控拒絕 arm 的理由（"Arming denied: ..."）只從這裡來。
                text = msg.text
                if isinstance(text, (bytes, bytearray)):
                    text = text.decode("utf-8", "replace")
                text = str(text).rstrip("\x00").strip()
                if text:
                    self.store.push_message(now, int(msg.severity), text)

    GCS_HEARTBEAT_INTERVAL_S = 1.0

    def _send_gcs_heartbeat(self) -> None:
        """以 ~1Hz 發送 GCS 心跳。

        MAVLink 規定每個節點都要週期性發心跳，這是對方判斷「連線還在」的唯一
        依據。少了它，PX4 一收到我們的指令就登記「有地面站」，接著立刻認定
        **資料鏈路中斷**（COM_DL_LOSS_T）→ 進入 failsafe → **拒絕解鎖**。

        症狀非常反直覺，實機上就是這樣呈現的：
            完全不接數傳          → 可以 arm（PX4 從沒看過地面站）
            接上本地面站          → 不能 arm（看過地面站，但心跳沒了＝斷線）
            接上 QGC              → 可以 arm（QGC 有乖乖發心跳）
        只讀不寫的監看程式一樣要發心跳，「我只是聽而已」不是豁免理由。
        """
        if self._conn is None:
            return
        now = time.monotonic()
        if now - self._last_hb_sent_t < self.GCS_HEARTBEAT_INTERVAL_S:
            return
        self._last_hb_sent_t = now
        try:
            from pymavlink import mavutil

            with self._send_lock:
                self._conn.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, mavutil.mavlink.MAV_STATE_ACTIVE,
                )
            self.gcs_heartbeats_sent += 1
        except Exception:
            pass  # 心跳送不出去不該拖垮接收迴圈；斷線由讀取那側處理

    def _request_intervals(self) -> None:
        """跟飛控要固定頻率的訊息（SET_MESSAGE_INTERVAL）。"""
        name_to_id = {
            "ATTITUDE": MSG_ID_ATTITUDE,
            "GLOBAL_POSITION_INT": MSG_ID_GLOBAL_POSITION_INT,
            "GIMBAL_DEVICE_ATTITUDE_STATUS": MSG_ID_GIMBAL_DEVICE_ATTITUDE_STATUS,
            "GPS_RAW_INT": MSG_ID_GPS_RAW_INT,
            "EXTENDED_SYS_STATE": MSG_ID_EXTENDED_SYS_STATE,
        }
        for name, hz in self.stream_rates.items():
            msg_id = name_to_id.get(name)
            if msg_id is None or hz <= 0:
                continue
            self._command_long(511, msg_id, 1e6 / hz)  # MAV_CMD_SET_MESSAGE_INTERVAL
        self._command_long(511, MSG_ID_HOME_POSITION, 5e6)  # home 低頻即可

    # ---- 發送 ----

    def _command_long(self, command: int, p1=0.0, p2=0.0, p3=0.0, p4=0.0, p5=0.0, p6=0.0, p7=0.0) -> None:
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.command_long_send(
                self._target_sys, self._target_comp, command, 0,
                float(p1), float(p2), float(p3), float(p4), float(p5), float(p6), float(p7),
            )

    def _command_int(self, frame: int, command: int, p1, p2, p3, p4, x: int, y: int, z: float) -> None:
        if self._conn is None:
            return
        with self._send_lock:
            self._conn.mav.command_int_send(
                self._target_sys, self._target_comp, frame, command, 0, 0,
                float(p1), float(p2), float(p3), float(p4), int(x), int(y), float(z),
            )

    def send_reposition(
        self, lat: float, lon: float, alt_amsl: float,
        alt_rel_m: float | None = None,
        loiter_radius_m: float | None = None, loiter_ccw: bool = False,
    ) -> None:
        """DO_REPOSITION：旋翼=飛到點懸停；固定翼=以 NAV_LOITER_RAD（或 param3）繞行。

        alt_rel_m 僅供 LR24 後端用相對 home 高度；MAVLink 直連用 alt_amsl，故忽略。
        """
        MAV_FRAME_GLOBAL_INT = 5
        MAV_CMD_DO_REPOSITION = 192
        MAV_DO_REPOSITION_FLAGS_CHANGE_MODE = 1
        radius = float(loiter_radius_m) if loiter_radius_m else float("nan")
        yaw_param = (1.0 if loiter_ccw else 0.0) if loiter_radius_m else float("nan")
        self._command_int(
            MAV_FRAME_GLOBAL_INT, MAV_CMD_DO_REPOSITION,
            -1.0,                                  # p1 地速：預設
            MAV_DO_REPOSITION_FLAGS_CHANGE_MODE,   # p2 切到 Hold 執行
            radius,                                # p3 定翼繞行半徑（部分版本支援，否則用 NAV_LOITER_RAD）
            yaw_param,                             # p4 定翼繞向 0=CW 1=CCW
            round(lat * 1e7), round(lon * 1e7), alt_amsl,
        )

    def send_roi_location(self, lat: float, lon: float, alt_amsl: float) -> None:
        """DO_SET_ROI_LOCATION：飛控自動讓雲台持續指向該座標（用飛控 250Hz 姿態）。"""
        MAV_FRAME_GLOBAL_INT = 5
        MAV_CMD_DO_SET_ROI_LOCATION = 195
        self._command_int(
            MAV_FRAME_GLOBAL_INT, MAV_CMD_DO_SET_ROI_LOCATION,
            0, 0, 0, 0, round(lat * 1e7), round(lon * 1e7), alt_amsl,
        )

    def clear_roi(self) -> None:
        MAV_CMD_DO_SET_ROI_NONE = 197
        self._command_long(MAV_CMD_DO_SET_ROI_NONE)

    def send_gimbal_pitchyaw(self, pitch_deg: float, yaw_deg_earth: float) -> None:
        """直接下雲台角度（地理系 yaw；pitchyaw 控制模式用）。"""
        MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW = 1000
        GIMBAL_MANAGER_FLAGS_YAW_LOCK = 16
        self._command_long(
            MAV_CMD_DO_GIMBAL_MANAGER_PITCHYAW,
            pitch_deg, yaw_deg_earth, float("nan"), float("nan"),
            GIMBAL_MANAGER_FLAGS_YAW_LOCK, 0, 0,
        )
