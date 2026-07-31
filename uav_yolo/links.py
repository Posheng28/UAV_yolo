"""通訊後端：讓引擎的 `link` 可插拔。

兩種指揮後端（config: link.command_backend）：

  direct : UAV_yolo 直接對 PX4 發 MAVLink DO_REPOSITION（單一數傳電台，
           讀遙測也發指令）。四軸首次驗證、無 companion 時用這條，最簡單。
           → 直接用 mavlink_io.MavlinkConnection，不經本檔。

  lr24   : 整合 NYCU_UAV_offboard。目標 GPS 經 LR24-F 傳字串指令 $CMD,seq,GOTO,...
           給機上 companion 的 global_goto_node，由它套用安全 gate 再 DO_REPOSITION。
           但 LR24 不回傳姿態/位置，所以 pose 仍要靠另一條 MAVLink(SiK) 電台讀。
           → CompositeLink：MAVLink 讀 pose、LR24 送 GOTO、雲台 ROI 走 MAVLink。

LR24 幀格式與 NYCU tools/send_lr24_command.py 逐位元一致（見 build_frame / parse_response）。
"""

from __future__ import annotations

import threading
import time
from dataclasses import dataclass

# ---------------------------------------------------------------------------
# LR24-F 線路協定（對齊 NYCU send_lr24_command.py / lr24_command_node.cpp）
# ---------------------------------------------------------------------------

GOTO_COMMANDS = frozenset({"GOTO", "GOTO_AMSL"})
RESPONSE_TYPES = frozenset({"ACK", "ERR", "STAT"})


def checksum(payload: str) -> int:
    """payload（$ 與 * 之間的字串）逐位元 XOR。"""
    value = 0
    for byte in payload.encode("ascii"):
        value ^= byte
    return value


def build_frame(sequence: str, command: str, *arguments) -> str:
    """組一個 checksummed 指令幀，結尾含 \\n。格式：$CMD,seq,COMMAND[,arg...]*HH\\n"""
    seq = str(sequence)
    cmd = str(command).strip().upper()
    if not seq or any(c in seq for c in ",*$\r\n"):
        raise ValueError("sequence 含保留字元或為空")
    if not cmd or any(c in cmd for c in ",*$\r\n"):
        raise ValueError("command 含保留字元或為空")
    args = [str(a) for a in arguments]
    for a in args:
        if any(c in a for c in ",*$\r\n"):
            raise ValueError("argument 含保留字元")
    fields = ["CMD", seq, cmd, *args]
    payload = ",".join(fields)
    if len(payload) + 4 > 256:  # 加上 $ * HH，全幀 ≤256 bytes（LR24 限制）
        raise ValueError("指令幀超過 256 bytes")
    return f"${payload}*{checksum(payload):02X}\n"


@dataclass(frozen=True)
class ResponseFrame:
    frame_type: str  # ACK | ERR | STAT
    sequence: str
    command: str
    message: str


def parse_response(line: str | bytes) -> ResponseFrame:
    """解析並驗 checksum 一個回覆幀。格式錯誤丟 ValueError。"""
    text = line.decode("ascii") if isinstance(line, bytes) else line
    text = text.rstrip("\r\n")
    if not text.startswith("$"):
        raise ValueError("回覆未以 $ 起始")
    star = text.find("*")
    if star < 0 or star + 3 != len(text):
        raise ValueError("checksum 格式錯誤")
    cs_text = text[star + 1:]
    payload = text[1:star]
    if checksum(payload) != int(cs_text, 16):
        raise ValueError("checksum 不符")
    fields = payload.split(",", 3)
    if len(fields) != 4 or fields[0] not in RESPONSE_TYPES or not fields[1]:
        raise ValueError("回覆欄位格式錯誤")
    return ResponseFrame(fields[0], fields[1], fields[2], fields[3])


# ---------------------------------------------------------------------------
# LR24 指令通道：獨立執行緒送指令、收 ACK/ERR，coalesce 最新目標
# ---------------------------------------------------------------------------

class Lr24CommandChannel:
    """把目標 GPS 以 GOTO 幀送給機上 companion；請求/回覆式、嚴格單筆在途。

    引擎的即時迴圈只呼叫 goto()（非阻塞、寫入「最新目標」槽）；真正的送收在
    背景執行緒進行，因為單筆 LR24 往返可能達數秒（node service timeout 7s）。
    """

    def __init__(
        self,
        port: str = "",
        baud: int = 115200,
        *,
        alt_ref: str = "rel_home",
        response_timeout_s: float = 8.0,
        status_interval_s: float = 3.0,
        retry_backoff_s: float = 1.0,
        goto_max_age_s: float = 10.0,
        transport=None,
        seq_start: int | None = None,
    ):
        self.port = port
        self.baud = baud
        self.alt_ref = alt_ref  # rel_home -> GOTO | amsl -> GOTO_AMSL
        self.response_timeout_s = response_timeout_s
        self.status_interval_s = status_interval_s
        self.retry_backoff_s = retry_backoff_s
        self.goto_max_age_s = goto_max_age_s

        self._transport = transport  # 可注入（測試）；None 則 start() 開 pyserial
        self._seq = seq_start if seq_start is not None else self._time_seq()
        self._lock = threading.Lock()
        # (lat, lon, alt, ref, enqueued_at)
        self._pending_goto: tuple[float, float, float, str, float] | None = None
        self.retry_count = 0
        self.dropped_stale = 0
        self._emergency: str | None = None  # "RTL" / "ABORT"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None

        # 對外可觀察狀態（UI/引擎讀）
        self.error: str | None = None
        self.connected = False
        self.ready_for_goto: bool | None = None
        self.last_response: ResponseFrame | None = None
        self.last_response_t: float | None = None
        self.last_sent_frame: str = ""
        self.sent_count = 0
        self.ack_count = 0
        self.err_count = 0

    @staticmethod
    def _time_seq() -> int:
        return time.time_ns()

    def _next_seq(self) -> str:
        # 單調遞增保證唯一（避免 node「同 seq 不同 payload」拒絕）
        self._seq += 1
        return str(self._seq)

    # ---- 引擎端 API（非阻塞） ----

    def goto(self, lat: float, lon: float, alt_m: float, ref: str | None = None) -> None:
        """設定/更新最新目標（coalescing：只保留最後一筆）。"""
        with self._lock:
            self._pending_goto = (
                float(lat), float(lon), float(alt_m), ref or self.alt_ref, time.monotonic()
            )

    def request_rtl(self) -> None:
        with self._lock:
            self._emergency = "RTL"
            # 返航是緊急指令：把排隊中的目標丟掉，
            # 否則 RTL 送出後下一輪又把飛機導回目標——完全違背操作員意圖。
            self._pending_goto = None

    # ---- 生命週期 ----

    def start(self) -> bool:
        if self._transport is None:
            try:
                import serial  # 延遲載入
                self._transport = serial.Serial(self.port, self.baud, timeout=0.2)
            except Exception as exc:
                self.error = f"LR24 序列埠開啟失敗（{self.port}）：{exc}"
                return False
        self.connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._loop, daemon=True, name="lr24-cmd")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._transport is not None:
            try:
                self._transport.close()
            except Exception:
                pass

    # ---- 背景送收 ----

    def _loop(self) -> None:
        # 從 now 起算：待機 status_interval 才發第一筆 STATUS，避免蓋掉剛入列的目標
        last_status = time.monotonic()
        while not self._stop.is_set():
            emergency = goto = None
            with self._lock:
                if self._emergency:
                    emergency, self._emergency = self._emergency, None
                elif self._pending_goto is not None:
                    goto, self._pending_goto = self._pending_goto, None

            if emergency == "RTL":
                self._send_and_wait("RTL")
            elif goto is not None:
                lat, lon, alt, ref, queued_at = goto
                if time.monotonic() - queued_at > self.goto_max_age_s:
                    self.dropped_stale += 1  # 太舊（目標早已丟失）就別再送
                    continue
                verb = "GOTO_AMSL" if ref == "amsl" else "GOTO"
                # node 要求 lat/lon 全精度、alt 一位小數即可
                resp = self._send_and_wait(verb, f"{lat:.7f}", f"{lon:.7f}", f"{alt:.1f}")
                if resp is None or resp.frame_type == "ERR":
                    # 逾時或被拒（busy / 尚未 ready / GPS gate）→ 重排重試。
                    # 引擎有 deadband，目標不動就不會再呼叫 send_reposition，
                    # 這裡不重試的話「一次 ERR 就再也不送」，靜止目標將永遠收不到指令。
                    # ⚠ 重排必須刷新時間戳：帶原 queued_at 的話，一次 8s 逾時
                    # ＋1s backoff 後第二次取出就 >goto_max_age_s，被 stale 丟棄
                    # ——「重試」實際上最多只做一次就永久靜默。staleness 防的是
                    # 「在佇列裡等太久的舊願望」；重試中的目標仍是引擎的最新
                    # 願望（有更新的話 _pending_goto 早被覆蓋、這裡不會重排）。
                    with self._lock:
                        if self._pending_goto is None:  # 沒有更新的目標才重排
                            self._pending_goto = (lat, lon, alt, ref, time.monotonic())
                            self.retry_count += 1
                    time.sleep(self.retry_backoff_s)
            elif time.monotonic() - last_status >= self.status_interval_s:
                self._send_and_wait("STATUS")
                last_status = time.monotonic()
            else:
                time.sleep(0.02)

    def _send_and_wait(self, command: str, *args) -> ResponseFrame | None:
        seq = self._next_seq()
        try:
            frame = build_frame(seq, command, *args)
        except ValueError as exc:
            self.error = f"指令組幀失敗：{exc}"
            return None
        try:
            self._transport.write(frame.encode("ascii"))
            if hasattr(self._transport, "flush"):
                self._transport.flush()
        except Exception as exc:
            self.error = f"LR24 寫入失敗：{exc}"
            self.connected = False
            return None
        self.last_sent_frame = frame.strip()
        self.sent_count += 1

        deadline = time.monotonic() + self.response_timeout_s
        while time.monotonic() < deadline and not self._stop.is_set():
            try:
                raw = self._transport.readline()
            except Exception as exc:
                self.error = f"LR24 讀取失敗：{exc}"
                self.connected = False
                return None
            if not raw:
                continue
            try:
                resp = parse_response(raw)
            except ValueError:
                continue  # 略過雜訊/半幀
            self._observe(resp)
            if resp.frame_type in ("ACK", "ERR") and resp.sequence == seq:
                return resp
            # STAT 或別的 seq：繼續等本次 seq 的終局回覆
        return None  # timeout

    def _observe(self, resp: ResponseFrame) -> None:
        self.error = None
        self.connected = True
        self.last_response = resp
        self.last_response_t = time.monotonic()
        if resp.frame_type == "ACK":
            self.ack_count += 1
        elif resp.frame_type == "ERR":
            self.err_count += 1
        # 從 STATUS 的 STAT/ACK 訊息抓 ready_for_goto
        if "ready_for_goto=true" in resp.message:
            self.ready_for_goto = True
        elif "ready_for_goto=false" in resp.message:
            self.ready_for_goto = False

    def snapshot(self) -> dict:
        return {
            "connected": self.connected,
            "error": self.error,
            "ready_for_goto": self.ready_for_goto,
            "sent": self.sent_count,
            "ack": self.ack_count,
            "err": self.err_count,
            "retry": self.retry_count,
            "dropped_stale": self.dropped_stale,
            "last_sent": self.last_sent_frame,
            "last_response": (
                f"{self.last_response.frame_type}:{self.last_response.message}"
                if self.last_response else None
            ),
        }


# ---------------------------------------------------------------------------
# 複合 link：遙測來源（讀 pose）＋ 指令通道（送 GOTO），對引擎呈現單一介面
# ---------------------------------------------------------------------------

class CompositeLink:
    """鴨子型別等同 MavlinkConnection：
        .store           來自遙測來源（MAVLink SiK）——引擎讀 pose 用
        .send_reposition 走 LR24 GOTO（給 companion 的 global_goto_node）
        .send_roi_location / .send_gimbal_pitchyaw / .clear_roi 走遙測 MAVLink（直接給 PX4 驅動雲台）
    """

    def __init__(self, telemetry_link, command_channel: Lr24CommandChannel, *, goto_alt_ref: str = "rel_home"):
        self.telemetry = telemetry_link
        self.command = command_channel
        self.goto_alt_ref = goto_alt_ref
        self.store = telemetry_link.store

    # ---- 生命週期 ----

    def start(self) -> bool:
        tele_ok = self.telemetry.start()
        cmd_ok = self.command.start()
        return bool(tele_ok) and bool(cmd_ok)

    def stop(self) -> None:
        try:
            self.command.stop()
        finally:
            self.telemetry.stop()

    @property
    def error(self) -> str | None:
        return self.telemetry.error or self.command.error

    # ---- 指令 ----

    def send_reposition(self, lat, lon, alt_amsl, alt_rel_m=None, loiter_radius_m=None,
                        loiter_ccw=False, speed_ms=None):
        # LR24 GOTO 幀只有 lat/lon/alt，帶不了 loiter 半徑/方向與速度上限：
        # 半徑由 PX4 NAV_LOITER_RAD 決定，速度由機上 global_goto_node 或
        # MPC_XY_CRUISE 決定。走這條後端時 speed_ms 不會生效。
        if self.goto_alt_ref == "rel_home" and alt_rel_m is not None:
            self.command.goto(lat, lon, alt_rel_m, ref="rel_home")
        else:
            self.command.goto(lat, lon, alt_amsl, ref="amsl")

    def send_roi_location(self, lat, lon, alt_amsl):
        if hasattr(self.telemetry, "send_roi_location"):
            self.telemetry.send_roi_location(lat, lon, alt_amsl)

    def clear_roi(self):
        if hasattr(self.telemetry, "clear_roi"):
            self.telemetry.clear_roi()

    def send_gimbal_pitchyaw(self, pitch_deg, yaw_deg_earth):
        if hasattr(self.telemetry, "send_gimbal_pitchyaw"):
            self.telemetry.send_gimbal_pitchyaw(pitch_deg, yaw_deg_earth)

    def request_rtl(self):
        self.command.request_rtl()
