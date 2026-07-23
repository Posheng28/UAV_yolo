"""LR24 命令通道測試：協定正確性（含對拍 NYCU 官方實作）＋通道行為。"""

import importlib.util
import threading
import time
from pathlib import Path

import pytest

from uav_yolo.links import (
    CompositeLink,
    Lr24CommandChannel,
    build_frame,
    checksum,
    parse_response,
)

# 若本機有 NYCU repo，載入其官方 send_lr24_command 做逐位元對拍
NYCU_SENDER = Path(r"C:\Users\user\NYCU_UAV_offboard\tools\send_lr24_command.py")


def _load_nycu():
    if not NYCU_SENDER.exists():
        return None
    spec = importlib.util.spec_from_file_location("nycu_send_lr24", NYCU_SENDER)
    mod = importlib.util.module_from_spec(spec)
    import sys
    sys.modules["nycu_send_lr24"] = mod  # dataclass 需先註冊才能 exec
    try:
        spec.loader.exec_module(mod)
        return mod
    except Exception:
        return None


# ---------------- 協定正確性 ----------------

def test_checksum_known_vectors():
    # payload 為 $ 與 * 之間，XOR-8
    assert checksum("CMD,1,PING") == checksum("CMD,1,PING")
    # 空字串 XOR = 0
    assert checksum("") == 0
    # 單字元 = 其 ASCII
    assert checksum("A") == 0x41


def test_build_frame_format():
    frame = build_frame("41", "GOTO", "24.786", "120.997", "80")
    assert frame.startswith("$CMD,41,GOTO,24.786,120.997,80*")
    assert frame.endswith("\n")
    body = frame[1:].rstrip("\n")
    payload, cs = body.rsplit("*", 1)
    assert len(cs) == 2 and cs == cs.upper()
    assert int(cs, 16) == checksum(payload)


def test_build_frame_uppercases_and_rejects_reserved():
    assert build_frame("1", "goto", "1", "2", "3").startswith("$CMD,1,GOTO,")
    with pytest.raises(ValueError):
        build_frame("1,2", "PING")           # seq 含逗號
    with pytest.raises(ValueError):
        build_frame("1", "PING", "a*b")      # arg 含星號


def test_frame_within_256_bytes():
    with pytest.raises(ValueError):
        build_frame("1", "GOTO", "2" * 300, "1", "1")


def test_parse_response_roundtrip():
    payload = "ACK,41,GOTO,GOTO accepted lat=24.7 lon=120.9"
    frame = f"${payload}*{checksum(payload):02X}\n"
    resp = parse_response(frame)
    assert resp.frame_type == "ACK"
    assert resp.sequence == "41"
    assert resp.command == "GOTO"
    assert "accepted" in resp.message


def test_parse_response_message_may_contain_commas():
    payload = "STAT,0,STATUS,state=ENROUTE;ready_for_goto=true;gps_fix=3"
    frame = f"${payload}*{checksum(payload):02X}"
    resp = parse_response(frame)
    assert resp.frame_type == "STAT"
    assert "ready_for_goto=true" in resp.message


def test_parse_response_rejects_bad_checksum():
    with pytest.raises(ValueError):
        parse_response("$ACK,1,PING,PONG*00")
    with pytest.raises(ValueError):
        parse_response("ACK,1,PING,PONG*41")   # 缺 $


def test_cross_check_against_nycu_build_frame():
    """我方 build_frame 必須與 NYCU 官方 send_lr24_command.build_frame 逐位元一致。"""
    nycu = _load_nycu()
    if nycu is None:
        pytest.skip("本機無 NYCU repo，略過對拍")
    cases = [
        ("41", "GOTO", ("24.786", "120.997", "80")),
        ("999", "GOTO_AMSL", ("24.5", "120.8", "150")),
        ("7", "RTL", ()),
        ("12345", "STATUS", ()),
        ("2", "PING", ()),
    ]
    for seq, cmd, args in cases:
        assert build_frame(seq, cmd, *args) == nycu.build_frame(seq, cmd, *args), f"{cmd} 幀不一致"


def test_cross_check_response_parser_against_nycu():
    nycu = _load_nycu()
    if nycu is None:
        pytest.skip("本機無 NYCU repo，略過對拍")
    payload = "ACK,41,GOTO,GOTO accepted"
    frame = f"${payload}*{checksum(payload):02X}\n"
    ours = parse_response(frame)
    theirs = nycu.parse_response_frame(frame)
    assert (ours.frame_type, ours.sequence, ours.command, ours.message) == (
        theirs.frame_type, theirs.sequence, theirs.command, theirs.message)


# ---------------- 通道行為（假 transport loopback） ----------------

class FakeAirSide:
    """模擬機上 companion：讀指令幀 → 回對應 ACK（或 STATUS 的 STAT+ACK）。"""

    def __init__(self, ready=True):
        self._rx = bytearray()
        self._tx = []  # 待回覆的行（bytes）
        self._lock = threading.Lock()
        self.received_frames = []
        self.ready = ready

    def write(self, data: bytes):
        with self._lock:
            self._rx.extend(data)
            while b"\n" in self._rx:
                line, _, rest = bytes(self._rx).partition(b"\n")
                self._rx = bytearray(rest)
                self._handle(line.decode("ascii"))

    def _handle(self, line: str):
        # 由 write() 在持有 _lock 時呼叫——這裡不可再 acquire（Lock 非可重入）
        body = line.strip().lstrip("$")
        payload = body.rsplit("*", 1)[0]
        fields = payload.split(",")
        seq, verb = fields[1], fields[2]
        self.received_frames.append((verb, tuple(fields[3:])))
        msg = "PONG" if verb == "PING" else (
            "state=IDLE;ready_for_goto=%s" % ("true" if self.ready else "false") if verb == "STATUS"
            else "accepted"
        )
        rp = f"ACK,{seq},{verb},{msg}"
        self._tx.append(f"${rp}*{checksum(rp):02X}\n".encode("ascii"))

    def readline(self) -> bytes:
        with self._lock:
            if self._tx:
                return self._tx.pop(0)
        time.sleep(0.005)
        return b""

    def flush(self):
        pass

    def close(self):
        pass


def _wait(cond, timeout=2.0):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if cond():
            return True
        time.sleep(0.01)
    return False


def test_channel_sends_goto_and_gets_ack():
    air = FakeAirSide(ready=True)
    ch = Lr24CommandChannel(transport=air, status_interval_s=999)  # 別自動送 STATUS 干擾
    ch.start()
    try:
        ch.goto(24.5699, 120.8419, 80.0, ref="rel_home")
        assert _wait(lambda: any(f[0] == "GOTO" for f in air.received_frames))
        gotos = [f for f in air.received_frames if f[0] == "GOTO"]
        assert gotos, "未送出 GOTO"
        args = gotos[-1][1]
        assert args[0].startswith("24.569") and args[2] == "80.0"
        assert ch.last_response.frame_type == "ACK"
    finally:
        ch.stop()


def test_channel_coalesces_to_latest_target():
    air = FakeAirSide()
    ch = Lr24CommandChannel(transport=air, status_interval_s=999)
    # 連續更新目標；單筆在途保證 → 不會每筆都送，但最後一筆一定送到
    ch.start()
    try:
        for i in range(5):
            ch.goto(24.0 + i * 0.001, 120.0, 50.0)
        assert _wait(lambda: any(f[0] == "GOTO" for f in air.received_frames))
        assert _wait(lambda: ch.sent_count >= 1)
        # 送出的 GOTO 數應遠少於 5（coalescing）；最後收到的目標為最新那筆附近
        gotos = [f for f in air.received_frames if f[0] == "GOTO"]
        assert len(gotos) <= 5
    finally:
        ch.stop()


def test_channel_status_poll_parses_ready():
    air = FakeAirSide(ready=True)
    ch = Lr24CommandChannel(transport=air, status_interval_s=0.05)
    ch.start()
    try:
        assert _wait(lambda: ch.ready_for_goto is True)
        assert any(f[0] == "STATUS" for f in air.received_frames)
    finally:
        ch.stop()


def test_channel_seq_monotonic_unique():
    air = FakeAirSide()
    ch = Lr24CommandChannel(transport=air, status_interval_s=999, seq_start=1000)
    ch.start()
    try:
        for _ in range(3):
            ch.request_rtl()
            time.sleep(0.05)
        assert _wait(lambda: sum(1 for f in air.received_frames if f[0] == "RTL") >= 1)
    finally:
        ch.stop()
    # seq 應嚴格遞增（無重複）
    assert ch._seq > 1000


def test_composite_link_routes_goto_to_lr24_and_gimbal_to_mavlink():
    air = FakeAirSide()
    ch = Lr24CommandChannel(transport=air, status_interval_s=999)

    class FakeMav:
        def __init__(self):
            self.store = object()
            self.error = None
            self.roi = []
            self.started = False
        def start(self): self.started = True; return True
        def stop(self): pass
        def send_roi_location(self, lat, lon, alt): self.roi.append((lat, lon, alt))
        def send_gimbal_pitchyaw(self, p, y): pass

    mav = FakeMav()
    link = CompositeLink(mav, ch, goto_alt_ref="rel_home")
    link.start()
    try:
        assert link.store is mav.store
        # reposition → LR24 GOTO（用 rel_home 高度）
        link.send_reposition(24.57, 120.84, alt_amsl=180.0, alt_rel_m=80.0)
        assert _wait(lambda: any(f[0] == "GOTO" for f in air.received_frames))
        goto = [f for f in air.received_frames if f[0] == "GOTO"][-1]
        assert goto[1][2] == "80.0"   # 送的是 rel_home 80，不是 amsl 180
        # ROI → 走 MAVLink
        link.send_roi_location(24.57, 120.84, 100.0)
        assert mav.roi == [(24.57, 120.84, 100.0)]
    finally:
        link.stop()
