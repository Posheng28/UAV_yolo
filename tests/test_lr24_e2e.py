"""lr24 後端 × 引擎端到端閉環：SimWorld 遙測 + LR24 假機上端 + CompositeLink。

補上整合路徑的最後一塊拼圖：之前 LR24 通道只有單元測試，
這裡讓「引擎 → CompositeLink → LR24 幀 → (假)機上端 → 世界動力學」整條閉環。
"""

import threading
import time

import numpy as np
import pytest

from uav_yolo.config import Config
from uav_yolo.links import CompositeLink, Lr24CommandChannel, checksum
from uav_yolo.simulation import build_sim_engine

DT = 0.05


class WorldAirSide:
    """假機上端：收 GOTO 就 ACK 並把目標套進 SimWorld（模擬 global_goto_node）。"""

    def __init__(self, world):
        self.world = world
        self._rx = bytearray()
        self._tx = []
        self._lock = threading.Lock()
        self.gotos: list[tuple[float, float, float]] = []

    def write(self, data: bytes):
        with self._lock:
            self._rx.extend(data)
            while b"\n" in self._rx:
                line, _, rest = bytes(self._rx).partition(b"\n")
                self._rx = bytearray(rest)
                self._handle(line.decode("ascii"))

    def _handle(self, line: str):
        payload = line.strip().lstrip("$").rsplit("*", 1)[0]
        fields = payload.split(",")
        seq, verb = fields[1], fields[2]
        if verb == "GOTO":
            lat, lon, rel_alt = map(float, fields[3:6])
            self.gotos.append((lat, lon, rel_alt))
            ned = self.world.georef.lla_to_ned(lat, lon, 0.0)
            self.world.cmd_point = np.array([ned[0], ned[1]])
            self.world.cmd_alt_rel = rel_alt
            # 重點：LR24 幀沒有半徑欄位 → cmd_radius 維持 None（PX4 用 NAV_LOITER_RAD）
            msg = "GOTO accepted"
        elif verb == "STATUS":
            msg = "state=ENROUTE;ready_for_goto=true"
        else:
            msg = "ok"
        rp = f"ACK,{seq},{verb},{msg}"
        self._tx.append(f"${rp}*{checksum(rp):02X}\n".encode("ascii"))

    def readline(self) -> bytes:
        with self._lock:
            if self._tx:
                return self._tx.pop(0)
        time.sleep(0.002)
        return b""

    def flush(self):
        pass

    def close(self):
        pass


@pytest.fixture
def lr24_engine(tmp_path):
    cfg = Config()
    cfg._local_path = tmp_path / "local.yaml"
    cfg.update(
        {
            "system": {"mode": "sim"},
            "vehicle": {"airframe": "fixedwing"},
            "video": {"width": 960, "height": 540},
            "detector": {"lock_mode": "auto", "min_lock_frames": 6},
            "sim": {"patrol": False},  # 依賴「急停後圓心收斂」，關掉巡邏
        }
    )
    engine = build_sim_engine(cfg, realtime=False)
    world = engine.sim_world
    # 本測試主題是「指令通道」不是搜獲：把定翼放到目標上空附近，立刻能鎖定
    world.veh_pos = np.array([0.0, 0.0])

    air = WorldAirSide(world)
    channel = Lr24CommandChannel(transport=air, status_interval_s=999, retry_backoff_s=0.05)
    # 遙測沿用 SimLink（pose/雲台 ROI），指令改走 LR24 —— 與實機 lr24 模式同構
    engine.link = CompositeLink(engine.link, channel, goto_alt_ref="rel_home")
    channel.start()
    yield engine, world, air, channel
    channel.stop()


def crank_realtime(engine, world, seconds: float):
    """手搖模擬 + 讓 LR24 執行緒有真實時間送收。"""
    steps = int(seconds / DT)
    for i in range(steps):
        world.step(DT)
        engine.step()
        if i % 4 == 0:
            time.sleep(0.004)  # 給通道執行緒送收的空檔


def test_lr24_backend_closes_the_loop(lr24_engine):
    engine, world, air, channel = lr24_engine
    engine.set_guidance_enabled(True)

    crank_realtime(engine, world, 40.0)
    time.sleep(0.3)  # 收尾：讓最後的 ACK 回來

    # 引擎狀態正確反映 lr24 後端
    st = engine.status()
    assert st.mavlink["backend"] == "lr24"
    assert st.mavlink["lr24"]["sent"] >= 1

    # LR24 幀真的送達且被 ACK
    assert len(air.gotos) >= 2, f"只送達 {len(air.gotos)} 筆 GOTO"
    assert channel.ack_count >= 2
    assert channel.err_count == 0

    # 高度走的是「相對 home」（30~120m 之內），不是 AMSL
    for _lat, _lon, rel_alt in air.gotos:
        assert 20.0 <= rel_alt <= 120.0, f"GOTO 高度 {rel_alt} 不是相對 home 語意"

    # 閉環有效：圓心收斂到目標附近（急停後）
    last = air.gotos[-1]
    ned = world.georef.lla_to_ned(last[0], last[1], 0.0)
    err = float(np.linalg.norm(np.array([ned[0], ned[1]]) - world.target_pos))
    assert err < 40.0, f"LR24 圓心離目標 {err:.0f}m"

    # LR24 幀不帶半徑 → world.cmd_radius 必須是 None（半徑由 NAV_LOITER_RAD 決定）
    assert world.cmd_radius is None

    # 雲台 ROI 仍走遙測 MAVLink 那條（SimLink），不受指令改道影響
    assert world.rois, "雲台 ROI 沒送（該走遙測鏈路）"


def test_lr24_backend_stops_on_pilot_override(lr24_engine):
    engine, world, air, channel = lr24_engine
    engine.set_guidance_enabled(True)
    crank_realtime(engine, world, 15.0)
    n_before = len(air.gotos)
    assert n_before >= 1

    from uav_yolo.mavlink_io.telemetry import Heartbeat

    orig_push = world._push_telemetry

    def override_push():
        orig_push()
        world.store.set_heartbeat(Heartbeat(world.sim_t, mode="POSCTL", armed=True))

    world._push_telemetry = override_push
    crank_realtime(engine, world, 5.0)
    time.sleep(0.2)
    n_after = len(air.gotos)

    world._push_telemetry = orig_push
    crank_realtime(engine, world, 5.0)
    time.sleep(0.2)
    # 接管後閂鎖：切回也不恢復發送
    assert len(air.gotos) == n_after
    assert engine.gates.pilot_override_latched
