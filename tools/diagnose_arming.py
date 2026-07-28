"""問飛控「你為什麼不肯 arm」，把它自己的答案挖出來。

PX4 拒絕 arm 時會用 STATUSTEXT 講原因，但那是一閃即逝的廣播；
平常還會持續發 SYS_STATUS（感測器健康位元）與 EKF_STATUS_REPORT
（EKF 各項是否收斂）。這支工具把三者一起收下來並翻譯成人話。

用法：
    python tools/diagnose_arming.py --port COM10 --baud 115200
    python tools/diagnose_arming.py --port COM10 --try-arm    # 送一次 arm 指令抓拒絕理由

⚠ --try-arm 會真的對飛控送出解鎖指令。**務必先拆掉螺旋槳。**
   若飛控接受，馬達會轉。預設不會送，要明確加旗標。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymavlink import mavutil

SEVERITY = {0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL", 3: "ERROR",
            4: "WARNING", 5: "NOTICE", 6: "INFO", 7: "DEBUG"}

FIX_TYPE = {0: "無 GPS", 1: "無定位", 2: "2D", 3: "3D", 4: "DGPS",
            5: "RTK float", 6: "RTK fixed"}

# MAV_SYS_STATUS_SENSOR：只列與 arm 相關的
SENSORS = [
    (1 << 0, "3D 陀螺儀"), (1 << 1, "3D 加速度計"), (1 << 2, "3D 磁力計"),
    (1 << 3, "絕對氣壓計"), (1 << 5, "GPS"), (1 << 12, "AHRS/姿態"),
    (1 << 15, "馬達輸出"), (1 << 16, "RC 接收機"),
    (1 << 22, "預飛檢查"), (1 << 24, "GPS 定位品質"),
]

# ESTIMATOR_STATUS.flags（EKF_STATUS_FLAGS）——PX4 實際發的是這則，不是 EKF_STATUS_REPORT。
# 「水平位置(絕對)」沒亮就是 arm 被擋最典型的原因：GPS 有值 ≠ EKF 接受它。
ESTIMATOR_FLAGS = [
    (1 << 0, "姿態"), (1 << 1, "水平速度"), (1 << 2, "垂直速度"),
    (1 << 3, "水平位置(相對)"), (1 << 4, "水平位置(絕對)"),
    (1 << 5, "垂直位置(絕對)"), (1 << 6, "垂直位置(對地)"),
    (1 << 7, "常數位置模式"), (1 << 8, "可預測水平位置(相對)"),
    (1 << 9, "可預測水平位置(絕對)"), (1 << 10, "GPS 跳變"), (1 << 11, "加速度計異常"),
]

# EKF_STATUS_REPORT flags
EKF_FLAGS = [
    (1 << 0, "姿態"), (1 << 1, "水平速度"), (1 << 2, "垂直速度"),
    (1 << 3, "水平位置(相對)"), (1 << 4, "水平位置(絕對)"),
    (1 << 5, "垂直位置(絕對)"), (1 << 6, "垂直位置(地面)"),
    (1 << 7, "常數位置模式"), (1 << 8, "可預測水平位置(相對)"),
    (1 << 9, "可預測水平位置(絕對)"),
]


def decode_param(value: float, param_type: int):
    """PARAM_VALUE.param_value 一律是 float32，但整數型參數是「把位元原封搬進去」。

    不照 param_type 重新解讀的話，整數 4 會被印成 5.60519e-45 這種非正規化浮點
    ——看起來像壞掉的浮點數，實際上是正常的整數。誤判過一次就知道多坑。
    """
    import struct

    INT_TYPES = {1: "B", 2: "b", 3: "H", 4: "h", 5: "I", 6: "i"}  # MAV_PARAM_TYPE
    fmt = INT_TYPES.get(param_type)
    if fmt is None:
        return value                      # REAL32 / REAL64：本來就是浮點
    raw = struct.pack("<f", value)
    size = struct.calcsize(fmt)
    return struct.unpack_from("<" + fmt, raw[:size])[0]


def parse_event(payload: bytes) -> dict | None:
    """手解 MAVLink EVENT（訊息 410）。

    pymavlink 2.4.42 的 dialect 沒有這則訊息（收到只會變成 UNKNOWN_410），
    但 PX4 1.15+ **已經把「拒絕 arm 的理由」從 STATUSTEXT 改走 EVENT 介面**。
    也就是說不解析它，就完全聽不到飛控在講什麼——這正是先前
    「TEMPORARILY_REJECTED 但一句 STATUSTEXT 都沒有」的原因。

    欄位依 MAVLink 打包規則（大的在前）：
        uint32 id, uint32 event_time_boot_ms, uint16 sequence,
        uint8 destination_component, uint8 destination_system,
        uint8 log_levels, uint8 arguments[40]
    """
    import struct

    if len(payload) < 13:
        return None
    ev_id, boot_ms, seq, dst_comp, dst_sys, log_levels = struct.unpack_from("<IIHBBB", payload, 0)
    args = payload[13:53]
    return {
        "id": ev_id,
        "boot_ms": boot_ms,
        "sequence": seq,
        # log_levels: 高 4 bit = 內部級別，低 4 bit = 外部（給操作員看的）級別
        "severity": log_levels & 0x0F,
        "arguments": args,
    }


def bits(value: int, table) -> tuple[list[str], list[str]]:
    on = [name for bit, name in table if value & bit]
    off = [name for bit, name in table if not (value & bit)]
    return on, off


def main() -> int:
    ap = argparse.ArgumentParser(description="診斷 PX4 為什麼不能 arm")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=15.0)
    ap.add_argument("--try-arm", action="store_true",
                    help="送出一次 arm 指令以取得拒絕理由（務必先拆槳）")
    ap.add_argument("--params", action="store_true",
                    help="讀取與 arm 相關的飛控參數（電池校正/門檻/斷路器）")
    args = ap.parse_args()

    print(f">>> 連線 {args.port} @ {args.baud}")
    m = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=255)
    if not m.wait_heartbeat(timeout=10):
        print("!! 收不到心跳；確認電台通電、埠與鮑率正確")
        return 1
    print(f">>> 心跳來自 system={m.target_system} component={m.target_component}\n")

    def req(msg_id: int, hz: float) -> None:
        m.mav.command_long_send(m.target_system, m.target_component,
                                511, 0, msg_id, 1e6 / hz, 0, 0, 0, 0, 0)

    for msg_id, hz in ((1, 2), (24, 2), (33, 4), (30, 4), (0, 1)):  # SYS_STATUS,GPS_RAW,GLOBAL_POS,ATTITUDE,HEARTBEAT
        req(msg_id, hz)
    try:
        req(193, 2)   # EKF_STATUS_REPORT（PX4 不一定支援，失敗無妨）
    except Exception:
        pass

    counts: dict[str, int] = {}
    seen_events: dict[int, dict] = {}
    texts: list[tuple[float, int, str]] = []
    latest: dict[str, object] = {}
    armed_now = None

    t0 = time.monotonic()
    deadline = t0 + args.seconds
    arm_sent_at = None
    ack = None

    while time.monotonic() < deadline:
        if args.try_arm and arm_sent_at is None and time.monotonic() - t0 > 4.0:
            print(">>> 送出 arm 指令（MAV_CMD_COMPONENT_ARM_DISARM, param1=1）…\n")
            m.mav.command_long_send(m.target_system, m.target_component,
                                    400, 0, 1, 0, 0, 0, 0, 0, 0)
            arm_sent_at = time.monotonic()
            deadline = max(deadline, arm_sent_at + 8.0)

        msg = m.recv_match(blocking=True, timeout=0.5)
        if msg is None:
            continue
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1

        if t == "STATUSTEXT":
            txt = msg.text
            if isinstance(txt, (bytes, bytearray)):
                txt = txt.decode("utf-8", "replace")
            txt = str(txt).rstrip("\x00").strip()
            if txt and (not texts or texts[-1][2] != txt):
                texts.append((time.monotonic() - t0, int(msg.severity), txt))
                print(f"  [{SEVERITY.get(msg.severity, msg.severity):9}] {txt}")
        elif t == "COMMAND_ACK" and msg.command == 400:
            ack = msg.result
        elif t.startswith("UNKNOWN_410"):
            buf = getattr(msg, "_msgbuf", None)
            if buf:
                # MAVLink2 標頭 10 bytes，尾端 2 bytes CRC（可能再加 13 bytes 簽章）
                ev = parse_event(bytes(buf)[10:-2])
                if ev and ev["id"] not in seen_events:
                    seen_events[ev["id"]] = ev
                    print(f"  [EVENT] id={ev['id']}  severity={SEVERITY.get(ev['severity'], ev['severity'])}"
                          f"  args={ev['arguments'][:12].hex()}")
        elif t in ("SYS_STATUS", "GPS_RAW_INT", "GLOBAL_POSITION_INT",
                   "EKF_STATUS_REPORT", "ESTIMATOR_STATUS", "BATTERY_STATUS",
                   "EXTENDED_SYS_STATE"):
            latest[t] = msg
        elif t == "HEARTBEAT" and msg.get_srcComponent() == 1:
            armed_now = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

    print("\n" + "=" * 66)
    print(f"收到的訊息種類：" + "  ".join(f"{k}×{v}" for k, v in sorted(counts.items())))
    print("=" * 66)

    if args.params:
        # 電壓分壓校正歪掉時，飛控會以為健康的電池是低電量——量測值與平衡頭
        # 電表對不上就是這個症狀，而低電量會直接擋 arm。
        wanted = [
            "BAT1_V_DIV", "BAT1_N_CELLS", "BAT1_V_EMPTY", "BAT1_V_CHARGED",
            "BAT1_CAPACITY", "BAT1_SOURCE",
            "BAT_LOW_THR", "BAT_CRIT_THR", "BAT_EMERGEN_THR",
            "CBRK_SUPPLY_CHK", "COM_ARM_WO_GPS", "COM_ARM_CHK_ESCS",
            "COM_ARM_MAG_ANG", "COM_PREARM_MODE", "COM_DISARM_PRFLT",
        ]
        print("\n讀取參數中…")
        got: dict[str, float] = {}
        for name in wanted:
            m.mav.param_request_read_send(m.target_system, m.target_component,
                                          name.encode(), -1)
        t_end = time.monotonic() + 6.0
        while time.monotonic() < t_end and len(got) < len(wanted):
            pm = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.4)
            if pm is None:
                continue
            pid = pm.param_id
            if isinstance(pid, (bytes, bytearray)):
                pid = pid.decode("utf-8", "replace")
            pid = pid.rstrip("\x00")
            if pid in wanted:
                got[pid] = decode_param(pm.param_value, pm.param_type)
        for name in wanted:
            v = got.get(name)
            print(f"  {name:<18} {'(讀不到)' if v is None else f'{v:g}'}")

        ncell = got.get("BAT1_N_CELLS")
        vdiv = got.get("BAT1_V_DIV")
        low_thr = got.get("BAT_LOW_THR")
        bat = latest.get("BATTERY_STATUS")
        if bat and ncell:
            volt = sum(x for x in bat.voltages[:12] if 0 < x < 65535) / 1000.0
            per_cell = volt / ncell
            print(f"\n  飛控認為：{volt:.2f} V ÷ {ncell:g}S = 每 cell {per_cell:.2f} V"
                  f"，剩餘 {bat.battery_remaining}%")
            if low_thr is not None:
                margin = bat.battery_remaining / 100.0 - low_thr
                verdict = ("高於低電量門檻，電池不是 arm 被擋的原因"
                           if margin > 0 else "已低於低電量門檻 → 會擋 arm")
                print(f"  低電量門檻 {low_thr*100:.0f}%：{verdict}"
                      f"（餘裕 {margin*100:+.0f} 個百分點）")
            print(f"  對帳：拿平衡頭電表量總電壓。若電表比 {volt:.2f} V 明顯高，"
                  f"就是 BAT1_V_DIV（目前 {vdiv:g}）校歪，")
            print(f"        飛控會把健康電池當低電量。QGC → Power → 電池校正可修。")

    gps = latest.get("GPS_RAW_INT")
    if gps:
        print(f"\nGPS_RAW_INT  定位={FIX_TYPE.get(gps.fix_type, gps.fix_type)}"
              f"  衛星={gps.satellites_visible}"
              f"  HDOP={gps.eph/100:.2f}  VDOP={gps.epv/100:.2f}")
        h_acc = getattr(gps, "h_acc", None)
        if h_acc:
            print(f"             水平精度 {h_acc/1000:.2f} m｜垂直精度 {getattr(gps,'v_acc',0)/1000:.2f} m")
    else:
        print("\n!! 完全沒收到 GPS_RAW_INT")

    gp = latest.get("GLOBAL_POSITION_INT")
    if gp:
        print(f"GLOBAL_POSITION_INT  {gp.lat/1e7:.7f}, {gp.lon/1e7:.7f}  相對高度 {gp.relative_alt/1000:.1f} m")
        print("  → EKF 有全球定位輸出 ✓")
    else:
        print("!! 沒收到 GLOBAL_POSITION_INT")
        print("  → EKF 沒有輸出全球位置。這是 arm 被擋最常見的直接原因；")
        print("     GPS_RAW_INT 有值不代表 EKF 接受它（EKF 會自己判斷品質）。")

    est = latest.get("ESTIMATOR_STATUS")
    if est:
        on, off = bits(est.flags, ESTIMATOR_FLAGS)
        print(f"\nESTIMATOR_STATUS（EKF 實際有效的輸出）flags=0x{est.flags:04x}")
        print(f"  有效：{'、'.join(on) or '(無)'}")
        print(f"  無效：{'、'.join(n for n in off if '跳變' not in n and '異常' not in n) or '(無) ✓'}")
        if not (est.flags & (1 << 4)):
            print("  ⚠ 『水平位置(絕對)』無效 → EKF 沒有全球定位，PX4 會拒絕 arm。")
            print("     注意：GPS_RAW_INT 有 3D fix 不代表 EKF 接受它。")

    ss = latest.get("SYS_STATUS")
    if ss:
        present, _ = bits(ss.onboard_control_sensors_present, SENSORS)
        enabled, _ = bits(ss.onboard_control_sensors_enabled, SENSORS)
        healthy, _ = bits(ss.onboard_control_sensors_health, SENSORS)
        bad = [s for s in present if s not in healthy]
        print(f"\nSYS_STATUS 感測器（present 但 health 沒亮 = 有問題）：")
        print(f"  正常  ：{'、'.join(healthy) or '(無)'}")
        print(f"  有問題：{'、'.join(bad) or '(無) ✓'}")
        prearm_bit = 1 << 22
        if ss.onboard_control_sensors_present & prearm_bit:
            ok = bool(ss.onboard_control_sensors_health & prearm_bit)
            print(f"  預飛檢查（MAV_SYS_STATUS_PREARM_CHECK）：{'通過 ✓' if ok else '未通過 ❌ ← 這就是擋你的'}")

    bat = latest.get("BATTERY_STATUS")
    if bat:
        v = sum(x for x in bat.voltages[:12] if 0 < x < 65535) / 1000.0
        print(f"\n電池：{v:.2f} V   剩餘 {bat.battery_remaining}%")
        if 0 <= bat.battery_remaining < 30:
            print("  ⚠ 電量偏低，PX4 低電量檢查會擋 arm")

    ess = latest.get("EXTENDED_SYS_STATE")
    if ess:
        land = {0: "未知", 1: "已落地", 2: "空中", 3: "起飛中", 4: "降落中"}
        print(f"落地狀態：{land.get(ess.landed_state, ess.landed_state)}")

    ekf = latest.get("EKF_STATUS_REPORT")
    if ekf:
        on, off = bits(ekf.flags, EKF_FLAGS)
        print(f"\nEKF 狀態：")
        print(f"  已收斂：{'、'.join(on) or '(無)'}")
        print(f"  未收斂：{'、'.join(off) or '(無) ✓'}")

    if args.try_arm:
        names = {0: "ACCEPTED（已解鎖！）", 1: "TEMPORARILY_REJECTED", 2: "DENIED",
                 3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
        print(f"\narm 指令結果：{names.get(ack, ack) if ack is not None else '沒收到 COMMAND_ACK'}")
        print(f"心跳回報 armed = {armed_now}")

    if seen_events:
        print(f"\n飛控 EVENT（PX4 1.15+ 用它取代 STATUSTEXT 報告 arm 拒絕理由）：")
        for ev in sorted(seen_events.values(), key=lambda e: e["boot_ms"]):
            print(f"  id={ev['id']:<12} severity={SEVERITY.get(ev['severity'], ev['severity']):9}"
                  f" args={ev['arguments'][:16].hex()}")
        print("  ↑ 事件 id 要對照韌體的 event metadata 才有名字；")
        print("    QGroundControl 內建那份對照表，會直接顯示成人話。")

    print("\n" + "=" * 66)
    if texts:
        print("飛控說的話（依時間）：")
        for dt, sev, txt in texts:
            print(f"  +{dt:5.1f}s [{SEVERITY.get(sev, sev):9}] {txt}")
    else:
        print("這段期間飛控沒有發任何 STATUSTEXT。")
        print("（拒絕 arm 的理由只在「嘗試 arm 的當下」才會發，")
        print("  請加 --try-arm，或在遙控器上撥解鎖的同時跑這支工具。）")
    print("=" * 66)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
