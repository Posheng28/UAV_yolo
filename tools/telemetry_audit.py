"""飛行前遙測全面體檢：把飛控回報的每一項攤開，並主動標出異常。

跟 diagnose_arming.py 的分工：那支專門查「為什麼不能 arm」，這支是
「所有資料看起來合理嗎」——高度基準是否一致、串流有沒有真的照要求的頻率
進來、姿態/震動/電池/RC 有沒有離譜的值。

異常判定都寫成明確門檻並印出理由，不是「看起來怪怪的」。

用法：
    python tools/telemetry_audit.py --port COM10 --seconds 20
（序列埠獨佔，先跑 stop.bat 關掉地面站）
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time
from collections import defaultdict
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymavlink import mavutil

OK, WARN, BAD = "✅", "⚠️ ", "❌"
findings: list[tuple[str, str]] = []


def note(level: str, text: str) -> None:
    findings.append((level, text))


def main() -> int:
    ap = argparse.ArgumentParser(description="飛行前遙測體檢")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--seconds", type=float, default=20.0)
    args = ap.parse_args()

    m = mavutil.mavlink_connection(args.port, baud=args.baud, source_system=255)
    print(f">>> 連線 {args.port}…")
    if not m.wait_heartbeat(timeout=10):
        print("!! 收不到心跳")
        return 1

    # 要串流；沒發心跳的話 PX4 不會把我們當地面站，STATUSTEXT 也不會來
    want = {33: 4, 24: 2, 1: 2, 30: 10, 242: 1, 230: 2, 32: 4, 241: 1, 147: 1, 245: 1}
    for mid, hz in want.items():
        m.mav.command_long_send(m.target_system, m.target_component, 511, 0,
                                mid, 1e6 / hz, 0, 0, 0, 0, 0)

    counts: dict[str, int] = defaultdict(int)
    latest: dict[str, object] = {}
    texts: list[tuple[int, str]] = []
    att_hist: list[tuple[float, float]] = []
    t0 = time.monotonic()
    last_hb = 0.0

    while time.monotonic() - t0 < args.seconds:
        if time.monotonic() - last_hb >= 1.0:
            last_hb = time.monotonic()
            m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                 mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0,
                                 mavutil.mavlink.MAV_STATE_ACTIVE)
        msg = m.recv_match(blocking=True, timeout=0.4)
        if msg is None:
            continue
        t = msg.get_type()
        counts[t] += 1
        latest[t] = msg
        if t == "ATTITUDE":
            att_hist.append((msg.roll, msg.pitch))
        elif t == "STATUSTEXT":
            txt = msg.text
            if isinstance(txt, (bytes, bytearray)):
                txt = txt.decode("utf-8", "replace")
            txt = str(txt).rstrip("\x00").strip()
            if txt and (not texts or texts[-1][1] != txt):
                texts.append((int(msg.severity), txt))

    dur = time.monotonic() - t0
    print(f">>> 收集 {dur:.0f} 秒\n" + "=" * 68)

    # ---------------- 高度：四個來源互相對帳 ----------------
    print("【高度】")
    gp = latest.get("GLOBAL_POSITION_INT")
    home = latest.get("HOME_POSITION")
    gps = latest.get("GPS_RAW_INT")
    if gp:
        print(f"  EKF 融合高度 (AMSL)   {gp.alt/1000:8.1f} m")
        print(f"  EKF 相對 home         {gp.relative_alt/1000:8.1f} m")
    if home:
        print(f"  HOME_POSITION (AMSL)  {home.altitude/1000:8.1f} m")
    if gps:
        print(f"  GPS 原始高度 (AMSL)   {gps.alt/1000:8.1f} m")
    if gp and gps:
        d = abs(gp.alt - gps.alt) / 1000.0
        (note(OK, f"EKF 與 GPS 高度一致（差 {d:.1f} m）") if d < 15 else
         note(WARN, f"EKF 高度與 GPS 差 {d:.1f} m —— 高度基準不一致，"
                    f"別用『home_alt + 相對高度』算 AMSL 下指令"))
    if gp:
        rel = gp.relative_alt / 1000.0
        (note(OK, f"停在地面時相對 home 高度 {rel:+.1f} m，合理") if abs(rel) < 5 else
         note(WARN, f"停在地面卻回報相對 home {rel:+.1f} m —— 導引高度已改走"
                    f"相對座標框，不受影響；但 UI 顯示的高度會偏"))

    # ---------------- GPS ----------------
    print("\n【GPS】")
    if gps:
        fix = {0: "無", 1: "無定位", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK-float", 6: "RTK-fix"}
        h_acc = getattr(gps, "h_acc", 0) / 1000.0
        v_acc = getattr(gps, "v_acc", 0) / 1000.0
        print(f"  定位 {fix.get(gps.fix_type, gps.fix_type)}   衛星 {gps.satellites_visible} 顆"
              f"   HDOP {gps.eph/100:.2f}   VDOP {gps.epv/100:.2f}")
        print(f"  水平精度 {h_acc:.2f} m   垂直精度 {v_acc:.2f} m")
        note(OK if gps.fix_type >= 3 else BAD, f"定位類型 {fix.get(gps.fix_type)}")
        note(OK if gps.satellites_visible >= 8 else WARN,
             f"衛星數 {gps.satellites_visible}（門檻 8）")
        note(OK if h_acc and h_acc <= 5.0 else WARN, f"水平精度 {h_acc:.2f} m（門檻 5）")

    # ---------------- EKF ----------------
    print("\n【EKF】")
    est = latest.get("ESTIMATOR_STATUS")
    if est:
        flags = [(1, "姿態"), (2, "水平速度"), (4, "垂直速度"), (8, "水平位置(相對)"),
                 (16, "水平位置(絕對)"), (32, "垂直位置(絕對)"), (64, "垂直位置(對地)"),
                 (128, "常數位置模式"), (256, "預測水平(相對)"), (512, "預測水平(絕對)"),
                 (1024, "GPS 跳變"), (2048, "加速度計異常")]
        on = [n for b, n in flags if est.flags & b]
        print(f"  flags=0x{est.flags:04x}  有效：{'、'.join(on)}")
        for bit, name, need in ((16, "水平位置(絕對)", True), (32, "垂直位置(絕對)", True),
                                (1024, "GPS 跳變", False), (2048, "加速度計異常", False)):
            has = bool(est.flags & bit)
            note(OK if has == need else BAD if need else WARN,
                 f"{name}：{'有' if has else '無'}")
        for k in ("vel_ratio", "pos_horiz_ratio", "pos_vert_ratio", "mag_ratio"):
            v = getattr(est, k, None)
            if v is not None:
                note(OK if v < 1.0 else WARN, f"{k} = {v:.2f}（>1 表示該項融合發散）")

    # ---------------- 姿態 / 震動 ----------------
    print("\n【姿態與震動】")
    if att_hist:
        rolls = [math.degrees(r) for r, _ in att_hist]
        pitches = [math.degrees(p) for _, p in att_hist]
        print(f"  roll  平均 {statistics.mean(rolls):+.1f}°  變動 {statistics.pstdev(rolls):.2f}°")
        print(f"  pitch 平均 {statistics.mean(pitches):+.1f}°  變動 {statistics.pstdev(pitches):.2f}°")
        for name, vals in (("roll", rolls), ("pitch", pitches)):
            avg = statistics.mean(vals)
            note(OK if abs(avg) < 10 else WARN,
                 f"靜止時 {name} 平均 {avg:+.1f}°（>10° 可能是水平沒校正或機體真的沒放平）")
    vib = latest.get("VIBRATION")
    if vib:
        print(f"  震動 x={vib.vibration_x:.1f} y={vib.vibration_y:.1f} z={vib.vibration_z:.1f}"
              f"   clipping {vib.clipping_0}/{vib.clipping_1}/{vib.clipping_2}")
        worst = max(vib.vibration_x, vib.vibration_y, vib.vibration_z)
        note(OK if worst < 30 else WARN, f"最大震動 {worst:.1f}（<30 佳，>60 需處理）")

    # ---------------- 電池 / RC / 感測器 ----------------
    print("\n【電力・RC・感測器】")
    bat = latest.get("BATTERY_STATUS")
    if bat:
        v = sum(x for x in bat.voltages[:12] if 0 < x < 65535) / 1000.0
        print(f"  電池 {v:.2f} V   剩餘 {bat.battery_remaining}%   電流 {bat.current_battery/100:.1f} A")
        note(OK if bat.battery_remaining >= 30 else WARN,
             f"剩餘電量 {bat.battery_remaining}%")
    ss = latest.get("SYS_STATUS")
    if ss:
        print(f"  5V 航電軌 —— 見下方 STATUSTEXT（PX4 用它報告 Avionics Power）")
        sensors = [(1 << 0, "陀螺儀"), (1 << 1, "加速度計"), (1 << 2, "磁力計"),
                   (1 << 3, "氣壓計"), (1 << 5, "GPS"), (1 << 15, "馬達輸出"),
                   (1 << 16, "RC 接收機"), (1 << 22, "預飛檢查")]
        bad = [n for b, n in sensors
               if (ss.onboard_control_sensors_present & b)
               and not (ss.onboard_control_sensors_health & b)]
        note(OK if not bad else BAD, f"感測器健康：{'全部正常' if not bad else '異常 ' + '、'.join(bad)}")
    rc = latest.get("RC_CHANNELS")
    if rc:
        print(f"  RC 訊號強度 {rc.rssi}   通道數 {rc.chancount}")
        note(OK if rc.chancount >= 8 else WARN, f"RC 通道數 {rc.chancount}")

    # ---------------- 串流頻率：要的有沒有真的來 ----------------
    print("\n【訊息串流】")
    expect = {"ATTITUDE": 10, "GLOBAL_POSITION_INT": 4, "GPS_RAW_INT": 2, "SYS_STATUS": 2}
    for name, hz in expect.items():
        got = counts.get(name, 0) / dur
        ratio = got / hz if hz else 1
        print(f"  {name:<22} 要求 {hz:>2} Hz   實得 {got:5.1f} Hz")
        note(OK if ratio > 0.5 else WARN,
             f"{name} 實得 {got:.1f} Hz（要求 {hz}）{'' if ratio > 0.5 else ' —— 頻寬不足或未生效'}")

    if texts:
        print("\n【飛控訊息】")
        sev = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR", 4: "WARN", 5: "NOTICE", 6: "INFO"}
        for s, txt in texts:
            print(f"  [{sev.get(s, s):6}] {txt}")
            if s <= 4:
                note(BAD if s <= 3 else WARN, f"飛控警告：{txt}")

    print("\n" + "=" * 68 + "\n【體檢結論】")
    for level in (BAD, WARN, OK):
        for lv, text in findings:
            if lv == level:
                print(f"  {lv} {text}")
    n_bad = sum(1 for lv, _ in findings if lv == BAD)
    n_warn = sum(1 for lv, _ in findings if lv == WARN)
    print(f"\n  嚴重 {n_bad}｜提醒 {n_warn}｜正常 {len(findings)-n_bad-n_warn}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
