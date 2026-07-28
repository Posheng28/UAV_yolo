"""解析 PX4 .ulg：回答「模式為什麼不可用」與「哪一項 GPS 檢查在失敗」。

不猜、不推論——直接讀飛控自己記下的旗標：
    estimator_gps_status : 十項 GNSS 檢查各自的 check_fail_* 布林
    failsafe_flags       : 哪一個條件讓需要位置的模式變成不可用
    vehicle_status       : nav_state 隨時間變化（模式實際切到哪）

用法：
    python tools/analyse_ulog.py <檔案.ulg> [更多檔案...]
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from pyulog import ULog

GPS_CHECKS = [
    ("check_fail_gps_fix", "定位類型不足"),
    ("check_fail_min_sat_count", "衛星數不足"),
    ("check_fail_max_pdop", "PDOP 超標 ★"),
    ("check_fail_max_horiz_err", "水平精度超標"),
    ("check_fail_max_vert_err", "垂直精度超標"),
    ("check_fail_max_spd_err", "速度精度超標"),
    ("check_fail_max_horiz_drift", "水平漂移超標"),
    ("check_fail_max_vert_drift", "垂直漂移超標"),
    ("check_fail_max_horiz_spd_err", "水平速度誤差超標"),
    ("check_fail_max_vert_spd_err", "垂直速度誤差超標"),
    ("check_fail_spoofed", "疑似欺騙訊號"),
]

FAILSAFE_FLAGS = [
    ("global_position_invalid", "全域位置無效 → Hold 不可用"),
    ("local_position_invalid", "本地位置無效 → Position 不可用"),
    ("local_position_invalid_relaxed", "本地位置無效(寬鬆) → Position 不可用"),
    ("local_altitude_invalid", "本地高度無效"),
    ("attitude_invalid", "姿態無效"),
    ("manual_control_signal_lost", "遙控訊號遺失 → Position 不可用"),
    ("gcs_connection_lost", "地面站連線遺失"),
    ("battery_low_remaining_time", "電量剩餘時間不足"),
    ("battery_unhealthy", "電池不健康"),
    ("home_position_invalid", "Home 位置無效"),
    ("wind_limit_exceeded", "風速超限"),
    ("flight_time_limit_exceeded", "飛行時間超限"),
    ("position_accuracy_low", "位置精度低 → 可能觸發 RTL"),
]

NAV_STATE = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "AUTO.MISSION", 4: "AUTO.LOITER(Hold)",
    5: "AUTO.RTL", 10: "ACRO", 12: "DESCEND", 13: "TERMINATION", 14: "OFFBOARD",
    15: "STAB", 17: "AUTO.TAKEOFF", 18: "AUTO.LAND", 19: "AUTO.FOLLOW",
    20: "AUTO.PRECLAND", 21: "ORBIT", 22: "AUTO.VTOL_TAKEOFF",
}


KEY_PARAMS = {
    "EKF2_HGT_REF": ("EKF 高度基準", {0: "氣壓計", 1: "GPS", 2: "測距儀", 3: "視覺"}),
    "EKF2_GPS_CTRL": ("GNSS 融合開關（0=完全不用 GPS）", None),
    "EKF2_GPS_CHECK": ("GNSS 檢查遮罩（245=1.15 / 1023=1.16 / 2047=1.17）", None),
    "EKF2_REQ_PDOP": ("PDOP 上限", None),
    "COM_POS_FS_EPH": ("Hold 的 eph 門檻 m", None),
    "COM_ARM_WO_GPS": ("無 GPS 可否解鎖", None),
}


def report_params(ulog: ULog) -> None:
    """設定錯誤造成的模式不可用，看數值序列是看不出來的——要看參數。

    實例：EKF2_HGT_REF=2（測距儀）但機上沒有測距儀，於是 alt_valid 永遠 false、
    global_position_invalid 永遠成立、Hold 永遠不可用。所有 GNSS 檢查卻全綠。
    """
    p = ulog.initial_parameters
    print("\n【關鍵參數】")
    for name, (label, table) in KEY_PARAMS.items():
        v = p.get(name)
        if v is None:
            continue
        extra = ""
        if table is not None:
            extra = f"  → {table.get(int(v), '?')}"
            if name == "EKF2_HGT_REF" and int(v) == 2:
                extra += "  ⚠️ 沒裝測距儀就會讓 alt_valid 永遠 false → Hold 不可用"
        print(f"  {name:<16} {v:<10}{label}{extra}")


def get(ulog: ULog, name: str, idx: int = 0):
    for d in ulog.data_list:
        if d.name == name and d.multi_id == idx:
            return d
    return None


def spans(t: np.ndarray, flag: np.ndarray) -> list[tuple[float, float]]:
    """把布林序列變成 [(起, 迄)] 的 True 區間（秒）。"""
    out, start = [], None
    for i, v in enumerate(flag):
        if v and start is None:
            start = t[i]
        elif not v and start is not None:
            out.append((start, t[i])); start = None
    if start is not None:
        out.append((start, t[-1]))
    return out


def analyse(path: Path) -> None:
    u = ULog(str(path))
    t0 = u.start_timestamp
    print("=" * 72)
    print(f"{path.name}   時長 {(u.last_timestamp - t0)/1e6:.0f} 秒")
    print("=" * 72)

    report_params(u)

    # ---- 全域位置有效性：Hold 要的就是這個 ----
    gp = get(u, "vehicle_global_position")
    if gp:
        print("\n【全域位置有效性】（Hold 需要它，Position 不需要）")
        for key, label in (("lat_lon_valid", "經緯度"), ("alt_valid", "高度")):
            arr = gp.data.get(key)
            if arr is not None:
                pct = 100 * np.mean(arr.astype(bool))
                mark = "✅" if pct > 99 else "❌"
                print(f"  {mark} {label:<6} 有效比例 {pct:5.1f}%"
                      + ("" if pct > 99 else "  ← global_position_invalid 由此而來"))

    # ---- 模式歷程 ----
    vs = get(u, "vehicle_status")
    if vs:
        t = (vs.data["timestamp"] - t0) / 1e6
        nav = vs.data.get("nav_state")
        if nav is not None:
            changes = [0] + list(np.flatnonzero(np.diff(nav)) + 1)
            print("\n【飛行模式歷程】")
            for i in changes:
                print(f"  +{t[i]:6.1f}s  {NAV_STATE.get(int(nav[i]), f'狀態{int(nav[i])}')}")
        armed = vs.data.get("arming_state")
        if armed is not None:
            arm_spans = spans(t, armed == 2)
            if arm_spans:
                print("  解鎖區間：" + "、".join(f"{a:.0f}~{b:.0f}s" for a, b in arm_spans))

    # ---- GPS 檢查 ----
    gs = get(u, "estimator_gps_status")
    if gs:
        t = (gs.data["timestamp"] - t0) / 1e6
        print(f"\n【GNSS 品質檢查】（{len(t)} 筆取樣）")
        any_fail = False
        for key, label in GPS_CHECKS:
            arr = gs.data.get(key)
            if arr is None:
                continue
            fail = arr.astype(bool)
            n = int(fail.sum())
            if n:
                any_fail = True
                sp = spans(t, fail)
                pct = 100 * n / len(fail)
                shown = "、".join(f"{a:.0f}~{b:.0f}s" for a, b in sp[:6])
                print(f"  ❌ {label:<22} 失敗 {n:>5} 筆 ({pct:4.1f}%)  {shown}"
                      + (" …" if len(sp) > 6 else ""))
        if not any_fail:
            print("  ✅ 十一項檢查全程零失敗")

    # ---- failsafe 旗標 ----
    ff = get(u, "failsafe_flags")
    if ff:
        t = (ff.data["timestamp"] - t0) / 1e6
        print("\n【failsafe 旗標】（哪一項讓模式不可用）")
        any_on = False
        for key, label in FAILSAFE_FLAGS:
            arr = ff.data.get(key)
            if arr is None:
                continue
            on = arr.astype(bool)
            n = int(on.sum())
            if n:
                any_on = True
                sp = spans(t, on)
                shown = "、".join(f"{a:.0f}~{b:.0f}s" for a, b in sp[:6])
                print(f"  ⚠️  {label:<34} {n:>5} 筆  {shown}"
                      + (" …" if len(sp) > 6 else ""))
        if not any_on:
            print("  ✅ 全程沒有任何 failsafe 旗標被設立")

    # ---- 位置精度 ----
    lp = get(u, "vehicle_local_position")
    if lp and "eph" in lp.data:
        eph = lp.data["eph"]; epv = lp.data.get("epv")
        finite = eph[np.isfinite(eph)]
        if finite.size:
            print(f"\n【EKF 融合後位置精度】（Commander 真正用來判斷的量）")
            print(f"  eph  中位 {np.median(finite):.2f} m  最大 {finite.max():.2f} m"
                  f"   （COM_POS_FS_EPH 門檻 5.0 m）")
            if epv is not None:
                fv = epv[np.isfinite(epv)]
                if fv.size:
                    print(f"  epv  中位 {np.median(fv):.2f} m  最大 {fv.max():.2f} m")
            over = (finite > 5.0).sum()
            print(f"  超過 5 m 的取樣：{over} 筆 / {finite.size}"
                  + ("  ← Hold 會在這些時刻不可用" if over else "  ✅ 從未超過"))

    # ---- 實際 DOP ----
    gps = get(u, "vehicle_gps_position") or get(u, "sensor_gps")
    if gps and "hdop" in gps.data:
        h, v = gps.data["hdop"], gps.data.get("vdop")
        pdop = np.hypot(h, v) if v is not None else h
        print(f"\n【原始 DOP】（PX4 用 sqrt(hdop²+vdop²) 比對 EKF2_REQ_PDOP=2.5）")
        print(f"  HDOP 中位 {np.median(h):.2f} 最大 {h.max():.2f}"
              f"   VDOP 中位 {np.median(v):.2f} 最大 {v.max():.2f}" if v is not None else "")
        print(f"  合成 PDOP 中位 {np.median(pdop):.2f}  最大 {pdop.max():.2f}"
              f"   超過 2.5 的比例 {100*(pdop>2.5).mean():.1f}%")
        sats = gps.data.get("satellites_used")
        if sats is not None:
            print(f"  衛星數 中位 {int(np.median(sats))}  最少 {int(sats.min())}")
    print()


def main() -> int:
    if len(sys.argv) < 2:
        print(__doc__)
        return 1
    for arg in sys.argv[1:]:
        analyse(Path(arg))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
