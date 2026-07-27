"""拆槳台架：遠端切換 PX4 飛行模式，驗證雙向控制與模式語意。

送 MAV_CMD_DO_SET_MODE 依序切 Altitude → Hold → Position，
每段等 COMMAND_ACK ＋讀心跳確認模式真的變了。
不 arm、不碰馬達；仍請拆槳測試。

用法：python tools/bench_mode_test.py --port COM10 --baud 115200
"""

from __future__ import annotations

import argparse
import time

CMD_DO_SET_MODE = 176
MODE_FLAG_CUSTOM = 1  # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED

# (顯示名, PX4 main_mode, sub_mode)；QGC 顯示名對照
SEQUENCE = [
    ("Altitude（定高）", 2, 0),
    ("Hold（定點盤旋）", 4, 3),   # AUTO.LOITER
    ("Position（位置）", 3, 0),
]

MAIN_NAMES = {1: "Manual", 2: "Altitude", 3: "Position", 4: "AUTO", 5: "Acro",
              6: "Offboard", 7: "Stabilized"}
AUTO_SUB = {2: "Takeoff", 3: "Hold", 4: "Mission", 5: "RTL", 6: "Land"}

RESULTS = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED", 3: "UNSUPPORTED", 4: "FAILED"}


def mode_name(custom_mode: int) -> str:
    main = (custom_mode >> 16) & 0xFF
    sub = (custom_mode >> 24) & 0xFF
    if main == 4:
        return f"AUTO.{AUTO_SUB.get(sub, sub)}"
    return MAIN_NAMES.get(main, f"main={main}")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    from pymavlink import mavutil

    conn = mavutil.mavlink_connection(args.port, baud=args.baud,
                                      source_system=255, source_component=190)
    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=8)
    if hb is None:
        print("!! 收不到心跳")
        return 1
    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
    print(f"[起點] 目前模式：{mode_name(hb.custom_mode)}\n")

    for label, main_mode, sub_mode in SEQUENCE:
        print(f">>> 切換到 {label} …")
        conn.mav.command_long_send(
            sysid, compid, CMD_DO_SET_MODE, 0,
            float(MODE_FLAG_CUSTOM), float(main_mode), float(sub_mode), 0, 0, 0, 0,
        )
        # 等 ACK
        ack_txt = "無 ACK"
        end = time.time() + 3
        while time.time() < end:
            m = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.3)
            if m and m.command == CMD_DO_SET_MODE:
                ack_txt = RESULTS.get(m.result, str(m.result))
                break
        # 等心跳反映新模式
        newmode = "?"
        end = time.time() + 3
        while time.time() < end:
            m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=0.5)
            if m and m.get_srcComponent() == 1:
                newmode = mode_name(m.custom_mode)
                target_main = (m.custom_mode >> 16) & 0xFF
                if target_main == main_mode:
                    break
        print(f"    ACK={ack_txt}｜心跳回報模式：{newmode}")
        print("    ← 看另一台 QGC 左上角，應顯示同名模式\n")
        time.sleep(2.0)  # 給使用者看一眼

    print("完成。最終模式應為 Position。")
    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
