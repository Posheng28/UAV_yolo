"""拆槳台架測試：驗證「飛控收得到、且看得懂」我們送的指令。

原理：PX4 對每個 COMMAND_LONG/COMMAND_INT 都會回一個 COMMAND_ACK。
      ACK 的 result 告訴我們飛控怎麼看待這個指令：
        ACCEPTED(0)          → 收到且執行
        TEMPORARILY_REJECTED(1)/DENIED(2) → **收到且看懂了，但當下條件不允許**（室內沒位置就是這個）
        UNSUPPORTED(3)       → 收到但不支援這個指令（代表我們用錯指令）
        FAILED(4)            → 收到但執行失敗
      **只要有 ACK 回來，就證明鏈路通、飛控看得懂。** 拒絕不代表壞掉。

安全：本腳本只送「查詢/設定串流/雲台指向」與「導引目標」類指令，
      **不會 arm、不會起飛、不會解鎖馬達**。仍請務必拆槳測試。

用法：
    python tools/bench_command_test.py --port COM10 --baud 115200
"""

from __future__ import annotations

import argparse
import time

RESULT_NAMES = {
    0: "ACCEPTED（接受）",
    1: "TEMPORARILY_REJECTED（暫時拒絕：條件未滿足）",
    2: "DENIED（拒絕：條件不允許）",
    3: "UNSUPPORTED（不支援此指令）",
    4: "FAILED（執行失敗）",
    5: "IN_PROGRESS（執行中）",
    6: "CANCELLED（已取消）",
}

# 指令代碼
CMD_REQUEST_MESSAGE = 512
CMD_SET_MESSAGE_INTERVAL = 511
CMD_DO_SET_ROI_LOCATION = 195
CMD_DO_SET_ROI_NONE = 197
CMD_DO_REPOSITION = 192
MSG_ID_AUTOPILOT_VERSION = 148

MAV_FRAME_GLOBAL_INT = 5


def wait_ack(conn, command: int, timeout: float = 3.0):
    """等這個指令的 ACK。回 (result_code, 說明) 或 None。"""
    end = time.time() + timeout
    while time.time() < end:
        msg = conn.recv_match(type="COMMAND_ACK", blocking=True, timeout=0.3)
        if msg and msg.command == command:
            return msg.result, RESULT_NAMES.get(msg.result, f"未知({msg.result})")
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="拆槳台架：驗證飛控收得到/看得懂指令")
    ap.add_argument("--port", default="COM10", help="送指令用的埠（LR24 地面端）")
    ap.add_argument("--baud", type=int, default=115200)
    args = ap.parse_args()

    from pymavlink import mavutil

    print(f">>> 連線 {args.port} @ {args.baud} …")
    conn = mavutil.mavlink_connection(
        args.port, baud=args.baud, source_system=255, source_component=190
    )

    hb = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=8)
    if hb is None:
        print("!! 收不到心跳：檢查電台/飛控供電、埠與鮑率")
        return 1
    sysid, compid = hb.get_srcSystem(), hb.get_srcComponent()
    print(f"[心跳] sys={sysid} comp={compid} type={hb.type} autopilot={hb.autopilot}")
    print(f"       模式 custom_mode={hb.custom_mode}\n")

    def cmd_long(command, *params, label=""):
        p = list(params) + [0.0] * (7 - len(params))
        conn.mav.command_long_send(sysid, compid, command, 0, *[float(x) for x in p])
        ack = wait_ack(conn, command)
        status = "無 ACK（飛控沒回應）" if ack is None else f"result={ack[0]} {ack[1]}"
        ok = "OK " if ack is not None else "!! "
        print(f"{ok}[{label or command}] → {status}")
        return ack

    def cmd_int(frame, command, p1, p2, p3, p4, x, y, z, label=""):
        conn.mav.command_int_send(
            sysid, compid, frame, command, 0, 0,
            float(p1), float(p2), float(p3), float(p4), int(x), int(y), float(z),
        )
        ack = wait_ack(conn, command)
        status = "無 ACK（飛控沒回應）" if ack is None else f"result={ack[0]} {ack[1]}"
        ok = "OK " if ack is not None else "!! "
        print(f"{ok}[{label or command}] → {status}")
        return ack

    print("=" * 66)
    print("測試 1｜最無害的查詢：要飛控回報自己的版本")
    print("  有 ACK = 鏈路雙向通、飛控看得懂指令")
    print("=" * 66)
    cmd_long(CMD_REQUEST_MESSAGE, MSG_ID_AUTOPILOT_VERSION, label="REQUEST_MESSAGE(版本)")
    ver = conn.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=3)
    if ver:
        fv = ver.flight_sw_version
        print(f"    → 飛控回報版本：PX4 {(fv >> 24) & 0xFF}.{(fv >> 16) & 0xFF}.{(fv >> 8) & 0xFF}")

    print("\n" + "=" * 66)
    print("測試 2｜設定訊息串流（我們平常在用的）")
    print("=" * 66)
    cmd_long(CMD_SET_MESSAGE_INTERVAL, 33, 200000, label="SET_MESSAGE_INTERVAL(位置 5Hz)")
    cmd_long(CMD_SET_MESSAGE_INTERVAL, 30, 100000, label="SET_MESSAGE_INTERVAL(姿態 10Hz)")

    print("\n" + "=" * 66)
    print("測試 3｜雲台指向 ROI（追蹤時用來讓雲台鎖住目標）")
    print("  不會動馬達；沒裝雲台可能回 UNSUPPORTED/DENIED，屬正常")
    print("=" * 66)
    pos = conn.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
    if pos and pos.lat != 0:
        lat, lon = pos.lat, pos.lon
        print(f"    （用目前位置附近當 ROI：{lat/1e7:.6f}, {lon/1e7:.6f}）")
    else:
        lat, lon = int(24.7870 * 1e7), int(120.9970 * 1e7)
        print(f"    （無位置遙測，用交大附近固定座標當 ROI）")
    cmd_int(MAV_FRAME_GLOBAL_INT, CMD_DO_SET_ROI_LOCATION, 0, 0, 0, 0,
            lat, lon, 50.0, label="DO_SET_ROI_LOCATION")
    time.sleep(0.5)
    cmd_long(CMD_DO_SET_ROI_NONE, label="DO_SET_ROI_NONE(取消)")

    print("\n" + "=" * 66)
    print("測試 4｜★ 導引目標 DO_REPOSITION（追蹤系統真正用來下令飛的指令）")
    print("  拆槳＋室內：預期被拒（沒有效位置），**被拒也是成功的證明**")
    print("  → 有 ACK 就代表飛控收到且看得懂；等有 GPS 位置就會 ACCEPTED")
    print("=" * 66)
    cmd_int(MAV_FRAME_GLOBAL_INT, CMD_DO_REPOSITION,
            -1, 1, 0, float("nan"), lat, lon, 30.0, label="DO_REPOSITION(目標點)")

    print("\n" + "=" * 66)
    print("判讀：")
    print("  每項都有 result=... → 飛控**收得到也看得懂**，鏈路完全正常。")
    print("  DENIED/TEMPORARILY_REJECTED = 條件不足（室內無位置），不是壞掉。")
    print("  UNSUPPORTED = 那個指令飛控不支援（雲台類常見，無雲台時正常）。")
    print("  無 ACK = 沒收到 → 才是鏈路問題。")
    print("=" * 66)
    conn.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
