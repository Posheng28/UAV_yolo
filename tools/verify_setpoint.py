"""驗證「Pixhawk 讀懂了指令點」——讀回它內部的目標點逐欄位對帳。

為什麼 COMMAND_ACK 不夠
------------------------------------------------
ACCEPTED 只代表「指令被收下」，不代表「參數被照你的意思解讀」。
本專案實際踩過：用相對高度座標框送 4m，PX4 回 ACCEPTED——但它把 param7
一律當 AMSL，實際上是「飛到海拔 4 公尺＝往地下 109m」。ACK 騙過了我們。

真正的判準是 PX4 的 **POSITION_TARGET_GLOBAL_INT**（訊息 87）：navigator
解析完指令後，會把「我現在要飛去哪」的內部 setpoint 廣播出來。
讀回它、跟我們發的逐欄位比對，一致才算「讀懂」。

用法（拆槳！飛機通電、LR24 接上）：
    python tools/verify_setpoint.py --port COM10
    python tools/verify_setpoint.py --north 5 --alt 6   # 自訂偏移與高度

安全：未解鎖時 PX4 只會記下目標點不會移動；若偵測到已解鎖會直接中止，
除非明確加 --allow-armed（那代表飛機真的會飛過去，確保空域淨空）。
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from pymavlink import mavutil

from uav_yolo.geometry import GeoRef
from uav_yolo.mavlink_io.telemetry import MavlinkConnection

ACK = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED", 3: "UNSUPPORTED",
       4: "FAILED", 5: "IN_PROGRESS"}
MSG_POSITION_TARGET_GLOBAL_INT = 87


def main() -> int:
    ap = argparse.ArgumentParser(description="DO_REPOSITION 讀回對帳")
    ap.add_argument("--port", default="COM10")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--north", type=float, default=3.0, help="目標點在目前位置北方幾公尺")
    ap.add_argument("--east", type=float, default=0.0)
    ap.add_argument("--alt", type=float, default=5.0, help="相對 home 高度 m")
    ap.add_argument("--allow-armed", action="store_true")
    args = ap.parse_args()

    # 用專案自己的連線類別——測的必須是真實程式路徑，不是另寫的簡化版
    # （上次就是簡化版漏了 CHANGE_MODE 旗標，得出錯誤結論）
    link = MavlinkConnection(args.port, args.baud, stream_rates={
        "GLOBAL_POSITION_INT": 4, "ATTITUDE": 5, "EXTENDED_SYS_STATE": 1,
    })
    link.start()
    print(f">>> 連線 {args.port}，等遙測…")
    t0 = time.time()
    while time.time() - t0 < 12:
        if link.store.position_at(time.monotonic()) and link.store.home:
            break
        time.sleep(0.3)
    pos = link.store.position_at(time.monotonic())
    home = link.store.home
    hb = link.store.heartbeat
    if pos is None or home is None:
        print("!! 12 秒內沒收齊位置/home；確認飛機通電、GPS 有 fix")
        link.stop()
        return 1

    if hb and hb.armed and not args.allow_armed:
        print("!! 飛機已解鎖——此工具會下真實的移動指令。拆槳、上鎖後再跑，"
              "或明確加 --allow-armed（飛機會真的飛過去）")
        link.stop()
        return 1

    # 跟 engine._run_guidance 完全相同的換算路徑
    georef = GeoRef(home.lat, home.lon)
    veh_ne = georef.lla_to_ned(pos.lat, pos.lon, pos.rel_alt)
    import numpy as np

    target_ne = np.array([veh_ne[0] + args.north, veh_ne[1] + args.east, 0.0])
    lat, lon, _ = georef.ned_to_lla(target_ne)
    alt_amsl = home.alt_amsl + args.alt

    print(f"\n目前位置  {pos.lat:.7f}, {pos.lon:.7f}  相對高度 {pos.rel_alt:.1f} m")
    print(f"home 海拔 {home.alt_amsl:.1f} m")
    print(f"發送目標  北 {args.north:+.1f}m / 東 {args.east:+.1f}m / 相對高度 {args.alt:.1f}m")
    print(f"  → lat {lat:.7f}  lon {lon:.7f}  AMSL {alt_amsl:.1f} m")

    # 要求 PX4 廣播內部 setpoint
    link._command_long(511, MSG_POSITION_TARGET_GLOBAL_INT, 5e5)

    link.last_ack.pop(192, None)
    t_sent = time.monotonic()
    link.send_reposition(lat, lon, alt_amsl, alt_rel_m=args.alt)
    print("\n>>> DO_REPOSITION 已送出，等 ACK 與 setpoint 讀回…")

    ack_result = None
    sp = None
    t1 = time.time()
    while time.time() - t1 < 10 and (ack_result is None or sp is None):
        if 192 in link.last_ack and ack_result is None:
            ack_result = link.last_ack[192][0]
            print(f"  COMMAND_ACK: {ACK.get(ack_result, ack_result)}")
        # newer_than=t_sent：送出前的 setpoint 是舊狀態，拿它對帳是假陽性
        raw = link.poll_raw("POSITION_TARGET_GLOBAL_INT", newer_than=t_sent)
        if raw is not None:
            sp = raw
        time.sleep(0.1)

    hb = link.store.heartbeat
    print(f"  飛控模式  : {hb.mode if hb else '?'}")
    link.stop()

    print("\n" + "=" * 64)
    if sp is None:
        print("❌ 10 秒內沒讀到 POSITION_TARGET_GLOBAL_INT。")
        print("   可能：指令被拒（看上面 ACK）、或韌體不廣播此訊息。")
        return 1

    sp_lat, sp_lon = sp.lat_int / 1e7, sp.lon_int / 1e7
    dn = (sp_lat - lat) * 111320.0
    de = (sp_lon - lon) * 111320.0 * math.cos(math.radians(lat))
    dalt = float(sp.alt) - alt_amsl
    print("【逐欄位對帳：我發的 vs PX4 內部目標點】")
    print(f"  lat   {lat:.7f}  vs  {sp_lat:.7f}   差 {dn:+.2f} m")
    print(f"  lon   {lon:.7f}  vs  {sp_lon:.7f}   差 {de:+.2f} m")
    print(f"  alt   {alt_amsl:.1f}  vs  {sp.alt:.1f} (AMSL)   差 {dalt:+.2f} m")
    pos_ok = abs(dn) < 1.0 and abs(de) < 1.0
    alt_ok = abs(dalt) < 2.0
    print()
    print(f"  {'✅' if pos_ok else '❌'} 水平位置：PX4 {'照收' if pos_ok else '解讀不一致！'}")
    print(f"  {'✅' if alt_ok else '❌'} 高度：PX4 {'照收' if alt_ok else '解讀不一致（基準錯了？）！'}")
    if pos_ok and alt_ok:
        print("\n  ✅ Pixhawk 讀懂了指令點，且內部目標＝我們要它去的地方。")
    return 0 if (pos_ok and alt_ok) else 1


if __name__ == "__main__":
    raise SystemExit(main())
