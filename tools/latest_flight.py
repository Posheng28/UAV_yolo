"""把最近一次任務記錄整理成一份可讀的覆盤報告。

    python tools/latest_flight.py              # 最新那一份
    python tools/latest_flight.py --list       # 列出全部
    python tools/latest_flight.py --pick 3     # 倒數第 3 份
    python tools/latest_flight.py --file data/missions/mission_20260804_102026.jsonl

為什麼要有這支：每次覆盤都在重寫同一批分析（狀態歷程、偵測率、指令速度、
閘門原因、影像健康度），而其中最有價值的一項——**反推 NED 原點，算出車子
相對飛機的偏移序列**——手算很容易出錯。2026-08-04 那次就是靠它才分辨出
「飛機在繞圈」而不是「車子跑太快」，光看座標序列完全看不出來。

原理：command 事件同時記了 (n,e) 與 (lat,lon)，兩者互為同一點的兩種表示，
所以可以解出 NED 原點；有了原點就能把每個 snap 的載具經緯度換算成 NED，
與目標估計相減得到偏移。實測反推誤差約 4 cm。
"""

from __future__ import annotations

import argparse
import collections
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
M_PER_DEG_LAT = 111320.0


def load(path: Path) -> list[dict]:
    rows = []
    for line in path.open(encoding="utf-8", errors="replace"):
        line = line.strip()
        if not line:
            continue
        try:
            rows.append(json.loads(line))
        except json.JSONDecodeError:
            pass          # 半行（當機時最後一行可能寫到一半）直接跳過
    return rows


def find_missions(cfg_dir: Path) -> list[Path]:
    return sorted(cfg_dir.glob("mission_*.jsonl"), key=lambda p: p.stat().st_mtime)


def ned_origin(cmds: list[dict]) -> tuple[float, float] | None:
    """由 command 同時記錄的 (n,e) 與 (lat,lon) 反推 NED 原點。"""
    for c in cmds:
        if all(k in c for k in ("lat", "lon", "n", "e")):
            lat0 = c["lat"] - c["n"] / M_PER_DEG_LAT
            lon0 = c["lon"] - c["e"] / (M_PER_DEG_LAT * math.cos(math.radians(c["lat"])))
            return lat0, lon0
    return None


def pct(n: int, d: int) -> str:
    return "—" if not d else f"{100.0 * n / d:.0f}%"


def median(xs):
    xs = sorted(xs)
    return xs[len(xs) // 2] if xs else None


def report(path: Path) -> None:
    rows = load(path)
    if not rows:
        print(f"{path.name}: 空檔或無法解析")
        return
    t0 = rows[0]["t"]
    span = rows[-1]["t"] - t0
    kinds = collections.Counter(r.get("event") for r in rows)
    snaps = [r for r in rows if r.get("event") == "snap"]
    cmds = [r for r in rows if r.get("event") == "command"]

    print("=" * 72)
    print(f"任務記錄  {path.name}")
    print(f"  牆鐘 {rows[0].get('wall')} → {rows[-1].get('wall')}   時長 {span / 60:.1f} 分鐘"
          f"   大小 {path.stat().st_size / 1e6:.1f} MB")
    on = next((r for r in rows if r.get("event") == "guidance_on"), None)
    if on:
        print(f"  載體 {on.get('airframe')}  跟隨高度 {on.get('follow_alt_m')}m  "
              f"退距 {on.get('standoff_m')}m  上限速度 {on.get('max_speed_ms')}m/s")
        print(f"  權重 {on.get('weights')}  切塊 {on.get('tiling')}")
    off = next((r for r in rows if r.get("event") == "guidance_off"), None)
    print(f"  結束：{off.get('reason') if off else '未關閉（記錄仍開著）'}"
          + (f"  共發指令 {off.get('commands_sent')} 筆" if off else ""))
    print(f"  事件：{dict(kinds)}")

    # ---- 影片 ----
    mp4 = path.with_suffix(".mp4")
    if mp4.exists():
        info = f"{mp4.stat().st_size / 1e6:.1f} MB"
        try:
            import cv2

            cap = cv2.VideoCapture(str(mp4))
            if cap.isOpened():
                n = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
                fps = cap.get(cv2.CAP_PROP_FPS) or 0
                w = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                h = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                info += f"，{w}x{h}，{n} 幀 @{fps:.0f}fps ≈ {n / fps:.0f}s" if fps else ""
            cap.release()
        except Exception:
            pass
        print(f"  ▶ 影片：{mp4.name}（{info}）")
    else:
        print("  ▶ 影片：無（這次沒錄到，或是錄影功能上線前的記錄）")

    if not snaps:
        print("  沒有 snap，無法進一步分析")
        return

    # 🔴 只看「真正在飛」的那一段。導引忘了關的話，後面幾小時的落地閒置
    # 會把所有比例稀釋掉（實例：160 分鐘的記錄裡實際飛行只有前 50 秒，
    # 偵測率被稀釋成 0%，看起來像整場都沒偵測到）。
    # 定義：從第一筆有目標估計的 snap，到最後一筆有偵測或有指令的時刻，
    # 再往後留 10 秒尾巴。
    def active_window(all_snaps, all_cmds):
        marks = [s["t"] for s in all_snaps if s.get("tgt") or (s.get("dets") or 0)]
        marks += [c["t"] for c in all_cmds]
        if not marks:
            return all_snaps, None
        lo, hi = min(marks), max(marks) + 10.0
        return [s for s in all_snaps if lo <= s["t"] <= hi], (lo - t0, hi - t0)

    snaps_all, snaps = snaps, None
    snaps, win = active_window(snaps_all, cmds)
    if win and (win[1] - win[0]) < 0.9 * span:
        print(f"\n⚠ 只分析實際作用時段 t+{win[0]:.0f}s ~ t+{win[1]:.0f}s"
              f"（{win[1] - win[0]:.0f} 秒），其餘 {span - (win[1] - win[0]):.0f} 秒是落地/閒置。"
              "\n  （導引結束後記得關掉，否則記錄會一直長）")
    if not snaps:
        snaps = snaps_all

    # ---- 狀態歷程 ----
    print("\n[狀態歷程]")
    prev = None
    for s in snaps:
        st = s.get("state")
        if st != prev:
            print(f"  t+{s['t'] - t0:7.1f}s {s.get('wall')}  {prev or '—'} → {st}"
                  f"   偵測數={s.get('dets')}")
            prev = st

    # ---- 偵測 ----
    dets = [s.get("dets") or 0 for s in snaps]
    print(f"\n[偵測] 有目標的 snap {pct(sum(1 for d in dets if d), len(dets))}"
          f"   每幀中位 {median(dets)}  最多 {max(dets)}"
          f"   （>1 表示同時有誤判，會搶鎖定）")

    # ---- 指令 ----
    if cmds:
        sp = [c["speed"] for c in cmds if c.get("speed") is not None]
        al = [c["alt_rel"] for c in cmds if c.get("alt_rel") is not None]
        acks = collections.Counter(
            r.get("result") for r in rows if r.get("event") == "command_ack")
        act_span = (win[1] - win[0]) if win else span
        print(f"\n[指令] {len(cmds)} 筆，作用時段內平均每 {act_span / max(len(cmds), 1):.1f} 秒一筆")
        if sp:
            print(f"  下達速度 m/s：中位 {median(sp):.1f}  最大 {max(sp):.1f}")
        if al:
            print(f"  指令高度 m：中位 {median(al):.1f}  範圍 {min(al):.1f}~{max(al):.1f}")
        if acks:
            print(f"  飛控回應：{dict(acks)}")
            bad = sum(n for k, n in acks.items() if k not in ("ACCEPTED", "IN_PROGRESS"))
            if bad:
                print(f"  ⚠ 有 {bad} 筆被飛控拒收")
        else:
            print("  飛控回應：無記錄（舊版記錄沒有 command_ack，或指令從未被 ACK）")
    else:
        print("\n[指令] 一筆都沒發")

    # ---- 閘門 ----
    gc = collections.Counter()
    for s in snaps:
        for g in (s.get("gates") or []):
            gc[g] += 1
    if gc:
        print("\n[閘門阻擋原因]")
        for g, n in gc.most_common(6):
            print(f"  {pct(n, len(snaps)):>4} 的時間  {g[:60]}")

    # ---- 影像健康 ----
    vids = [s["video"] for s in snaps if s.get("video")]
    if vids and len(vids[0]) >= 2:
        fps = [v[1] for v in vids if v[1] is not None]
        offline = sum(1 for v in vids if not v[0])
        line = f"\n[影像] 未連線 {pct(offline, len(vids))} 的時間"
        if fps:
            line += f"   fps 中位 {median(fps):.1f}"
        if len(vids[0]) >= 6:
            reo = [v[4] for v in vids if v[4] is not None]
            lum = [v[5] for v in vids if v[5] is not None]
            if reo:
                line += f"   期間重開擷取 {reo[-1] - reo[0]} 次"
            if lum:
                line += f"   平均亮度中位 {median(lum):.0f}"
        print(line)

    # ---- 🔴 相對偏移：飛機到底有沒有追上 ----
    origin = ned_origin(cmds)
    if origin and any(s.get("veh") and s.get("tgt") for s in snaps):
        lat0, lon0 = origin
        cosl = math.cos(math.radians(lat0))
        offs = []
        for s in snaps:
            veh, tgt = s.get("veh"), s.get("tgt")
            if not veh or not tgt or veh[0] is None:
                continue
            vn = (veh[0] - lat0) * M_PER_DEG_LAT
            ve = (veh[1] - lon0) * M_PER_DEG_LAT * cosl
            offs.append((s["t"] - t0, math.hypot(tgt[0] - vn, tgt[1] - ve),
                         veh[2] if len(veh) > 2 else None))
        if offs:
            d = [o[1] for o in offs]
            ds = sorted(d)
            alt = median([o[2] for o in offs if o[2] is not None]) or 0.0
            # 短軸半視野 ≈ 0.587×高度（實測自本專案內參）
            half = 0.587 * alt
            inside = sum(1 for x in d if x <= half)
            print(f"\n[追蹤品質] 目標離飛機的水平距離（反推 NED 原點求得）")
            print(f"  中位 {ds[len(ds) // 2]:.2f}m   p90 {ds[int(len(ds) * 0.9)]:.2f}m   "
                  f"最大 {max(d):.2f}m")
            print(f"  飛行高度中位 {alt:.1f}m → 短軸半視野約 {half:.2f}m")
            print(f"  目標落在畫面內的時間比例：{pct(inside, len(d))}"
                  + ("   ← 偏低代表飛機沒追上或在繞圈" if inside < 0.7 * len(d) else ""))


def trim(path: Path) -> None:
    """就地剪掉作用時段之後的閒置 snap（導引忘了關時，尾巴可以是好幾小時）。

    保留全部非 snap 事件（指令、ACK、影像事件、開關記錄），只砍 snap，
    所以覆盤需要的東西一個都不會少。
    """
    rows = load(path)
    if not rows:
        print(f"{path.name}: 空檔")
        return
    cmds = [r for r in rows if r.get("event") == "command"]
    marks = [r["t"] for r in rows
             if r.get("event") == "snap" and (r.get("tgt") or (r.get("dets") or 0))]
    marks += [c["t"] for c in cmds]
    if not marks:
        print(f"{path.name}: 找不到作用時段（沒有偵測也沒有指令），不動它")
        return
    hi = max(marks) + 10.0
    kept = [r for r in rows if r.get("event") != "snap" or r["t"] <= hi]
    if len(kept) == len(rows):
        print(f"{path.name}: 沒有多餘的閒置尾巴")
        return
    before = path.stat().st_size
    tmp = path.with_suffix(".jsonl.tmp")
    with tmp.open("w", encoding="utf-8") as f:
        for r in kept:
            f.write(json.dumps(r, ensure_ascii=False) + "\n")
    tmp.replace(path)                     # 原子置換，中途斷電不會留半個檔
    print(f"{path.name}: {len(rows)} → {len(kept)} 筆，"
          f"{before / 1e6:.2f} → {path.stat().st_size / 1e6:.2f} MB"
          f"（剪掉 {len(rows) - len(kept)} 筆閒置 snap）")


def main() -> None:
    if hasattr(sys.stdout, "reconfigure"):
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    ap = argparse.ArgumentParser(description="整理最近一次任務記錄")
    ap.add_argument("--dir", default=str(ROOT / "data" / "missions"))
    ap.add_argument("--file", default=None, help="指定某一份記錄")
    ap.add_argument("--list", action="store_true", help="列出全部")
    ap.add_argument("--pick", type=int, default=1, help="倒數第 N 份（預設 1＝最新）")
    ap.add_argument("--trim", action="store_true",
                    help="就地剪掉作用時段之後的閒置 snap（導引忘了關時用）")
    args = ap.parse_args()

    if args.file:
        (trim if args.trim else report)(Path(args.file))
        return

    missions = find_missions(Path(args.dir))
    if not missions:
        print(f"{args.dir} 底下找不到 mission_*.jsonl")
        return
    if args.list:
        for i, p in enumerate(reversed(missions), 1):
            mp4 = p.with_suffix(".mp4")
            print(f"  [{i:2d}] {p.name}  {p.stat().st_size / 1e6:6.1f}MB"
                  f"  {'＋影片' if mp4.exists() else '（無影片）'}")
        return
    if args.trim:
        for p in missions:            # 一次把所有肥掉的都剪乾淨
            trim(p)
        return
    report(missions[-args.pick])


if __name__ == "__main__":
    main()
