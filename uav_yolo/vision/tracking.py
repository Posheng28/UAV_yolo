"""穩定 ID 指派：把每幀的偵測框對應到「跨幀不變」的識別碼。

為什麼不直接用 ultralytics 追蹤器給的 ID
-------------------------------------------------
BoT-SORT/ByteTrack 的新軌跡第一幀是 unconfirmed 狀態，`results.boxes.id`
為 None，必須連續配對成功才會轉正並發出 ID。目標移動快時幀間位移大、
IoU 配對失不到，於是每幀都退回 unconfirmed → 永遠拿不到 ID。

實測（使用者實際蒐集的 200 幀移動序列，weights/toycar.pt、conf 0.55）：
    model.predict 偵測到車          128 幀 (64%)
    model.track   給得出 track id     5 幀 ( 2%)
    → 偵測到但沒 ID                 123 幀
舊版 detect() 是「沒 ID 就 continue 丟掉」，所以那 123 幀畫面上完全沒有框——
這正是「車一移動框就消失、停下來才偵測得到」的成因（車停 → 位移小 → 配對
成功 → 軌跡轉正 → 框回來）。

而且追蹤器每次重建軌跡都換一個新號碼，鎖定用的 `locked_id` 會指向再也不會
出現的死號 → 目標重新被偵測到也鎖不回去、框從紅色掉回綠色。

所以識別由本層自己做：以「上一幀位置 + 速度外推」預測下一幀位置，用相對於
目標尺寸的距離當門檻（大目標容許大位移、小目標嚴格），並保留短暫失聯記憶，
讓同一個 ID 撐過偵測空窗。純邏輯、不碰影像，可完整單元測試。
"""

from __future__ import annotations

from dataclasses import dataclass, field

Bbox = tuple[float, float, float, float]  # x1, y1, x2, y2


def _centre_size(bbox: Bbox) -> tuple[float, float, float, float]:
    x1, y1, x2, y2 = bbox
    return (x1 + x2) / 2.0, (y1 + y2) / 2.0, max(1e-6, x2 - x1), max(1e-6, y2 - y1)


@dataclass
class _Track:
    sid: int
    cx: float
    cy: float
    w: float
    h: float
    vx: float = 0.0          # 像素/秒
    vy: float = 0.0
    t: float = 0.0
    missed: int = 0
    hits: int = 1

    @property
    def radius(self) -> float:
        """目標「半徑」：門檻以此為尺度，讓大小目標用同一組參數。"""
        return 0.5 * (self.w + self.h) / 2.0

    def predict(self, t: float) -> tuple[float, float]:
        dt = max(0.0, min(t - self.t, 1.0))  # 夾住：長時間空窗不要外推到天邊
        return self.cx + self.vx * dt, self.cy + self.vy * dt


@dataclass
class StableIdAssigner:
    """把 (bbox, 類別) 的序列指派成穩定 ID。

    參數都以「目標尺寸的倍數」表示，因此不隨解析度或飛行高度失準。
    """

    gate_radii: float = 4.0      # 允許的預測誤差，單位＝目標半徑；4 倍半徑 ≈ 兩個身長
    size_ratio_max: float = 3.0  # 前後框大小差超過此倍數視為不同物體
    max_missed: int = 45         # 連續幾幀沒配到就淘汰該 ID（30fps ≈ 1.5 秒）
    vel_alpha: float = 0.5       # 速度低通；太高會被單幀雜訊帶偏

    _tracks: list[_Track] = field(default_factory=list, init=False)
    _next_sid: int = field(default=1, init=False)

    def reset(self) -> None:
        self._tracks.clear()
        self._next_sid = 1

    def assign(self, boxes: list[Bbox], t: float) -> list[int]:
        """回傳與 boxes 等長、順序對應的穩定 ID 串列。"""
        dets = [_centre_size(b) for b in boxes]

        # 成本 = 預測位置到偵測中心的距離 ÷ 目標半徑（無因次）
        pairs: list[tuple[float, int, int]] = []
        for ti, tr in enumerate(self._tracks):
            px, py = tr.predict(t)
            for di, (cx, cy, w, h) in enumerate(dets):
                ratio = max(w / tr.w, tr.w / w, h / tr.h, tr.h / h)
                if ratio > self.size_ratio_max:
                    continue
                dist = ((cx - px) ** 2 + (cy - py) ** 2) ** 0.5
                cost = dist / max(tr.radius, 1e-6)
                # 空窗越久預測越不可信 → 門檻隨失聯幀數放寬，才接得回來
                if cost > self.gate_radii * (1.0 + 0.1 * tr.missed):
                    continue
                pairs.append((cost, ti, di))

        pairs.sort(key=lambda p: p[0])
        used_t: set[int] = set()
        used_d: set[int] = set()
        matched: dict[int, int] = {}  # det index -> track index
        for _cost, ti, di in pairs:
            if ti in used_t or di in used_d:
                continue
            used_t.add(ti)
            used_d.add(di)
            matched[di] = ti

        sids: list[int] = []
        for di, (cx, cy, w, h) in enumerate(dets):
            ti = matched.get(di)
            if ti is None:
                tr = _Track(sid=self._next_sid, cx=cx, cy=cy, w=w, h=h, t=t)
                self._next_sid += 1
                self._tracks.append(tr)
                used_t.add(len(self._tracks) - 1)
            else:
                tr = self._tracks[ti]
                dt = t - tr.t
                if dt > 1e-6:
                    a = self.vel_alpha
                    tr.vx = (1 - a) * tr.vx + a * (cx - tr.cx) / dt
                    tr.vy = (1 - a) * tr.vy + a * (cy - tr.cy) / dt
                tr.cx, tr.cy, tr.w, tr.h, tr.t = cx, cy, w, h, t
                tr.missed = 0
                tr.hits += 1
            sids.append(tr.sid)

        # 本幀沒配到的軌跡：記一次失聯，超時淘汰（保留記憶才能撐過偵測空窗）
        keep: list[_Track] = []
        for ti, tr in enumerate(self._tracks):
            if ti not in used_t:
                tr.missed += 1
                if tr.missed > self.max_missed:
                    continue
            keep.append(tr)
        self._tracks = keep
        return sids
