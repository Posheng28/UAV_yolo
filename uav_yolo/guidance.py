"""導引律：把目標估計轉成「載具該去哪」。

固定翼的核心觀念（解決「車急停、飛機飛過頭」）：
    永遠不叫飛機飛到目標頭上——那是旋翼思維。固定翼的指令是
    「以目標的預測位置為圓心、半徑 R 繞行」（standoff orbit）。
    PX4 收到 DO_REPOSITION 後自然以 loiter 圓實現；目標動，圓心跟著動；
    目標停，就繞著停點轉，不存在過頭問題。

旋翼則直接飛到目標上方/後方偏移點懸停跟隨。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from .estimation import TargetEstimator


@dataclass
class GuidanceCommand:
    """引擎會把 point_ne 轉成經緯度後用 DO_REPOSITION 發出。"""

    point_ne: np.ndarray          # 指令點（NED 的 N/E，公尺）
    alt_rel_m: float              # 相對 home 高度
    loiter_radius_m: float | None = None   # 固定翼繞行半徑（旋翼為 None）
    loiter_ccw: bool = False
    speed_ms: float | None = None # 地速上限；None = 用飛控預設
    label: str = ""


class MultirotorGuidance:
    """旋翼：跟在目標後方 standoff_m 的位置（standoff_m=0 則正上方）。"""

    MIN_SPEED_FOR_BEARING = 1.5  # m/s，低於此速度沿用上次方位，避免方向亂跳
    SPEED_MARGIN = 1.5           # 指令速度＝目標速度的幾倍（要追得上又不過衝）
    SPEED_FLOOR_MS = 1.0         # 目標靜止時的最低移動速度

    def __init__(self, follow_alt_m: float, standoff_m: float,
                 max_speed_ms: float = 5.0):
        self.follow_alt_m = float(follow_alt_m)
        self.standoff_m = float(standoff_m)
        # 指令速度上限。不設的話 PX4 用 MPC_XY_CRUISE，低空做小距離修正會衝到
        # 約 3m/s、傾角約 17 度——**而傾角會把相機視軸甩開，相機幾何正是
        # 目標座標的來源**。所以速度不是舒適度問題，是精度問題。
        self.max_speed_ms = float(max_speed_ms)
        self._last_bearing = math.pi  # 預設從南側接近（bearing 指「目標→載具」方向）

    def _command_speed(self, target_speed: float) -> float | None:
        """指令速度跟著目標速度走，而不是寫死一個值。

        寫死太慢 → 追不上會動的車；寫死太快 → 靜止目標的小修正也全速衝，
        機身一傾斜相機就甩開。跟著目標速度給，兩種情境都合理。
        """
        if self.max_speed_ms <= 0:
            return None                    # 0 = 交給飛控預設
        want = target_speed * self.SPEED_MARGIN + self.SPEED_FLOOR_MS
        return min(max(want, self.SPEED_FLOOR_MS), self.max_speed_ms)

    def compute(self, est: TargetEstimator) -> GuidanceCommand:
        target = est.predict_ahead(1.0)  # 旋翼反應快，輕微前置即可
        if self.standoff_m > 0.0:
            if est.speed >= self.MIN_SPEED_FOR_BEARING:
                vel = est.vel_ne
                self._last_bearing = math.atan2(-vel[1], -vel[0])  # 目標速度反方向=後方
            offset = self.standoff_m * np.array(
                [math.cos(self._last_bearing), math.sin(self._last_bearing)]
            )
            point = target + offset
        else:
            point = target
        return GuidanceCommand(
            point_ne=point,
            alt_rel_m=self.follow_alt_m,
            speed_ms=self._command_speed(est.speed),
            label="follow" if self.standoff_m > 0 else "overhead",
        )


class FixedWingGuidance:
    """固定翼：standoff 繞行，圓心 = KF 預測位置（前置 lead_time_s 秒）。"""

    def __init__(self, orbit_radius_m: float, orbit_dir: str, lead_time_s: float, alt_m: float):
        self.orbit_radius_m = float(orbit_radius_m)
        self.loiter_ccw = str(orbit_dir).lower() == "ccw"
        self.lead_time_s = float(lead_time_s)
        self.alt_m = float(alt_m)

    def compute(self, est: TargetEstimator) -> GuidanceCommand:
        center = est.predict_ahead(self.lead_time_s)
        return GuidanceCommand(
            point_ne=center,
            alt_rel_m=self.alt_m,
            loiter_radius_m=self.orbit_radius_m,
            loiter_ccw=self.loiter_ccw,
            label="standoff-orbit",
        )


def build_guidance(airframe: str, cfg_guidance: dict):
    """依載體種類建對應導引律（UI 切換「四軸/固定翼」就是切這裡）。"""
    if airframe == "fixedwing":
        fw = cfg_guidance.get("fixedwing", {})
        return FixedWingGuidance(
            orbit_radius_m=fw.get("orbit_radius_m", 150.0),
            orbit_dir=fw.get("orbit_dir", "cw"),
            lead_time_s=fw.get("lead_time_s", 3.0),
            alt_m=fw.get("alt_m", 80.0),
        )
    mc = cfg_guidance.get("multirotor", {})
    return MultirotorGuidance(
        follow_alt_m=mc.get("follow_alt_m", 40.0),
        standoff_m=mc.get("standoff_m", 15.0),
        max_speed_ms=mc.get("max_speed_ms", 5.0),
    )
