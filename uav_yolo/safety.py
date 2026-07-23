"""安全閘門：每個導引指令發出前的最後防線。

承襲並修正舊系統的規格：
    - 數傳斷線停止發送（保留）
    - 指令速率上限（保留）
    - 飛行員接管即停發（保留，但改用 PX4 正確模式判斷 + 閂鎖：
      一旦接管，即使切回允許模式也不自動恢復，須在 UI 重新啟用）
    - 新增：home 距離圍欄、高度上下限、目標估計逾時
所有未通過的閘門都會回報原因字串，UI 直接顯示「為什麼現在沒在發指令」。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class GateReport:
    ok: bool
    blocked: list[str] = field(default_factory=list)


class SafetyGates:
    def __init__(self, cfg_safety: dict, rate_hz: float):
        self.link_timeout_s = float(cfg_safety.get("link_timeout_s", 2.0))
        self.allowed_modes = set(cfg_safety.get("allowed_modes", ["AUTO.LOITER"]))
        self.max_cmd_distance_m = float(cfg_safety.get("max_cmd_distance_m", 500.0))
        self.min_cmd_alt_m = float(cfg_safety.get("min_cmd_alt_m", 20.0))
        self.max_cmd_alt_m = float(cfg_safety.get("max_cmd_alt_m", 120.0))
        self.min_interval_s = 1.0 / max(float(rate_hz), 0.1)

        self.pilot_override_latched = False
        self._last_mode: str | None = None
        self._last_send_t: float | None = None

    # ---- 狀態維護 ----

    def observe_mode(self, mode: str | None, guidance_enabled: bool) -> None:
        """每個週期呼叫：偵測「曾在允許模式 → 離開」的接管動作並閂鎖。"""
        if (
            guidance_enabled
            and mode is not None
            and self._last_mode in self.allowed_modes
            and mode not in self.allowed_modes
        ):
            self.pilot_override_latched = True
        self._last_mode = mode

    def reset_override(self) -> None:
        """UI 重新啟用導引時呼叫。"""
        self.pilot_override_latched = False

    def mark_sent(self, now: float) -> None:
        self._last_send_t = now

    def clamp_alt(self, alt_rel_m: float) -> float:
        return min(max(alt_rel_m, self.min_cmd_alt_m), self.max_cmd_alt_m)

    # ---- 評估 ----

    def evaluate(
        self,
        now: float,
        *,
        guidance_enabled: bool,
        mode: str | None,
        armed: bool,
        link_ok: bool,
        est_initialized: bool,
        est_age_s: float,
        coast_timeout_s: float,
        cmd_point_ne: np.ndarray | None,
    ) -> GateReport:
        blocked: list[str] = []

        if not guidance_enabled:
            blocked.append("導引開關未開啟（UI 儀表板打開）")
        if self.pilot_override_latched:
            blocked.append("飛行員已接管（模式曾切出允許清單；重新啟用導引解除）")
        if not link_ok:
            blocked.append("數傳斷線/逾時")
        if mode is None:
            blocked.append("尚未收到心跳")
        elif mode not in self.allowed_modes:
            blocked.append(f"模式 {mode} 不在允許清單 {sorted(self.allowed_modes)}")
        if not armed:
            blocked.append("載具未 arm")
        if not est_initialized:
            blocked.append("尚未鎖定目標")
        elif est_age_s > coast_timeout_s:
            blocked.append(f"目標估計逾時 {est_age_s:.1f}s（> coast {coast_timeout_s:.0f}s）")
        if cmd_point_ne is not None:
            dist_home = float(np.linalg.norm(np.asarray(cmd_point_ne, dtype=float)))
            if dist_home > self.max_cmd_distance_m:
                blocked.append(f"指令點離 home {dist_home:.0f}m 超出圍欄 {self.max_cmd_distance_m:.0f}m")
        if self._last_send_t is not None and (now - self._last_send_t) < self.min_interval_s:
            blocked.append("速率限制（正常節流）")

        return GateReport(ok=not blocked, blocked=blocked)
