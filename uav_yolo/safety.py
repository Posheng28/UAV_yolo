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
    # 節流「不是安全阻擋」，是流量控制：每秒最多發一次指令，正常運作時
    # 大半時間都成立。混進 blocked 會讓 UI 一直閃紅，操作員很快學會忽略
    # 紅色項目——真的出現 GPS 不良/飛行員接管時就看不到了（警報疲勞）。
    throttled: bool = False


class SafetyGates:
    # 這兩個資料流的要求速率分別約 2Hz 與 1Hz；容忍數倍抖動後仍算過期。
    GPS_MAX_AGE_S = 3.0
    LANDED_MAX_AGE_S = 5.0

    def __init__(self, cfg_safety: dict, rate_hz: float):
        self.pilot_override_latched = False
        self._last_mode: str | None = None
        self._last_send_t: float | None = None
        self.update_limits(cfg_safety, rate_hz)

    def update_limits(self, cfg_safety: dict, rate_hz: float) -> None:
        """就地更新門檻，**保留接管閂鎖與速率狀態**。

        供 UI 儲存設定時熱套用——把圍欄改嚴後必須立刻生效，
        不能等到重啟引擎（操作員會以為已經生效）。
        """
        self.link_timeout_s = float(cfg_safety.get("link_timeout_s", 2.0))
        self.allowed_modes = set(cfg_safety.get("allowed_modes", ["AUTO.LOITER"]))
        self.max_cmd_distance_m = float(cfg_safety.get("max_cmd_distance_m", 500.0))
        self.min_cmd_alt_m = float(cfg_safety.get("min_cmd_alt_m", 20.0))
        self.max_cmd_alt_m = float(cfg_safety.get("max_cmd_alt_m", 120.0))
        self.min_interval_s = 1.0 / max(float(rate_hz), 0.1)

        # GPS 品質／離地檢查（direct 模式下取代機上 global_goto_node 的同名 gate）
        self.require_gps_quality = bool(cfg_safety.get("require_gps_quality", True))
        self.gps_min_fix_type = int(cfg_safety.get("gps_min_fix_type", 3))
        self.gps_min_satellites = int(cfg_safety.get("gps_min_satellites", 8))
        self.gps_max_eph_m = float(cfg_safety.get("gps_max_eph_m", 5.0))
        self.gps_max_epv_m = float(cfg_safety.get("gps_max_epv_m", 8.0))
        # 老韌體沒有 h_acc/v_acc（公尺精度）時退回 DOP 門檻
        self.gps_max_hdop = float(cfg_safety.get("gps_max_hdop", 2.5))
        self.gps_max_vdop = float(cfg_safety.get("gps_max_vdop", 3.5))
        self.require_airborne = bool(cfg_safety.get("require_airborne", True))

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
        gps=None,
        landed=None,
        video_frozen: bool = False,
        target_locked: bool = False,
        meas_note: str = "",
    ) -> GateReport:
        blocked: list[str] = []

        if not guidance_enabled:
            blocked.append("導引開關未開啟（UI 儀表板打開）")
        # 影像停格 ＝ 畫面死了但擷取還在吐幀（圖傳失鎖的典型長相）。此時
        # 天底相機的固定像素會對應到「飛機下方固定偏移」的地面點，飛機一動
        # 目標就跟著動——實測 12 秒漂 158m、KF 認定靜止的車以 15m/s 逃逸。
        # 引擎已停止餵量測（第一道防線），這裡是第二道：明確告訴操作員為何不發。
        if video_frozen:
            blocked.append("影像停格中（畫面沒更新），不由死畫面推導指令")
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
        # 🔴 這兩個是「最後一次收到的快照」，不會過期自清——資料流單獨死掉時
        # （LR24 半雙工實測只拿到約 37% 的要求速率，1Hz 訊息掉幾秒很正常），
        # 閘門會拿一份幾分鐘前的樣本回答「一切正常」。位置/姿態有新鮮度保護，
        # 這兩個沒有，所以要在這裡補。
        if self.require_airborne:
            if landed is None:
                blocked.append("尚未收到離地狀態（EXTENDED_SYS_STATE）")
            elif (now - getattr(landed, "t", now)) > self.LANDED_MAX_AGE_S:
                blocked.append(
                    f"離地狀態已過期 {now - landed.t:.0f}s（EXTENDED_SYS_STATE 停止更新）")
            elif not landed.airborne:
                blocked.append("載具尚未離地（本系統不負責起飛）")
        if self.require_gps_quality:
            if gps is None:
                blocked.append("尚未收到 GPS 品質（GPS_RAW_INT）")
            elif (now - getattr(gps, "t", now)) > self.GPS_MAX_AGE_S:
                blocked.append(f"GPS 品質資料已過期 {now - gps.t:.0f}s（GPS_RAW_INT 停止更新）")
            else:
                import math as _math

                if gps.fix_type < self.gps_min_fix_type:
                    blocked.append(f"GPS fix_type={gps.fix_type} < {self.gps_min_fix_type}")
                if gps.satellites < self.gps_min_satellites:
                    blocked.append(f"GPS 衛星數 {gps.satellites} < {self.gps_min_satellites}")
                # 優先用公尺精度（h_acc/v_acc）；沒有就退 DOP；兩者皆無 → 擋下
                if _math.isfinite(gps.eph_m):
                    if gps.eph_m > self.gps_max_eph_m:
                        blocked.append(f"GPS 水平誤差 {gps.eph_m:.1f}m > {self.gps_max_eph_m}m")
                elif gps.hdop is not None:
                    if gps.hdop > self.gps_max_hdop:
                        blocked.append(f"GPS HDOP {gps.hdop:.1f} > {self.gps_max_hdop}")
                else:
                    blocked.append("GPS 未提供水平精度（h_acc/HDOP 皆無）")
                if _math.isfinite(gps.epv_m):
                    if gps.epv_m > self.gps_max_epv_m:
                        blocked.append(f"GPS 垂直誤差 {gps.epv_m:.1f}m > {self.gps_max_epv_m}m")
                elif gps.vdop is not None and gps.vdop > self.gps_max_vdop:
                    blocked.append(f"GPS VDOP {gps.vdop:.1f} > {self.gps_max_vdop}")
        if not est_initialized:
            # 🔴 「沒有鎖定」與「鎖定了但算不出座標」是兩件完全不同的事，
            # 處置也不同（前者去點選目標，後者去查測地為什麼失敗）。
            # 舊版兩者都印「尚未鎖定目標」，於是畫面上框是紅的、清單寫
            # 「已鎖定」，閘門卻說沒鎖定——操作員只能困惑。
            if target_locked:
                why = meas_note or "尚無有效量測"
                blocked.append(f"已鎖定目標，但還算不出地面座標：{why}")
            else:
                blocked.append("尚未鎖定目標（在畫面或偵測列表點選一台車）")
        elif est_age_s > coast_timeout_s:
            blocked.append(f"目標估計逾時 {est_age_s:.1f}s（> coast {coast_timeout_s:.0f}s）")
        if cmd_point_ne is not None:
            pt = np.asarray(cmd_point_ne, dtype=float)
            dist_home = float(np.linalg.norm(pt))
            # NaN 對 `>` 恆為 False——不明確擋下的話，一個 NaN 指令點會
            # 直接穿過圍欄（fail-open）。安全檢查必須 fail-closed。
            if not np.all(np.isfinite(pt)):
                blocked.append("指令點含非有限值（NaN/Inf），拒絕發送")
            elif dist_home > self.max_cmd_distance_m:
                blocked.append(f"指令點離 home {dist_home:.0f}m 超出圍欄 {self.max_cmd_distance_m:.0f}m")
        throttled = (
            self._last_send_t is not None
            and (now - self._last_send_t) < self.min_interval_s
        )

        # 節流一樣會擋住這次發送（ok=False），但不列入 blocked，UI 另外用中性樣式顯示
        return GateReport(ok=(not blocked and not throttled), blocked=blocked, throttled=throttled)
