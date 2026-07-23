"""PX4 custom_mode 解析。

PX4 的 HEARTBEAT.custom_mode 編碼：main_mode 在 bits 16-23、sub_mode 在 bits 24-31。
舊系統的致命傷之一：拿 ArduPilot Copter 的模式編號（GUIDED=4）去判斷，
接上 PX4 或定翼機都會誤判「飛行員接管」狀態——這裡用 PX4 正確表。
"""

from __future__ import annotations

MAIN_MODES = {
    1: "MANUAL",
    2: "ALTCTL",
    3: "POSCTL",
    4: "AUTO",
    5: "ACRO",
    6: "OFFBOARD",
    7: "STABILIZED",
    8: "RATTITUDE",
}

AUTO_SUB_MODES = {
    1: "READY",
    2: "TAKEOFF",
    3: "LOITER",
    4: "MISSION",
    5: "RTL",
    6: "LAND",
    7: "RTGS",
    8: "FOLLOW_TARGET",
    9: "PRECLAND",
}


def mode_string(custom_mode: int) -> str:
    """custom_mode → 人類可讀模式名，例：AUTO.LOITER / POSCTL / MANUAL。"""
    main = (int(custom_mode) >> 16) & 0xFF
    sub = (int(custom_mode) >> 24) & 0xFF
    name = MAIN_MODES.get(main, f"UNKNOWN({main})")
    if main == 4:
        return f"AUTO.{AUTO_SUB_MODES.get(sub, sub)}"
    return name


def encode_custom_mode(main: int, sub: int = 0) -> int:
    """測試/模擬用：組回 custom_mode。"""
    return ((sub & 0xFF) << 24) | ((main & 0xFF) << 16)
