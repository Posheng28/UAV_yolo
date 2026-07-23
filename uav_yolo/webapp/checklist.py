"""任務前檢查清單：預設模板 + data/checklist.json 持久化。"""

from __future__ import annotations

import json
import threading
import uuid
from pathlib import Path

from ..config import PROJECT_ROOT

CHECKLIST_PATH = PROJECT_ROOT / "data" / "checklist.json"

DEFAULT_ITEMS: list[tuple[str, str]] = [
    # (分組, 項目)
    ("行前準備（前一天）", "所有電池充飽：飛機動力、遙控器、筆電、數傳"),
    ("行前準備（前一天）", "螺旋槳與備品清點（含備用槳、工具）"),
    ("行前準備（前一天）", "確認天氣與風速（定翼注意側風、陣風上限）"),
    ("行前準備（前一天）", "場地/空域申請與周邊淨空確認"),
    ("行前準備（前一天）", "最新訓練權重已拷入 weights/best.pt"),
    ("行前準備（前一天）", "用模擬模式跑一次完整流程（鎖定→開導引→接管演練）"),
    ("硬體組裝", "雲台安裝穩固、減震球完好"),
    ("硬體組裝", "相機 HDMI → 採集卡 → 筆電接妥，OBS/其他佔用程式已關閉"),
    ("硬體組裝", "數傳電台兩端天線鎖緊、GPS 天線朝上無遮擋"),
    ("硬體組裝", "遙控器對頻正常、接管開關（模式切換）位置確認"),
    ("PX4 / 雲台參數", "MNT_MODE_IN=4（MAVLink 雲台協定 v2）"),
    ("PX4 / 雲台參數", "MNT_MODE_OUT 依接線設定（MAVLink 或 AUX PWM）"),
    ("PX4 / 雲台參數", "C-20T 控制模式與行程設定完成（GimbalConfig 軟體）"),
    ("PX4 / 雲台參數", "定翼：NAV_LOITER_RAD 設為需求繞行半徑"),
    ("地面站軟體", "影像來源有畫面、FPS 正常（儀表板左上）"),
    ("地面站軟體", "相機校正已完成（設定頁顯示 calibrated，RMS < 1px）"),
    ("地面站軟體", "MAVLink 連線正常：心跳/模式/高度/姿態即時更新"),
    ("地面站軟體", "雲台 ROI 地面測試：點鎖目標後雲台確實轉向"),
    ("地面站軟體", "YOLO 偵測正常：對地面測試物有框、ID 穩定"),
    ("地面站軟體", "設定頁：載體種類、雲台有無、允許模式都選對"),
    ("起飛前", "安全開關/arm 流程口頭複誦、圍欄參數（距離/高度上限）確認"),
    ("起飛前", "導引開關確認為關閉狀態（起飛後手動開啟）"),
    ("起飛前", "RC 接管演練：切出 Hold → 確認系統停發並閂鎖"),
    ("任務中", "鎖定目標後先確認估計穩定（速度合理、誤差小）再開導引"),
    ("任務中", "持續監看「未發送原因」清單與數傳訊號強度"),
    ("返航後", "回收 log 與影像、記錄電池狀態、問題筆記回寫"),
]


class Checklist:
    def __init__(self, path: Path = CHECKLIST_PATH):
        self.path = path
        self._lock = threading.Lock()
        self.items: list[dict] = []
        self._load()

    def _default_items(self) -> list[dict]:
        return [
            {"id": uuid.uuid4().hex[:8], "group": group, "text": text, "done": False, "custom": False}
            for group, text in DEFAULT_ITEMS
        ]

    def _load(self) -> None:
        if self.path.exists():
            try:
                with open(self.path, "r", encoding="utf-8") as f:
                    self.items = json.load(f)
                return
            except (json.JSONDecodeError, OSError):
                pass
        self.items = self._default_items()
        self._save()

    def _save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump(self.items, f, ensure_ascii=False, indent=1)

    def all(self) -> list[dict]:
        with self._lock:
            return [dict(i) for i in self.items]

    def toggle(self, item_id: str) -> None:
        with self._lock:
            for item in self.items:
                if item["id"] == item_id:
                    item["done"] = not item["done"]
                    break
            self._save()

    def add(self, group: str, text: str) -> dict:
        item = {"id": uuid.uuid4().hex[:8], "group": group, "text": text, "done": False, "custom": True}
        with self._lock:
            self.items.append(item)
            self._save()
        return item

    def remove(self, item_id: str) -> None:
        with self._lock:
            self.items = [i for i in self.items if i["id"] != item_id]
            self._save()

    def reset(self) -> None:
        with self._lock:
            self.items = self._default_items()
            self._save()

    def uncheck_all(self) -> None:
        with self._lock:
            for item in self.items:
                item["done"] = False
            self._save()
