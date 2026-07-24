"""設定載入：default.yaml 為基底，local.yaml 覆蓋，UI 修改寫回 local.yaml。"""

from __future__ import annotations

import copy
import threading
from pathlib import Path

import yaml

PROJECT_ROOT = Path(__file__).resolve().parent.parent
DEFAULT_PATH = PROJECT_ROOT / "config" / "default.yaml"
LOCAL_PATH = PROJECT_ROOT / "config" / "local.yaml"

# 單一事實來源：這兩個路徑在引擎/模擬/webapp 三處都會讀，
# 各自寫自己的 fallback 會漂移（且 YAML null 不會觸發 dict.get 的預設）。
DEFAULT_INTRINSICS_FILE = "config/camera_intrinsics.yaml"
DEFAULT_WEIGHTS_FILE = "weights/best.pt"


def _deep_merge(base: dict, override: dict) -> dict:
    """override 蓋到 base 上（巢狀 dict 遞迴合併，其他型別直接取代）。"""
    out = copy.deepcopy(base)
    for key, value in (override or {}).items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = _deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


class Config:
    """執行緒安全的設定容器，用點路徑存取：cfg.get("gimbal.control")。"""

    def __init__(self, default_path: Path = DEFAULT_PATH, local_path: Path = LOCAL_PATH):
        self._default_path = Path(default_path)
        self._local_path = Path(local_path)
        self._lock = threading.Lock()
        self.reload()

    def reload(self) -> None:
        with self._lock:
            with open(self._default_path, "r", encoding="utf-8") as f:
                defaults = yaml.safe_load(f) or {}
            overrides = {}
            if self._local_path.exists():
                with open(self._local_path, "r", encoding="utf-8") as f:
                    overrides = yaml.safe_load(f) or {}
            self._defaults = defaults
            self._overrides = overrides
            self._merged = _deep_merge(defaults, overrides)

    def get(self, dotted: str, default=None):
        with self._lock:
            node = self._merged
            for part in dotted.split("."):
                if not isinstance(node, dict) or part not in node:
                    return default
                node = node[part]
            return copy.deepcopy(node)

    def section(self, dotted: str) -> dict:
        value = self.get(dotted, {})
        return value if isinstance(value, dict) else {}

    def as_dict(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._merged)

    def update(self, patch: dict) -> None:
        """深層合併 patch 進 local.yaml 並立即生效。"""
        with self._lock:
            self._overrides = _deep_merge(self._overrides, patch)
            self._merged = _deep_merge(self._defaults, self._overrides)
            self._local_path.parent.mkdir(parents=True, exist_ok=True)
            with open(self._local_path, "w", encoding="utf-8") as f:
                yaml.safe_dump(self._overrides, f, allow_unicode=True, sort_keys=False)

    def override(self, patch: dict) -> None:
        """僅本次執行生效的覆蓋（不寫入 local.yaml；CLI 旗標用）。

        注意 reload() 會清掉 override——引擎重啟沿用檔案設定屬預期行為。
        """
        with self._lock:
            self._merged = _deep_merge(self._merged, patch)
