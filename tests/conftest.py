"""測試共用防護。

任務記錄隔離：任何測試若沒有明確指定 system.mission_log_dir，啟用導引時
會寫進專案真正的 data/missions/——實際發生過：一次 pytest 在使用者的任務
目錄裡留下 12 份假任務檔，跟真飛行記錄混在一起，覆盤時差點分不出來。
（與先前 Config() 寫壞真 local.yaml 是同一類病：測試觸碰使用者的真實資料。）
"""

from pathlib import Path

import pytest

from uav_yolo import engine as engine_mod


@pytest.fixture(autouse=True)
def _isolate_mission_logs(tmp_path, monkeypatch):
    """沒明確指定記錄目錄的測試，一律導到 tmp_path。

    有明確指定的（如 test_mission_log 自己驗證檔案位置）不受影響。
    """
    original = engine_mod.TrackerEngine._mission_dir

    def patched(self):
        # 注意不能檢查「有沒有設定」：default.yaml 本身就帶著預設值
        # data/missions，cfg.get 永遠拿得到（第一版 conftest 就是這樣失效、
        # 又污染了一輪）。判斷「還是預設值」才對。
        configured = str(self.cfg.get("system.mission_log_dir", "data/missions"))
        if configured == "data/missions":
            d = tmp_path / "_missions"
            d.mkdir(parents=True, exist_ok=True)
            return d
        return original(self)

    monkeypatch.setattr(engine_mod.TrackerEngine, "_mission_dir", patched)
