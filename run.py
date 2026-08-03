"""UAV_yolo 地面站啟動入口。

    python run.py               # 模式由設定檔／UI 切換鈕決定（單一事實來源）
    python run.py --sim         # 可選：把模式寫成模擬（持久化到 local.yaml）
    python run.py --port 8610

模式的「正常」切法是 UI 設定頁的模擬/實機鈕 → 按「重啟引擎」即生效；
不需要用不同的 .bat 或 CLI 旗標。--sim/--live 只是把設定檔改掉的捷徑。
"""

import argparse
import os
import sys
import time
from pathlib import Path


def _tee_stderr_to_log() -> str | None:
    """把 stderr 在**作業系統層**（fd 2）導進 data/server.log。

    為什麼重要：伺服器視窗一關，OpenCV 的訊息就永遠消失了——上次要查「飛行中
    圖傳為什麼斷」時，唯一的線索就是這些訊息，而它們全丟在一個被關掉的視窗裡。
    只導 stderr，啟動網址等正常訊息仍留在視窗上，操作流程完全不變。

    🔴 兩個都是實測出來的順序限制，改動前先看這裡：
      · Python 的 logging / redirect_stderr **攔不到**：OpenCV 的警告是 C 層
        直接寫 fd 2 的，只有 os.dup2 這種 OS 層重導有效。
      · **必須在 import cv2 之前執行**。OpenCV 的 CRT 在 DLL 載入時就把 stderr
        的 handle 快取起來了，之後才 dup2 完全無效（實測：同一段程式碼放在
        import cv2 之後，log 只有 Python 那行、C 層警告全部漏掉）。
        所以這個函式在模組頂端就呼叫，uav_yolo 的 import 刻意排在它後面。
    """
    try:
        log_dir = Path(__file__).resolve().parent / "data"
        log_dir.mkdir(parents=True, exist_ok=True)
        path = log_dir / "server.log"
        fh = open(path, "a", buffering=1, encoding="utf-8", errors="replace")
        fh.write(f"\n===== session start {time.strftime('%Y-%m-%d %H:%M:%S')} =====\n")
        fh.flush()
        os.dup2(fh.fileno(), 2)          # fd 2 之後由這個檔案接手（含 C 層寫入）
        sys.stderr = fh                   # Python 端也指過去，兩邊一致
        return str(path)
    except Exception:
        return None                       # 記不了 log 不能擋開機


# 只有「真的當啟動器執行」時才重導；被 import（測試/工具）時不動別人的 stderr。
_LOG_PATH = _tee_stderr_to_log() if __name__ == "__main__" else None

import uvicorn  # noqa: E402  ← 以下 import 會拉進 cv2，必須排在重導之後

from uav_yolo.config import Config  # noqa: E402
from uav_yolo.webapp import create_app  # noqa: E402


def _keep_awake() -> bool:
    """要求 Windows 在地面站執行期間不要進入睡眠／Modern Standby。

    🔴 實測（2026-07-31 那次任務）：筆電在任務進行中進入 Modern Standby，
    Kernel-Power 506/507 顯示 19:06:59 睡著、19:11:08 才醒——整整 4 分 9 秒
    地面站完全凍結，USB 上的採集卡與數傳一起斷，任務記錄中間是一段空白。
    當天總共睡了 36 次。飛行中的地面站絕不能被 OS 凍住。

    ES_CONTINUOUS 讓這個要求持續有效（不必定期重下）；ES_DISPLAY_REQUIRED
    連螢幕一起擋掉，因為操作員要看畫面。非 Windows 或呼叫失敗就靜默略過。
    """
    try:
        import ctypes

        ES_CONTINUOUS = 0x80000000
        ES_SYSTEM_REQUIRED = 0x00000001
        ES_DISPLAY_REQUIRED = 0x00000002
        rv = ctypes.windll.kernel32.SetThreadExecutionState(
            ES_CONTINUOUS | ES_SYSTEM_REQUIRED | ES_DISPLAY_REQUIRED)
        return bool(rv)
    except Exception:
        return False


def _allow_sleep() -> None:
    try:
        import ctypes

        ctypes.windll.kernel32.SetThreadExecutionState(0x80000000)  # ES_CONTINUOUS
    except Exception:
        pass


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV_yolo 地面站")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--sim", action="store_true", help="把模式寫成模擬（持久化到 local.yaml）")
    parser.add_argument("--live", action="store_true", help="把模式寫成實機（持久化到 local.yaml）")
    args = parser.parse_args()

    cfg = Config()
    # 用 update（寫入 local.yaml）而非 override（僅記憶體）：否則按「重啟引擎」時
    # reload() 會洗掉 CLI 覆蓋、模式跳回設定檔，造成「頂端顯示 vs 設定鈕」不一致。
    if args.sim:
        cfg.update({"system": {"mode": "sim"}})
    elif args.live:
        cfg.update({"system": {"mode": "live"}})

    host = args.host or cfg.get("system.web_host", "127.0.0.1")
    port = args.port or int(cfg.get("system.web_port", 8600))

    log_path = _LOG_PATH
    app = create_app(cfg)
    awake = _keep_awake()
    print(f">>> UAV_yolo 地面站 http://{host}:{port}  （模式：{cfg.get('system.mode')}）")
    print(">>> 睡眠抑制：" + ("已啟用（執行期間不會進入睡眠／關螢幕）"
                             if awake else "無法啟用，請自行確認電源設定"))
    if log_path:
        print(f">>> 診斷記錄：{log_path}（影像/驅動訊息；查斷線看這個檔）")
    try:
        uvicorn.run(app, host=host, port=port, log_level="warning")
    finally:
        _allow_sleep()   # 還原，否則這台筆電從此不睡


if __name__ == "__main__":
    main()
