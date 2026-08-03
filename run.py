"""UAV_yolo 地面站啟動入口。

    python run.py               # 模式由設定檔／UI 切換鈕決定（單一事實來源）
    python run.py --sim         # 可選：把模式寫成模擬（持久化到 local.yaml）
    python run.py --port 8610

模式的「正常」切法是 UI 設定頁的模擬/實機鈕 → 按「重啟引擎」即生效；
不需要用不同的 .bat 或 CLI 旗標。--sim/--live 只是把設定檔改掉的捷徑。
"""

import argparse

import uvicorn

from uav_yolo.config import Config
from uav_yolo.webapp import create_app


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

    app = create_app(cfg)
    awake = _keep_awake()
    print(f">>> UAV_yolo 地面站 http://{host}:{port}  （模式：{cfg.get('system.mode')}）")
    print(">>> 睡眠抑制：" + ("已啟用（執行期間不會進入睡眠／關螢幕）"
                             if awake else "無法啟用，請自行確認電源設定"))
    try:
        uvicorn.run(app, host=host, port=port, log_level="warning")
    finally:
        _allow_sleep()   # 還原，否則這台筆電從此不睡


if __name__ == "__main__":
    main()
