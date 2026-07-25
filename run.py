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
    print(f">>> UAV_yolo 地面站 http://{host}:{port}  （模式：{cfg.get('system.mode')}）")
    uvicorn.run(app, host=host, port=port, log_level="warning")


if __name__ == "__main__":
    main()
