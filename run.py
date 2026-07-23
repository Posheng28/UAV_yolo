"""UAV_yolo 地面站啟動入口。

    python run.py               # 依 config（live 或 sim）
    python run.py --sim         # 強制模擬模式（無硬體演練）
    python run.py --port 8600
"""

import argparse

import uvicorn

from uav_yolo.config import Config
from uav_yolo.webapp import create_app


def main() -> None:
    parser = argparse.ArgumentParser(description="UAV_yolo 地面站")
    parser.add_argument("--host", default=None)
    parser.add_argument("--port", type=int, default=None)
    parser.add_argument("--sim", action="store_true", help="以模擬模式啟動（覆蓋設定檔）")
    parser.add_argument("--live", action="store_true", help="以實機模式啟動（覆蓋設定檔）")
    args = parser.parse_args()

    cfg = Config()
    if args.sim:
        cfg.override({"system": {"mode": "sim"}})
    elif args.live:
        cfg.override({"system": {"mode": "live"}})

    host = args.host or cfg.get("system.web_host", "127.0.0.1")
    port = args.port or int(cfg.get("system.web_port", 8600))

    app = create_app(cfg)
    print(f">>> UAV_yolo 地面站 http://{host}:{port}  （模式：{cfg.get('system.mode')}）")
    uvicorn.run(app, host=host, port=port, log_level="warning")


if __name__ == "__main__":
    main()
