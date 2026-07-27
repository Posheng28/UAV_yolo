"""從實機影像鏈路收集訓練影像（給玩具車 domain adaptation 用）。

為什麼需要：現成模型（你的空拍模型、COCO）都認不得室內玩具車——
實測 5 幀全 0 偵測，卻把鞋子誤判成 Car。玩具車的尺度與紋理不在
任何既有訓練分佈裡，只能補自己的資料。

用法：
    python tools/collect_dataset.py --out data/toycar --every 0.7 --count 200

    收集時請「拖著車在地板上慢慢移動」，並涵蓋：
      - 不同位置（畫面中央、四個角落）
      - 不同高度（相機 40cm ~ 1.5m）
      - 不同朝向（車頭朝各方向）
      - 有無干擾物（鞋子、箱子入鏡也要拍，模型才學得會分辨）
      - 不同光線

輸出：out/images/*.jpg，之後用 label_dataset.py 半自動標註。
"""

from __future__ import annotations

import argparse
import time
from pathlib import Path

import cv2


def main() -> int:
    ap = argparse.ArgumentParser(description="從實機影像收集訓練圖")
    # 一定要用 frame_raw：/frame.jpg 帶偵測框與狀態文字，
    # 拿去訓練會讓模型學成「看到綠框就是車」。
    ap.add_argument("--url", default="http://localhost:8610/frame_raw.jpg",
                    help="地面站的『原始』單幀端點（無疊加層）")
    ap.add_argument("--out", default="data/toycar")
    ap.add_argument("--every", type=float, default=0.7, help="每幾秒存一張")
    ap.add_argument("--count", type=int, default=200, help="收集幾張")
    args = ap.parse_args()

    import urllib.request

    import numpy as np

    out_dir = Path(args.out) / "images"
    out_dir.mkdir(parents=True, exist_ok=True)
    existing = len(list(out_dir.glob("*.jpg")))
    print(f">>> 輸出：{out_dir}（已有 {existing} 張）")
    print(">>> 請拖著車移動，涵蓋不同位置/高度/朝向/干擾物。Ctrl+C 可隨時停止\n")

    saved = 0
    try:
        while saved < args.count:
            try:
                raw = urllib.request.urlopen(args.url, timeout=5).read()
            except Exception as exc:
                print(f"  取幀失敗（{exc}），重試中…")
                time.sleep(1.0)
                continue
            img = cv2.imdecode(np.frombuffer(raw, np.uint8), cv2.IMREAD_COLOR)
            if img is None:
                time.sleep(0.3)
                continue
            idx = existing + saved
            path = out_dir / f"toycar_{idx:05d}.jpg"
            cv2.imwrite(str(path), img)
            saved += 1
            print(f"  [{saved}/{args.count}] {path.name}", end="\r")
            time.sleep(args.every)
    except KeyboardInterrupt:
        print("\n>>> 使用者中止")

    print(f"\n>>> 共存 {saved} 張，總計 {existing + saved} 張於 {out_dir}")
    print(">>> 下一步：python tools/label_dataset.py --dir " + str(Path(args.out)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
