"""用你自己收集的玩具車影像微調（fine-tune）YOLO。

策略：從既有的空拍權重接續訓練（transfer learning），而不是從零開始——
既保留原本對「車輛形狀」的認識，又學會你的玩具車與地板環境。

用法：
    python tools/train_toycar.py                    # 預設 100 epochs
    python tools/train_toycar.py --epochs 150 --base weights/best.pt

訓練完會輸出 runs/toycar/weights/best.pt，並提示如何套用。
"""

from __future__ import annotations

import argparse
import random
import shutil
from pathlib import Path


def split_dataset(root: Path, val_ratio: float = 0.2) -> Path:
    """把 images/labels 分成 train/val（YOLO 需要的目錄結構）。"""
    imgs = sorted((root / "images").glob("*.jpg"))
    labeled = [p for p in imgs if (root / "labels" / (p.stem + ".txt")).exists()]
    if len(labeled) < 20:
        raise SystemExit(f"已標註影像只有 {len(labeled)} 張，太少。請先標到 100 張以上")

    random.seed(42)
    random.shuffle(labeled)
    n_val = max(int(len(labeled) * val_ratio), 5)
    splits = {"val": labeled[:n_val], "train": labeled[n_val:]}

    ds = root / "_yolo"
    if ds.exists():
        shutil.rmtree(ds)
    for split, files in splits.items():
        (ds / "images" / split).mkdir(parents=True, exist_ok=True)
        (ds / "labels" / split).mkdir(parents=True, exist_ok=True)
        for p in files:
            shutil.copy2(p, ds / "images" / split / p.name)
            shutil.copy2(root / "labels" / (p.stem + ".txt"),
                         ds / "labels" / split / (p.stem + ".txt"))

    yaml_path = ds / "data.yaml"
    yaml_path.write_text(
        f"path: {ds.resolve().as_posix()}\n"
        "train: images/train\n"
        "val: images/val\n"
        "nc: 1\n"
        "names: [Car]\n",
        encoding="utf-8",
    )
    print(f">>> 資料集：訓練 {len(splits['train'])} 張、驗證 {len(splits['val'])} 張")
    return yaml_path


def main() -> int:
    ap = argparse.ArgumentParser(description="微調玩具車偵測模型")
    ap.add_argument("--data", default="data/toycar")
    ap.add_argument("--base", default="weights/best.pt",
                    help="起始權重；沒有就用 yolo26n.pt")
    ap.add_argument("--epochs", type=int, default=100)
    ap.add_argument("--imgsz", type=int, default=960)
    ap.add_argument("--batch", type=int, default=8)
    args = ap.parse_args()

    root = Path(args.data)
    yaml_path = split_dataset(root)

    base = Path(args.base)
    weights = str(base) if base.exists() else "yolo26n.pt"
    print(f">>> 起始權重：{weights}")

    from ultralytics import YOLO

    model = YOLO(weights)
    results = model.train(
        data=str(yaml_path.resolve()),
        epochs=args.epochs,
        imgsz=args.imgsz,
        batch=args.batch,
        patience=30,
        cos_lr=True,
        # 微調用較小的學習率，避免把原本學到的車輛特徵洗掉
        lr0=0.002,
        # 增強：玩具車會從各種角度/距離出現，加強幾何與色彩變化
        degrees=180.0,      # 車頭可能朝任何方向
        scale=0.6,          # 不同高度 → 大小差異大
        fliplr=0.5,
        hsv_v=0.5,          # 室內光線變化
        mosaic=1.0,
        project="runs",
        name="toycar",
        exist_ok=True,
    )

    # 輸出目錄要跟 ultralytics 問，不能自己拼：相對的 project 會被 cfg 前綴成
    # `settings['runs_dir']/<task>/`，實際落在 runs/detect/runs/toycar/，
    # 於是訓練成功也會印「找不到輸出權重」。
    save_dir = Path(getattr(results, "save_dir", "runs/toycar"))
    best = save_dir / "weights" / "best.pt"
    print("\n" + "=" * 60)
    if best.exists():
        print(f">>> 訓練完成：{best}")
        print(">>> 套用方式：")
        print(f"       copy {best} weights\\toycar.pt")
        print("    然後在設定頁把「權重」改成 weights/toycar.pt、按重啟引擎")
        print(">>> 先用 tools/eval_on_live.py 驗證效果再上機")
    else:
        print("!! 找不到輸出權重，檢查上方訓練日誌")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
