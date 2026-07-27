"""用「已標好的少量資料」訓一個快速模型，替剩下的影像預標註。

人在迴圈（human-in-the-loop）流程，比全手工快 3~5 倍：
    1. 手動標 40~60 張        (tools/label_dataset.py)
    2. 跑這支                  → 訓快速模型、自動預標其餘影像
    3. 再開標註工具快速修正    (框大多已畫好，只要改錯的)
    4. 正式訓練                (tools/train_toycar.py)

自動標註不會憑空變準，但「修正既有框」遠比「從頭畫」快。
低於 conf 門檻的影像會留空標籤讓你自己判斷，不會亂塞框。
"""

from __future__ import annotations

import argparse
import shutil
from pathlib import Path


def main() -> int:
    ap = argparse.ArgumentParser(description="用少量標註預標其餘影像")
    ap.add_argument("--dir", default="data/toycar")
    ap.add_argument("--epochs", type=int, default=40, help="快速模型訓練輪數")
    ap.add_argument("--conf", type=float, default=0.35, help="預標註信心門檻")
    ap.add_argument("--imgsz", type=int, default=960)
    args = ap.parse_args()

    root = Path(args.dir)
    img_dir, lbl_dir = root / "images", root / "labels"
    lbl_dir.mkdir(parents=True, exist_ok=True)

    all_imgs = sorted(img_dir.glob("*.jpg"))
    labeled = [p for p in all_imgs if (lbl_dir / (p.stem + ".txt")).exists()]
    todo = [p for p in all_imgs if p not in set(labeled)]

    print(f">>> 共 {len(all_imgs)} 張；已標 {len(labeled)}、待預標 {len(todo)}")
    if len(labeled) < 25:
        raise SystemExit("已標註太少（<25 張）。請先手動標 40~60 張再跑這支")
    if not todo:
        print(">>> 全部都標好了，直接跑 train_toycar.py")
        return 0

    # 用已標註的資料訓一個快速模型（不求最好，只求能預標）
    import random

    random.seed(0)
    pool = labeled[:]
    random.shuffle(pool)
    n_val = max(len(pool) // 5, 3)
    ds = root / "_prelabel"
    if ds.exists():
        shutil.rmtree(ds)
    for split, files in {"val": pool[:n_val], "train": pool[n_val:]}.items():
        (ds / "images" / split).mkdir(parents=True, exist_ok=True)
        (ds / "labels" / split).mkdir(parents=True, exist_ok=True)
        for p in files:
            shutil.copy2(p, ds / "images" / split / p.name)
            shutil.copy2(lbl_dir / (p.stem + ".txt"), ds / "labels" / split / (p.stem + ".txt"))
    yaml_path = ds / "data.yaml"
    yaml_path.write_text(
        f"path: {ds.resolve().as_posix()}\ntrain: images/train\nval: images/val\nnc: 1\nnames: [Car]\n",
        encoding="utf-8",
    )

    from ultralytics import YOLO

    base = Path("weights/best.pt")
    model = YOLO(str(base) if base.exists() else "yolo26n.pt")
    print(f">>> 訓練快速預標模型（{args.epochs} epochs）…")
    model.train(data=str(yaml_path.resolve()), epochs=args.epochs, imgsz=args.imgsz,
                batch=8, lr0=0.003, degrees=180.0, scale=0.6, fliplr=0.5,
                project="runs", name="prelabel", exist_ok=True, verbose=False, plots=False)

    quick = YOLO("runs/prelabel/weights/best.pt")
    print(f">>> 預標 {len(todo)} 張（conf={args.conf}）…")
    with_box = 0
    for i, p in enumerate(todo):
        r = quick.predict(str(p), conf=args.conf, imgsz=args.imgsz, verbose=False)[0]
        lines = []
        if r.boxes is not None:
            for b in r.boxes:
                cx, cy, bw, bh = [float(v) for v in b.xywhn[0]]
                lines.append(f"0 {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        (lbl_dir / (p.stem + ".txt")).write_text("\n".join(lines))
        if lines:
            with_box += 1
        print(f"  {i+1}/{len(todo)}", end="\r")

    print(f"\n>>> 完成：{with_box} 張有預標框、{len(todo)-with_box} 張空白（需你判斷）")
    print(">>> 下一步：python tools/label_dataset.py --dir " + str(root))
    print("    逐張檢查，框錯就按 c 清掉重畫、沒車就按 x")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
