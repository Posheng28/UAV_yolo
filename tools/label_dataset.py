"""極簡標註工具：拖曳畫框標出玩具車，輸出 YOLO 格式標籤。

操作：
    滑鼠左鍵拖曳  畫一個框
    u            復原上一個框
    c            清掉這張的所有框
    空白鍵 / d   存檔並下一張
    a            上一張
    x            這張沒有目標（存空標籤——負樣本，很重要！）
    q            離開（進度會存著，下次接續）

用法：
    python tools/label_dataset.py --dir data/toycar

輸出 YOLO 格式：labels/xxx.txt，每行 "class cx cy w h"（都是 0~1 正規化）。
class 0 = Car（與既有資料集一致）。
"""

from __future__ import annotations

import argparse
from pathlib import Path

import cv2

WINDOW = "label (drag=box  u=undo  c=clear  space=next  a=prev  x=no target  q=quit)"
CLASS_ID = 0  # Car


class Labeler:
    def __init__(self, img_dir: Path, lbl_dir: Path):
        self.paths = sorted(img_dir.glob("*.jpg"))
        if not self.paths:
            raise SystemExit(f"找不到影像：{img_dir}")
        self.lbl_dir = lbl_dir
        self.lbl_dir.mkdir(parents=True, exist_ok=True)
        self.idx = self._first_unlabeled()
        self.boxes: list[tuple[int, int, int, int]] = []
        self.drag_start = None
        self.cur = None
        self.disp_scale = 1.0

    def _label_path(self, i: int) -> Path:
        return self.lbl_dir / (self.paths[i].stem + ".txt")

    def _first_unlabeled(self) -> int:
        for i in range(len(self.paths)):
            if not self._label_path(i).exists():
                return i
        return 0

    def load(self):
        """載入這張已存的標籤（可續標/修正）。"""
        self.boxes = []
        p = self._label_path(self.idx)
        img = cv2.imread(str(self.paths[self.idx]))
        h, w = img.shape[:2]
        if p.exists():
            for line in p.read_text().strip().splitlines():
                parts = line.split()
                if len(parts) != 5:
                    continue
                _, cx, cy, bw, bh = map(float, parts)
                x1 = int((cx - bw / 2) * w); y1 = int((cy - bh / 2) * h)
                x2 = int((cx + bw / 2) * w); y2 = int((cy + bh / 2) * h)
                self.boxes.append((x1, y1, x2, y2))
        return img

    def save(self):
        img = cv2.imread(str(self.paths[self.idx]))
        h, w = img.shape[:2]
        lines = []
        for x1, y1, x2, y2 in self.boxes:
            x1, x2 = sorted((max(0, x1), min(w, x2)))
            y1, y2 = sorted((max(0, y1), min(h, y2)))
            if x2 - x1 < 4 or y2 - y1 < 4:
                continue
            cx = (x1 + x2) / 2 / w; cy = (y1 + y2) / 2 / h
            bw = (x2 - x1) / w; bh = (y2 - y1) / h
            lines.append(f"{CLASS_ID} {cx:.6f} {cy:.6f} {bw:.6f} {bh:.6f}")
        # 空檔案 = 負樣本（明確告訴模型「這張沒有車」），對減少誤判很重要
        self._label_path(self.idx).write_text("\n".join(lines))

    def on_mouse(self, event, x, y, flags, param):
        rx, ry = int(x / self.disp_scale), int(y / self.disp_scale)
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drag_start = (rx, ry)
            self.cur = (rx, ry, rx, ry)
        elif event == cv2.EVENT_MOUSEMOVE and self.drag_start:
            self.cur = (self.drag_start[0], self.drag_start[1], rx, ry)
        elif event == cv2.EVENT_LBUTTONUP and self.drag_start:
            x1, y1 = self.drag_start
            if abs(rx - x1) > 4 and abs(ry - y1) > 4:
                self.boxes.append((x1, y1, rx, ry))
            self.drag_start = None
            self.cur = None

    def run(self):
        cv2.namedWindow(WINDOW, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(WINDOW, self.on_mouse)
        img = self.load()

        while True:
            view = img.copy()
            for (x1, y1, x2, y2) in self.boxes:
                cv2.rectangle(view, (x1, y1), (x2, y2), (80, 220, 80), 3)
            if self.cur:
                cv2.rectangle(view, self.cur[:2], self.cur[2:], (60, 200, 255), 2)

            done = sum(1 for i in range(len(self.paths)) if self._label_path(i).exists())
            hud = f"{self.idx + 1}/{len(self.paths)}  已標 {done}  本張 {len(self.boxes)} 框"
            cv2.rectangle(view, (0, 0), (760, 46), (0, 0, 0), -1)
            cv2.putText(view, hud, (12, 33), cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                        (255, 255, 255), 2, cv2.LINE_AA)

            h, w = view.shape[:2]
            self.disp_scale = min(1500 / w, 850 / h, 1.0)
            cv2.imshow(WINDOW, cv2.resize(view, None, fx=self.disp_scale, fy=self.disp_scale))

            k = cv2.waitKey(20) & 0xFF
            if k == ord("q"):
                self.save()
                break
            elif k in (ord(" "), ord("d")):
                self.save()
                if self.idx < len(self.paths) - 1:
                    self.idx += 1
                    img = self.load()
            elif k == ord("a"):
                self.save()
                if self.idx > 0:
                    self.idx -= 1
                    img = self.load()
            elif k == ord("u") and self.boxes:
                self.boxes.pop()
            elif k == ord("c"):
                self.boxes = []
            elif k == ord("x"):
                self.boxes = []
                self.save()
                if self.idx < len(self.paths) - 1:
                    self.idx += 1
                    img = self.load()

        cv2.destroyAllWindows()
        done = sum(1 for i in range(len(self.paths)) if self._label_path(i).exists())
        print(f">>> 已標註 {done}/{len(self.paths)} 張")
        if done >= 50:
            print(">>> 下一步：python tools/train_toycar.py")
        else:
            print(">>> 建議至少標到 100 張再訓練")


def main() -> int:
    ap = argparse.ArgumentParser(description="YOLO 標註小工具")
    ap.add_argument("--dir", default="data/toycar")
    args = ap.parse_args()
    root = Path(args.dir)
    Labeler(root / "images", root / "labels").run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
