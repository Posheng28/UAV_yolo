"""資料集工具測試：標註存讀往返、train/val 切分。

標註格式錯了會安靜地毀掉整批訓練資料（模型學到錯的框卻不會報錯），
所以座標換算一定要測。
"""

import sys
from pathlib import Path

import cv2
import numpy as np
import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "tools"))
from label_dataset import Labeler  # noqa: E402
from train_toycar import split_dataset  # noqa: E402


def make_images(root: Path, n: int, size=(1080, 1920)) -> None:
    (root / "images").mkdir(parents=True, exist_ok=True)
    for i in range(n):
        img = np.full((*size, 3), 120, np.uint8)
        cv2.imwrite(str(root / "images" / f"img_{i:03d}.jpg"), img)


def test_box_roundtrip_preserves_pixels(tmp_path):
    """畫的框存成 YOLO 正規化座標、再讀回來，必須回到同樣的像素位置。"""
    make_images(tmp_path, 1)
    lab = Labeler(tmp_path / "images", tmp_path / "labels")
    original = (400, 300, 700, 560)
    lab.boxes = [original]
    lab.save()

    lab2 = Labeler(tmp_path / "images", tmp_path / "labels")
    lab2.idx = 0
    lab2.load()
    assert len(lab2.boxes) == 1
    for got, want in zip(lab2.boxes[0], original):
        assert abs(got - want) <= 2, f"座標換算失真：{lab2.boxes[0]} vs {original}"


def test_yolo_format_is_normalised(tmp_path):
    make_images(tmp_path, 1)
    lab = Labeler(tmp_path / "images", tmp_path / "labels")
    lab.boxes = [(0, 0, 1920, 1080)]  # 整張畫面
    lab.save()
    line = (tmp_path / "labels" / "img_000.txt").read_text().strip()
    cls, cx, cy, w, h = line.split()
    assert cls == "0"
    assert float(cx) == pytest.approx(0.5, abs=0.01)
    assert float(cy) == pytest.approx(0.5, abs=0.01)
    assert float(w) == pytest.approx(1.0, abs=0.01)
    assert float(h) == pytest.approx(1.0, abs=0.01)


def test_empty_label_is_written_for_negative_sample(tmp_path):
    """沒有目標的畫面要存空標籤（負樣本），這是減少誤判的關鍵。"""
    make_images(tmp_path, 1)
    lab = Labeler(tmp_path / "images", tmp_path / "labels")
    lab.boxes = []
    lab.save()
    p = tmp_path / "labels" / "img_000.txt"
    assert p.exists(), "沒有目標時也必須寫出標籤檔，否則該張會被訓練忽略"
    assert p.read_text().strip() == ""


def test_degenerate_boxes_are_dropped(tmp_path):
    make_images(tmp_path, 1)
    lab = Labeler(tmp_path / "images", tmp_path / "labels")
    lab.boxes = [(100, 100, 101, 101), (200, 200, 500, 460)]  # 第一個太小
    lab.save()
    lines = [l for l in (tmp_path / "labels" / "img_000.txt").read_text().splitlines() if l]
    assert len(lines) == 1


def test_resume_skips_to_first_unlabeled(tmp_path):
    make_images(tmp_path, 5)
    (tmp_path / "labels").mkdir()
    for i in range(3):
        (tmp_path / "labels" / f"img_{i:03d}.txt").write_text("")
    lab = Labeler(tmp_path / "images", tmp_path / "labels")
    assert lab.idx == 3, "應從第一張未標註的接續"


def test_split_dataset_creates_yolo_layout(tmp_path):
    make_images(tmp_path, 30, size=(360, 640))
    (tmp_path / "labels").mkdir()
    for i in range(30):
        (tmp_path / "labels" / f"img_{i:03d}.txt").write_text("0 0.5 0.5 0.2 0.2")

    yaml_path = split_dataset(tmp_path, val_ratio=0.2)
    ds = tmp_path / "_yolo"
    n_train = len(list((ds / "images" / "train").glob("*.jpg")))
    n_val = len(list((ds / "images" / "val").glob("*.jpg")))
    assert n_train + n_val == 30
    assert n_val >= 5
    # 每張圖都要有對應標籤，否則訓練會靜默漏掉
    for split in ("train", "val"):
        for img in (ds / "images" / split).glob("*.jpg"):
            assert (ds / "labels" / split / (img.stem + ".txt")).exists()
    assert "names: [Car]" in yaml_path.read_text(encoding="utf-8")


def test_split_refuses_too_few_labels(tmp_path):
    make_images(tmp_path, 5)
    (tmp_path / "labels").mkdir()
    for i in range(5):
        (tmp_path / "labels" / f"img_{i:03d}.txt").write_text("")
    with pytest.raises(SystemExit):
        split_dataset(tmp_path)
