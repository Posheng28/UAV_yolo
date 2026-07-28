"""切塊推論的幾何與合併。

存在理由（實測，見 tiling.py 開頭）：目標縮到約 24px 時整幀推論偵測率是 **0%**，
切塊後回到 88%。空拍小目標不是「信心低一點」，是完全看不到。
"""

import pytest

from uav_yolo.vision.tiling import (auto_grid, iou, merge_detections,
                                    offset_boxes, should_tile, tile_rects)


# ---------------- 自動格數：切塊唯一的效果來源 ----------------

def test_auto_grid_makes_tiles_smaller_than_the_model_input():
    """塊必須比模型輸入小，否則前處理會把它縮小——切了等於沒切。

    這不是理論：實測 1920x1080 進 960x544 的模型，寫死 3x2 切出 768x648
    （縮放 0.84），偵測率只有 10%；auto 推出的 3x3 是 80%。
    """
    W, H, IW, IH, ov = 1920, 1080, 960, 544, 0.2
    nx, ny = auto_grid(W, H, IW, IH, ov)
    x0, y0, tw, th = tile_rects(W, H, nx, ny, ov)[0]
    assert tw <= IW and th <= IH, f"{nx}x{ny} 切出 {tw}x{th}，比模型輸入 {IW}x{IH} 大"
    assert min(IH / th, IW / tw) >= 1.0, "塊會被縮小，切塊完全失去意義"


def test_auto_grid_for_this_aircraft_is_3x3():
    """1920x1080 進 960x544：實測 3x3 是「剛好不縮小」的最小格數。"""
    assert auto_grid(1920, 1080, 960, 544, 0.2) == (3, 3)


def test_auto_grid_needs_no_tiles_when_input_is_large_enough():
    assert auto_grid(1920, 1080, 4096, 4096, 0.2) == (1, 1)


def test_auto_grid_scales_with_how_small_the_input_is():
    small = auto_grid(1920, 1080, 480, 288, 0.2)
    large = auto_grid(1920, 1080, 960, 544, 0.2)
    assert small[0] >= large[0] and small[1] >= large[1]


# ---------------- 切塊幾何 ----------------

def test_tiles_cover_the_whole_frame():
    """漏掉任何一角，目標飛到那裡就消失——而且不會有任何錯誤訊息。"""
    W, H = 1920, 1080
    rects = tile_rects(W, H, 3, 2, overlap=0.2)
    covered = [[False] * 20 for _ in range(20)]
    for x0, y0, tw, th in rects:
        for gy in range(20):
            for gx in range(20):
                px, py = (gx + 0.5) * W / 20, (gy + 0.5) * H / 20
                if x0 <= px <= x0 + tw and y0 <= py <= y0 + th:
                    covered[gy][gx] = True
    missed = [(gx, gy) for gy in range(20) for gx in range(20) if not covered[gy][gx]]
    assert not missed, f"這些位置沒有被任何一塊涵蓋：{missed[:8]}"


def test_tiles_stay_inside_the_frame():
    W, H = 1920, 1080
    for x0, y0, tw, th in tile_rects(W, H, 3, 2):
        assert 0 <= x0 and x0 + tw <= W, "切塊超出畫面右緣"
        assert 0 <= y0 and y0 + th <= H, "切塊超出畫面下緣"


def test_tiles_overlap_so_boundary_targets_survive():
    """沒有重疊的話，剛好在切線上的目標會被兩塊各切一半，兩邊都認不出來。"""
    rects = tile_rects(1920, 1080, 3, 2, overlap=0.2)
    xs = sorted({r[0] for r in rects})
    tw = rects[0][2]
    assert xs[1] < xs[0] + tw, "相鄰塊沒有重疊"


def test_single_tile_is_the_whole_frame():
    assert tile_rects(1920, 1080, 1, 1, 0.0) == [(0, 0, 1920, 1080)]


def test_rejects_nonsense_parameters():
    with pytest.raises(ValueError):
        tile_rects(1920, 1080, 0, 2)
    with pytest.raises(ValueError):
        tile_rects(1920, 1080, 3, 2, overlap=0.95)


# ---------------- 座標還原 ----------------

def test_boxes_map_back_to_frame_coordinates():
    """切塊裡的座標是相對該塊的；沒平移回去，框會全部擠在畫面左上角。"""
    assert offset_boxes([(10.0, 20.0, 50.0, 60.0)], 640, 360) == [(650.0, 380.0, 690.0, 420.0)]


# ---------------- 合併去重 ----------------

def test_same_object_seen_by_two_tiles_becomes_one():
    """重疊區的目標會被相鄰兩塊各偵測一次，不合併就會變成兩台車。"""
    a = ((100.0, 100.0, 200.0, 200.0), "Car", 0.9)
    b = ((105.0, 102.0, 203.0, 198.0), "Car", 0.7)   # 幾乎同一個框
    assert len(merge_detections([a, b])) == 1


def test_merge_keeps_the_more_confident_box():
    a = ((100.0, 100.0, 200.0, 200.0), "Car", 0.6)
    b = ((104.0, 101.0, 202.0, 199.0), "Car", 0.95)
    kept = merge_detections([a, b])
    assert len(kept) == 1 and kept[0][2] == pytest.approx(0.95)


def test_two_real_cars_are_not_merged():
    a = ((100.0, 100.0, 200.0, 200.0), "Car", 0.9)
    b = ((900.0, 700.0, 1000.0, 800.0), "Car", 0.8)
    assert len(merge_detections([a, b])) == 2


def test_different_classes_are_never_merged():
    a = ((100.0, 100.0, 200.0, 200.0), "Car", 0.9)
    b = ((101.0, 101.0, 199.0, 199.0), "Truck", 0.8)
    assert len(merge_detections([a, b])) == 2


def test_iou_basics():
    box = (0.0, 0.0, 10.0, 10.0)
    assert iou(box, box) == pytest.approx(1.0)
    assert iou(box, (20.0, 20.0, 30.0, 30.0)) == pytest.approx(0.0)
    assert iou(box, (5.0, 0.0, 15.0, 10.0)) == pytest.approx(50 / 150)


# ---------------- auto 模式的判斷 ----------------

def test_auto_tiles_when_target_is_small():
    assert should_tile([(0.0, 0.0, 50.0, 40.0)], 1920) is True


def test_auto_skips_tiling_when_target_is_large():
    """目標夠大時切塊反而把它切斷（實測 100%→52%），而且白花數倍算力。"""
    assert should_tile([(0.0, 0.0, 400.0, 300.0)], 1920) is False


def test_auto_tiles_when_nothing_was_detected():
    """什麼都沒看到，正是目標太小的典型症狀——這時候更該切塊。"""
    assert should_tile([], 1920) is True
