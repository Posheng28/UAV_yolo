"""ONNX / DirectML 推論後端。

會跑真模型的測試在沒有 weights/toycar.onnx 或沒裝 onnxruntime 時自動跳過，
純幾何的 letterbox 測試永遠會跑（它才是座標算錯時的第一道防線）。
"""

from pathlib import Path

import numpy as np
import pytest

from uav_yolo.vision.detector import Detector
from uav_yolo.vision.onnx_backend import export_input_size, letterbox

ROOT = Path(__file__).resolve().parent.parent
ONNX = ROOT / "weights" / "toycar.onnx"


# ---------------- 匯出尺寸（踩過的雷） ----------------

def test_export_input_size_rounds_short_side_up_to_stride():
    """短邊要補到 stride 32 的倍數，不是任意比例。

    實際踩過：1920x1080 @960 我算成 576（=960×0.6），正確是 544
    （540 補到 32 的倍數）。匯出尺寸不對，網路看到的尺度就與 .pt 路徑不同，
    框位置偏掉約 10px——而且畫面看起來完全正常，只有對拍才驗得出來。
    """
    assert export_input_size((1080, 1920), 640) == (384, 640)
    assert export_input_size((1080, 1920), 960) == (544, 960)
    assert export_input_size((1080, 1920), 1280) == (736, 1280)


def test_export_input_size_handles_portrait_and_square():
    assert export_input_size((1920, 1080), 640) == (640, 384)   # 直立
    assert export_input_size((720, 720), 640) == (640, 640)     # 正方


@pytest.mark.skipif(not ONNX.exists(), reason="尚未匯出 weights/toycar.onnx")
def test_exported_model_matches_the_computed_size():
    """實際匯出的模型必須符合上面的規則，否則就是又手算錯了。"""
    import onnxruntime as ort

    shape = ort.InferenceSession(str(ONNX), providers=["CPUExecutionProvider"]).get_inputs()[0].shape
    h, w = int(shape[2]), int(shape[3])
    assert (h, w) == export_input_size((1080, 1920), w), (
        f"匯出尺寸 {w}x{h} 與 1920x1080 來源的 rect letterbox 不符"
    )


# ---------------- letterbox 幾何（不需要模型） ----------------

def test_letterbox_pads_16x9_source_without_distortion():
    """1920x1080 進 640x384：等比縮到 640x360，上下各補 12px。

    比例算錯就等於整幀被拉伸，框會系統性偏移——而且畫面看起來完全正常。
    """
    frame = np.zeros((1080, 1920, 3), np.uint8)
    out, r, pad_x, pad_y = letterbox(frame, (384, 640))
    assert out.shape[:2] == (384, 640)
    assert r == pytest.approx(640 / 1920)
    assert pad_x == 0
    assert pad_y == pytest.approx(12)


def test_letterbox_inverse_recovers_original_coordinates():
    """把框從 letterbox 座標反算回原始幀，必須落回原處。"""
    frame = np.zeros((1080, 1920, 3), np.uint8)
    _, r, pad_x, pad_y = letterbox(frame, (384, 640))

    for orig in [(0.0, 0.0, 100.0, 80.0), (900.0, 500.0, 1010.0, 560.0),
                 (1820.0, 1000.0, 1920.0, 1080.0)]:
        x1, y1, x2, y2 = orig
        # 正向：原始 → letterbox
        lb = (x1 * r + pad_x, y1 * r + pad_y, x2 * r + pad_x, y2 * r + pad_y)
        # 反向：letterbox → 原始（backend.infer 用的公式）
        back = tuple((v - p) / r for v, p in zip(lb, (pad_x, pad_y, pad_x, pad_y)))
        assert back == pytest.approx(orig, abs=1e-6)


def test_letterbox_handles_square_source():
    frame = np.zeros((720, 720, 3), np.uint8)
    out, r, pad_x, pad_y = letterbox(frame, (384, 640))
    assert out.shape[:2] == (384, 640)
    assert r == pytest.approx(384 / 720)
    assert pad_y == 0
    assert pad_x > 0


# ---------------- Detector 分流 ----------------

def test_detector_routes_by_extension():
    assert Detector("weights/toycar.onnx", 0.5, 640, ["Car"]).is_onnx
    assert not Detector("weights/toycar.pt", 0.5, 640, ["Car"]).is_onnx


def test_set_imgsz_updates_pytorch_backend():
    d = Detector("weights/does-not-exist.pt", conf=0.5, imgsz=1280, class_names=["Car"])
    assert d.set_imgsz(640) is None
    assert d.imgsz == 640


@pytest.mark.skipif(not ONNX.exists(), reason="尚未匯出 weights/toycar.onnx")
def test_set_imgsz_refuses_to_pretend_on_static_onnx():
    """ONNX 尺寸是匯出時固定的。改設定值不會有效果——必須回報原因。

    默默忽略就是本專案最忌諱的靜默失效：操作員以為調了推論尺寸，
    實際上模型連理都沒理。
    """
    d = Detector(str(ONNX), conf=0.5, imgsz=640, class_names=["Car"])
    d._ensure_model()
    exported = max(d._model.input_hw)        # 尺寸要跟模型問，不能寫死在測試裡
    assert d.set_imgsz(exported) is None     # 與匯出尺寸相符 → 沒問題
    reason = d.set_imgsz(exported * 2)
    assert reason and "重新匯出" in reason


@pytest.mark.skipif(not ONNX.exists(), reason="尚未匯出 weights/toycar.onnx")
def test_onnx_detector_returns_same_detection_shape():
    """ONNX 路徑要回傳與 PyTorch 路徑同形狀的 Detection，下游才不用改。"""
    import cv2

    imgs = sorted((ROOT / "data" / "toycar" / "images").glob("*.jpg"))
    if not imgs:
        pytest.skip("沒有測試影像")
    frame = cv2.imread(str(imgs[0]))

    d = Detector(str(ONNX), conf=0.45, imgsz=640, class_names=["Car"])
    dets = d.detect(frame, t=0.0)
    for det in dets:
        assert isinstance(det.track_id, int)
        assert det.cls_name == "Car"
        assert 0.0 <= det.conf <= 1.0
        x1, y1, x2, y2 = det.bbox
        assert 0 <= x1 < x2 <= frame.shape[1]
        assert 0 <= y1 < y2 <= frame.shape[0]


@pytest.mark.skipif(not ONNX.exists(), reason="尚未匯出 weights/toycar.onnx")
def test_onnx_and_pytorch_agree_on_the_same_frames():
    """兩條路徑必須給出實質相同的框。

    速度再快，框跑掉就不能上機——DirectML 與 CPU 的數值差異是真實風險。
    """
    import cv2

    imgs = sorted((ROOT / "data" / "toycar" / "images").glob("*.jpg"))[:15]
    pt = ROOT / "weights" / "toycar.pt"
    if not imgs or not pt.exists():
        pytest.skip("缺少比對用的影像或 .pt 權重")

    b = Detector(str(ONNX), conf=0.45, imgsz=640, class_names=["Car"])
    b._ensure_model()
    # .pt 要用與 ONNX 匯出相同的推論尺寸，否則比的是兩種尺寸不是兩種後端
    a = Detector(str(pt), conf=0.45, imgsz=max(b._model.input_hw), class_names=["Car"])
    for i, p in enumerate(imgs):
        frame = cv2.imread(str(p))
        da = sorted(a.detect(frame, i / 30.0), key=lambda d: d.bbox)
        db = sorted(b.detect(frame, i / 30.0), key=lambda d: d.bbox)
        assert len(da) == len(db), f"{p.name}: 框數不同 {len(da)} vs {len(db)}"
        for x, y in zip(da, db):
            assert np.allclose(x.bbox, y.bbox, atol=2.0), f"{p.name}: 框位置偏差過大"
