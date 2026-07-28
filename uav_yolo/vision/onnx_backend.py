"""ONNX Runtime 推論後端（可用 DirectML 跑 AMD GPU）。

為什麼要自己寫，而不是 `YOLO("model.onnx")`
------------------------------------------------
ultralytics 8.4.104 把 execution provider 寫死成 CUDA / CoreML / CPU
（`ultralytics/nn/backends/onnx.py`），**沒有 DirectML**，所以走它的 ONNX 路徑
在 AMD 上只會拿到 CPU＝白做。更糟的是它的 `check_requirements(("onnx",
"onnxruntime"))` 用發行版名稱判斷是否安裝，而我們裝的是 `onnxruntime-directml`
（dist 名對不上）→ AUTOINSTALL 會 `pip install onnxruntime` 直接把 DirectML 版
蓋掉（兩者裝進同一個 site-packages/onnxruntime/ 目錄）。

前後處理也刻意自己實作：letterbox 與反算只有十幾行，寫在這裡就能單元測試、
也不會哪天被 ultralytics 內部改動咬到（`boxes.id` 那次就是這樣栽的）。

模型必須用 `end2end=True` 匯出（YOLO26 預設就是），輸出 `(1, N, 6)` =
[x1, y1, x2, y2, conf, cls]，NMS 已內建在圖裡——這同時避開了 DirectML
沒有 NonMaxSuppression kernel 的問題。
"""

from __future__ import annotations

import ast

import cv2
import numpy as np


def export_input_size(src_hw: tuple[int, int], imgsz: int, stride: int = 32) -> tuple[int, int]:
    """算出 ONNX 應該匯出的輸入尺寸 (高, 寬)，對齊 ultralytics 的 rect letterbox。

    ultralytics predict 走 rect 模式：長邊縮到 imgsz，短邊按比例縮後**補到 stride
    的倍數**（不是補成正方形，也不是任意比例）。匯出尺寸沒對齊就等於網路看到的
    尺度和 .pt 路徑不同——實測 1920x1080 @960 匯出成 576（而非正確的 544）時，
    框位置就偏掉約 10px。

        >>> export_input_size((1080, 1920), 640)
        (384, 640)
        >>> export_input_size((1080, 1920), 960)
        (544, 960)
    """
    h0, w0 = src_hw
    r = imgsz / max(h0, w0)
    long_side, short_side = imgsz, int(np.ceil(min(h0, w0) * r / stride)) * stride
    return (short_side, long_side) if w0 >= h0 else (long_side, short_side)


def letterbox(frame, out_hw: tuple[int, int]) -> tuple[np.ndarray, float, float, float]:
    """等比縮放置中補邊到固定尺寸，回傳 (影像, 縮放比, 左padding, 上padding)。"""
    out_h, out_w = out_hw
    h0, w0 = frame.shape[:2]
    r = min(out_h / h0, out_w / w0)
    nw, nh = int(round(w0 * r)), int(round(h0 * r))
    resized = cv2.resize(frame, (nw, nh), interpolation=cv2.INTER_LINEAR)
    pad_x, pad_y = (out_w - nw) / 2.0, (out_h - nh) / 2.0
    top, left = int(round(pad_y - 0.1)), int(round(pad_x - 0.1))
    bottom, right = out_h - nh - top, out_w - nw - left
    padded = cv2.copyMakeBorder(
        resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114)
    )
    return padded, r, float(left), float(top)


class OnnxRuntimeBackend:
    """載入 .onnx 並在 DirectML（有的話）上推論。"""

    def __init__(self, path: str, prefer_dml: bool = True):
        import onnxruntime as ort

        so = ort.SessionOptions()
        # 這兩項是 DirectML 官方要求，不設會有正確性/效能問題
        so.enable_mem_pattern = False
        so.execution_mode = ort.ExecutionMode.ORT_SEQUENTIAL

        available = ort.get_available_providers()
        want = (
            ["DmlExecutionProvider", "CPUExecutionProvider"]
            if prefer_dml and "DmlExecutionProvider" in available
            else ["CPUExecutionProvider"]
        )
        self.session = ort.InferenceSession(path, so, providers=want)
        self.provider = self.session.get_providers()[0]
        # 「我以為在跑 GPU、其實退回 CPU」是最容易自欺的失效——一定要能查
        self.fallback_reason = (
            None if self.provider == "DmlExecutionProvider"
            else f"DirectML 不可用，退回 {self.provider}"
        )

        inp = self.session.get_inputs()[0]
        self.input_name = inp.name
        if any(not isinstance(d, int) for d in inp.shape[2:]):
            raise ValueError(
                f"{path} 是 dynamic shape 匯出；請改用 dynamic=False 重匯出"
                "（DirectML 對 free dimension 會明顯變慢）"
            )
        self.input_hw = (int(inp.shape[2]), int(inp.shape[3]))

        meta = self.session.get_modelmeta().custom_metadata_map or {}
        if "names" not in meta:
            raise ValueError(
                f"{path} 缺少 ultralytics 的 names metadata；"
                "請用 YOLO(...).export(format='onnx') 重匯出"
            )
        self.names = {int(k): str(v) for k, v in ast.literal_eval(meta["names"]).items()}

        args = ast.literal_eval(meta.get("args", "{}"))
        self.end2end = str(meta.get("end2end", "False")) == "True" or bool(args.get("nms"))
        if not self.end2end:
            raise ValueError(
                f"{path} 不是 end2end 匯出（圖裡沒有 NMS）。本後端只支援 end2end，"
                "因為 DirectML 沒有 NonMaxSuppression kernel。"
            )

    def warmup(self, n: int = 3) -> None:
        """DirectML 首次執行要編譯 shader，冷啟動可達數秒——先在載入時付掉。"""
        blob = np.zeros((1, 3, *self.input_hw), dtype=np.float32)
        for _ in range(n):
            self.session.run(None, {self.input_name: blob})

    def infer(self, frame, conf: float):
        """回傳 (bbox 串列[原始幀座標], [(cls_id, conf), ...])。"""
        padded, r, pad_x, pad_y = letterbox(frame, self.input_hw)
        blob = padded[..., ::-1].transpose(2, 0, 1)          # BGR→RGB, HWC→CHW
        blob = np.ascontiguousarray(blob, dtype=np.float32)[None] / 255.0
        out = self.session.run(None, {self.input_name: blob})[0][0]  # (N, 6)

        h0, w0 = frame.shape[:2]
        boxes, meta = [], []
        for x1, y1, x2, y2, c, cls in out:
            if c < conf:
                continue  # end2end 輸出已按信心排序，但仍逐筆檢查較穩健
            bx1 = float(np.clip((x1 - pad_x) / r, 0, w0))
            by1 = float(np.clip((y1 - pad_y) / r, 0, h0))
            bx2 = float(np.clip((x2 - pad_x) / r, 0, w0))
            by2 = float(np.clip((y2 - pad_y) / r, 0, h0))
            boxes.append((bx1, by1, bx2, by2))
            meta.append((int(cls), float(c)))
        return boxes, meta
