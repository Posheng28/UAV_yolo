"""YOLO 偵測與單一目標鎖定。

修正舊系統「只拿第一個框就 break、多車亂跳」的問題：
    - 使用 model.track 的追蹤 ID，鎖定單一 ID 跟到底。
    - auto 模式：同一 ID 連續出現 min_lock_frames 幀才鎖定（承襲原規格）。
    - manual 模式：UI 點選畫面上的框才鎖定。
    - 測地點用 bbox「底邊中點」（車輛接地位置），斜視角時比中心點準。
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path


@dataclass
class Detection:
    track_id: int
    cls_name: str
    conf: float
    bbox: tuple[float, float, float, float]  # x1, y1, x2, y2

    @property
    def ground_pixel(self) -> tuple[float, float]:
        """接地參考像素：底邊中點。"""
        x1, y1, x2, y2 = self.bbox
        return (x1 + x2) / 2.0, y2

    @property
    def area(self) -> float:
        x1, y1, x2, y2 = self.bbox
        return max(0.0, x2 - x1) * max(0.0, y2 - y1)


def build_class_filter(model_names: dict[int, str], allowed: list[str]) -> set[int] | None:
    """允許類別名 → 類別 id 集合（不分大小寫）。

    對不上任何類別時回 None（=不過濾），避免換權重（如 COCO fallback）後全滅。
    """
    if not allowed:
        return None
    lookup = {name.lower(): idx for idx, name in model_names.items()}
    ids = {lookup[n.lower()] for n in allowed if n.lower() in lookup}
    return ids or None


class Detector:
    """ultralytics YOLO 包裝：延遲載入、track 模式、類別過濾。"""

    def __init__(self, weights: str, conf: float, imgsz: int, class_names: list[str]):
        self.weights_path = self._resolve_weights(weights)
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.allowed_names = class_names
        self._model = None
        self._class_ids: set[int] | None = None

    @staticmethod
    def _resolve_weights(weights: str) -> str:
        """自訓權重存在就用，否則退 COCO 預訓練 yolo26n（自動下載）。"""
        if weights and Path(weights).exists():
            return weights
        return "yolo26n.pt"

    def _ensure_model(self):
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(self.weights_path)
            self._class_ids = build_class_filter(self._model.names, self.allowed_names)
            # COCO fallback 時允許 car/truck 同義映射
            if self._class_ids is None and self.allowed_names:
                coco_alias = {"car", "truck", "bus"}
                self._class_ids = build_class_filter(
                    self._model.names, [n for n in coco_alias]
                )
        return self._model

    def detect(self, frame) -> list[Detection]:
        model = self._ensure_model()
        results = model.track(
            frame, persist=True, conf=self.conf, imgsz=self.imgsz, verbose=False
        )
        detections: list[Detection] = []
        for r in results:
            if r.boxes is None or r.boxes.id is None:
                continue
            names = r.names
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if self._class_ids is not None and cls_id not in self._class_ids:
                    continue
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                detections.append(
                    Detection(
                        track_id=int(box.id[0]),
                        cls_name=str(names.get(cls_id, cls_id)),
                        conf=float(box.conf[0]),
                        bbox=(x1, y1, x2, y2),
                    )
                )
        return detections


class TargetLock:
    """單一目標鎖定狀態機（純邏輯，可獨立測試）。"""

    def __init__(self, mode: str = "auto", min_lock_frames: int = 6):
        self.mode = mode  # auto | manual
        self.min_lock_frames = int(min_lock_frames)
        self.locked_id: int | None = None
        self.pending_manual_id: int | None = None
        self._candidate_id: int | None = None
        self._candidate_streak = 0

    @property
    def locked(self) -> bool:
        return self.locked_id is not None

    def request_manual_lock(self, track_id: int) -> None:
        self.pending_manual_id = int(track_id)

    def unlock(self) -> None:
        self.locked_id = None
        self._candidate_id = None
        self._candidate_streak = 0
        self.pending_manual_id = None

    def update(self, detections: list[Detection]) -> Detection | None:
        """每幀呼叫，回傳目前鎖定目標的偵測（本幀沒看到回 None）。"""
        by_id = {d.track_id: d for d in detections}

        # UI 手動指定（auto 模式也允許點選改鎖）
        if self.pending_manual_id is not None:
            if self.pending_manual_id in by_id:
                self.locked_id = self.pending_manual_id
                self.pending_manual_id = None
            elif self.mode == "manual":
                return None  # 等點選的 ID 出現

        if self.locked_id is not None:
            return by_id.get(self.locked_id)

        if self.mode != "auto" or not detections:
            self._candidate_streak = 0
            return None

        # auto：最大框連續 N 幀才鎖定（防單幀誤偵測）
        best = max(detections, key=lambda d: d.area)
        if best.track_id == self._candidate_id:
            self._candidate_streak += 1
        else:
            self._candidate_id = best.track_id
            self._candidate_streak = 1
        if self._candidate_streak >= self.min_lock_frames:
            self.locked_id = self._candidate_id
            return by_id.get(self.locked_id)  # 鎖定當幀立即回傳，讓第一筆量測不延遲
        return None
