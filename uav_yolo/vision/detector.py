"""YOLO 偵測與單一目標鎖定。

修正舊系統「只拿第一個框就 break、多車亂跳」的問題：
    - 每個框都指派一個跨幀穩定的 ID，鎖定單一 ID 跟到底。
    - auto 模式：同一 ID 連續出現 min_lock_frames 幀才鎖定（承襲原規格）。
    - manual 模式：UI 點選畫面上的框才鎖定。
    - 測地點用 bbox「底邊中點」（車輛接地位置），斜視角時比中心點準。

識別碼刻意不用 ultralytics 追蹤器的 `boxes.id`：它在目標移動快時給不出 ID
（實測 200 幀移動序列只有 2% 拿得到），而舊版「沒 ID 就丟掉」會讓框在車一動
時整段消失。理由與實測數據見 tracking.StableIdAssigner。
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from pathlib import Path

from .tracking import StableIdAssigner


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
    """ultralytics YOLO 包裝：延遲載入、類別過濾、穩定 ID 指派。"""

    def __init__(self, weights: str, conf: float, imgsz: int, class_names: list[str]):
        self.weights_path = self._resolve_weights(weights)
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.allowed_names = class_names
        self.assigner = StableIdAssigner()
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

    def detect(self, frame, t: float | None = None) -> list[Detection]:
        """偵測本幀並回傳帶穩定 ID 的框；t 為該幀時間戳（秒），供速度外推用。"""
        model = self._ensure_model()
        # 用 predict 而非 track：識別交給 StableIdAssigner，追蹤器的 ID 反而是
        # 框消失的主因（見 tracking.py 開頭的實測），且省下每幀的光流補償運算。
        results = model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)
        stamp = time.monotonic() if t is None else float(t)

        boxes: list[tuple[float, float, float, float]] = []
        meta: list[tuple[str, float]] = []
        for r in results:
            if r.boxes is None:
                continue
            names = r.names
            for box in r.boxes:
                cls_id = int(box.cls[0])
                if self._class_ids is not None and cls_id not in self._class_ids:
                    continue
                x1, y1, x2, y2 = map(float, box.xyxy[0])
                boxes.append((x1, y1, x2, y2))
                meta.append((str(names.get(cls_id, cls_id)), float(box.conf[0])))

        sids = self.assigner.assign(boxes, stamp)
        return [
            Detection(track_id=sid, cls_name=cls_name, conf=conf, bbox=bbox)
            for sid, bbox, (cls_name, conf) in zip(sids, boxes, meta)
        ]


class TargetLock:
    """單一目標鎖定狀態機（純邏輯，可獨立測試）。"""

    PENDING_EXPIRE_FRAMES = 60  # 點選的 ID 若 ~3 秒內沒出現就放棄（防舊 ID 之後亂劫持）
    REACQUIRE_FRAMES = 90       # 鎖定 ID 消失後，還願意用影像位置重新綁定的幀數
    REACQUIRE_GATE_RADII = 4.0  # 重綁距離門檻，單位＝目標半徑（隨失聯時間放寬）

    def __init__(self, mode: str = "auto", min_lock_frames: int = 6):
        self.mode = mode  # auto | manual
        self.min_lock_frames = int(min_lock_frames)
        self.locked_id: int | None = None
        self.pending_manual_id: int | None = None
        self._pending_age = 0
        self._candidate_id: int | None = None
        self._candidate_streak = 0
        # 鎖定目標最後一次出現的影像位置與尺寸；ID 死掉時靠它重新綁定
        self._last_box: tuple[float, float, float, float] | None = None
        self._miss_age = 0

    @property
    def locked(self) -> bool:
        return self.locked_id is not None

    def request_manual_lock(self, track_id: int) -> None:
        self.pending_manual_id = int(track_id)
        self._pending_age = 0

    def unlock(self) -> None:
        self.locked_id = None
        self._candidate_id = None
        self._candidate_streak = 0
        self.pending_manual_id = None
        self._pending_age = 0
        self._last_box = None
        self._miss_age = 0

    def _remember(self, det: Detection) -> None:
        self._last_box = det.bbox
        self._miss_age = 0

    def _reacquire_by_image(self, detections: list[Detection]) -> Detection | None:
        """鎖定的 ID 不見了 → 用「最後出現的影像位置」找回同一個目標。

        沒有 GPS/遙測時這是唯一能重鎖的依據（世界座標重鎖需要 home 與位置，
        室內測試根本拿不到，等於永遠不會執行）。門檻用目標半徑的倍數表示，
        並隨失聯幀數放寬——目標可能一直在動。
        """
        if self._last_box is None or not detections:
            return None
        x1, y1, x2, y2 = self._last_box
        cx, cy = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        w, h = max(1e-6, x2 - x1), max(1e-6, y2 - y1)
        radius = 0.5 * (w + h) / 2.0
        gate = self.REACQUIRE_GATE_RADII * (1.0 + 0.05 * self._miss_age)

        best, best_cost = None, gate
        for d in detections:
            bx1, by1, bx2, by2 = d.bbox
            bw, bh = max(1e-6, bx2 - bx1), max(1e-6, by2 - by1)
            if max(bw / w, w / bw, bh / h, h / bh) > 3.0:
                continue
            dist = (((bx1 + bx2) / 2.0 - cx) ** 2 + ((by1 + by2) / 2.0 - cy) ** 2) ** 0.5
            cost = dist / max(radius, 1e-6)
            if cost < best_cost:
                best, best_cost = d, cost
        return best

    def update(self, detections: list[Detection]) -> Detection | None:
        """每幀呼叫，回傳目前鎖定目標的偵測（本幀沒看到回 None）。"""
        by_id = {d.track_id: d for d in detections}

        # UI 手動指定（auto 模式也允許點選改鎖）
        if self.pending_manual_id is not None:
            if self.pending_manual_id in by_id:
                self.locked_id = self.pending_manual_id
                self.pending_manual_id = None
                self._pending_age = 0
            else:
                # 點選的 ID 一直沒出現就放棄：ByteTrack ID 不會回收，
                # 但偵測器重啟後可能重複——過期的 pending 之後突然「劫持」鎖定很危險
                self._pending_age += 1
                if self._pending_age >= self.PENDING_EXPIRE_FRAMES:
                    self.pending_manual_id = None
                    self._pending_age = 0
                elif self.mode == "manual":
                    return None  # 等點選的 ID 出現

        if self.locked_id is not None:
            det = by_id.get(self.locked_id)
            if det is not None:
                self._remember(det)
                return det
            # 鎖定的 ID 消失了。可能是目標被遮蔽/離開畫面，也可能只是識別斷了；
            # 先用最後的影像位置嘗試接回同一個目標，接不回才算本幀沒看到。
            self._miss_age += 1
            if self._miss_age <= self.REACQUIRE_FRAMES:
                cand = self._reacquire_by_image(detections)
                if cand is not None:
                    self.locked_id = cand.track_id
                    self._remember(cand)
                    return cand
            return None

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
            det = by_id.get(self.locked_id)  # 鎖定當幀立即回傳，讓第一筆量測不延遲
            if det is not None:
                self._remember(det)
            return det
        return None
