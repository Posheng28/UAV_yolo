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

PROJECT_ROOT = Path(__file__).resolve().parents[2]

# 模型載入失敗後隔多久才重試。用意見 Detector._ensure_model。
MODEL_RETRY_S = 60.0


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

    def __init__(self, weights: str, conf: float, imgsz: int, class_names: list[str],
                 tiling: str = "off", tile_grid: tuple[int, int] | None = None,
                 tile_overlap: float = 0.2):
        self.weights_path = self._resolve_weights(weights)
        self.conf = float(conf)
        self.imgsz = int(imgsz)
        self.allowed_names = class_names
        # 切塊推論：小目標的救命稻草。實測 8m 等效高度下整幀 0%、切塊 88%。
        # 詳見 tiling.py 開頭的量測表。
        self.tiling = tiling            # off | auto | on（setter 會正規化）
        self.tile_grid = tile_grid
        self.tile_overlap = float(tile_overlap)
        self._last_boxes: list[tuple[float, float, float, float]] = []
        self.assigner = StableIdAssigner()
        self.backend_name = "ultralytics"
        self.provider: str | None = None       # ONNX 時是實際的 execution provider
        self.fallback_reason: str | None = None  # 想用 DirectML 卻退回 CPU 的原因
        self._model = None
        self._class_ids: set[int] | None = None
        self._load_error: str | None = None    # 上次載入失敗的原因（見 _ensure_model）
        self._load_error_t = 0.0

    @property
    def tiling(self) -> str:
        return self._tiling

    @tiling.setter
    def tiling(self, value) -> None:
        # YAML 把 off/on 讀成布林，直接拿去比字串會永遠不相等（靜默失效）
        from .tiling import normalise_mode

        self._tiling = normalise_mode(value)

    @property
    def is_onnx(self) -> bool:
        return str(self.weights_path).lower().endswith(".onnx")

    @staticmethod
    def _resolve_weights(weights: str) -> str:
        """自訓權重存在就用，否則退 COCO 預訓練 yolo26n（自動下載）。

        退回時給**絕對路徑**指向 weights/：ultralytics 拿到相對名稱會下載到當下的
        工作目錄，也就是專案根目錄——檔案掉在 repo 根、設定頁的權重下拉又只掃
        weights/，於是每次啟動都重新下載一次，而使用者看不到它在哪。
        """
        if weights and Path(weights).exists():
            return weights
        fallback = PROJECT_ROOT / "weights" / "yolo26n.pt"
        try:
            fallback.parent.mkdir(parents=True, exist_ok=True)
        except OSError:
            return "yolo26n.pt"        # 目錄建不出來就照舊，至少還能跑
        return str(fallback)

    def _apply_class_filter(self, names: dict[int, str]) -> None:
        self._class_ids = build_class_filter(names, self.allowed_names)
        # COCO fallback 時允許 car/truck 同義映射
        if self._class_ids is None and self.allowed_names:
            self._class_ids = build_class_filter(names, ["car", "truck", "bus"])

    def _ensure_model(self):
        """載入模型；失敗的話記住原因，別每幀重試。

        🔴 沒有這個記憶的話：權重不在本機（新 clone 的 weights/ 是空的）而網路
        又不通時，ultralytics 每次呼叫都會重新嘗試下載那 5.3MB 的 COCO 模型——
        而這是在引擎迴圈裡、每一幀都跑一次。整個地面站會卡在逾時上，UI 看起來
        像當掉，而真正的原因（沒有權重、也連不出去）一句都沒說。
        野外沒有網路正是最可能發生的地方。
        """
        if self._model is not None:
            return self._model

        now = time.monotonic()
        if self._load_error is not None and now - self._load_error_t < MODEL_RETRY_S:
            raise RuntimeError(self._load_error)

        try:
            if self.is_onnx:
                from .onnx_backend import OnnxRuntimeBackend

                backend = OnnxRuntimeBackend(self.weights_path)
                backend.warmup()
                self._model = backend
                self.backend_name = "onnx"
                self.provider = backend.provider
                self.fallback_reason = backend.fallback_reason
                self._apply_class_filter(backend.names)
            else:
                from ultralytics import YOLO

                self._model = YOLO(self.weights_path)
                self._apply_class_filter(self._model.names)
        except Exception as exc:
            self._load_error = (
                f"模型載入失敗（{self.weights_path}）：{exc}。"
                f"權重不在本機時會自動下載，需要網路；{int(MODEL_RETRY_S)} 秒後才會再試一次")
            self._load_error_t = now
            raise RuntimeError(self._load_error) from exc

        self._load_error = None
        return self._model

    def set_imgsz(self, value: int) -> str | None:
        """熱套用推論尺寸；不能套用時回傳原因字串（給 UI 顯示）。

        ONNX 是靜態尺寸匯出（DirectML 對 dynamic shape 會明顯變慢），改設定值
        不會有任何效果——這種「改了沒反應也沒提示」正是本專案要避免的靜默失效。
        """
        value = int(value)
        if self.is_onnx:
            if self._model is not None and value != max(self._model.input_hw):
                return (
                    f"ONNX 模型固定為 {self._model.input_hw[1]}x{self._model.input_hw[0]}，"
                    f"要改成 {value} 請重新匯出"
                )
            return None
        self.imgsz = value
        return None

    def _raw_detect(self, frame) -> tuple[list, list]:
        """對一張影像做一次推論，回傳 (bbox 串列, [(類別名, 信心)])。"""
        model = self._ensure_model()
        boxes: list[tuple[float, float, float, float]] = []
        meta: list[tuple[str, float]] = []
        if self.is_onnx:
            raw_boxes, raw_meta = model.infer(frame, self.conf)
            for bbox, (cls_id, conf) in zip(raw_boxes, raw_meta):
                if self._class_ids is not None and cls_id not in self._class_ids:
                    continue
                boxes.append(bbox)
                meta.append((model.names.get(cls_id, str(cls_id)), conf))
        else:
            # 用 predict 而非 track：識別交給 StableIdAssigner，追蹤器的 ID 反而是
            # 框消失的主因（見 tracking.py 開頭的實測），且省下每幀的光流補償運算。
            results = model.predict(frame, conf=self.conf, imgsz=self.imgsz, verbose=False)
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
        return boxes, meta

    def _model_input_hw(self) -> tuple[int, int]:
        """模型實際吃的輸入尺寸。ONNX 是匯出時固定的；.pt 走 rect letterbox。"""
        if self.is_onnx and self._model is not None:
            return self._model.input_hw
        return (int(self.imgsz * 9 / 16), int(self.imgsz))   # 16:9 來源的 rect 尺寸

    def _effective_grid(self, frame_w: int, frame_h: int) -> tuple[int, int]:
        """tile_grid 為 None（auto）時，從模型輸入尺寸推出不會縮小的最小格數。"""
        from . import tiling

        if self.tile_grid is not None:
            return self.tile_grid
        ih, iw = self._model_input_hw()
        return tiling.auto_grid(frame_w, frame_h, iw, ih, self.tile_overlap)

    def _tiled_detect(self, frame) -> tuple[list, list]:
        """整幀 + 各切塊都推論後合併。

        整幀那一次不能省：目標夠大時會被切線切斷（實測從 100% 掉到 52%），
        兩者取聯集再去重才是穩健的做法。
        """
        from . import tiling

        h, w = frame.shape[:2]
        boxes, meta = self._raw_detect(frame)          # 整幀
        items = [(b, m[0], m[1]) for b, m in zip(boxes, meta)]

        nx, ny = self._effective_grid(w, h)
        for x0, y0, tw, th in tiling.tile_rects(w, h, nx, ny, self.tile_overlap):
            tb, tm = self._raw_detect(frame[y0:y0 + th, x0:x0 + tw])
            for b, m in zip(tiling.offset_boxes(tb, x0, y0), tm):
                items.append((b, m[0], m[1]))

        merged = tiling.merge_detections(items)
        return [it[0] for it in merged], [(it[1], it[2]) for it in merged]

    def detect(self, frame, t: float | None = None) -> list[Detection]:
        """偵測本幀並回傳帶穩定 ID 的框；t 為該幀時間戳（秒），供速度外推用。"""
        from . import tiling

        stamp = time.monotonic() if t is None else float(t)
        use_tiles = self.tiling == "on" or (
            self.tiling == "auto"
            and tiling.should_tile(self._last_boxes, frame.shape[1])
        )
        boxes, meta = self._tiled_detect(frame) if use_tiles else self._raw_detect(frame)
        self._last_boxes = list(boxes)

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
        # 夾成 >=1：設定檔曾出現 -3（manual 模式下無害，一旦切成自動就是
        # 「第一幀看到什麼就鎖什麼」，反閃爍確認完全失效）。設定值本來就
        # 可能被手改或從舊檔繼承，門檻這種東西要在程式裡守住。
        self.min_lock_frames = max(1, int(min_lock_frames))
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
                elif self.mode == "manual" and self.locked_id is None:
                    # 只有「沒有現任鎖定」才空等點選的 ID。已有鎖定時不能
                    # return None：那會讓現任目標整整 3 秒收不到量測（KF 進
                    # coast）——一次誤點就把好好追著的目標晾在一旁。
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
