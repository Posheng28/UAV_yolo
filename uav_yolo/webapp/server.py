"""地面站 Web UI 伺服器（FastAPI）。

    GET  /                    儀表板（單頁應用）
    GET  /stream.mjpg         即時影像（MJPEG，含偵測疊加）
    GET  /api/status          引擎狀態（前端 400ms 輪詢）
    POST /api/guidance        {"enabled": bool} 導引總開關
    POST /api/lock            {"track_id": int} 手動鎖定
    POST /api/unlock          解鎖
    GET/PUT /api/config       讀/寫設定（寫入 config/local.yaml）
    POST /api/restart         用最新設定重建引擎
    GET  /api/devices         影像裝置清單
    檢查清單與相機校正端點見下。
"""

from __future__ import annotations

import dataclasses
import threading
import time
from pathlib import Path

import cv2
from fastapi import Body, FastAPI
from fastapi.responses import FileResponse, JSONResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles

from ..config import (
    DEFAULT_INTRINSICS_FILE,
    DEFAULT_WEIGHTS_FILE,
    PROJECT_ROOT,
    Config,
)
from ..engine import create_engine
from ..vision.calibration import CalibrationSession
from ..vision.source import list_video_devices
from .checklist import Checklist

STATIC_DIR = Path(__file__).parent / "static"


def _ui_version() -> str:
    """UI 資產指紋（檔案 mtime 總和）。前端比對它，變了就自動重載頁面——
    根治「伺服器更新了、瀏覽器還跑舊 JS」這類看過三次的災難。"""
    total = 0
    for f in STATIC_DIR.glob("*"):
        try:
            total ^= int(f.stat().st_mtime_ns) & 0xFFFFFFFF
        except OSError:
            pass
    return f"{total:08x}"


class EngineManager:
    """持有目前引擎；設定變更後可整組重建。"""

    def __init__(self, cfg: Config):
        self.cfg = cfg
        self._lock = threading.Lock()
        self.engine = None

    def start(self) -> None:
        with self._lock:
            self.engine = create_engine(self.cfg)
            self.engine.start()

    def restart(self) -> None:
        with self._lock:
            if self.engine is not None:
                self.engine.stop()
            self.cfg.reload()
            self.engine = create_engine(self.cfg)
            self.engine.start()


def create_app(cfg: Config | None = None) -> FastAPI:
    cfg = cfg or Config()
    manager = EngineManager(cfg)
    checklist = Checklist()
    calib: dict = {"session": None}

    app = FastAPI(title="UAV_yolo 地面站")
    app.state.manager = manager
    ui_version = _ui_version()  # 啟動時定版；重啟後檔案有變=新指紋

    @app.on_event("startup")
    def _startup():
        manager.start()

    @app.on_event("shutdown")
    def _shutdown():
        if manager.engine:
            manager.engine.stop()

    # ---------------- 頁面與影像 ----------------

    @app.get("/")
    def index():
        return FileResponse(STATIC_DIR / "index.html")

    @app.get("/frame.jpg")
    def frame():
        """單幀 JPEG。前端用輪詢這個端點取代 MJPEG 長連線——
        任何伺服器/引擎重啟都能自動恢復，不會卡在死掉的串流上。"""
        engine = manager.engine
        jpeg = engine.jpeg_frame() if engine else None
        if jpeg is None:
            return JSONResponse({"error": "no frame"}, status_code=503)
        from fastapi.responses import Response

        return Response(
            content=jpeg,
            media_type="image/jpeg",
            headers={"Cache-Control": "no-store, no-cache, must-revalidate"},
        )

    @app.get("/frame_raw.jpg")
    def frame_raw():
        """未疊加的原始幀。**訓練資料一定要用這個**——
        /frame.jpg 帶有偵測框與狀態文字，拿去訓練會讓模型學到
        「綠框＝車」這種災難性關聯。"""
        import cv2
        from fastapi.responses import Response

        engine = manager.engine
        frame = engine.raw_frame() if engine else None
        if frame is None:
            return JSONResponse({"error": "no frame"}, status_code=503)
        ok, buf = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, 92])
        if not ok:
            return JSONResponse({"error": "encode failed"}, status_code=500)
        return Response(content=buf.tobytes(), media_type="image/jpeg",
                        headers={"Cache-Control": "no-store"})

    @app.get("/stream.mjpg")
    def stream():
        boundary = b"--frame\r\nContent-Type: image/jpeg\r\n\r\n"

        def gen():
            while True:
                engine = manager.engine
                jpeg = engine.jpeg_frame() if engine else None
                if jpeg is not None:
                    yield boundary + jpeg + b"\r\n"
                time.sleep(0.07)  # ~14 fps 串流即可

        return StreamingResponse(gen(), media_type="multipart/x-mixed-replace; boundary=frame")

    # ---------------- 狀態與操作 ----------------

    @app.get("/api/status")
    def status():
        engine = manager.engine
        if engine is None:
            return {"ready": False, "ui_version": ui_version}
        data = dataclasses.asdict(engine.status())
        data["ready"] = True
        data["ui_version"] = ui_version
        return data

    @app.post("/api/guidance")
    def set_guidance(body: dict = Body(...)):
        if manager.engine:
            manager.engine.set_guidance_enabled(bool(body.get("enabled")))
        return {"ok": True}

    @app.post("/api/lock")
    def lock(body: dict = Body(...)):
        if manager.engine and "track_id" in body:
            manager.engine.manual_lock(int(body["track_id"]))
        return {"ok": True}

    @app.post("/api/unlock")
    def unlock():
        if manager.engine:
            manager.engine.unlock()
        return {"ok": True}

    # ---------------- 設定 ----------------

    # 已知權重的用途說明（下拉選單顯示用）。實測結論見 integration/TRAIN_YOUR_TARGET.md：
    # 只用玩具車微調的模型會 100% 遺忘真車（44→0），所以必須保留多個模型切換。
    WEIGHT_NOTES = {
        "toycar.pt": "室內玩具車（自訓 200 張＠960）— 真車無效",
        "toycar_v1_640.pt": "玩具車舊版（67 張＠640）— 已被 toycar.pt 取代，留作對照",
        "best.pt": "空拍真車（自訓）— 玩具車無效",
        "yolo26n.pt": "COCO 通用 nano — 真實車輛/行人等 80 類",
        "yolo11s.pt": "COCO 通用 small — 較準但較慢",
    }

    def _list_weights() -> list[dict]:
        wdir = PROJECT_ROOT / "weights"
        out = []
        if wdir.is_dir():
            for p in sorted(list(wdir.glob("*.pt")) + list(wdir.glob("*.onnx"))):
                # .onnx 是同名 .pt 匯出的，用途說明沿用；後綴標明它走 GPU 推論
                note = WEIGHT_NOTES.get(p.name) or WEIGHT_NOTES.get(p.stem + ".pt", "")
                if p.suffix == ".onnx":
                    note = (note + "｜ONNX：DirectML GPU 推論、尺寸固定").lstrip("｜")
                out.append({
                    "path": f"weights/{p.name}",
                    "name": p.name,
                    "size_mb": round(p.stat().st_size / 1e6, 1),
                    "note": note,
                })
        return out

    @app.get("/api/weights")
    def list_weights():
        return {"weights": _list_weights()}

    @app.get("/api/config")
    def get_config():
        weights = cfg.get("detector.weights") or DEFAULT_WEIGHTS_FILE
        weights_exists = (PROJECT_ROOT / weights).exists()
        intrinsics = PROJECT_ROOT / (cfg.get("camera.intrinsics_file") or DEFAULT_INTRINSICS_FILE)
        return {
            "config": cfg.as_dict(),
            "meta": {
                "weights_exists": weights_exists,
                "calibrated": intrinsics.exists(),
                "devices": list_video_devices(),
                "weights": _list_weights(),   # 供設定頁做模型下拉選單
            },
        }

    @app.put("/api/config")
    def put_config(body: dict = Body(...)):
        cfg.update(body)
        # 安全/導引/偵測/延遲這些門檻立刻套用到執行中的引擎——
        # 操作員把圍欄改嚴按下儲存，就必須當場生效，不能等重啟。
        applied: list[str] = []
        if manager.engine is not None:
            try:
                applied = manager.engine.apply_live_config()
            except Exception as exc:  # 熱套用失敗不該讓儲存整個失敗
                return {"ok": True, "note": f"已存檔，但熱套用失敗（請按重啟引擎）：{exc}"}
        return {
            "ok": True,
            "applied": applied,
            "note": "已存檔並即時套用安全/導引/偵測門檻；標 ⟳ 的項目"
                    "（影像來源、載體、雲台、MAVLink/LR24 連線）仍需按「重啟引擎」",
        }

    @app.post("/api/restart")
    def restart():
        manager.restart()
        return {"ok": True}

    @app.get("/api/devices")
    def devices():
        return {"devices": list_video_devices()}

    @app.post("/api/video/test")
    def video_test(body: dict = Body(default={})):
        """起飛前測試影像來源：回報實際解析度/fps，不動到執行中的引擎。"""
        from ..vision.source import probe_source

        video_cfg = dict(cfg.section("video"))
        video_cfg.update(body or {})  # 允許帶入尚未儲存的設定先試
        return probe_source(video_cfg)

    # ---------------- 鏈路自檢 ----------------

    @app.post("/api/selfcheck")
    def selfcheck():
        """實測每條鏈路是否通。只讀不寫，不會發送任何導引指令。"""
        from ..selfcheck import run_selfcheck

        engine = manager.engine
        if engine is None:
            return JSONResponse({"error": "引擎未就緒"}, status_code=503)
        return run_selfcheck(engine, cfg)

    # ---------------- 檢查清單 ----------------

    @app.get("/api/checklist")
    def get_checklist():
        items = checklist.all()
        done = sum(1 for i in items if i["done"])
        return {"items": items, "done": done, "total": len(items)}

    @app.post("/api/checklist/toggle")
    def checklist_toggle(body: dict = Body(...)):
        checklist.toggle(str(body.get("id")))
        return {"ok": True}

    @app.post("/api/checklist/add")
    def checklist_add(body: dict = Body(...)):
        item = checklist.add(str(body.get("group", "自訂")), str(body.get("text", "")).strip())
        return {"ok": True, "item": item}

    @app.post("/api/checklist/remove")
    def checklist_remove(body: dict = Body(...)):
        checklist.remove(str(body.get("id")))
        return {"ok": True}

    @app.post("/api/checklist/uncheck_all")
    def checklist_uncheck():
        checklist.uncheck_all()
        return {"ok": True}

    @app.post("/api/checklist/reset")
    def checklist_reset():
        checklist.reset()
        return {"ok": True}

    # ---------------- 相機校正 ----------------

    @app.post("/api/calib/start")
    def calib_start(body: dict = Body(...)):
        calib["session"] = CalibrationSession(
            board_cols=int(body.get("cols", 9)),
            board_rows=int(body.get("rows", 6)),
            square_mm=float(body.get("square_mm", 25.0)),
        )
        return {"ok": True}

    @app.post("/api/calib/capture")
    def calib_capture():
        sess: CalibrationSession | None = calib["session"]
        engine = manager.engine
        if sess is None or engine is None:
            return JSONResponse({"ok": False, "error": "校正未開始或引擎未就緒"}, status_code=400)
        frame = engine.raw_frame()
        if frame is None:
            return JSONResponse({"ok": False, "error": "目前沒有影像"}, status_code=400)
        try:
            found = sess.capture(frame)
        except ValueError as exc:  # 解析度中途改變
            return JSONResponse({"ok": False, "error": str(exc)}, status_code=400)
        return {"ok": True, "found": found, "count": sess.count}

    @app.post("/api/calib/compute")
    def calib_compute():
        sess: CalibrationSession | None = calib["session"]
        if sess is None:
            return JSONResponse({"ok": False, "error": "校正未開始"}, status_code=400)
        try:
            result = sess.compute()
        except ValueError as exc:
            return JSONResponse({"ok": False, "error": str(exc)}, status_code=400)
        model = result["camera_model"]
        return {
            "ok": True,
            "rms": round(result["rms"], 3),
            "hfov_deg": round(result["hfov_deg"], 1),
            "fx": round(float(model.K[0, 0]), 1),
            "fy": round(float(model.K[1, 1]), 1),
            "cx": round(float(model.K[0, 2]), 1),
            "cy": round(float(model.K[1, 2]), 1),
            "dist": [round(float(d), 4) for d in model.dist],
        }

    @app.post("/api/calib/save")
    def calib_save():
        sess: CalibrationSession | None = calib["session"]
        if sess is None or sess.result is None:
            return JSONResponse({"ok": False, "error": "尚未計算校正結果"}, status_code=400)
        path = PROJECT_ROOT / (cfg.get("camera.intrinsics_file") or DEFAULT_INTRINSICS_FILE)
        sess.save(str(path))
        return {"ok": True, "path": str(path), "note": "重啟引擎後生效"}

    @app.get("/api/calib/status")
    def calib_status():
        sess: CalibrationSession | None = calib["session"]
        if sess is None:
            return {"active": False}
        return {
            "active": True,
            "count": sess.count,
            "board": list(sess.board_size),
            "last_found": sess.last_found,
            "has_result": sess.result is not None,
        }

    @app.middleware("http")
    async def no_cache_ui(request, call_next):
        """UI 資產一律不快取。

        地面站是本機服務、檔案很小，快取沒有效益，卻會造成很危險的情況：
        更新後瀏覽器仍載入舊的 app.js/index.html，操作員看到的是舊版介面
        （例如新加的安全檢查按鈕不見了、或行為與程式碼不符）卻毫無察覺。
        """
        response = await call_next(request)
        path = request.url.path
        if path.startswith("/static") or path == "/":
            response.headers["Cache-Control"] = "no-store, no-cache, must-revalidate, max-age=0"
            response.headers["Pragma"] = "no-cache"
        return response

    app.mount("/static", StaticFiles(directory=str(STATIC_DIR)), name="static")
    return app
