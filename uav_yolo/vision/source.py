"""影像來源：HDMI 採集卡（UVC）直開 / OBS 虛擬相機 / RTSP / 影片檔。

比舊系統的改進：
    - 直開採集卡（跳過 OBS 一手），少一段延遲與相依。
    - 擷取跑獨立執行緒、只保留「最新」一幀＋單調時間戳；
      主迴圈永遠拿到最新畫面，時間戳供遙測內插對齊。
    - 斷線自動重連（承襲舊系統的重開相機行為）。
"""

from __future__ import annotations

import os
import threading
import time

import cv2


def list_video_devices() -> list[str]:
    """Windows DirectShow 裝置清單（UI 下拉選單用）。"""
    try:
        from pygrabber.dshow_graph import FilterGraph

        return FilterGraph().get_input_devices()
    except Exception:
        return []


def _find_device_index(name_hint: str) -> int | None:
    if not name_hint:
        return None
    for idx, name in enumerate(list_video_devices()):
        if name_hint.lower() in name.lower():
            return idx
    return None


# ---------------------------------------------------------------------------
# RTSP 支援
# ---------------------------------------------------------------------------

def has_gstreamer() -> bool:
    """pip 版 opencv-python 通常沒編 GStreamer；有的話 RTSP 延遲會更低。"""
    try:
        info = cv2.getBuildInformation()
    except Exception:
        return False
    for line in info.splitlines():
        if "GStreamer" in line:
            return "YES" in line.upper()
    return False


def build_gst_pipeline(url: str, transport: str = "udp", timeout_s: float = 5.0) -> str:
    """低延遲 GStreamer RTSP pipeline（codec 無關，靠 decodebin）。

    latency=0 + drop=true + max-buffers=1 才不會累積緩衝——這正是 RTSP
    追蹤最容易踩的坑（預設緩衝可以到好幾秒）。
    """
    proto = "tcp" if transport == "tcp" else "udp"
    return (
        f"rtspsrc location={url} latency=0 protocols={proto} "
        f"tcp-timeout={int(timeout_s * 1_000_000)} timeout={int(timeout_s * 1_000_000)} ! "
        "rtpjitterbuffer latency=0 drop-on-latency=true ! "
        "decodebin ! videoconvert ! "
        "appsink drop=true sync=false max-buffers=1"
    )


def _ffmpeg_rtsp_options(transport: str, timeout_s: float = 5.0) -> str:
    # stimeout/timeout 單位是「微秒」。不設的話，連不到的位址會讓
    # VideoCapture 卡住非常久——UI 會凍住，飛行中擷取執行緒也會僵死。
    # 新舊 FFmpeg 對 rtsp 分別認 stimeout / timeout，兩個都給。
    us = int(timeout_s * 1_000_000)
    return (
        f"rtsp_transport;{transport}|fflags;nobuffer|flags;low_delay"
        f"|max_delay;0|reorder_queue_size;0|stimeout;{us}|timeout;{us}|rw_timeout;{us}"
    )


def _open_with_timeout(open_fn, timeout_s: float):
    """在背景執行緒開串流並限時等待；逾時就放棄並交給背景回收。

    為什麼要這樣做：OpenCV 4.11 的 FFMPEG 後端內建 30 秒中斷逾時，
    **實測 `OPENCV_FFMPEG_OPEN_TIMEOUT_MS` / `READ_TIMEOUT_MS` 環境變數
    完全不生效**（連 import cv2 前就設也一樣），串流層的 stimeout/timeout
    也管不到它。位址打錯或圖傳沒開時會整整卡 30 秒——UI 凍住、飛行中擷取
    執行緒僵死。唯一可靠的辦法就是自己在外面限時。
    """
    holder: dict = {}
    done = threading.Event()

    def worker():
        try:
            holder["cap"] = open_fn()
        except Exception as exc:
            holder["error"] = exc
        finally:
            done.set()

    threading.Thread(target=worker, daemon=True, name="rtsp-open").start()

    if done.wait(timeout_s):
        return holder.get("cap")

    def reap():  # 逾時後仍要把最終冒出來的 cap 釋放掉，別洩漏
        done.wait()
        late = holder.get("cap")
        if late is not None:
            try:
                late.release()
            except Exception:
                pass

    threading.Thread(target=reap, daemon=True, name="rtsp-open-reap").start()
    return None


def open_rtsp(
    url: str, transport: str, backend: str = "auto", timeout_s: float = 5.0
) -> tuple[cv2.VideoCapture | None, str]:
    """開一路 RTSP，回 (cap, 實際用的描述)。失敗回 (None, 原因)。"""
    if backend in ("auto", "gstreamer") and has_gstreamer():
        pipeline = build_gst_pipeline(url, transport, timeout_s)
        cap = _open_with_timeout(
            lambda: cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER), timeout_s)
        if cap is not None and cap.isOpened():
            return cap, f"gstreamer/{transport}"
        if cap is not None:
            cap.release()
        if backend == "gstreamer":
            return None, "GStreamer 開啟失敗或逾時"

    # FFMPEG：選項必須在建立 VideoCapture 之前寫進環境變數。
    # 注意：OpenCV 的 FFMPEG 後端有「自己的」中斷逾時（預設 30 秒），
    # 上面的 stimeout/timeout 串流選項管不到它——連不到的位址會整整卡 30 秒
    # （實測會噴 "Stream timeout triggered after 30042ms"）。真正的旋鈕是這兩個
    # OpenCV 層級環境變數，必須在建立 VideoCapture 前設好。
    timeout_ms = str(int(timeout_s * 1000))
    os.environ["OPENCV_FFMPEG_OPEN_TIMEOUT_MS"] = timeout_ms
    os.environ["OPENCV_FFMPEG_READ_TIMEOUT_MS"] = timeout_ms
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = _ffmpeg_rtsp_options(transport, timeout_s)
    cap = _open_with_timeout(lambda: cv2.VideoCapture(url, cv2.CAP_FFMPEG), timeout_s)
    if cap is None:
        return None, f"RTSP 連線逾時 {timeout_s:.0f}s（{transport}）"
    if cap.isOpened():
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass
        return cap, f"ffmpeg/{transport}"
    cap.release()
    return None, f"無法開啟 RTSP（{transport}）"


def probe_source(cfg_video: dict, sample_frames: int = 12) -> dict:
    """起飛前測試：照目前設定開一次來源，量實際解析度與 fps。

    不影響執行中的引擎（RTSP 可多開一路；UVC 若被佔用會回報 busy）。
    """
    src = VideoSource(dict(cfg_video))
    cap = src._open()
    if cap is None:
        return {
            "ok": False,
            "error": src.error or f"無法開啟來源（{src.mode}）；"
                                  f"{'裝置可能被佔用（關掉 OBS/其他程式）' if src.mode in ('uvc', 'obs') else '檢查網址與網段'}",
            "mode": src.mode,
        }
    try:
        t0 = time.monotonic()
        got = 0
        w = h = 0
        for _ in range(sample_frames):
            ok, frame = cap.read()
            if not ok or frame is None:
                continue
            got += 1
            h, w = frame.shape[:2]
        elapsed = time.monotonic() - t0
        if got == 0:
            return {"ok": False, "error": "已連線但取不到畫面", "mode": src.mode,
                    "device": src.device_label}
        return {
            "ok": True,
            "mode": src.mode,
            "device": src.device_label,
            "width": w,
            "height": h,
            "fps": round(got / elapsed, 1) if elapsed > 0 else None,
            "frames": got,
            "gstreamer": has_gstreamer(),
        }
    finally:
        cap.release()


class VideoSource:
    """統一的影像來源介面：start() 後用 get_frame() 拿 (frame, t_mono)。"""

    def __init__(self, cfg_video: dict):
        self.cfg = cfg_video
        self.mode = cfg_video.get("source", "uvc")
        self.width = int(cfg_video.get("width", 1280))
        self.height = int(cfg_video.get("height", 720))
        self._cap: cv2.VideoCapture | None = None
        self._lock = threading.Lock()
        self._frame = None
        self._frame_t: float | None = None
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self.fps = 0.0
        self.connected = False
        self.error: str | None = None
        self.device_label = ""

    # ---- 開啟各種來源 ----

    def _open(self) -> cv2.VideoCapture | None:
        mode = self.mode
        if mode == "rtsp":
            url = self.cfg.get("rtsp_url", "")
            backend = self.cfg.get("rtsp_backend", "auto")
            configured = self.cfg.get("rtsp_transport", "udp")
            # auto：先試低延遲的 UDP，不通再退 TCP（有些網路擋 UDP）
            timeout_s = float(self.cfg.get("rtsp_timeout_s", 5.0))
            order = ["udp", "tcp"] if configured == "auto" else [configured]
            for transport in order:
                cap, note = open_rtsp(url, transport, backend, timeout_s)
                if cap is not None:
                    ok, _ = cap.read()  # 真的能取到一幀才算通
                    if ok:
                        self.device_label = f"{url}  [{note}]"
                        return cap
                    cap.release()
            self.error = f"RTSP 無法取得畫面：{url}"
            return None

        if mode == "file":
            path = self.cfg.get("file_path", "")
            self.device_label = path
            cap = cv2.VideoCapture(path)
            return cap if cap.isOpened() else None

        # uvc / obs：DirectShow 名稱鎖定優先，其次索引
        if mode == "obs":
            hint = self.cfg.get("obs_name_hint", "OBS Virtual Camera")
            index = _find_device_index(hint)
            candidates = [index] if index is not None else [1, 2, 3]
        else:  # uvc
            hint = self.cfg.get("uvc_name_hint", "")
            index = _find_device_index(hint)
            candidates = [index] if index is not None else [int(self.cfg.get("uvc_index", 1))]

        for idx in candidates:
            cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
            if not cap.isOpened():
                cap.release()
                continue
            # 便宜的 HDMI 採集卡多半是 USB2：用預設的 YUY2 未壓縮格式在 1080p
            # 會被頻寬卡到剩幾 fps。改要 MJPG 才吃得下 1080p30。
            fourcc = str(self.cfg.get("fourcc", "MJPG")).strip().upper()
            if len(fourcc) == 4:  # VideoWriter_fourcc 需要「剛好」4 字元，少一個就 TypeError
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
            elif fourcc:
                self.error = f"fourcc 設定「{fourcc}」無效（需 4 字元，如 MJPG），已用裝置預設"
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 只留最新一幀，別累積延遲
            except Exception:
                pass
            ok, _ = cap.read()
            if ok:
                names = list_video_devices()
                self.device_label = names[idx] if idx < len(names) else f"index {idx}"
                return cap
            cap.release()
        return None

    # ---- 生命週期 ----

    def start(self) -> bool:
        self._cap = self._open()
        if self._cap is None:
            self.error = f"無法開啟影像來源（{self.mode}）"
            return False
        self.connected = True
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name="video-capture")
        self._thread.start()
        return True

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2.0)
        if self._cap:
            self._cap.release()
        self.connected = False

    def _reconnect(self) -> None:
        """先放掉舊的、再開新的：DirectShow 相機是獨占的，
        舊 handle 還握著時第二次開同一台一定失敗 → 重連會永遠打不通。"""
        self.connected = False
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        try:
            cap = self._open()
        except Exception as exc:
            self.error = f"重新開啟影像來源失敗：{exc}"
            return
        if cap is not None:
            self._cap = cap
            self.connected = True
            self.error = None

    def _capture_loop(self) -> None:
        fps_t0 = time.monotonic()
        fps_n = 0
        last_good = time.monotonic()
        stall_timeout = float(self.cfg.get("stall_timeout_s", 4.0))
        while not self._stop.is_set():
            try:
                ok, frame = (self._cap.read() if self._cap else (False, None))
            except Exception as exc:
                # cv2 偶爾會直接丟例外（驅動/解碼問題）。不擋的話執行緒靜默死掉，
                # UI 還顯示 connected=True、FPS 停在舊值——操作員完全看不出來。
                self.error = f"影像擷取異常：{exc}（重試中）"
                time.sleep(1.0)
                self._reconnect()
                last_good = time.monotonic()
                continue
            now = time.monotonic()

            # RTSP／網路來源可能「不報錯但也不再吐新幀」——read() 成功卻停更。
            # 沒有 watchdog 的話追蹤會抱著一張舊畫面繼續算，非常危險。
            if ok and frame is not None:
                last_good = now
            elif now - last_good > stall_timeout and self.mode in ("rtsp", "uvc", "obs"):
                self.error = f"影像停滯超過 {stall_timeout:.0f}s，重新連線中"
                self._reconnect()
                last_good = now
                continue

            if not ok or frame is None:
                if self.mode == "file" and self._cap is not None:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 影片檔循環播放
                    continue
                time.sleep(0.5)
                self._reconnect()  # 圖傳中斷自動重連
                continue

            with self._lock:
                self._frame = frame
                self._frame_t = now
            fps_n += 1
            if now - fps_t0 >= 2.0:
                self.fps = fps_n / (now - fps_t0)
                fps_t0, fps_n = now, 0

    def get_frame(self):
        """回傳 (frame, t_mono)；還沒有畫面回 (None, None)。"""
        with self._lock:
            if self._frame is None:
                return None, None
            return self._frame.copy(), self._frame_t
