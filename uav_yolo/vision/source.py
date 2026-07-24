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
            self.device_label = url
            # OpenCV/FFMPEG 預設會緩衝數秒——對追蹤是致命的。
            # 走 UDP、關 buffer、低延遲旗標；必須在建立 VideoCapture 前設好環境變數。
            transport = self.cfg.get("rtsp_transport", "udp")
            os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = (
                f"rtsp_transport;{transport}|fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0"
            )
            cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
            if not cap.isOpened():
                cap.release()
                return None
            try:
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 只留最新一幀
            except Exception:
                pass
            return cap

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
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
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

    def _capture_loop(self) -> None:
        fps_t0 = time.monotonic()
        fps_n = 0
        while not self._stop.is_set():
            ok, frame = (self._cap.read() if self._cap else (False, None))
            now = time.monotonic()
            if not ok or frame is None:
                if self.mode == "file" and self._cap is not None:
                    self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 影片檔循環播放
                    continue
                self.connected = False
                time.sleep(0.5)
                new_cap = self._open()  # 圖傳中斷自動重連
                if new_cap is not None:
                    if self._cap:
                        self._cap.release()
                    self._cap = new_cap
                    self.connected = True
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
