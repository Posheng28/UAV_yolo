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
from collections import deque

import cv2
import numpy as np


def list_video_devices() -> list[str]:
    """Windows DirectShow 裝置清單（UI 下拉選單用）。

    pygrabber 走 COM，而 FastAPI 的同步端點跑在執行緒池——每條新執行緒都必須
    先 CoInitialize，否則會靜默失敗回空清單（症狀：設定頁「裝置清單」顯示
    偵測不到，但同一時間「測試影像來源」卻抓得到裝置名稱）。
    """
    initialized = False
    try:
        import comtypes  # pygrabber 的相依

        try:
            comtypes.CoInitialize()
            initialized = True
        except Exception:
            pass
    except Exception:
        pass

    try:
        from pygrabber.dshow_graph import FilterGraph

        return FilterGraph().get_input_devices()
    except Exception:
        return []
    finally:
        if initialized:
            try:
                import comtypes

                comtypes.CoUninitialize()
            except Exception:
                pass


def _index_of(names: list[str], name_hint: str) -> int | None:
    """在已取得的裝置清單裡找名稱含 hint 的索引。

    傳入清單而不是自己列舉：一次 _open() 原本會做兩次 COM 列舉（找索引一次、
    取標籤一次），而重連路徑上每一毫秒都算在黑畫面時間裡。
    """
    if not name_hint:
        return None
    for idx, name in enumerate(names):
        if name_hint.lower() in name.lower():
            return idx
    return None


def _find_device_index(name_hint: str) -> int | None:
    return _index_of(list_video_devices(), name_hint)


# DirectShow 開啟/格式協商全行程序列化（見 _open 內的說明）
_device_open_lock = threading.Lock()


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


# FFMPEG 選項走「行程級」環境變數：設定頁的 probe（HTTP 執行緒）與引擎的
# 自動重連（擷取執行緒）同時開 RTSP 時會互蓋 transport/timeout。序列化整段
# 「設環境變數 → 建 VideoCapture」；鎖最多持有 timeout_s（開啟本身有限時）。
_ffmpeg_env_lock = threading.Lock()


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
    with _ffmpeg_env_lock:
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
    try:
        cap = src._open()
    except Exception as exc:
        # 開啟過程丟例外（驅動/格式協商）不該變成 HTTP 500，
        # 操作員需要看到原因而不是一個紅色的伺服器錯誤
        return {"ok": False, "mode": src.mode, "error": f"開啟時發生例外：{exc}"}
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
            # 實際談成的格式：FPS 偏低時用來分辨「沒談成 MJPG」還是「沒有 HDMI 訊號」
            "fourcc": VideoSource.read_fourcc(cap),
            "note": src.error or "",
            "gstreamer": has_gstreamer(),
        }
    finally:
        cap.release()


# 🔴 讀取失敗的容忍預算。UVC/HDMI 這條鏈路偶爾掉一兩顆幀是常態（USB 排程、
# 採集卡重新同步、VRX 切模式），但舊版是「一次 read() 失敗就 sleep(0.5) 再把
# 整個 DirectShow graph 拆掉重建」。實測（本機真實 UVC 裝置、1920×1080 MJPG）：
# 注入**一次**失敗讀取 → 畫面凍結 3.7~8.8 秒、connected=False 3.2~7.2 秒，
# 放大約 100~250 倍。重開的成本大宗是 _negotiate_format（0.83~5.9s）與驗證
# 讀取（~1.0s），不是裝置列舉。所以：先原地重試，真的連續失敗才升級重連。
READ_FAIL_TOLERANCE = 8      # 連續失敗數；30fps 下約 0.27 秒
READ_RETRY_SLEEP_S = 0.005   # 重試間隔（不是 0.5 秒！）
# 但重連失敗要退避：裝置真的不在（採集卡沒插、被別的程式佔住）時，若還用
# 5ms 的節奏重試會把 CPU 和 log 洗爆——舊版每 0.54 秒重試一次就已經在
# data/server.log.err 產生 12037 行。退避讓「暫時性」與「真的沒插」分道揚鑣。
RECONNECT_BACKOFF_S = (0.5, 1.0, 2.0, 3.0)

# 內容活性檢查：採集卡有訊號但畫面不動（VRX 失鎖後吐靜止的「無訊號」畫面、
# 或圖傳凍結）時，read() 一路成功、fps 正常、connected=True——舊版完全偵測
# 不到，凍住的畫面還會被當成新量測餵進 KF。這是軟體端唯一能指認 RF 失鎖的訊號。
FREEZE_ALERT_S = 1.5         # 畫面實質不變超過這麼久 → 判定停格
# 🔴 停格持續這麼久就主動重開一次裝置（有冷卻，不是每秒亂重開）。
# MS2109 這類 HDMI dongle 會在「開啟當下」鎖定輸入格式：先開了擷取、HDMI
# 訊號後到（VRX 晚供電、圖傳晚鎖定、來源換解析度），dongle 就會一直吐全黑
# 畫面，read() 全部成功、fps 卻只有 1——**重開才會重新鎖定**。
# 舊版「一次讀取失敗就整組重開」雖然會把打嗝放大成數秒黑畫面，卻意外具備
# 這條恢復路徑；改成容忍瞬時失敗之後就消失了。這裡把它以正確的形式加回來：
# 只在「確定畫面是死的」時重開，而不是任何一次抖動都重開。
FREEZE_RELATCH_S = 6.0
FREEZE_RELATCH_COOLDOWN_S = 15.0
# 取樣點變動比例低於此就算「沒在動」。
#
# 兩種誤判的代價完全不對稱，所以門檻刻意偏高：
#   漏報（死畫面被當成活的）→ 導引拿死畫面推導指令 → 飛機追幻影，危險。
#   誤報（活畫面被當成死的）→ 停止發指令並顯示原因，畫面一動就自動解除，安全。
# 飛行中的實拍畫面幾乎每個取樣點都在變（機體晃動＋雜訊），遠高於 5%；
# 而 VRX 無訊號畫面上的 OSD 條（1080p 下 300×30 ≈ 0.4%）遠低於 5%。
FREEZE_CHANGED_FRAC = 0.05
# 全黑判定要有遲滯，否則亮度在門檻附近擺盪（黃昏、深色柏油、自動曝光呼吸）
# 會讓 blank 每次取樣都翻面，事件洗版還會把真正重要的斷線事件擠出佇列。
# 進入要「持續夠久」，離開只要亮度明顯回來——寧可晚報，不可亂報。
BLANK_LUMA = 8.0             # 平均亮度低於此 → 疑似無訊號畫面
BLANK_EXIT_LUMA = 15.0       # 高於此才算恢復
BLANK_ENTER_S = 1.0          # 連續多久才判定
LIVENESS_EVERY = 10          # 每 N 幀取樣一次（取樣成本 <0.1ms）


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

        # ---- 可觀察狀態（UI 與任務記錄用；全部只由擷取執行緒更新） ----
        # 沒有這些，事後只能面對「圖傳有時候會斷線」而完全無從分辨
        # 「USB 掉線」「畫面凍住」「RF 失鎖」——三者的處置完全不同。
        self.frames_total = 0
        self.read_fail_streak = 0
        self.read_fail_total = 0
        self.reopen_total = 0
        self.reopen_fail_total = 0
        self.reopen_fail_streak = 0
        self.last_open_ms: float | None = None
        self.last_reason: str | None = None      # 上次重連的觸發原因
        self.actual_wh: tuple[int, int] | None = None
        self.actual_fourcc = ""
        self.frozen = False                       # 有訊號但畫面不動
        self.blank = False                        # 畫面幾乎全黑（疑似無訊號）
        self.mean_luma: float | None = None
        self._events: deque = deque(maxlen=200)   # 引擎每輪汲取寫進任務記錄
        self._last_sample = None      # 上一次的取樣（逐元素比對用）
        self._last_change_t: float | None = None
        self._liveness_n = 0
        self._dark_since: float | None = None
        self._last_relatch_t = 0.0    # 上次因停格而重開的時刻（冷卻用）

    # ---- 事件記錄 ----

    def _note(self, kind: str, **detail) -> None:
        """記一筆影像事件（含單調時間與牆鐘），供任務記錄與 UI 取用。"""
        self._events.append({
            "kind": kind,
            "t": round(time.monotonic(), 3),
            "wall": time.strftime("%H:%M:%S"),
            **detail,
        })

    def drain_events(self) -> list[dict]:
        """取走累積的事件（引擎呼叫；取走即清空）。"""
        out = []
        while self._events:
            out.append(self._events.popleft())
        return out

    def snapshot(self) -> dict:
        """給 /api/status 與任務快照的一份影像健康度。"""
        return {
            "frames_total": self.frames_total,
            "reopen_total": self.reopen_total,
            "reopen_fail_total": self.reopen_fail_total,
            "read_fail_total": self.read_fail_total,
            "last_reason": self.last_reason,
            "last_open_ms": self.last_open_ms,
            "frozen": self.frozen,
            "blank": self.blank,
            "mean_luma": None if self.mean_luma is None else round(self.mean_luma, 1),
            "actual": (list(self.actual_wh) + [self.actual_fourcc]) if self.actual_wh else None,
        }

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
        names = list_video_devices()
        if mode == "obs":
            hint = self.cfg.get("obs_name_hint", "OBS Virtual Camera")
            index = _index_of(names, hint)
            candidates = [index] if index is not None else [1, 2, 3]
        else:  # uvc
            hint = self.cfg.get("uvc_name_hint", "")
            index = _index_of(names, hint)
            if index is None and hint:
                # 名稱對不上就退索引是有風險的：實測本機索引 1 是 OBS 虛擬相機。
                # 採集卡掉線時退回去等於默默改看別台裝置，畫面「有」但完全是錯的。
                self.error = (f"找不到名稱含「{hint}」的擷取裝置"
                              f"（目前有：{'、'.join(names) or '無'}），改用索引")
            candidates = [index] if index is not None else [int(self.cfg.get("uvc_index", 1))]

        for idx in candidates:
            # DirectShow 的開啟/格式協商不是執行緒安全的：舊擷取執行緒還在
            # _negotiate_format 下 cap.set() 時，另一條（引擎重啟、或設定頁的
            # 影像測試）同時開同一台，重現過 access violation／heap corruption
            # 直接殺掉整個行程。全行程序列化開啟這段。
            with _device_open_lock:
                if self._stop.is_set():
                    return None
                cap = cv2.VideoCapture(idx, cv2.CAP_DSHOW)
                if not cap.isOpened():
                    cap.release()
                    continue
                self._negotiate_format(cap)
                try:
                    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # 只留最新一幀，別累積延遲
                except Exception:
                    pass
                ok, _ = cap.read()
            if ok:
                self.device_label = names[idx] if idx < len(names) else f"index {idx}"
                return cap
            cap.release()
        return None

    @staticmethod
    def read_fourcc(cap) -> str:
        try:
            code = int(cap.get(cv2.CAP_PROP_FOURCC))
        except Exception:
            return ""
        if not code:
            return ""
        return "".join(chr((code >> (8 * i)) & 0xFF) for i in range(4)).strip()

    def _negotiate_format(self, cap) -> None:
        """談定像素格式與解析度，**設定後讀回驗證**。

        便宜的 HDMI 採集卡多半走 USB2：未壓縮 YUY2 在 1920×1080 只有約 5 FPS
        （頻寬硬限制），必須談成 MJPG 才吃得下 30 FPS。但 DirectShow 驅動常常
        「接受了 set() 卻不套用」，而且對『先設格式還是先設解析度』的順序敏感——
        實測某採集卡在預設順序下仍停在 YUY2。因此這裡設完要讀回確認，
        沒成功就換順序再試一次。
        """
        want = str(self.cfg.get("fourcc", "MJPG")).strip().upper()
        if want and len(want) != 4:
            self.error = f"fourcc 設定「{want}」無效（需 4 字元，如 MJPG），改用裝置預設"
            want = ""

        def apply(fourcc_first: bool) -> None:
            if fourcc_first and want:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*want))
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            if not fourcc_first and want:
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*want))

        def record() -> None:
            # 解析度也要讀回：原本只驗 fourcc，裝置默默給了別的尺寸完全看不到。
            try:
                self.actual_wh = (int(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                                  int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
            except Exception:
                self.actual_wh = None
            self.actual_fourcc = self.read_fourcc(cap).upper()

        apply(fourcc_first=True)
        if not want:
            record()
            return
        if self.read_fourcc(cap).upper() == want:
            record()
            return

        # 沒談成：換「先解析度後格式」再試（部分驅動只吃這個順序）
        apply(fourcc_first=False)
        record()
        got = self.actual_fourcc
        if got != want:
            self.error = (
                f"要求 {want} 但裝置談成 {got or '未知'}；"
                f"{self.width}×{self.height} 走未壓縮格式在 USB2 只有約 5 FPS。"
                "可改用 USB3 採集卡，或把解析度降到 1280×720"
            )

    # ---- 生命週期 ----

    def start(self) -> bool:
        """啟動擷取。**即使一開始開不了也會啟動重連執行緒**。

        以前開啟失敗就直接 return False 且不啟動執行緒 → 引擎永久失明、
        不會自己恢復（例如重啟引擎時前一個 capture 尚未釋放裝置、或圖傳
        還沒供電）。這種暫時性失敗必須能自動康復。
        """
        t0 = time.monotonic()
        self._cap = self._open()
        self.last_open_ms = round((time.monotonic() - t0) * 1000)
        self.connected = self._cap is not None
        self.last_reason = "startup"
        if self._cap is None:
            self.error = f"無法開啟影像來源（{self.mode}），持續重試中"
        self._note("video_start", ok=self.connected, open_ms=self.last_open_ms,
                   device=self.device_label[:60], mode=self.mode)
        self._stop.clear()
        self._thread = threading.Thread(target=self._capture_loop, daemon=True, name="video-capture")
        self._thread.start()
        return self.connected

    def stop(self) -> None:
        self._stop.set()
        if self._thread:
            # join 要撐得過進行中的 _open()：實測本機真實 UVC 裝置冷開機
            # 可達 5.2 秒，2 秒的 join 會讓舊擷取執行緒與新引擎的執行緒同時
            # 對 DirectShow 下 cap.set()，重現過 access violation 直接殺行程。
            self._thread.join(timeout=10.0)
        # cap 的釋放歸擷取執行緒所有（_capture_loop 的 finally）。join 逾時代表
        # 執行緒還卡在 cap.read()（RTSP 斷線時可長達數秒）——這時從別的執行緒
        # release() 同一個 cap 是對使用中物件動手，OpenCV 會直接當掉整個行程。
        if self._thread is not None and self._thread.is_alive():
            return
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        self.connected = False

    def _reconnect(self, reason: str = "unknown") -> None:
        """先放掉舊的、再開新的：DirectShow 相機是獨占的，
        舊 handle 還握著時第二次開同一台一定失敗 → 重連會永遠打不通。"""
        was_connected = self.connected
        self.connected = False
        self.last_reason = reason
        if was_connected:
            self._note("video_lost", reason=reason, fps_before=round(self.fps, 1),
                       fail_streak=self.read_fail_streak, frames_total=self.frames_total)
        if self._cap:
            try:
                self._cap.release()
            except Exception:
                pass
            self._cap = None
        # stop() 之後不該再開新裝置：晚到的重連會把相機重新佔住，
        # 下一個引擎（重啟流程）開同一台就 busy。
        if self._stop.is_set():
            return
        t0 = time.monotonic()
        try:
            cap = self._open()
        except Exception as exc:
            self.error = f"重新開啟影像來源失敗：{exc}"
            self.reopen_fail_total += 1
            self._note("video_reopen", ok=False, err=str(exc)[:120],
                       open_ms=round((time.monotonic() - t0) * 1000))
            return
        self.reopen_total += 1
        self.last_open_ms = round((time.monotonic() - t0) * 1000)
        if cap is not None:
            self._cap = cap
            self.connected = True
            # _open 自己設的警告（例如「談成 YUY2、只有約 5 FPS」）不能被清掉，
            # 那正是操作員最需要看到的東西。只清「重連中」這類過期訊息。
            if self.error and (self.error.startswith("影像讀取失敗")
                               or self.error.startswith("影像停滯")
                               or self.error.startswith("無法開啟")
                               or self.error.startswith("重新開啟")
                               or self.error.startswith("找不到名稱")):
                self.error = None
            self.read_fail_streak = 0
            # 🔴 重連**不清** frozen。清掉的話，會抖動的採集卡（本機一天 1540 次
            # USB 突發移除）每次重連都把停格判定歸零、重新計時 1.5 秒——實測
            # 每 1.2 秒重連一次時，frozen 為真的時間佔比是 0%，等於這道防線
            # 完全不存在。取樣基準重置即可，「是否停格」要等真的看到畫面在動
            # 才由 _check_liveness 解除。
            self._last_sample = None
            self._note("video_restored", reason=reason, open_ms=self.last_open_ms,
                       device=self.device_label[:60], reopen_total=self.reopen_total)
        else:
            self.reopen_fail_total += 1
            self._note("video_reopen", ok=False, open_ms=self.last_open_ms)

    # ---- 內容活性（有訊號 ≠ 有畫面） ----

    def _check_liveness(self, frame, now: float) -> None:
        """read() 成功不代表畫面在動。

        RF 失鎖時 VRX 多半仍持續輸出 HDMI（靜止的「無訊號」畫面），採集卡照樣
        吐幀：connected=True、fps 正常、看門狗永遠不跳，而追蹤會抱著一張死畫面
        繼續算並餵給 KF——沒有這個檢查，那是完全沉默的失效。
        """
        self._liveness_n += 1
        if self._liveness_n % LIVENESS_EVERY:
            return
        try:
            # 🔴 抽樣要夠密、比較要逐元素。原本是 frame[::32,::32].sum() 壓成
            # 一個整數（1080p 下只有 6120 個值＝畫面的 0.098%）：任何一個
            # 會動的角落（VRX 的 OSD 計時器、閃爍圖示）只要落在格線上，
            # 整張死畫面就會被判成「活的」——而這是 RF 失鎖的唯一軟體特徵。
            # 改成看「有多少比例的取樣點真的變了」，單一角落動不了大局。
            sample = np.asarray(frame[::16, ::16], dtype=np.int16)
            luma = float(sample.mean())
        except Exception:
            return
        self.mean_luma = luma
        prev = self._last_sample
        self._last_sample = sample
        if prev is None or prev.shape != sample.shape:
            # 重連後（或解析度改變後）的第一筆只建基準，不下判斷：這裡若當成
            # 「畫面有變」就會把 frozen 清掉，抖動的裝置每次重連都能洗掉判定。
            return
        changed_frac = float(np.count_nonzero(sample != prev)) / sample.size

        # 全黑判定（帶遲滯，見常數處的說明）
        if luma < BLANK_LUMA:
            if self._dark_since is None:
                self._dark_since = now
            if not self.blank and (now - self._dark_since) >= BLANK_ENTER_S:
                self.blank = True
                self._note("video_blank", mean_luma=round(luma, 1))
        elif luma > BLANK_EXIT_LUMA:
            self._dark_since = None
            if self.blank:
                self.blank = False
                self._note("video_signal", mean_luma=round(luma, 1))

        if changed_frac >= FREEZE_CHANGED_FRAC:
            was_still_since = self._last_change_t
            self._last_change_t = now
            if self.frozen:
                self.frozen = False
                # 停格持續多久要用「開始不動的時刻」算，不能用剛更新過的值（恆為 0）
                self._note("video_unfroze",
                           gap_s=round(now - (was_still_since or now), 1))
                if self.error and self.error.startswith("影像停格"):
                    self.error = None
            return
        # 畫面實質上沒有變化
        if self._last_change_t is None:
            self._last_change_t = now
            return
        held = now - self._last_change_t
        if held >= FREEZE_ALERT_S:
            if not self.frozen:
                self.frozen = True
                self._note("video_freeze", gap_s=round(held, 1), mean_luma=round(luma, 1))
            # 錯誤字串每次都重寫：它是唯一告訴操作員「這是圖傳失鎖、不是 USB」
            # 的診斷。原本只在進入停格的那一瞬間寫一次，之後任何一次讀取失敗
            # 都會把它蓋掉且永不回來（實測 7 次失敗後訊息永久消失）。
            #
            # 全黑與「有畫面但不動」要分開講，處置完全不同：全黑＝採集卡根本
            # 沒收到 HDMI 訊號（VRX 沒電／線沒插好／圖傳沒鎖定），去查機器；
            # 有內容但不動＝訊號在、來源端凍結，去查相機/雲台/RF。
            if self.blank:
                self.error = (
                    f"採集卡收不到 HDMI 訊號（畫面全黑 {held:.0f}s，"
                    f"{self.fps:.0f} FPS）：檢查 VRX 電源、Mini HDMI 接頭、"
                    "以及機上圖傳是否已鎖定")
            else:
                self.error = (f"影像停格 {held:.0f}s：採集卡有訊號但畫面內容沒變"
                              "（疑似圖傳失鎖或相機/雲台端凍結）")

    def _capture_loop(self) -> None:
        try:
            self._capture_loop_inner()
        finally:
            # 執行緒是 cap 的擁有者：自己收尾，stop() 只在確定執行緒
            # 已結束時才代為釋放（見 stop() 的說明）。
            if self._cap:
                try:
                    self._cap.release()
                except Exception:
                    pass
                self._cap = None
            self.connected = False

    def _capture_loop_inner(self) -> None:
        fps_t0 = time.monotonic()
        fps_n = 0
        last_good = time.monotonic()
        stall_timeout = float(self.cfg.get("stall_timeout_s", 4.0))
        file_rewinds = 0
        while not self._stop.is_set():
            try:
                ok, frame = (self._cap.read() if self._cap else (False, None))
            except Exception as exc:
                # cv2 偶爾會直接丟例外（驅動/解碼問題）。不擋的話執行緒靜默死掉，
                # UI 還顯示 connected=True、FPS 停在舊值——操作員完全看不出來。
                self.error = f"影像擷取異常：{exc}（重試中）"
                self._note("video_exception", err=str(exc)[:120])
                time.sleep(1.0)
                self._reconnect("exception")
                last_good = time.monotonic()
                continue
            now = time.monotonic()

            if ok and frame is not None:
                last_good = now
                if self.read_fail_streak:
                    # 撐過去了：這種「本來會變成數秒黑畫面」的瞬斷值得留痕，
                    # 否則修好之後就再也不知道鏈路到底有多不乾淨。
                    self._note("video_hiccup", fails=self.read_fail_streak)
                    self.read_fail_streak = 0
                    if self.error and self.error.startswith("影像讀取失敗"):
                        self.error = None
                self.frames_total += 1
                self._check_liveness(frame, now)
                file_rewinds = 0
                with self._lock:
                    self._frame = frame
                    self._frame_t = now
                fps_n += 1
                if now - fps_t0 >= 2.0:
                    self.fps = fps_n / (now - fps_t0)
                    fps_t0, fps_n = now, 0
                # 畫面死透了就重開一次去重新鎖定 HDMI 輸入（見 FREEZE_RELATCH_S）
                if (self.frozen and self.mode in ("uvc", "obs")
                        and (now - self._last_relatch_t) >= FREEZE_RELATCH_COOLDOWN_S
                        and (now - (self._last_change_t or now)) >= FREEZE_RELATCH_S):
                    self._last_relatch_t = now
                    self._note("video_relatch",
                               still_s=round(now - (self._last_change_t or now), 1))
                    self._reconnect("frozen_relatch")
                continue

            # ---- 讀取失敗 ----
            if self.mode == "file" and self._cap is not None:
                self._cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # 影片檔循環播放
                # 損壞/空影片會讓「讀失敗→倒帶→讀失敗」變成無 sleep 的
                # 忙迴圈，吃滿一顆核心還不報錯。連續倒帶仍讀不到就要說。
                file_rewinds += 1
                if file_rewinds >= 3:
                    self.error = f"影片檔讀不到任何幀：{self.device_label}"
                    time.sleep(0.5)
                continue

            self.read_fail_streak += 1
            self.read_fail_total += 1
            # 🔴 先原地重試，別急著拆裝置：一次失敗就重開會把 33ms 的 USB
            # 打嗝放大成數秒凍結（實測 3.7~8.8s）。連續失敗才是真的斷了。
            if self.read_fail_streak < READ_FAIL_TOLERANCE:
                if self.read_fail_streak == 1:
                    self.error = "影像讀取失敗，重試中"
                time.sleep(READ_RETRY_SLEEP_S)
                continue

            # 停滯逾時（read 成功但畫面不更新的情境由 _check_liveness 負責，
            # 這裡處理的是「read 一直失敗」）
            reason = ("stall_timeout" if now - last_good > stall_timeout else "read_false")
            self._reconnect(reason)
            if self.connected:
                self.reopen_fail_streak = 0
            else:
                idx = min(self.reopen_fail_streak, len(RECONNECT_BACKOFF_S) - 1)
                self.reopen_fail_streak += 1
                time.sleep(RECONNECT_BACKOFF_S[idx])
            last_good = time.monotonic()

    def get_frame(self):
        """回傳 (frame, t_mono)；還沒有畫面回 (None, None)。"""
        with self._lock:
            if self._frame is None:
                return None, None
            return self._frame.copy(), self._frame_t
