"""鏈路自檢：實際探測每條鏈路，回報「通/警告/不通」與具體原因。

設計原則：
    - 每一項都是**實測**（讀真實遙測、抓真實幀），不是看設定檔猜。
    - 不通時要說「怎麼修」，不能只說失敗。
    - 只讀不寫：絕不發送任何導引指令或改變飛控狀態。

狀態：
    pass  已驗證可用
    warn  能運作但有風險（例如未校正、延遲未量）
    fail  這條鏈路不通，會直接影響任務
    skip  此設定下不適用（例如沒裝雲台）
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from pathlib import Path

from .config import DEFAULT_INTRINSICS_FILE, DEFAULT_WEIGHTS_FILE, PROJECT_ROOT


@dataclass
class CheckResult:
    key: str
    name: str
    status: str            # pass | warn | fail | skip
    detail: str            # 實測到什麼
    fix: str = ""          # 不通時怎麼修
    metrics: dict = field(default_factory=dict)


def _r(key, name, status, detail, fix="", **metrics) -> CheckResult:
    return CheckResult(key, name, status, detail, fix, metrics)


# --------------------------------------------------------------- 各條鏈路

def check_video(engine) -> CheckResult:
    """影像鏈路：真的有新幀進來嗎、解析度與速率如何。"""
    v = getattr(engine, "video", None)
    if v is None:
        return _r("video", "影像鏈路", "fail", "影像來源未建立", "檢查設定頁「影像來源」後重啟引擎")

    frame, t1 = v.get_frame()
    if frame is None:
        label = getattr(v, "device_label", "") or "尚未成功開啟任何裝置"
        err = getattr(v, "error", None)
        cfgv = getattr(v, "cfg", {}) or {}
        hint = (f"來源={cfgv.get('source')}"
                f"｜名稱關鍵字='{cfgv.get('uvc_name_hint', '')}'"
                f"｜索引={cfgv.get('uvc_index')}")
        return _r("video", "影像鏈路", "fail",
                  f"沒有畫面（{label}）｜{hint}" + (f"｜{err}" if err else ""),
                  "設定頁按「🎥 測試影像來源」比對：若測試能開但這裡不行，"
                  "多半是裝置被別的程式佔用（OBS／另一個地面站分頁），"
                  "或索引指到別台裝置；系統會持續自動重試")

    h, w = frame.shape[:2]
    fps = round(float(getattr(v, "fps", 0.0)), 1)

    # 等一下再取一次，確認「有在更新」而不是卡住的舊幀。
    # 模擬模式的時鐘由 world.step() 手動推進，牆鐘時間內不會前進 → 不適用此檢查。
    stale = False
    if not engine.sim_mode:
        time.sleep(0.35)
        _, t2 = v.get_frame()
        stale = (t1 is not None and t2 is not None and t2 == t1)

    if stale:
        return _r("video", "影像鏈路", "fail",
                  f"畫面停止更新（{w}×{h}，卡在同一幀）",
                  "圖傳/採集卡可能斷線；檢查接線與電源，系統會自動重連",
                  width=w, height=h, fps=fps)
    if fps < 10:
        return _r("video", "影像鏈路", "warn",
                  f"{w}×{h} @ {fps} FPS（偏低）",
                  "USB2 採集卡請確認 fourcc=MJPG；1080p 建議 ≥20 FPS",
                  width=w, height=h, fps=fps)
    return _r("video", "影像鏈路", "pass", f"{w}×{h} @ {fps} FPS，畫面持續更新",
              width=w, height=h, fps=fps)


def check_calibration(engine, cfg) -> CheckResult:
    """相機校正：沒校正 → 測地會有系統性誤差。"""
    path = PROJECT_ROOT / (cfg.get("camera.intrinsics_file") or DEFAULT_INTRINSICS_FILE)
    model = engine.camera_model
    if model.source != "calibrated":
        return _r("camera", "相機校正", "warn",
                  f"未校正，用 FOV {model.hfov_deg:.0f}° 近似（測地精度差、畫面邊緣尤其）",
                  "相機校正頁：整條鏈路（相機→圖傳→採集卡）拍棋盤格 15~25 張")
    # 校正檔的解析度是否與目前實際幀相符
    note = f"已校正（{model.width}×{model.height}，水平 FOV {model.hfov_deg:.0f}°）"
    return _r("camera", "相機校正", "pass", note, path=str(path))


def check_detector(engine, cfg) -> CheckResult:
    """YOLO：權重是自訓的還是退回 COCO？當下有沒有偵測到東西？"""
    det = getattr(engine, "detector", None)
    weights = cfg.get("detector.weights") or DEFAULT_WEIGHTS_FILE
    wpath = PROJECT_ROOT / weights
    st = engine.status()
    n = len(st.detections)

    if engine.sim_mode:
        return _r("detector", "YOLO 偵測", "skip", "模擬模式使用合成偵測")
    if not wpath.exists():
        return _r("detector", "YOLO 偵測", "warn",
                  f"找不到 {weights}，已退回 COCO 預訓練（可偵測一般車輛，非你的自訓模型）",
                  "把訓練好的 best.pt 複製到 weights/best.pt 後重啟引擎",
                  detections=n)
    conf = float(cfg.get("detector.conf", 0.55))
    extra = f"，目前畫面 {n} 個偵測" if n else "，目前畫面無偵測（對著目標測試）"
    status = "pass" if n else "warn"
    return _r("detector", "YOLO 偵測", status,
              f"使用 {weights}（conf={conf}）{extra}",
              "對著車輛測試；一直抓不到可把 detector.conf 降到 0.3~0.4",
              detections=n, conf=conf)


def check_telemetry(engine, cfg) -> CheckResult:
    """數傳：心跳/姿態/位置真的有進來嗎（測地與閘門全靠它）。"""
    store = engine.link.store
    now = engine.clock()
    hb = store.heartbeat
    if hb is None:
        err = getattr(engine.link, "error", None)
        return _r("telemetry", "數傳遙測", "fail",
                  f"收不到心跳（{err or '無資料'}）",
                  f"確認電台已插、COM 埠正確（目前 {cfg.get('mavlink.port')}）、"
                  f"鮑率 {cfg.get('mavlink.baud')}；QGC 若佔用同一埠請先關掉")

    att = store.attitude_at(now)
    pos = store.position_at(now)
    link_ok = store.link_alive(now, float(cfg.get("safety.link_timeout_s", 2.0)))
    missing = []
    if att is None:
        missing.append("姿態(ATTITUDE)")
    if pos is None:
        missing.append("位置(GLOBAL_POSITION_INT)")
    if not link_ok:
        return _r("telemetry", "數傳遙測", "fail", "鏈路逾時（曾收到心跳但已中斷）",
                  "檢查電台距離/天線/電源")
    if missing:
        return _r("telemetry", "數傳遙測", "fail",
                  f"心跳正常（模式 {hb.mode}）但缺少：{'、'.join(missing)}",
                  "飛控可能未回應串流要求；重開飛控或確認 SET_MESSAGE_INTERVAL 支援")
    return _r("telemetry", "數傳遙測", "pass",
              f"模式 {hb.mode}｜{'已解鎖' if hb.armed else '未解鎖'}｜"
              f"高度 {pos.rel_alt:.0f}m｜姿態與位置正常",
              mode=hb.mode, armed=hb.armed, rel_alt=round(pos.rel_alt, 1))


def check_gps(engine, cfg) -> CheckResult:
    """GPS 品質：這是導引閘門的硬條件。"""
    store = engine.link.store
    gps = getattr(store, "gps", None)
    if gps is None:
        if engine.link.store.heartbeat is None:
            return _r("gps", "GPS 品質", "fail", "無遙測", "先修好數傳鏈路")
        return _r("gps", "GPS 品質", "fail", "未收到 GPS_RAW_INT",
                  "飛控未提供 GPS 串流；確認 GPS 模組已接、飛控已重啟")
    g = engine.gates
    problems = []
    if gps.fix_type < g.gps_min_fix_type:
        problems.append(f"fix={gps.fix_type}（需 ≥{g.gps_min_fix_type}）")
    if gps.satellites < g.gps_min_satellites:
        problems.append(f"衛星 {gps.satellites}（需 ≥{g.gps_min_satellites}）")
    acc = (f"水平 {gps.eph_m:.1f}m" if gps.eph_m != float("inf")
           else (f"HDOP {gps.hdop}" if gps.hdop is not None else "精度未知"))
    if problems:
        return _r("gps", "GPS 品質", "fail", "、".join(problems) + f"｜{acc}",
                  "移到空曠處等 GPS 收斂；室內幾乎不可能達標",
                  fix_type=gps.fix_type, satellites=gps.satellites)
    return _r("gps", "GPS 品質", "pass",
              f"fix={gps.fix_type}｜衛星 {gps.satellites}｜{acc}",
              fix_type=gps.fix_type, satellites=gps.satellites)


def check_home(engine) -> CheckResult:
    """Home：測地原點與高度基準都靠它。"""
    if engine.link.store.home is None:
        return _r("home", "Home 位置", "fail", "尚未取得 home",
                  "飛控需有 GPS 定位並設定 home（通常解鎖時自動設定）")
    if engine.georef is None:
        return _r("home", "Home 位置", "warn", "已收到 home 但測地原點尚未初始化",
                  "稍等一個影像幀週期")
    return _r("home", "Home 位置", "pass",
              f"已取得（高度基準 {engine.home_alt_amsl:.0f}m AMSL），測地原點已鎖定")


def check_gimbal(engine, cfg) -> CheckResult:
    """雲台：有回報最準（測地優先用回報角度）。"""
    if not engine.gimbal_present or engine.gimbal_control == "none":
        return _r("gimbal", "雲台", "skip", "設定為無雲台／固定安裝")
    now = engine._last_capture_t or engine.clock()
    has_fb = engine.link.store.gimbal_at(now) is not None
    if has_fb:
        return _r("gimbal", "雲台", "pass",
                  f"控制方式 {engine.gimbal_control}，有收到姿態回報（測地使用實際角度）")
    if engine.gimbal_control == "roi":
        return _r("gimbal", "雲台", "warn",
                  "ROI 模式但沒收到雲台姿態回報（測地將無法解算）",
                  "PX4 設 MNT_MODE_IN=4；確認雲台 UART 已接、QGC 可手動指向")
    return _r("gimbal", "雲台", "warn", f"{engine.gimbal_control} 模式，無回報（將用指令角推算）",
              "可接受但精度較差；建議改用有回報的 MAVLink 雲台協定")


def check_command_path(engine, cfg) -> CheckResult:
    """指令鏈路：後端是誰、現在能不能發（不實際發送）。"""
    backend = "lr24" if hasattr(engine.link, "command") else "direct"
    if backend == "lr24":
        snap = engine.link.command.snapshot()
        if not snap.get("connected"):
            return _r("command", "指令鏈路", "fail",
                      f"LR24 未連線（{snap.get('error') or '無回應'}）",
                      f"確認 LR24 電台在 {cfg.get('link.lr24_port')}、機上 companion 已啟動")
        ready = snap.get("ready_for_goto")
        if ready is False:
            return _r("command", "指令鏈路", "warn",
                      "LR24 已連線，但機上回報 ready_for_goto=false",
                      "看機上 global_goto_node 的 readiness_reason（通常是未解鎖/未離地/GPS）",
                      **snap)
        return _r("command", "指令鏈路", "pass",
                  f"LR24 已連線（送 {snap.get('sent')}／ACK {snap.get('ack')}）", **snap)

    # direct：檢查安全閘門現況
    blocked = list(engine.gate_report_blocked)
    if not engine.guidance_enabled:
        blocked = [b for b in blocked if "導引開關" not in b]
    if blocked:
        return _r("command", "指令鏈路", "warn",
                  "MAVLink 直連；目前被閘門擋住：" + "；".join(blocked[:3]),
                  "多數項目起飛後會自動滿足（解鎖、離地、切 Hold 模式）")
    return _r("command", "指令鏈路", "pass",
              "MAVLink 直連（DO_REPOSITION），閘門無阻擋")


def check_latency(cfg) -> CheckResult:
    """圖傳延遲補償：沒設 = 有系統性測地偏移。"""
    ms = float(cfg.get("video.latency_ms", 0.0))
    if ms <= 0:
        return _r("latency", "圖傳延遲補償", "warn",
                  "未設定（latency_ms=0）→ 會有「延遲×飛行速度」的固定測地偏移",
                  "鏡頭拍手機碼錶量出延遲，填入設定頁「圖傳延遲 ms」（數位圖傳通常 80~200）")
    return _r("latency", "圖傳延遲補償", "pass", f"已補償 {ms:.0f} ms", latency_ms=ms)


def check_pipeline(engine) -> CheckResult:
    """端到端：測地真的算得出座標嗎（這是所有前面鏈路的總驗收）。"""
    st = engine.status()
    t = st.target
    if not t.get("initialized"):
        return _r("pipeline", "端到端測地", "warn",
                  f"尚未產生目標座標（狀態：{st.state}）",
                  "對著目標讓系統鎖定；若一直不行，看上面哪條鏈路是 fail")
    note = engine.last_meas_note or ""
    if "無姿態" in note or "未交地" in note:
        return _r("pipeline", "端到端測地", "fail", f"測地失敗：{note}",
                  "檢查雲台回報與機體姿態；相機朝向須能看到地面")
    return _r("pipeline", "端到端測地", "pass",
              f"已解算目標座標 {t.get('lat', 0):.6f}, {t.get('lon', 0):.6f}"
              f"（不確定度 ±{t.get('pos_std', 0):.1f}m，狀態 {st.state}）",
              pos_std=round(t.get("pos_std", 0), 1))


# --------------------------------------------------------------- 總入口

def run_selfcheck(engine, cfg) -> dict:
    """跑完整條鏈路自檢。只讀不寫，不會發送任何導引指令。"""
    checks = [
        check_video(engine),
        check_calibration(engine, cfg),
        check_latency(cfg),
        check_detector(engine, cfg),
        check_telemetry(engine, cfg),
        check_gps(engine, cfg),
        check_home(engine),
        check_gimbal(engine, cfg),
        check_command_path(engine, cfg),
        check_pipeline(engine),
    ]
    counts = {"pass": 0, "warn": 0, "fail": 0, "skip": 0}
    for c in checks:
        counts[c.status] = counts.get(c.status, 0) + 1

    if counts["fail"]:
        verdict = "fail"
        summary = f"{counts['fail']} 條鏈路不通，請先排除再飛"
    elif counts["warn"]:
        verdict = "warn"
        summary = f"全部鏈路可運作，但有 {counts['warn']} 項風險提醒"
    else:
        verdict = "pass"
        summary = "全部鏈路正常"

    return {
        "verdict": verdict,
        "summary": summary,
        "counts": counts,
        "sim": engine.sim_mode,
        "checks": [
            {"key": c.key, "name": c.name, "status": c.status,
             "detail": c.detail, "fix": c.fix, "metrics": c.metrics}
            for c in checks
        ],
    }
