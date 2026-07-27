"""UI ↔ API ↔ 設定檔 的契約一致性（機械式交叉比對）。

這類不一致不會噴錯，只會「靜默失效」：設定頁某欄位打錯 key 就永遠存不進去、
前端讀一個後端沒發的狀態欄位就永遠顯示 undefined。人工看不出來，所以做成測試。
"""

import re
from pathlib import Path

import pytest
import yaml

ROOT = Path(__file__).resolve().parent.parent
APP_JS = (ROOT / "uav_yolo" / "webapp" / "static" / "app.js").read_text(encoding="utf-8")
INDEX_HTML = (ROOT / "uav_yolo" / "webapp" / "static" / "index.html").read_text(encoding="utf-8")
SERVER_PY = (ROOT / "uav_yolo" / "webapp" / "server.py").read_text(encoding="utf-8")
DEFAULTS = yaml.safe_load((ROOT / "config" / "default.yaml").read_text(encoding="utf-8"))


def _flatten(node, prefix=""):
    out = set()
    if isinstance(node, dict):
        for k, v in node.items():
            key = f"{prefix}.{k}" if prefix else k
            out.add(key)
            out |= _flatten(v, key)
    return out


CONFIG_KEYS = _flatten(DEFAULTS)


# ---------------- 前端呼叫的 API 都要存在 ----------------

def test_every_frontend_api_call_has_a_route():
    called = set(re.findall(r"""["'](/api/[a-zA-Z0-9_/]+)["']""", APP_JS))
    routes = set(re.findall(r"""@app\.(?:get|post|put|delete)\(\s*["']([^"']+)["']""", SERVER_PY))
    missing = called - routes
    assert not missing, f"前端呼叫了不存在的端點：{sorted(missing)}（會靜默 404）"


def test_every_route_uses_expected_method_from_frontend():
    """前端用 POST 打的端點，後端不能只註冊 GET（反之亦然）。"""
    post_calls = set(re.findall(r"""post\(\s*["'](/api/[a-zA-Z0-9_/]+)["']""", APP_JS))
    post_routes = set(re.findall(r"""@app\.post\(\s*["']([^"']+)["']""", SERVER_PY))
    bad = post_calls - post_routes
    assert not bad, f"前端以 POST 呼叫但後端未註冊 POST：{sorted(bad)}"


# ---------------- 設定頁欄位路徑都要真的存在於 default.yaml ----------------

def test_every_settings_field_path_exists_in_default_yaml():
    paths = set(re.findall(r"""path:\s*["']([a-zA-Z0-9_.]+)["']""", APP_JS))
    assert paths, "沒抓到任何設定欄位路徑，正規表示式可能失效"
    missing = {p for p in paths if p not in CONFIG_KEYS}
    assert not missing, (
        f"設定頁欄位對應到 default.yaml 不存在的 key：{sorted(missing)}；"
        "這種欄位改了會靜默無效"
    )


# ---------------- 前端引用的 DOM id 都要存在 ----------------

def test_every_referenced_element_id_exists():
    referenced = set(re.findall(r"""\$\(\s*["']#([a-zA-Z0-9_-]+)["']\s*\)""", APP_JS))
    declared = set(re.findall(r"""id=["']([a-zA-Z0-9_-]+)["']""", INDEX_HTML))
    missing = referenced - declared
    assert not missing, f"app.js 取用了 index.html 沒有的元素 id：{sorted(missing)}"


# ---------------- 前端讀的狀態欄位後端要真的發 ----------------

def test_status_payload_contains_fields_frontend_reads(tmp_path):
    """用真的 sim 引擎產一次狀態，比對前端 renderStatus 讀的頂層欄位。

    local_path 一定要指到 tmp_path：`Config()` 不帶參數會指向專案真正的
    config/local.yaml，`update()` 就會把使用者的設定改掉——實際發生過，
    跑一次測試就把操作員的地面站從實機模式偷偷切回模擬。
    """
    from uav_yolo.config import Config
    from uav_yolo.simulation import build_sim_engine

    cfg = Config(local_path=tmp_path / "local.yaml")
    cfg.update({"system": {"mode": "sim"}})
    engine = build_sim_engine(cfg, realtime=False)
    engine.sim_world.step(0.05)
    engine.step()

    import dataclasses

    payload = dataclasses.asdict(engine.status())

    # renderStatus 明確讀到的頂層欄位
    expected = {
        "state", "sim", "airframe", "guidance_enabled", "guidance_note",
        "gates", "detections", "target", "vehicle", "video", "mavlink",
        "gimbal", "last_command", "loop_hz",
        # 偵測/迴圈例外：症狀是「畫面正常但完全偵測不到」，沒有這兩個欄位
        # 操作員只會以為模型不準，往完全錯的方向查
        "detector_error", "loop_error",
    }
    missing = expected - set(payload)
    assert not missing, f"狀態少了前端會讀的欄位：{sorted(missing)}"

    # 巢狀欄位：前端直接取用，缺了就顯示 undefined
    for key in ("connected", "fps", "device", "error"):
        assert key in payload["video"], f"video 缺 {key}"
    for key in ("mode", "armed", "link_ok", "rel_alt", "home_set"):
        assert key in payload["vehicle"], f"vehicle 缺 {key}"
    for key in ("present", "control", "has_feedback"):
        assert key in payload["gimbal"], f"gimbal 缺 {key}"
    for key in ("error", "backend", "lr24"):
        assert key in payload["mavlink"], f"mavlink 缺 {key}"


# ---------------- 設定頁宣告需重啟的欄位，必須真的是重啟才生效的那些 ----------------

def test_restart_marked_fields_are_connection_shaped():
    """標 ⟳ = 需重啟。熱套用涵蓋的類別不該再標 ⟳，否則操作員會多按一次重啟。

    apply_live_config 涵蓋：safety / guidance / detector / estimator / video.latency_ms
    """
    live_prefixes = ("safety.", "guidance.", "detector.", "estimator.")
    # 例外：換權重檔要重新載入模型，apply_live_config 只改閾值不換模型，
    # 所以 detector.weights 確實需要重啟，標 ⟳ 是正確的。
    restart_required = {"detector.weights"}
    # 抓出每個 field 物件的 path 與其 hint 是否含 ⟳
    for block in re.findall(r"\{[^{}]*path:\s*[^{}]*\}", APP_JS):
        m = re.search(r"""path:\s*["']([a-zA-Z0-9_.]+)["']""", block)
        if not m:
            continue
        path = m.group(1)
        if path in restart_required:
            continue
        needs_restart = "⟳" in block
        if path.startswith(live_prefixes) or path == "video.latency_ms":
            assert not needs_restart, (
                f"{path} 已由 apply_live_config 熱套用，不該標 ⟳（會誤導操作員重啟）"
            )


def test_weights_switch_requires_restart():
    """換模型必須標 ⟳：apply_live_config 不會重新載入權重檔，
    不標的話操作員會以為切好了、實際還在跑舊模型——而模型間完全不通用
    （實測：玩具車模型對真車偵測數 44→0），這種誤解會直接毀掉任務。"""
    block = next(b for b in re.findall(r"\{[^{}]*path:\s*[^{}]*\}", APP_JS)
                 if 'detector.weights' in b)
    assert "⟳" in block, "detector.weights 需要重啟才生效，必須標 ⟳"
