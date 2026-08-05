"""啟動器與自動安裝的契約測試。

這裡守的是一個很特定的失敗：**別人的電腦**跑不起來，而我們的跑得起來。
開發機什麼都裝過了，所以「少宣告一個相依」在這裡永遠是綠的——實際上就發生過：
pymavlink 沒宣告 pyserial，開發機被別的套件帶進來過，同學的乾淨環境一開 COM 埠
就 No module named 'serial'，而錯誤訊息寫的是「COM 埠開啟失敗」看起來像接線問題。

所以這些測試不驗「現在能不能 import」，而驗**宣告的一致性**：
requirements.txt、bootstrap 的模組對照表、start.bat 三者不能各說各話。
"""

from __future__ import annotations

import ast
import importlib.util
import re
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parent.parent
BOOTSTRAP = ROOT / "tools" / "bootstrap.py"
START_BAT = ROOT / "start.bat"
REQUIREMENTS = ROOT / "requirements.txt"

sys.path.insert(0, str(ROOT / "tools"))
import bootstrap  # noqa: E402


def requirement_names() -> set[str]:
    """requirements.txt 裡的發行版名稱（去掉版本與 environment marker）。"""
    names = set()
    for raw in REQUIREMENTS.read_text(encoding="utf-8").splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        name = re.split(r"[<>=!~;\[]", line, 1)[0].strip()
        if name:
            names.add(name)
    return names


# ---------------------------------------------------------------- 宣告一致性


def test_every_requirement_is_classified():
    """requirements.txt 新增一項，就必須決定它是「啟動必要」還是「只有測試要」。

    沒有這個測試，新相依會靜靜地不被啟動器檢查——於是啟動器說「這個 Python 可以」，
    使用者按下去才在別的地方炸掉，而那正是這整套東西要消滅的體驗。
    """
    declared = requirement_names()
    classified = set(bootstrap.REQUIRED_MODULES) | bootstrap.DEV_ONLY
    unclassified = declared - classified
    assert not unclassified, (
        f"requirements.txt 有 {sorted(unclassified)} 沒有分類：請加進 bootstrap.py 的 "
        "REQUIRED_MODULES（啟動必要）或 DEV_ONLY（只有測試要）")


def test_no_phantom_requirements():
    """反向：對照表列的東西必須真的在 requirements.txt 裡，否則永遠裝不到。"""
    declared = requirement_names()
    phantom = (set(bootstrap.REQUIRED_MODULES) | bootstrap.DEV_ONLY) - declared
    assert not phantom, (
        f"bootstrap.py 要求 {sorted(phantom)}，但 requirements.txt 沒有列——"
        "自動安裝永遠不會把它裝起來，檢查卻會一直失敗")


def test_import_names_match_the_installed_distributions():
    """發行版名稱 → import 名稱的對照要正確。

    opencv-python→cv2、pyserial→serial、PyYAML→yaml 這種對不上時，症狀是
    「pip 明明裝好了，啟動器還是說少套件」——無限迴圈式的重裝。
    用實際安裝的中繼資料驗證，不是憑印象。
    """
    from importlib.metadata import PackageNotFoundError, distribution

    for dist_name, module in sorted(bootstrap.REQUIRED_MODULES.items()):
        try:
            dist = distribution(dist_name)
        except PackageNotFoundError:
            pytest.skip(f"{dist_name} 沒裝在這個環境，無法對拍")
        tops = (dist.read_text("top_level.txt") or "").split()
        if not tops:      # 有些新式打包沒有 top_level.txt，退而求其次
            assert importlib.util.find_spec(module) is not None, (
                f"{dist_name} 裝好了，但 import {module} 找不到")
            continue
        assert module in tops, (
            f"{dist_name} 提供的是 {tops}，對照表卻寫 import {module}")


def test_runtime_modules_are_actually_imported_somewhere():
    """啟動必要清單不該收留沒人用的東西（多裝一個就多一個裝不起來的風險）。"""
    sources = []
    for path in list(ROOT.glob("*.py")) + list((ROOT / "uav_yolo").rglob("*.py")):
        sources.append(path.read_text(encoding="utf-8", errors="replace"))
    blob = "\n".join(sources)
    for dist_name, module in sorted(bootstrap.REQUIRED_MODULES.items()):
        used = re.search(rf"^\s*(import|from)\s+{re.escape(module)}\b", blob, re.M)
        assert used, (
            f"{dist_name}（import {module}）被列為啟動必要，但 uav_yolo/ 與 run.py "
            "裡沒有任何 import——不是清單過時，就是名字寫錯")


# ---------------------------------------------------------------- 舊 Python


def test_bootstrap_parses_on_old_python():
    """bootstrap.py 自己必須能在 3.8 上跑起來。

    它存在的意義之一是告訴使用者「你的 Python 太舊」。如果它自己就用了 3.10 語法，
    對方只會看到一段 SyntaxError traceback，完全不知道該做什麼——最需要那句人話的人
    正好是唯一看不到它的人。
    """
    src = BOOTSTRAP.read_text(encoding="utf-8")
    try:
        ast.parse(src, feature_version=(3, 8))
    except SyntaxError as exc:
        pytest.fail(f"bootstrap.py 用了 3.8 沒有的語法：第 {exc.lineno} 行 {exc.text!r}")


def test_bootstrap_defers_annotations():
    """有 `from __future__ import annotations`，註解才不會在舊版被求值。"""
    tree = ast.parse(BOOTSTRAP.read_text(encoding="utf-8"))
    futures = [n for n in tree.body
               if isinstance(n, ast.ImportFrom) and n.module == "__future__"]
    assert any(a.name == "annotations" for n in futures for a in n.names), (
        "bootstrap.py 少了 from __future__ import annotations，"
        "型別註解會在舊版 Python 上直接 TypeError")


def test_bootstrap_output_is_ascii():
    """輸出到 cmd 視窗的字串一律 ASCII：代碼頁每台機器不同，中文會變一排問號。

    只檢查會被印出去的字串（out(...) 與 print(...)），註解與 docstring 不限。
    """
    tree = ast.parse(BOOTSTRAP.read_text(encoding="utf-8"))
    offenders = []
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        fn = node.func
        name = getattr(fn, "id", None) or getattr(fn, "attr", None)
        if name not in ("out", "print"):
            continue
        for arg in ast.walk(node):
            if isinstance(arg, ast.Constant) and isinstance(arg.value, str):
                if not arg.value.isascii():
                    offenders.append((node.lineno, arg.value[:40]))
    assert not offenders, f"這些輸出不是 ASCII：{offenders}"


# ---------------------------------------------------------------- --check


def test_check_reports_ready_in_this_environment():
    """跑得了測試的直譯器，照定義就該被啟動器接受。"""
    ok, why = bootstrap.is_ready()
    assert ok, f"目前這個環境跑得動測試，--check 卻說不行：{why}"


def test_check_exit_code_via_cli():
    proc = subprocess.run([sys.executable, str(BOOTSTRAP), "--check"],
                          capture_output=True, text=True)
    assert proc.returncode == 0, f"--check 應該回 0：{proc.stdout}{proc.stderr}"


def test_check_names_what_is_missing(monkeypatch):
    """少套件時要講出**是哪一個**，不是只說失敗。"""
    monkeypatch.setitem(bootstrap.REQUIRED_MODULES, "totally-not-real",
                        "uav_yolo_no_such_module")
    ok, why = bootstrap.is_ready()
    assert not ok
    assert "totally-not-real" in why, f"訊息沒點名少了什麼：{why!r}"


def test_windows_only_packages_are_skipped_off_windows():
    linux = bootstrap.needed_modules("linux")
    win = bootstrap.needed_modules("win32")
    assert "pygrabber" in win
    assert "pygrabber" not in linux, "非 Windows 不該要求 DirectShow 專用套件"


# ---------------------------------------------------------------- start.bat


def bat_text() -> str:
    return START_BAT.read_bytes().decode("ascii")


def test_start_bat_is_ascii_and_crlf():
    """cmd 用系統代碼頁讀 .bat，非 ASCII 會炸成亂碼視窗；LF 會讓多行區塊解析錯亂。"""
    raw = START_BAT.read_bytes()
    non_ascii = [b for b in raw if b > 127]
    assert not non_ascii, f"start.bat 有 {len(non_ascii)} 個非 ASCII 位元組"
    text = raw.decode("ascii")
    assert "\r\n" in text, "start.bat 必須是 CRLF"
    assert not re.search(r"(?<!\r)\n", text), "start.bat 有單獨的 LF 行尾"


def test_start_bat_has_no_machine_specific_paths():
    """寫死開發機的路徑正是同學按不動的原因。"""
    text = bat_text()
    assert "C:\\Users\\user" not in text, "start.bat 寫死了開發機的家目錄"
    assert "miniconda3\\python.exe" not in text.replace("%USERPROFILE%\\miniconda3\\python.exe", ""), (
        "start.bat 寫死了某個 conda 安裝路徑")


def test_start_bat_quotes_every_interpreter_use():
    """直譯器路徑可能含空白（C:\\Program Files\\...），沒加引號會被切成兩半。"""
    text = bat_text()
    for var in ("PY", "PY_ANY", "UAV_YOLO_RESOLVED_PY", "CAND"):
        # 取用（%VAR%）但沒被引號包住的地方；set "VAR=..." 的定義端不算。
        for m in re.finditer(rf"(?<!\")%{var}%(?!\")", text):
            line = text[text.rfind("\n", 0, m.start()) + 1:
                        text.find("\n", m.start())]
            if line.strip().startswith(("set ", "rem ", "echo ")):
                continue
            pytest.fail(f"%{var}% 沒加引號，路徑含空白就會壞：{line.strip()!r}")


def test_start_bat_every_goto_and_call_target_exists():
    """打錯一個標籤，cmd 會直接跳到檔尾靜靜結束——視窗一閃就沒了。"""
    text = bat_text()
    labels = set(re.findall(r"^:(\w+)", text, re.M))
    targets = set(re.findall(r"(?:goto|call)\s+:(\w+)", text))
    missing = targets - labels
    assert not missing, f"start.bat 跳到不存在的標籤：{sorted(missing)}"


def test_start_bat_has_no_duplicate_labels():
    """重複的標籤 = cmd 跳到第一個，第二個變成死碼，而且通常代表某段被貼錯位置。

    真的發生過：把 :log 的內容誤插進 :launch 中間，於是主流程還沒啟動伺服器
    就 exit /b 0——雙擊之後偵測完就安靜結束。標籤全都還「存在」，所以只檢查
    「跳轉目標存在」的測試抓不到。
    """
    labels = re.findall(r"^:(\w+)", bat_text(), re.M)
    dupes = sorted({l for l in labels if labels.count(l) > 1})
    assert not dupes, f"start.bat 有重複的標籤：{dupes}"


def test_start_bat_launch_actually_launches():
    """:launch 到下一個標籤之間，一定要真的把伺服器開起來。"""
    text = bat_text()
    body = text[text.index(":launch"):text.index(":already_running")]
    assert 'start "UAV_yolo Server"' in body, ":launch 區塊裡沒有啟動伺服器"
    assert "exit /b 0" not in body.split('start "UAV_yolo Server"')[0], (
        ":launch 在啟動伺服器之前就結束了——雙擊後會安靜地什麼都不做")


def test_start_bat_records_its_decisions(tmp_path):
    """視窗一關訊息就沒了，而使用者永遠是關掉之後才來問發生什麼事。"""
    text = bat_text()
    assert ":log" in text and "launcher.log" in text
    # %DATE% 在中文 Windows 是「週三 2026/08/05」——寫進 log 就是非 ASCII 位元組，
    # 貼到任何地方都是亂碼，而這個檔的用途正是「傳給我看」。（rem 註解不算）
    code = [l for l in text.splitlines() if not l.strip().lower().startswith("rem ")]
    assert not [l for l in code if "%DATE%" in l], (
        "log 用了本地化的日期變數，中文 Windows 會寫出亂碼")
    for expect in ('call :log using "%PY%"', "SETUP FAILED", "no Python 3.10+"):
        assert expect in text, f"少了關鍵決策記錄：{expect}"


def test_start_bat_says_which_environment_it_chose():
    """「為什麼沒有 .venv 資料夾」是第一個會被問的問題。

    答案幾乎都是「因為你不需要」，但畫面上不講，成功就看起來像失敗。
    """
    text = bat_text()
    assert "no .venv needed" in text, "沒有在畫面上說明為什麼沒有建 .venv"
    assert "auto-setup build" in text, (
        "少了版本橫幅——回報問題時無法分辨對方是不是還在用舊的 start.bat")


def test_start_bat_refers_to_the_real_bootstrap():
    text = bat_text()
    assert "tools\\bootstrap.py" in text, "start.bat 沒有呼叫自動安裝腳本"
    assert BOOTSTRAP.exists()
    assert "--check" in text, "start.bat 沒有用 --check 判斷候選直譯器"


def test_start_bat_keeps_the_close_window_to_stop_habit():
    """伺服器要開在自己的視窗：操作員的既有動作是「關掉視窗＝停伺服器」。"""
    text = bat_text()
    assert re.search(r'start "UAV_yolo Server"', text), "伺服器沒有開在獨立視窗"
    assert "--serve" in text, "少了 --serve 這條自我再呼叫的路徑"
    assert re.search(r"^:serve\b", text, re.M)


def test_start_bat_never_uses_if_errorlevel_for_child_processes():
    """🔴 `if errorlevel 1` 是「>= 1」，而 cmd 用**有號數**比較。

    當機的結束碼都是 NTSTATUS（0xC0000005 存取違規 = -1073741819，缺 DLL
    = -1073741515），全部是負數，於是 `if errorlevel 1` 判定為「成功」。
    後果有兩層：①伺服器當掉時 :serve 直接走到 exit /b 0，視窗無聲關閉——
    正是這次重寫要消滅的症狀；②:try_path 會把一個「連啟動都會當」的
    python.exe 當成合格直譯器，印出 Using Python 之後才閃退。
    改用 `%ERRORLEVEL% NEQ 0` / `EQU 0` 做無號比較。
    """
    text = bat_text()
    bad = [l.strip() for l in text.splitlines()
           if re.match(r"\s*if\s+(not\s+)?errorlevel\s", l, re.I)]
    assert not bad, (
        "這些行用了有號比較的 if errorlevel，當機碼（負數）會被當成成功：\n  "
        + "\n  ".join(bad))


def test_start_bat_treats_ctrl_c_as_a_clean_stop():
    """Ctrl+C 的結束碼是 0xC000013A，改用 NEQ 0 之後它也會變成「錯誤」。

    操作員按 Ctrl+C 停伺服器是正常動作，不該跳一頁錯誤訊息嚇人。
    """
    text = bat_text()
    assert "-1073741510" in text, (
        "沒有把 Ctrl+C（0xC000013A = -1073741510）當成正常結束")


def test_start_bat_does_not_claim_the_error_is_on_screen():
    """run.py 在 import 任何東西之前就把 fd 2 導進 data/server.log。

    所以伺服器視窗**真的什麼都沒有**。原本寫「原因在上面幾行」是假的：
    連綁不到埠這種最常見的失敗，畫面上也只有一行看起來成功的橫幅。
    失敗時要把 log 的尾巴印出來，而不是叫使用者去看不存在的東西。
    """
    text = bat_text()
    shown = [l.strip() for l in text.splitlines()
             if l.strip().lower().startswith("echo ")]      # rem 註解不算
    assert not [l for l in shown if "lines above" in l], (
        "還在對使用者宣稱錯誤訊息在畫面上，但那裡是空的")
    assert "server.log" in text and "-Tail" in text, (
        "失敗路徑沒有把 data\\server.log 的尾巴印出來")


def test_start_bat_waits_for_the_port_before_opening_the_browser():
    """固定睡幾秒是猜的：冷啟動光 import cv2+torch+ultralytics 就 5 秒以上。

    太早開瀏覽器 = 使用者看到「無法連線」，以為壞了。
    """
    text = bat_text()
    assert ":wait_port" in text, "沒有等埠開始監聽就開瀏覽器"
    assert "LISTENING" in text


def test_start_bat_refuses_to_start_a_second_server_on_a_busy_port():
    """🔴 已經有一台在跑時，絕對不能再開第二台然後把瀏覽器指過去。

    第二台根本綁不到埠會馬上死；而 :wait_port 只問「這個埠有沒有人在聽」，
    舊的那台還聽著 → 判定成功 → 開瀏覽器 → 操作員看到 UI，以為看的是自己
    剛啟動的那一份。實際踩到：使用者重新 clone 後開得起 UI，但那是幾小時前
    的舊行程在服務。開起飛前檢查清單時看錯行程是會出事的。
    """
    text = bat_text()
    assert ":port_busy" in text, "啟動前沒有先檢查埠是不是已經被佔用"
    launch = text[text.index(":launch"):text.index(":serve")]
    assert "call :port_busy" in launch, ":launch 沒有在 start 之前做這個檢查"
    assert launch.index("call :port_busy") < launch.index('start "UAV_yolo Server"'), (
        "檢查排在啟動第二台之後就沒有意義了")
    assert ":already_running" in text


def test_start_bat_uses_setx_for_the_persistent_override():
    """`set` 只活到那個 console 關閉為止——照著做完再雙擊 .bat 一定沒效。"""
    text = bat_text()
    assert "setx UAV_YOLO_PY" in text, (
        "提示使用者用 set 設定 UAV_YOLO_PY，但那個設定在下一個視窗就消失了")


def test_start_bat_pauses_on_every_dead_end():
    """雙擊執行時，任何 exit /b 1 之前都要 pause，否則錯誤訊息一閃就消失。

    這就是同學看到的原始症狀：視窗閃一下，沒人知道發生什麼事。
    """
    text = bat_text()
    lines = [l.strip() for l in text.splitlines()]
    for i, line in enumerate(lines):
        if line == "exit /b 1":
            window = lines[max(0, i - 14):i]
            assert "pause" in window, (
                f"第 {i + 1} 行的 exit /b 1 前面沒有 pause，錯誤訊息會來不及看")


# ---------------------------------------------------------------- venv 健康度


def test_broken_venv_is_rebuilt_not_reused(tmp_path, monkeypatch):
    """半成品或別台電腦拷來的 .venv 不能沿用。

    使用者在幾分鐘的安裝中途關掉視窗，venv 的 python.exe 早就複製進去了；
    沿用它會一路失敗，而訊息會指向網路問題——永遠修不好的重試迴圈。
    整包資料夾用隨身碟拷過來也一樣（.gitignore 擋 git，擋不了複製）。
    """
    venv_dir = tmp_path / ".venv"
    exe = bootstrap.venv_python(venv_dir)
    exe.parent.mkdir(parents=True)
    exe.write_bytes(b"not a real interpreter")     # 跑不起來的殼

    created = []

    def fake_run(cmd):
        created.append(cmd)
        exe.parent.mkdir(parents=True, exist_ok=True)   # 重建後才會有這個目錄
        exe.write_bytes(b"a fresh one")
        return True

    monkeypatch.setattr(bootstrap, "_run", fake_run)
    bootstrap.create_venv(venv_dir)
    assert created, "壞掉的 .venv 被沿用了，沒有重建"
    assert any("venv" in " ".join(map(str, c)) for c in created)


def test_healthy_venv_is_reused(tmp_path, monkeypatch):
    """反向：好的環境不該每次都被砍掉重建（那要再等好幾分鐘）。"""
    venv_dir = tmp_path / ".venv"
    exe = bootstrap.venv_python(venv_dir)
    exe.parent.mkdir(parents=True)
    exe.write_bytes(b"pretend")

    monkeypatch.setattr(bootstrap, "venv_is_alive", lambda e: True)
    monkeypatch.setattr(bootstrap, "_run",
                        lambda cmd: pytest.fail(f"健康的 venv 被重建了：{cmd}"))
    assert bootstrap.create_venv(venv_dir) == exe


def test_failed_venv_creation_leaves_nothing_behind(tmp_path, monkeypatch):
    """建失敗要收乾淨，否則殘骸會讓下一次執行以為「已經有了」。"""
    venv_dir = tmp_path / ".venv"

    def fake_run(cmd):
        venv_dir.mkdir(parents=True, exist_ok=True)   # 半成品
        (venv_dir / "pyvenv.cfg").write_text("half")
        return False

    monkeypatch.setattr(bootstrap, "_run", fake_run)
    assert bootstrap.create_venv(venv_dir) is None
    assert not venv_dir.exists(), "建失敗卻留下半成品目錄，會毒害之後每一次執行"


def test_user_site_flag_only_on_the_fallback_path(tmp_path, monkeypatch):
    """裝進使用者自己的 Python 時要走 --user（免管理員權限、不碰系統目錄）；
    裝進 venv 時絕對不能加（pip 會直接拒絕）。"""
    seen = []
    monkeypatch.setattr(bootstrap, "_run", lambda cmd: seen.append(cmd) or True)
    req = tmp_path / "requirements.txt"
    req.write_text("numpy\n")

    bootstrap.pip_install(Path("python"), req, user_site=False)
    assert not any("--user" in c for c in seen)

    seen.clear()
    bootstrap.pip_install(Path("python"), req, user_site=True)
    install = [c for c in seen if "install" in c and "-r" in c][0]
    assert "--user" in install
