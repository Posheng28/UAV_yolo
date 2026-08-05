"""把相依套件裝起來，讓沒有 conda 的人雙擊 start.bat 就能跑。

為什麼需要這支：同學 clone 下來按 start.bat，得到 `No module named uvicorn`。
啟動器原本只會「找一個已經裝好的 Python」，找不到就印錯誤——但對方要的不是
診斷訊息，是把東西裝好。這支就是那一步：建一個專案自己的 .venv、把
requirements.txt 裝進去、真的 import 一次確認它能載入。

裝進專案內的 .venv 而不是對方的系統 Python：
  · 不污染他們原本的環境（別人的專題可能鎖著別的 numpy 版本）
  · 不需要系統管理員權限
  · 砍掉 .venv 就完全復原——出事時「刪掉重來」是一句話講得完的

🔴 輸出一律 ASCII：cmd 視窗的代碼頁每台機器不一樣（CP950/CP437/CP1252），
非 ASCII 在英文版 Windows 上會變成一整排問號，安裝失敗時最需要讀懂的那幾行
反而看不懂。中文說明放 start_guide.md。

🔴 這支自己必須能在**舊版 Python 上執行**（3.8 也要能跑完並印出「你的 Python
太舊」）。所以只用 `from __future__ import annotations` 的延遲註解，不在執行期
用 3.10 才有的語法——不然使用者只會看到 SyntaxError，看不到那句人話。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parent.parent
REQUIREMENTS = PROJECT_ROOT / "requirements.txt"
VENV_DIR = PROJECT_ROOT / ".venv"
PIP_LOG = PROJECT_ROOT / "data" / "pip-install.log"

# 專案語法用到 `str | None` 這類 3.10 起才有的寫法，3.9 會在 import 當下 TypeError。
MIN_PYTHON = (3, 10)

# 發行版名稱 → import 名稱。兩者不同的（opencv-python→cv2、pyserial→serial、
# PyYAML→yaml）正是「pip list 看起來有、import 卻失敗」的來源，所以對照表寫死
# 在這裡；tests/test_bootstrap.py 會拿它跟 requirements.txt 對拍，往
# requirements.txt 加了套件卻忘了加進來，啟動器就永遠不會發現它沒裝。
REQUIRED_MODULES = {
    "numpy": "numpy",
    "opencv-python": "cv2",
    "pymavlink": "pymavlink",
    "pyserial": "serial",
    "ultralytics": "ultralytics",
    "fastapi": "fastapi",
    "uvicorn": "uvicorn",
    "PyYAML": "yaml",
    "pygrabber": "pygrabber",
}

# 只有 Windows 需要的（requirements.txt 用 environment marker 標了同一件事）。
WINDOWS_ONLY = {"pygrabber"}

# requirements.txt 裡但**不影響啟動**的：少了它們只是不能跑測試，地面站照開。
# 分開列是為了不讓啟動器變得吹毛求疵——沒裝 pytest 不該讓人開不了飛行任務。
DEV_ONLY = {"pytest", "httpx", "pyulog"}


def out(msg=""):
    """印一行。stdout 可能是任何代碼頁，所以永遠只送 ASCII。"""
    print(msg, flush=True)


# ---------------------------------------------------------------- 狀態判定


def needed_modules(platform=None):
    plat = platform if platform is not None else sys.platform
    if plat.startswith("win"):
        return dict(REQUIRED_MODULES)
    return dict((d, m) for d, m in REQUIRED_MODULES.items() if d not in WINDOWS_ONLY)


def missing_modules(platform=None):
    """這個直譯器少了哪些套件（用 find_spec，不真的 import）。

    find_spec 不執行模組，整個檢查是毫秒級——啟動器每個候選直譯器都要跑一次，
    真的 import 一次 ultralytics 要好幾秒，會讓「按下去到看見畫面」難以忍受。
    代價是抓不到「裝了但載不起來」（缺 VC++ runtime 的 cv2 之類），那個由
    安裝完的那一次完整 verify() 負責。
    """
    import importlib.util

    missing = []
    for dist, mod in sorted(needed_modules(platform).items()):
        try:
            found = importlib.util.find_spec(mod) is not None
        except (ImportError, ValueError):
            found = False        # 命名空間套件殘骸／壞掉的 .pth 都算沒有
        if not found:
            missing.append(dist)
    return missing


def is_ready():
    """跑著這支腳本的直譯器現在就能啟動地面站嗎？回 (可以嗎, 不行的原因)。

    刻意不留「已驗證」快取檔：快取會過期、會指到被刪掉的 venv、會在使用者
    手動 pip uninstall 之後繼續說「一切正常」。find_spec 掃一遍本來就夠快，
    每次重新問一次，就沒有第二種真相。
    """
    if sys.version_info < MIN_PYTHON:
        return False, "Python %d.%d is older than the required 3.10" % sys.version_info[:2]
    gone = missing_modules()
    if gone:
        return False, "missing packages: " + ", ".join(gone)
    return True, ""


# ---------------------------------------------------------------- 安裝


def venv_python(venv_dir):
    if sys.platform.startswith("win"):
        return venv_dir / "Scripts" / "python.exe"
    return venv_dir / "bin" / "python"


def _run(cmd):
    """跑一個子行程，輸出直接流到視窗。

    刻意不捕捉輸出：pip 裝 torch 要好幾分鐘，畫面上完全沒東西動的話使用者會
    以為當掉而去按叉叉——那正是留下半套環境的方法。
    """
    try:
        return subprocess.call(cmd) == 0
    except OSError as exc:
        out("[ERROR] could not run %s: %s" % (cmd[0], exc))
        return False


def venv_is_alive(exe):
    """那個 .venv 裡的 python 真的跑得動嗎？

    「檔案存在」不等於「能用」。實際會發生的兩種狀況：
      · 使用者在幾分鐘的安裝中途關掉視窗／按 Ctrl+C——venv 的 python.exe 很早
        就被複製進去了，所以留下一個殼；沿用它會一直失敗而且錯誤看起來像網路問題。
      · 整包資料夾用隨身碟／雲端硬碟從別台電腦拷過來，裡面連 .venv 一起拷了
        （.gitignore 擋得住 git，擋不住複製）——那個 venv 指向對方機器上的路徑。
    所以沿用前先問它一句話；答不出來就整個重建。
    """
    try:
        return subprocess.call(
            [str(exe), "-c", "import sys; sys.exit(0 if sys.version_info >= (3, 10) else 1)"],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL) == 0
    except OSError:
        return False


def create_venv(venv_dir=VENV_DIR):
    """建（或沿用）專案自己的 .venv，回傳裡面的 python。失敗回 None。"""
    import shutil

    exe = venv_python(venv_dir)
    if exe.exists():
        if venv_is_alive(exe):
            out("Reusing the existing environment: %s" % venv_dir)
            return exe
        out("The environment in %s is broken or came from another computer."
            % venv_dir)
        out("Rebuilding it from scratch.")
        try:
            shutil.rmtree(venv_dir)
        except OSError as exc:
            out("[ERROR] Could not remove it: %s" % exc)
            out("        Delete the .venv folder by hand and run this again.")
            return None

    out("Creating a private environment in %s" % venv_dir)
    # 用子行程而不是 venv.EnvBuilder：EnvBuilder 在某些 conda / Microsoft Store
    # 版 Python 上會拋出難懂的例外，子行程至少把 Python 自己的訊息原封印出來。
    ok = _run([sys.executable, "-m", "venv", str(venv_dir)])
    if not ok or not exe.exists():
        out("[ERROR] Could not create the environment in %s" % venv_dir)
        # 失敗留下的半成品會毒害之後每一次執行（下次看到 python.exe 就以為可以用），
        # 所以自己收乾淨。
        if venv_dir.exists():
            try:
                shutil.rmtree(venv_dir)
            except OSError:
                pass
        return None
    return exe


def pip_install(python, req=REQUIREMENTS, user_site=False):
    PIP_LOG.parent.mkdir(parents=True, exist_ok=True)
    # pip --log 是「附加」而且不管主控台多安靜都寫最詳細的內容——實測一次安裝
    # 就 41 MB。留著是為了讓裝失敗的人可以把檔案傳過來看，所以：每次先清掉
    # （不然重試幾次就滾成幾百 MB），成功了就刪掉（沒有東西要查了）。
    try:
        PIP_LOG.unlink()
    except OSError:
        pass
    base = [str(python), "-m", "pip", "--disable-pip-version-check"]

    # pip 太舊會挑不到新的 wheel（尤其 torch）。失敗不擋：離線或公司網路擋著
    # pypi 時，舊 pip 通常還是能從本機快取把東西裝起來。
    _run(base + ["install", "--upgrade", "pip"])

    out("")
    out("Installing dependencies. The first run downloads PyTorch and can")
    out("take several minutes and about 2 GB. Leave this window open.")
    out("")
    install = ["install", "--log", str(PIP_LOG), "-r", str(req)]
    # --user 只用在「裝進使用者自己的直譯器」那條退路上：它不需要系統管理員
    # 權限，也不會動到 Program Files 底下的共用 site-packages。在 venv 裡則
    # 完全不能用（pip 會直接拒絕）。
    if user_site:
        install.insert(1, "--user")
    if _run(base + install):
        try:
            PIP_LOG.unlink()
        except OSError:
            pass
        return True
    out("")
    out("[ERROR] Installing the dependencies failed.")
    # 只在檔案真的產生時才叫人去看它：pip 連啟動都失敗時根本沒有這個檔，
    # 指著一個不存在的路徑只會讓人以為自己又做錯了什麼。
    if PIP_LOG.exists():
        out("        Full pip log: %s" % PIP_LOG)
    else:
        out("        pip did not even start - see its output above.")
    out("        Usual causes: no internet, a proxy blocking pypi.org,")
    out("        or not enough free disk space (needs about 3 GB).")
    return False


def verify(python):
    """真的 import 一次。

    find_spec 只證明檔案在，import 才證明它載得起來——Windows 上 cv2 缺 VC++
    runtime、torch 缺 DLL，都是「裝好了但一 import 就炸」。那種失敗原本要等
    操作員按下啟動、視窗閃一下關掉才會發現；裝完付這幾秒，換掉那個場景。
    """
    # 一個一個 import 並且逐個回報。一次全部 import 要 7 秒以上（冷開機更久，
    # torch 光載入就很可觀），而那 7 秒正好接在好幾分鐘的安裝之後——畫面完全
    # 不動的時候，使用者會以為當掉而去關視窗，於是前功盡棄。
    out("")
    out("Checking that each package really loads:")
    failed = []
    for mod in sorted(set(needed_modules().values())):
        proc = subprocess.run([str(python), "-c", "import " + mod],
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out("  %-12s %s" % (mod, "ok" if proc.returncode == 0 else "FAILED"))
        if proc.returncode != 0:
            failed.append((mod, (proc.stderr or b"").decode("utf-8", "replace")))
    if not failed:
        return True

    out("")
    out("[ERROR] These installed but cannot be imported: %s"
        % ", ".join(m for m, _ in failed))
    for mod, err in failed:
        for line in err.strip().splitlines()[-3:]:
            out("        %s: %s" % (mod, line.encode("ascii", "replace").decode("ascii")))
    return False


def bootstrap(venv_dir=VENV_DIR, req=REQUIREMENTS, use_venv=True):
    out("=" * 64)
    out(" UAV_yolo setup - installing the packages the ground station needs")
    out("=" * 64)
    if sys.version_info < MIN_PYTHON:
        out("[ERROR] Running on Python %d.%d, but this project needs 3.10 or"
            % sys.version_info[:2])
        out("        newer. Install a newer Python from python.org and retry.")
        return 1
    if not req.exists():
        out("[ERROR] %s is missing - is this a complete checkout?" % req)
        return 1

    python = create_venv(venv_dir) if use_venv else Path(sys.executable)

    # 🔴 退路要講清楚，不能默默做。start.bat 剛剛才向使用者保證「只會動這個
    # 資料夾裡的東西」；建不出 venv 就把套件裝進他們自己的 Python，是違背那句
    # 承諾。所以明講裝到哪裡、並且走 --user（不需要管理員權限、不碰系統目錄）。
    user_site = False
    if python is None:
        user_site = True
        python = Path(sys.executable)
        out("")
        out("-" * 64)
        out("NOTE: a private environment could not be created, so the packages")
        out("      will go into your own Python instead of this folder:")
        out("          %s" % sys.executable)
        out("      They are installed per-user (pip --user): no administrator")
        out("      rights needed, and nothing system-wide is modified.")
        out("      Press Ctrl+C now if you would rather not.")
        out("-" * 64)
        out("")

    if not pip_install(python, req, user_site=user_site):
        return 1
    if not verify(python):
        return 1

    out("")
    out("Setup complete. Ground station will run on:")
    out("    %s" % python)
    return 0


# ---------------------------------------------------------------- CLI


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Install the packages UAV_yolo needs. ASCII output on purpose.")
    ap.add_argument("--check", action="store_true",
                    help="exit 0 if the Python running this script can start the app")
    ap.add_argument("--no-venv", action="store_true",
                    help="install into the current interpreter instead of .venv")
    ap.add_argument("--venv", default=str(VENV_DIR), help=argparse.SUPPRESS)
    args = ap.parse_args(argv)

    if args.check:
        ok, why = is_ready()
        if not ok:
            out("not ready: %s" % why)
        return 0 if ok else 1

    return bootstrap(venv_dir=Path(args.venv), use_venv=not args.no_venv)


if __name__ == "__main__":
    sys.exit(main())
