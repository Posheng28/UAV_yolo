"""測試不得寫入使用者真正的 config/local.yaml。

實際踩過：`tests/test_ui_contract.py` 用 `Config()`（不帶 local_path）再
`update({"system": {"mode": "sim"}})`，於是**跑一次 pytest 就把操作員的地面站
從實機模式偷偷切回模擬**。這種副作用不會有任何錯誤訊息，只會在下次重啟引擎時
變成「怎麼又跑去模擬了」，而且極難聯想到是測試造成的。
"""

import ast
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
TESTS = ROOT / "tests"


def test_no_test_constructs_config_without_an_explicit_local_path():
    """用 AST 找真正的 Config(...) 呼叫——正則會誤抓說明文字裡提到的名字。"""
    offenders = []
    for path in sorted(TESTS.glob("test_*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if not isinstance(node, ast.Call):
                continue
            func = node.func
            name = func.id if isinstance(func, ast.Name) else getattr(func, "attr", None)
            if name != "Config":
                continue
            if any(kw.arg == "local_path" for kw in node.keywords):
                continue
            offenders.append(f"{path.name}:{node.lineno}")
    assert not offenders, (
        "這些地方用了無參數的 Config()，會指向專案真正的 config/local.yaml；"
        f"改成 Config(local_path=tmp_path / 'local.yaml')：{offenders}"
    )


def test_local_yaml_is_not_modified_by_importing_the_test_suite():
    """真正的 local.yaml 若存在，內容不該被測試改動。

    這裡只驗「檔案沒有被測試建立出來」——CI 的乾淨 checkout 沒有 local.yaml，
    跑完測試後也不該冒出一個。
    """
    local = ROOT / "config" / "local.yaml"
    if local.exists():
        return  # 開發機上使用者本來就有這個檔，內容由上一個測試把關
    assert not local.exists(), "測試過程中生出了 config/local.yaml"
