# 啟動說明（start.bat）

**雙擊 `start.bat` 就好。**第一次會自動把需要的套件裝起來，之後直接開。
不需要先裝 conda、不需要自己打 `pip install`、不需要改任何路徑。

| | |
|---|---|
| 網址 | http://localhost:8610 |
| 停止伺服器 | 關掉那個「UAV_yolo Server」視窗（或在裡面按 Ctrl+C） |
| 模擬／實機 | 在網頁的**設定頁**切換 → 按「重啟引擎」。不是用不同的 .bat |

## 第一次啟動會發生什麼

1. 找一個能用的 Python（見下面「它怎麼找 Python」）。
2. 如果找到的 Python 沒有裝齊套件 → 在專案資料夾裡建一個 `.venv`，
   把 `requirements.txt` 裝進去，再真的 import 一次確認裝得起來。
   **會下載 PyTorch，大約 2 GB、數分鐘**，畫面會一直在動，別關掉它。
3. 裝完自動啟動，彈出瀏覽器。

`.venv` 完全關在專案資料夾裡：不會動到你原本的 Python、不需要系統管理員權限。
**想整個重來就把 `.venv` 資料夾刪掉**，下次啟動會重新裝一次。

> 沒有網路就裝不起來（要從 pypi.org 下載）。請在有網路的地方先跑過一次，
> 之後到野外就不需要網路了。

## 它怎麼找 Python

依序試這些，**第一個「真的裝齊套件」的就用它**：

1. 環境變數 `UAV_YOLO_PY` 指定的
2. 專案裡的 `.venv\Scripts\python.exe`
3. PATH 上的 `python`、`py -3`
4. `%USERPROFILE%\miniconda3`、`anaconda3`
5. python.org 的常見安裝位置（Python 3.10～3.13）

為什麼要「試」而不是「找到就用」：一台電腦常常裝了好幾個 Python，而只有一個
裝了套件。開發機實測 `py -3` 是沒有 cv2 的 3.12、`python` 才是裝齊的那個——
照「找到就用」會選到跑不起來的那個，然後視窗閃一下就關掉。

**已經有慣用環境（例如裝了 onnxruntime-directml 做 GPU 加速的 conda）**：
只要它裝齊 `requirements.txt`，就不會有人去建 `.venv`，會直接用它。
但**萬一 `.venv` 曾經被建出來，它的順位在 PATH 上的 `python` 之前**——
那個環境沒有 GPU 加速套件，推論會慢很多（自檢頁會警告「沒跑在 DirectML 上」）。
要釘死用自己的環境就設環境變數，它的順位最高：

```bash
set UAV_YOLO_PY=C:\path\to\python.exe
```

（或直接把 `.venv` 資料夾刪掉。）

## 完全沒有 Python 怎麼辦

start.bat 會把步驟印出來，照著做：

1. 到 https://www.python.org/downloads/ 下載 Python 3.12（Windows）
2. 安裝時**務必勾選 "Add python.exe to PATH"**（沒勾等於找不到）
3. 再雙擊一次 `start.bat`

需要 **3.10 以上**，比這舊的會被略過（程式用了 3.10 才有的語法）。

## 出問題時

- **視窗閃一下就不見了**：現在不會了——出錯會停在畫面上等你按鍵，
  訊息同時寫進 `data\server.log`。
- **安裝失敗**：完整記錄在 `data\pip-install.log`。最常見是沒網路、
  學校/公司網路擋 pypi.org、或磁碟空間不足（要 3 GB 左右）。
- **「A ground station is ALREADY running」**：已經有一台在跑了，啟動器**不會**再開
  第二台（第二台綁不到埠會馬上死），會直接幫你開那一台。**要重新啟動就先關掉
  它的「UAV_yolo Server」視窗**再雙擊一次。
  > 這點很重要：舊版會傻傻再開一台、然後把瀏覽器指到**舊的**那台，畫面看起來
  > 一切正常，但你看的不是剛啟動的那份。起飛前跑檢查清單時看錯行程會出事。

- **資料夾裡沒有 `.venv` 是正常的**：只有「這台電腦上沒有任何 Python 裝齊套件」
  才會建。你如果本來就有 conda 且裝齊了，啟動器會直接用它，不會多建一個。
  想知道它用了哪個，看啟動器視窗第一行 `Using Python: ...`。
- **想自己手動裝**：`python -m pip install -r requirements.txt`

## 實機模式上場前

在設定頁把模式切成實機並「重啟引擎」，先確認：影像來源、`mavlink.port`
（數傳 COM 埠）、載體種類、指令後端。細節見 `integration/PATH_MATRIX.md`
的每條路徑準備清單。

## 給維護者

- `.bat` **一律 ASCII + CRLF**。cmd 用系統代碼頁讀檔，中文會炸成亂碼視窗；
  LF 行尾會讓多行區塊解析錯亂。`tests/test_bootstrap.py` 會擋住這兩件事。
- **不要用 `timeout /t` 當等待**：stdin 被重導時它會噴
  `ERROR: Input redirection is not supported` 並**跳過等待**（實測 0.1 秒就回來），
  於是瀏覽器在伺服器還沒起來時就開了。改用 `call :sleep N`（內部走 `ping`）。
- 改了 `requirements.txt`，**同時要更新 `tools/bootstrap.py` 的 `REQUIRED_MODULES`
  或 `DEV_ONLY`**，否則啟動器不會檢查它——測試會擋住這件事。
- 實機模式需要 `pyserial`：pymavlink 開 COM 埠時 import `serial`，但它自己
  沒宣告這個相依。開發機剛好被別的套件帶進來過，乾淨環境沒有，
  症狀是「COM 埠開啟失敗」看起來像接線問題。
