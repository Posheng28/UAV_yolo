# 啟動說明（start.bat / start-live.bat）

雙擊即可啟動地面站，會自動開伺服器並彈出瀏覽器。

| 檔案 | 模式 | 用途 | 網址 |
|---|---|---|---|
| **`start.bat`** | 模擬（`--sim`） | 無硬體演練／看 UI／示範 | http://localhost:8610 |
| **`start-live.bat`** | 實機（`--live`） | 接上採集卡＋數傳的真實任務 | http://localhost:8600 |

## 怎麼用

1. 雙擊 `start.bat`（或 `start-live.bat`）。
2. 會跳出一個**伺服器視窗**（顯示 log），約 5 秒後自動開瀏覽器。
3. **關掉那個伺服器視窗＝停止伺服器**（或在視窗內按 Ctrl+C）。

> 瀏覽器沒自動開、或想重看：手動開 `http://localhost:8610`（sim）或 `http://localhost:8600`（live）。
> 換了頁面內容沒更新就按 **F5** 重整。

## 內含設定

- Python 路徑寫死為 `C:\Users\user\miniconda3\python.exe`（本機實測有裝齊相依套件的那個）。
  搬到別台電腦或換了環境 → 用記事本打開 .bat、改 `set "PY=..."` 那一行即可；
  找不到會直接報錯提示你改。
- Port：sim 用 8610、live 用 8600（避免兩個同時開時撞埠）。要改就改 `set "PORT=..."`。

## 為什麼有兩個檔而不是一個選單

`.bat` 對「多行 if/else 選單」在某些編碼／換行下容易解析錯亂（跳亂碼視窗）。
拆成兩個無分支的檔最穩，也一眼看得出自己開的是模擬還是實機——實機會真的對飛控發指令，
不該和模擬混淆。

## 實機模式（start-live.bat）上場前

先確認 `config/local.yaml`（或設定頁）已設好：影像來源、`mavlink.port`（數傳 COM 埠）、
載體種類、指令後端。細節見 `integration/PATH_MATRIX.md` 的每條路徑準備清單。
