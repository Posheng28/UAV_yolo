"""UAV_yolo — 固定翼/旋翼通用的視覺目標追蹤與導引系統。

模組總覽：
    config      設定載入/合併/存檔（default.yaml + local.yaml）
    geometry    相機模型、座標系旋轉、射線測地（像素 → 經緯度）
    estimation  目標 Kalman 濾波（位置+速度、盲區外推）
    guidance    導引律（旋翼跟隨 / 固定翼 standoff 繞行）
    mavlink_io  PX4 遙測接收與指令發送、安全閘門
    vision      影像來源、YOLO 偵測與目標鎖定、相機校正
    engine      主狀態機（把上面全部串起來）＋模擬模式
    webapp      地面站 Web UI（儀表板/檢查清單/設定/校正）
"""

__version__ = "0.1.0"
