"""離線驗證「像素 → 地面座標 → 導引指令」整條鏈路算得對不對。

不需要飛機、不需要遙測。分三段，每段都有**獨立於本專案程式碼**的對照答案：

  A. 合成往返：把已知地面點投影成像素，再測地回來，比對是否回到原位。
     絕對真值，能抓出旋轉矩陣、座標系慣例、內參縮放的錯誤。
  B. 真實偵測：拿真實照片跑真實模型，取框底邊中點做測地，
     再用 OpenCV undistortPoints + 三角函數獨立手算對拍。
  C. 導引指令：把測地結果餵進 KF 與導引律，檢查最後送出的
     經緯度/高度是否等於「目標位置 + 設定的退距」。

用法：
    python tools/verify_geolocation.py                    # 全部
    python tools/verify_geolocation.py --alt 20           # 指定飛行高度
    python tools/verify_geolocation.py --images 8         # 用幾張真實照片
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from uav_yolo.config import Config
from uav_yolo.estimation import TargetEstimator
from uav_yolo.geometry import GeoRef, geolocate_pixel
from uav_yolo.geometry.camera_model import CameraModel
from uav_yolo.geometry.frames import camera_rotation_gimbal_earth, euler_zyx_to_R
from uav_yolo.guidance import build_guidance
from uav_yolo.safety import SafetyGates

OK, BAD = "✅", "❌"


def load_camera(cfg: Config) -> CameraModel:
    return CameraModel.load(
        ROOT / (cfg.get("camera.intrinsics_file") or ""),
        float(cfg.get("camera.fallback_hfov_deg", 120.0)),
        int(cfg.get("video.width", 1920)),
        int(cfg.get("video.height", 1080)),
    )


# ------------------------------------------------------------------ A

def section_a(cam: CameraModel, alt: float) -> bool:
    """合成往返：已知地面點 → 像素 → 測地 → 應回到原點。"""
    print(f"\n{'='*66}\nA. 合成往返（絕對真值）  飛行高度 {alt:g}m\n{'='*66}")
    vehicle = np.array([0.0, 0.0, -alt])          # NED，z 向下故為負

    cases = [
        ("正下方",            0.0,   0.0,  -90.0, 0.0),
        ("北方 3m",           3.0,   0.0,  -90.0, 0.0),
        ("東方 2m",           0.0,   2.0,  -90.0, 0.0),
        ("東北各 2m",         2.0,   2.0,  -90.0, 0.0),
        ("機頭朝東(yaw90)",   2.0,   2.0,  -90.0, 90.0),
        ("雲台前傾 -60 度",   0.0,   0.0,  -60.0, 0.0),
        ("前傾+偏航",         4.0,  -1.5,  -60.0, 45.0),
    ]
    worst = 0.0
    for name, n, e, pitch_deg, yaw_deg in cases:
        R = camera_rotation_gimbal_earth(math.radians(yaw_deg), math.radians(pitch_deg))
        target = np.array([n, e, 0.0])
        # 投影：世界向量 → 相機系 → 像素
        pt_cam = R.T @ (target - vehicle)
        uv = cam.project(pt_cam)
        if uv is None:
            print(f"  {BAD} {name:16} 目標不在視野內（該姿態下投影失敗）")
            continue
        back = geolocate_pixel(uv[0], uv[1], cam, R, vehicle)
        if back is None:
            print(f"  {BAD} {name:16} 測地失敗")
            return False
        err = float(np.linalg.norm(back[:2] - target[:2]))
        worst = max(worst, err)
        mark = OK if err < 0.01 else BAD
        print(f"  {mark} {name:16} 真值 N{n:+6.2f} E{e:+6.2f}  →  像素({uv[0]:7.1f},{uv[1]:7.1f})"
              f"  →  測回 N{back[0]:+6.2f} E{back[1]:+6.2f}   誤差 {err*100:5.2f} cm")
    print(f"\n  最大往返誤差 {worst*100:.2f} cm  {'（純浮點誤差，正確）' if worst < 0.01 else '← 有問題'}")
    return worst < 0.01


# ------------------------------------------------------------------ B

def section_b(cam: CameraModel, cfg: Config, alt: float, n_images: int):
    """真實照片的偵測框 → 測地，與 OpenCV 獨立手算對拍。"""
    import cv2
    from uav_yolo.vision.detector import Detector

    print(f"\n{'='*66}\nB. 真實照片偵測 → 測地（獨立手算對拍）  假設高度 {alt:g}m、雲台垂直朝下\n{'='*66}")
    imgs = sorted((ROOT / "data" / "toycar" / "images").glob("*.jpg"))[:n_images]
    if not imgs:
        print("  找不到照片，略過")
        return True

    det = Detector(str(ROOT / cfg.get("detector.weights")), float(cfg.get("detector.conf", 0.45)),
                   int(cfg.get("detector.imgsz", 960)), cfg.get("detector.class_names") or ["Car"])
    vehicle = np.array([0.0, 0.0, -alt])
    R = camera_rotation_gimbal_earth(0.0, math.radians(-90.0))   # 機頭朝北、相機朝下

    print(f"  {'照片':<20}{'框底中點像素':>18}{'系統測地(N,E)':>20}{'獨立手算(N,E)':>20}{'差':>8}")
    worst = 0.0
    shown = 0
    for p in imgs:
        frame = cv2.imread(str(p))
        c = cam.scaled_to(frame.shape[1], frame.shape[0])
        dets = det.detect(frame, 0.0)
        if not dets:
            continue
        d = max(dets, key=lambda x: x.area)
        u, v = d.ground_pixel

        got = geolocate_pixel(u, v, c, R, vehicle)

        # --- 獨立對照：OpenCV 去畸變 → 正規化座標 → 垂直朝下時的簡單三角 ---
        und = cv2.undistortPoints(np.array([[[u, v]]], np.float64), c.K, c.dist)[0][0]
        # 相機朝下、yaw=0：影像右=東，影像下=南 → N = -y*alt, E = +x*alt
        ref = np.array([-und[1] * alt, und[0] * alt])

        err = float(np.linalg.norm(got[:2] - ref)) if got is not None else float("nan")
        worst = max(worst, err)
        if shown < 12:
            print(f"  {p.name:<20}({u:7.1f},{v:7.1f}) "
                  f"  N{got[0]:+6.2f} E{got[1]:+6.2f}"
                  f"   N{ref[0]:+6.2f} E{ref[1]:+6.2f}"
                  f"  {err*100:6.2f}cm")
            shown += 1
    print(f"\n  與獨立手算的最大差 {worst*100:.2f} cm  {'（一致）' if worst < 0.05 else '← 不一致，要查'}")
    return worst < 0.05


# ------------------------------------------------------------------ C

def section_c(cfg: Config, alt: float, vel_ne=(0.0, 0.0), label="靜止目標") -> bool:
    """測地結果 → KF → 導引 → 實際會送出的經緯度/高度。"""
    print(f"\n{'='*66}\nC. 導引指令｜{label}（目標起點：載具東北各 2m 的地面）\n{'='*66}")
    home = (24.569890, 120.841928)
    georef = GeoRef(*home)
    vehicle_ne = np.array([0.0, 0.0])       # 載具正好在 home 上方
    start = np.array([2.0, 2.0])
    v = np.array(vel_ne, dtype=float)

    ecfg = cfg.section("estimator")
    est = TargetEstimator(
        accel_std=ecfg.get("accel_std", 3.0),
        meas_std=ecfg.get("meas_std", 8.0),
        gate_sigma=ecfg.get("gate_sigma", 4.0),
        max_jump_m=cfg.get("safety.max_meas_jump_m", 30.0),
    )
    n = 25
    for i in range(n):                        # 餵一段觀測讓 KF 收斂到位置與速度
        t = i * 0.2
        est.predict_to(t)
        est.update(t, start + v * t)
    target_ne = start + v * ((n - 1) * 0.2)   # 最後一筆量測時的真實位置

    airframe = cfg.get("vehicle.airframe", "multirotor")
    guidance = build_guidance(airframe, cfg.section("guidance"))
    merged = {k: v for k, v in cfg.section("safety").items()
              if k not in ("multirotor", "fixedwing")}
    merged.update(cfg.section("safety").get(airframe) or {})
    gates = SafetyGates(merged, cfg.get("guidance.rate_hz", 1.0))

    cmd = guidance.compute(est)
    requested_alt = cmd.alt_rel_m
    cmd.alt_rel_m = gates.clamp_alt(cmd.alt_rel_m)
    lat, lon, _ = georef.ned_to_lla(np.array([cmd.point_ne[0], cmd.point_ne[1], 0.0]))

    standoff = float(cfg.get(f"guidance.{airframe}.standoff_m", 0.0)) if airframe == "multirotor" else 0.0
    offset = float(np.linalg.norm(cmd.point_ne - target_ne))

    # 獨立算一次「指令點應該在哪」：旋翼導引 = 目標往前推 1 秒，再往速度反方向退 standoff
    lead = target_ne + v * 1.0
    if standoff > 0 and float(np.linalg.norm(v)) >= 1.5:
        b = math.atan2(-v[1], -v[0])
        expect = lead + standoff * np.array([math.cos(b), math.sin(b)])
    elif standoff > 0:
        expect = lead + standoff * np.array([math.cos(math.pi), math.sin(math.pi)])  # 預設南側
    else:
        expect = lead
    point_err = float(np.linalg.norm(cmd.point_ne - expect))

    # 飛機實際會怎麼動（操作員最想看的就是這個）
    move = cmd.point_ne - vehicle_ne
    bearing = (math.degrees(math.atan2(move[1], move[0])) + 360) % 360
    compass = ["北", "東北", "東", "東南", "南", "西南", "西", "西北"][int((bearing + 22.5) % 360 // 45)]

    print(f"  載體種類            {airframe}")
    print(f"  目標真實位置        N{target_ne[0]:+.2f} E{target_ne[1]:+.2f}"
          f"   速度 N{v[0]:+.1f} E{v[1]:+.1f} m/s")
    print(f"  KF 估計位置         N{est.pos_ne[0]:+.2f} E{est.pos_ne[1]:+.2f}"
          f"   估計速度 N{est.vel_ne[0]:+.1f} E{est.vel_ne[1]:+.1f} m/s"
          f"   位置誤差 {np.linalg.norm(est.pos_ne - target_ne):.2f} m")
    print(f"  導引指令點          N{cmd.point_ne[0]:+.2f} E{cmd.point_ne[1]:+.2f}")
    print(f"  指令點離目標        {offset:.2f} m   (設定的水平退距 {standoff:.2f} m)")
    print(f"  ➜ 飛機會往          {compass}方（方位 {bearing:.0f}°）水平移動 "
          f"{np.linalg.norm(move):.2f} m")
    print(f"  指令高度            設定 {requested_alt:g}m → 實際送出 {cmd.alt_rel_m:g}m"
          f"{'  ← 被安全下限夾制！' if abs(cmd.alt_rel_m - requested_alt) > 1e-6 else ''}")
    print(f"  送出的經緯度        {lat:.7f}, {lon:.7f}")

    # 獨立驗算：把經緯度換算回離 home 的距離，與指令點比對
    dn = (lat - home[0]) * 111320.0
    de = (lon - home[1]) * 111320.0 * math.cos(math.radians(home[0]))
    err = math.hypot(dn - cmd.point_ne[0], de - cmd.point_ne[1])
    print(f"  經緯度反算對拍      N{dn:+.2f} E{de:+.2f}   差 {err*100:.1f} cm")
    print(f"  獨立算的應有指令點  N{expect[0]:+.2f} E{expect[1]:+.2f}   差 {point_err:.2f} m"
          f"   {'（含 1 秒前置量）' if float(np.linalg.norm(v)) > 0 else ''}")

    ok = point_err < 0.35 and err < 0.5
    print(f"\n  {OK if ok else BAD} 指令點與經緯度換算{'正確' if ok else '不符，要查'}")
    return ok


def main() -> int:
    ap = argparse.ArgumentParser(description="離線驗證測地與導引指令")
    ap.add_argument("--alt", type=float, default=None, help="假設飛行高度 m（預設用設定的跟隨高度）")
    ap.add_argument("--images", type=int, default=8, help="B 段用幾張真實照片")
    ap.add_argument("--skip-detect", action="store_true", help="跳過 B 段（不載模型，很快）")
    args = ap.parse_args()

    cfg = Config()
    alt = args.alt if args.alt is not None else float(cfg.get("guidance.multirotor.follow_alt_m", 40))
    cam = load_camera(cfg)
    print(f"相機：{cam.width}x{cam.height}  HFOV {cam.hfov_deg:.1f}deg  "
          f"（{'已校正' if (ROOT / (cfg.get('camera.intrinsics_file') or '')).exists() else '未校正，用近似值'}）")

    results = [("A 合成往返", section_a(cam, alt))]
    if not args.skip_detect:
        results.append(("B 真實偵測", section_b(cam, cfg, alt, args.images)))
    results.append(("C1 導引・靜止目標", section_c(cfg, alt)))
    results.append(("C2 導引・目標往東北 3m/s",
                    section_c(cfg, alt, vel_ne=(2.1, 2.1), label="目標往東北移動")))

    print(f"\n{'='*66}")
    for name, ok in results:
        print(f"  {OK if ok else BAD} {name}")
    return 0 if all(ok for _, ok in results) else 1


if __name__ == "__main__":
    raise SystemExit(main())
