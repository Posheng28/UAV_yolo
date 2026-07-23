# Patch：讓 `global_goto_node` 接受四軸（多旋翼）

## 為什麼需要

`NYCU_UAV_offboard/src/global_goto_node.cpp` 目前**硬鎖固定翼**，收到多旋翼的
GOTO 會直接拒絕（`validate_common_safety_locked`）：

```cpp
if (status.vehicle_type != VehicleStatus::VEHICLE_TYPE_FIXED_WING) {
    error = "Vehicle is not currently in fixed-wing configuration";
    return false;
}
```

你們的**首次整合測試是四軸**，所以若要跑「UAV_yolo → LR24 → global_goto_node」
這條完整整合路徑，必須放寬這個 gate。

> 這是 NYCU 團隊 repo（不可直接動 master）。請開 `feature/multirotor-goto` 分支套用，
> **並先在四軸 SITL（`gz_x500`）通過驗收再上實機**——與你們既有紀律一致。
>
> **替代方案（不改 C++）**：四軸只想驗證 UAV_yolo 的追蹤/測地本身時，把 UAV_yolo
> 設 `link.command_backend: direct`，直接對 PX4 發 MAVLink DO_REPOSITION（多旋翼原生支援），
> 完全不碰 global_goto_node。整合路徑（LR24）則留給固定翼——那正是本節點的設計目標。

## 改動 1／2：放寬載具型別 gate（加參數，預設仍只允許固定翼）

宣告一個新參數（建構子參數區，與其他 `declare_parameter` 並列）：

```cpp
allow_multirotor_ = this->declare_parameter<bool>("allow_multirotor", false);
```

成員變數區加：

```cpp
bool allow_multirotor_{false};
```

把型別 gate 改成：

```cpp
const bool is_fixed_wing = status.vehicle_type == VehicleStatus::VEHICLE_TYPE_FIXED_WING;
const bool is_multirotor = status.vehicle_type == VehicleStatus::VEHICLE_TYPE_ROTARY_WING;
if (!is_fixed_wing && !(allow_multirotor_ && is_multirotor)) {
    error = "Vehicle type not permitted (fixed-wing, or multirotor with allow_multirotor:=true)";
    return false;
}
```

## 改動 2／2：確認 setpoint 型別同時接受 POSITION（多旋翼）

`target_matches_setpoint_locked` 目前只認 `SETPOINT_TYPE_LOITER`：

```cpp
if (!setpoint.valid || setpoint.type != PositionSetpoint::SETPOINT_TYPE_LOITER ||
    ...
```

多旋翼 DO_REPOSITION 後，PX4 的 `position_setpoint_triplet.current.type` **可能是
`SETPOINT_TYPE_POSITION` 而非 `LOITER`**（旋翼是定點停懸、不繞圈）。改成兩者皆收：

```cpp
const bool type_ok =
    setpoint.type == PositionSetpoint::SETPOINT_TYPE_LOITER ||
    setpoint.type == PositionSetpoint::SETPOINT_TYPE_POSITION;
if (!setpoint.valid || !type_ok ||
    !valid_latitude(setpoint.lat) || !valid_longitude(setpoint.lon) ||
    !std::isfinite(setpoint.alt))
{
    return false;
}
```

> **實際型別以 SITL 實測為準**：在四軸 SITL 送一次 GOTO，用
> `ros2 topic echo /fmu/out/position_setpoint_triplet` 看 `current.type` 到底是 2(POSITION)
> 還是 3(LOITER)，據此保留需要的分支。`confirmation_timeout` 逾時且
> `matching_target_setpoint=false` 通常就是這裡型別不符。

## 建議一併調整的參數（四軸場測，launch 覆寫即可，不必改碼）

| 參數 | 固定翼預設 | 四軸建議 | 原因 |
|---|---:|---:|---|
| `allow_multirotor` | `false` | `true` | 啟用本 patch |
| `min_relative_altitude_m` | `30.0` | `10.0` | 四軸可低飛；仍保留下限 |
| `arrival_horizontal_threshold_m` | `100.0` | `15.0` | 四軸能精確到點，不需大到達圈 |
| `max_target_distance_m` | `2000.0` | `200.0` | 首測拉近、保守 |

```bash
ros2 launch my_offboard_cpp serial_gps_goto.launch.py \
  lr24_port:=/tmp/nycu_lr24_air \
  allow_multirotor:=true \
  min_relative_altitude_m:=10.0 \
  arrival_horizontal_threshold_m:=15.0 \
  max_target_distance_m:=200.0
```

## 四軸 SITL 驗收（沿用 gps_goto_program.md §9 精神）

```bash
# PX4 四軸 SITL
cd PX4-Autopilot && git checkout v1.17.0
make px4_sitl gz_x500
MicroXRCEAgent udp4 -p 8888
```

- 未 arm / 未離地 → GOTO 必須回 `ERR`，不得自行 arm。
- arm 且離地後送 GOTO → 進 `AUTO_LOITER`、`ENROUTE`，飛到點定點停懸。
- 座標超界 / NaN / 高度超界 / 超過 `max_target_distance_m` → 一律 `ERR`。
- 連送多筆 GOTO（模擬移動目標）→ loiter 點跟著更新，無「已有目標」誤拒。
- RC/QGC 切模式 → 立即接管，node 監控轉 `ABORTED` 但不代切 PX4 模式。
