# 车道保持 v2.1（RangeAware LaneKeeper）

> 参考：`LaneKeeping_V2_1_RangeAware`（2026-11-11）。
> 状态：**主机测试通过；未上机验证**。传感器安装几何、PD、通道宽度和
> 安全制动参数均未实车标定；不能声称“实车车道保持通过”。
>
> 范围：局部横向纠偏。不替代定位/EKF/地图/路径规划，不控制四个电机，
> 不拥有巡航速度，也不做前向急停。

---

## 1. v2.1 的核心变化

旧版把“墙是否存在”“近壁”“无目标”和“前向安全”混在 LaneKeeper 内部。
v2.1 改为：

1. **原始 ToF 是完整物理测量**。1.5 m / 1.9 m 的回波仍然有效，不能因为
   超过 150 mm 就写成 `ABSENT` 或 `NO_TARGET`；
2. 150 mm 只是 **LaneKeeper 内部近壁分类阈值**。`FAR_RETURN` 表示“有有效
   远距离回波”，不是“墙不存在”；`UNKNOWN` 表示近窗内没有可用数据，
   不是“空间畅通”；
3. 100–150 mm 首次观测连续 3 帧后进入 `MID_RETURN`，可以正常参与双墙判定，
   修复“上电落在滞回带 -> 永远 Unknown -> 故障”的问题；
4. **LaneKeeper 只输出横向 delta**：不修改巡航速度、不输出 `speedScale`、
   不做前向停车；前向 ToF 限速由独立 `SafetySupervisor` 负责；
5. 组选择使用**稳定通道方向**，不是每帧修正后的瞬时速度；没有新方向时
   沿用最后一次平移方向，停车仍可居中；
6. 墙距控制改为 PD（默认 Ki=0），降低对噪声的 D 敏感度；丢墙后按
   `degradedHoldMs` 缓释旧纠偏，不因单帧丢数重置。

---

## 2. 数据流与职责

```
Radio / 未来规划
      │ core::MotionCommand(base, 车体)
      ▼
 LaneKeeper ──(base + 横向 delta, omega 透传)──► SafetySupervisor
      ▲                                              │ ToF 制动/航向门控
      │ core::RangeSnapshot                          ▼
 DistanceArray ────────────────────────────────► Chassis
                                                    ├ IMU 航向 PID
                                                    └ X-Drive IK → 四电机
```

- LaneKeeper 不访问硬件，可在主机端完整测试；
- 原始 `RangeSnapshot` 是共享的**只读**测量。LaneKeeper 的 `NearClass`
  只供自身和诊断使用，定位/地图不得订阅；
- `omega` 原样透传，航向 PID 仍在 Chassis；
- 安全层只缩小 `vx/vy`，不改变 `omega`；当前只做 ToF 轴向约束，
  不含地图扫掠/未知挡板置信边界。

### 2.1 与旧版接口的差异

| 旧版 | v2.1 |
| --- | --- |
| `LaneState` 9 个状态（含左右单墙/转弯/故障） | `LaneMode` 7 个模式：DISABLED/NO_PAIR/INSUFFICIENT_DATA/NO_CORRIDOR/TRACKING/DEGRADED/TOO_CLOSE |
| `WallStatus Present/Absent/Unknown` | `NearClass UNKNOWN/NEAR_RETURN/MID_RETURN/FAR_RETURN` |
| `speedScale`、前向安全、故障停车 | LaneKeeper 不再拥有速度；移到 SafetySupervisor |
| 单墙跟随、走廊门控、停车自动居中开关 | 双墙可信才纠偏；停车按方向记忆继续居中 |
| 积分门控、抗饱和 | PD，默认 Ki=0；保留 PID 结构但不再使用积分 |

---

## 3. 坐标、单位与传感器几何

- 车体：`+X = 车头`，`+Y = 车体左侧`，`+Z` 向上；逆时针为正；
- `MotionCommand.vx/vy` 车体坐标；内部距离统一 mm，位置/宽度用 m；
- 横向误差/修正采用“**向右为正**”，最终用右法向 `-t` 映射回车体；
- 4 个 ToF 在正方形车架四个顶点，出射轴沿 ±X/±Y；
- 顶点半径 = 0.255/√2 ≈ **0.1803 m**；轮子在四条边中点，不是测距安装点。

默认映射：

| 逻辑 | 物理侧 | Smart Port | rangeSource | 车体位置 `(x,y)` [m] | 出射轴 [rad] | 组 |
| --- | --- | ---: | ---: | --- | ---: | --- |
| S0 | 前 | 7 | 0 | `(+0.1803, 0)` | `0` | A |
| S1 | 左 | 9 | 2 | `(0, +0.1803)` | `+π/2` | B |
| S2 | 后 | 8 | 1 | `(-0.1803, 0)` | `π` | A |
| S3 | 右 | 10 | 3 | `(0, -0.1803)` | `-π/2` | B |

- Group A = S0+S2 = 前后；Group B = S1+S3 = 左右；
- 实际安装位置/轴向不同，只改 `LaneConfig::sensors[]`，算法不改；
- `layoutVerified=false` 表示这些值尚未实测。

---

## 4. 单路近壁通道（`RangeChannel`）

每路独立维护原始有效性、近壁分类、中值/低通滤波和新鲜度。

### 4.1 有效回波

只有以下样本是**有效物理回波**：

- `RangeStatus::kOk`（20–2000 mm）；
- `RangeStatus::kTooClose`（<20 mm，距离不可信但确实有物体）。

`kNoTarget`（9999）、`kTooFar`（>2000）、`kReadError`、`kNotInstalled`
都是“本帧无有效回波”：不进入分类/滤波，不确认“无墙”。

### 4.2 分类（连续确认）

| 原始有效距离 | 分类变化 |
| --- | --- |
| `<100 mm` 连续 3 帧 | `NEAR_RETURN` |
| `>150 mm` 连续 3 帧 | `FAR_RETURN`（有效远距回波） |
| 100–150 mm（含端点） | 保持原近/远状态；首次观测连续 3 帧进入 `MID_RETURN` |
| 无有效回波 | 保持旧分类；超过 `maxAgeMs=180` 释放为 `UNKNOWN` |

- `FAR_RETURN` 是真实回波，不是无目标；
- `UNKNOWN` 不是空间畅通；
- 100–150 mm 的 `MID_RETURN` 可以作为双墙候选，不能单独证明通道安全。

### 4.3 滤波

- 3 点中值（2 点时取平均，1 点时直通）+ 一阶低通 `alpha=0.45`；
- 单帧跳变 `>60 mm` 先不改变滤波稳态，连续 2 帧才接受；
- 跳变确认与近壁分类解耦：分类仍按原始有效回波更新；
- `fresh` = 最近有效回波年龄 ≤42 ms（约 2 个控制周期）；
- `valid` = 年龄 ≤180 ms；
- 重复/乱序时间戳不重复计数。

---

## 5. 通道选对（`SensorGroupSelector`）

设车体通道方向 `d=(dx,dy)`，运动左法向 `t=(-dy,dx)`：

1. `score = max(|d·前后轴|, |d·左右轴|)`，即 `max(|tx|,|ty|)`；
2. `score < minAxisAlignment=0.82`（约 35°）不选对；45° 斜向不硬选；
3. 前后运动用 Group B（S1/S3），横向运动用 Group A（S0/S2）；
4. 两轴中在 `t` 上投影为正的一侧是运动左侧，另一侧是右侧；
5. 换对后清 PD 微分历史，并让模式机丢弃上一对的 hold 窗口。

方向来源：

- 当前工程没有全局路径层，使用基础命令的非零平移方向；
- 停车时沿用**最后一次**非零平移方向，因此停车仍可自动居中；
- 从未移动过（没有方向）且平移速度接近 0 时，按车体物理墙对自动回退：
  - B 对：S1/S3，法向 `t=(0,1)`；
  - A 对：S0/S2，法向 `t=(1,0)`；
  - 只有一对可信就用该对；两对都可信时选“更近的一对”，平局优先 B；
  - 该回退只用于停车/无方向，不会在 45° 行驶中硬选组；
- 未来接入路径规划时，应把 `RouteSegment` 的稳定切向量转换到车体后传入，
  不要使用每帧修正后的实际速度。

---

## 6. 走廊判定

只有在同一通道对上同时满足以下条件才进入双墙控制：

1. 两侧都是有效回波（`valid`）；
2. 分类不是 `FAR_RETURN` 或 `UNKNOWN`（`NEAR`/`MID` 均可）；
3. 两侧滤波距离都 `<150 mm`；
4. 几何宽度在 `[0.37, 0.70] m`：

```
width = (left_mm + right_mm)/1000
      + p_left·t + p_right·(-t)
```

默认对称安装时最后两项各为 0.1803 m，45 cm 通道：
`0.045+0.045+0.3606 ≈ 0.4506 m`。

- 单侧远回波、单侧无数据、宽度不合理 -> 不纠偏；
- `NO_CORRIDOR` = 两侧都有数据但几何/分类不可信；
- `INSUFFICIENT_DATA` = 至少一侧没有可用数据；
- 没有单墙跟随/无墙回退；没有“走廊外单侧过近自动远离”。

---

## 7. 模式机

```cpp
DISABLED, NO_PAIR, INSUFFICIENT_DATA, NO_CORRIDOR,
TRACKING, DEGRADED, TOO_CLOSE
```

| 模式 | 条件 | 输出 |
| --- | --- | --- |
| DISABLED | 模块关闭 / 配置非法 / 输入非法 | 透传或零 |
| NO_PAIR | 没有稳定通道方向 | 横向 0，不猜墙 |
| INSUFFICIENT_DATA | 有方向，但至少一侧无有效数据 | 旧纠偏按变化率回 0 |
| NO_CORRIDOR | 两侧有数据但宽度/分类不满足 | 旧纠偏缓释到 0 |
| TRACKING | 两侧可信且都新鲜 | PD 居中 |
| DEGRADED | 可信但一侧不新鲜，或丢墙 hold 窗口内 | 新鲜度不足仍算 PD；丢墙则缓释旧值 |
| TOO_CLOSE | 可信双墙且任一侧原始距离 <20 mm | 过近排斥，PD 旁路 |

- 丢墙后 `DEGRADED` 最多保持 `degradedHoldMs=150 ms`：
  `wanted = lastCorrection × (1 - elapsed/hold)`；
- 换对/重新获得可信双墙时清 PD 微分历史，避免 D 突跳；
- 模式切换只用于诊断和复位，不改变巡航速度。

---

## 8. 横向控制

### 8.1 误差

```
center_error_mm = 0.5 * ((right_mm + q_right) - (left_mm + q_left))
```

- 正 = 车体偏左 / 右墙更远 -> 向右修正；
- `q_left = p_left·t`，`q_right = p_right·(-t)`，用于兼容左右安装半径不一致；
- 默认对称安装时等价于 `(right_mm - left_mm)/2`；
- 误差使用**滤波后**距离；过近单侧排斥使用**原始**距离。

### 8.2 PD

```
correction = clamp(Kp*error + Kd*filtered(d error/dt),
                   -0.14, +0.14)  // m/s
```

默认：

| 参数 | 值 |
| --- | ---: |
| `Kp` | 0.0010 |
| `Ki` | 0 |
| `Kd` | 0.000025 |
| D 低通 alpha | 0.12 |
| 输出限幅 | 0.14 m/s |
| 横向变化率限幅 | 0.70 m/s² |

- 首帧 D=0；换对/重新获得可信双墙/离开 TOO_CLOSE 时复位；
- PD 输出先限幅，再按 `maxLateralAccelMps2 * dt` 做变化率限幅；
- 目标为 0 时也按变化率回到 0，不瞬间归零（`NO_PAIR` 除外：
  没有法向可用，横向 delta 直接为 0）。

### 8.3 停车静摩擦补偿

实车停车时，几毫米误差对应的 PD 输出只有几 mm/s，往往不足以克服静摩擦。
LaneKeeper 增加一个**仅停车生效**的最小横向速度：

```
if (基础平移速度 < minDirectionSpeedMps &&
    模式为 TRACKING/TOO_CLOSE &&
    |center_error| > standstillDeadbandMm &&
    |wanted| < standstillMinCorrectionMps)
    wanted = sign(error) * standstillMinCorrectionMps
```

- 默认 `standstillMinCorrectionMps=0.08 m/s`、`standstillDeadbandMm=3 mm`；
- 有平移速度时仍用正常 PD，不叠加该最小速度；
- 误差在死区内不施加，避免目标附近来回抖动；
- 如果实车仍不动，逐步增大该值；如果目标附近抖动，增大死区或降低该值。
- 主机闭环仿真（§12.4）：0.08 m/s 的停车距离约 13 mm，3 mm 死区会形成
  ±9 mm 的极限环；仿真中死区 6–10 mm 才消除。实车必须先按 §11 验证静摩擦，
  再定该值。

### 8.3 过近排斥

原始有效距离 `<20 mm`：

- 仅左侧过近：`wanted = +repulseMps`（向右远离）；
- 仅右侧过近：`wanted = -repulseMps`（向左远离）；
- 双侧都过近：`wanted = clamp(Kp*error, ±repulseMps)`，按差居中；
- 默认 `repulseMps=0.10 m/s`；
- PD 旁路，但仍受 `maxCorrectionMps` 和 `maxLateralAccelMps2` 限幅；
- LaneKeeper 不减速、不停车；是否允许该动作由 SafetySupervisor/上层决定。

### 8.4 输出

```
delta = -correction * t          // t 是运动左法向
command.vx = base.vx + delta.x
command.vy = base.vy + delta.y
command.omega = base.omega
```

- 前进时 `t=(0,1)`，正修正 -> `vy` 减小（车体向右）；
- 横移时 `t=(±1,0)`，修正落在 `vx`；
- 倒车时左右传感器对调，但右法向映射保持一致；
- **不乘速度比例，不修改基础速度大小**。

---

## 9. 前向安全（独立模块）

`SafetySupervisor` 在 LaneKeeper 之后、Chassis 之前：

```
$D / ToF ──► 轴对齐障碍距离
              │
command ─────►├─ d_brake = v*T + v²/(2a) 约束
              ▼
         限速后的 command
```

- 只对与运动方向轴对齐（`>=0.85`）且置信度 `>=8` 的 ToF 生效；
- 使用 `reactionS=0.12`、`brakeMps2=1.2`、`tofReserveM=0.022`；
- 只缩小 `vx/vy`，`omega` 保持；
- `requireHeading=true` 时航向无效/不可信直接禁止平移；默认关闭，
  避免 IMU 未校准时整车不能动；
- 配置非法时 **fail-closed**；
- 当前没有地图扫掠/未知挡板置信边界，不能单独作为实机零碰撞保证。

---

## 10. 在整车中的接入

`src/main.cpp` 每个 20 ms 周期：

```cpp
base = radio.valid ? radio.getDriveCommand() : zero;
const auto ranges = g_distance.toRangeSnapshot();
g_lane.update(base, ranges, g_imu.toHeadingState(), dt);
g_safety.update(g_lane.getState().command, ranges, g_imu.toHeadingState());
g_chassis.setCommand(g_safety.getState().command);
g_chassis.update(dt);
```

- `enableLaneAssist` 当前为 `true`（本次测试临时打开）；
- `enableSafety` 当前为 `true`；
- `disabled()` 同时 `lane.reset()` 和 `safety.reset()`；
- 不新增线程、不直接写电机、不绕过 IK。

---

## 11. 遥测

`$D` 末尾 30 个车道字段 + 3 个安全字段（详见 `docs/debug_log/07_调参工具.md` §7.5）：

- 模式/原因/组/近壁掩码/有效掩码/新鲜掩码；
- 控制侧滤波距离、中心误差、P/D、修正；
- 四路滤波距离、左右 `NearClass`、通道宽度、质量、对分；
- 物理左右原始距离、过近掩码；
- 输出 `vx/vy/omega`；
- 末尾 3 个安全层字段：限速比例、运动方向最近障碍、安全 flags。

关键点：

- `FAR_RETURN` 显示为“远”，不是“无墙”；
- `leftMm/rightMm=-1` 才表示当前控制侧不可用；
- `corridorWidthMm=-1` 表示几何不可用；
- LaneKeeper 不再有 `speedScale/forwardScale` 字段。

实时 UI：

```bash
python3 test/lane_plot.py /dev/ttyACM1 15 --csv lane.csv
python3 test/lane_plot.py /dev/ttyACM1 --headless 10 --csv lane.csv
```

---

## 12. 测试与验证状态

### 12.1 主机已执行

`bash scripts/test_native.sh`：

| 套件 | 内容 |
| --- | --- |
| `test_range_channel` | 99/100/150/151、MID 初始化、1.65 m 远距有效、丢帧保持/超时、跳变、重复时间戳、滤波 |
| `test_sensor_group_selector` | 前进/后退/横移/斜向/换组/方向无效 |
| `test_lane_state_machine` | TRACKING/DEGRADED/TOO_CLOSE/NO_CORRIDOR/INSUFFICIENT/NO_PAIR、hold 衰减 |
| `test_lane_config` | v2.1 默认值、几何/阈值/PD/宽度校验 |
| `test_lane_keeper` | 横向符号、横移/倒车选对、远距回波、宽度、方向记忆、无方向回退、停车最小速度、过近、丢墙缓释、无速度倍率 |
| `test_safety_supervisor` | 前向/侧向制动、45° 盲区、低置信度、航向门控、fail-closed |
| `test_maze_fixture` | 真实 maze_model.json 射线、45 cm 走廊双墙跟踪 |
| `test_lane_pid` | P/D、限幅、D 低通、复位 |

### 12.2 未验证

- 传感器实际安装位置/轴向/遮挡；
- `Kp/Kd` 未整定；
- 通道宽度范围 `0.37–0.70 m` 未按实车/赛场重标；仿真实际近壁门限约
  `0.40–0.66 m`（§12.4）；
- 过近排斥在实车静摩擦下是否能动；
- 安全层制动距离/传感器延迟未实测；
- 未上机验证，不能声称实车车道保持通过。

### 12.3 实车“不居中”排查顺序

1. 用 `test/lane_plot.py` 或 `distance_monitor.py` 看 `$D`：
   - `sideLeftRawMm/sideRightRawMm` 是否约 45/55 mm；
   - `leftClass/rightClass` 是否为 `NEAR/MID`；
   - `corridorWidthMm` 是否在约 400–660 mm（配置写 370–700，见 §12.4）；
   - `lane_mode`、`fallbackPair`、`corr`、`safetyScale`。
2. `lane_mode=NO_PAIR`：两侧还没有可信墙对。检查宽度/分类；停车时
   `fallbackPair` 应变为 1。若始终为 0，检查 `layoutVerified` 前的安装几何、
   传感器端口/轴向和 `corridorWidthMm`。
3. `lane_mode=TRACKING` 且 `corr` 非 0，但车不动：
   - 先看 `safetyScale` 是否为 1；为 0 说明安全层认为运动方向有障碍；
   - 再看 `standstillMinCorrectionMps` 是否足以克服静摩擦，逐步增大；
   - 检查 Chassis 的 slew/死区/轮子是否被卡。
4. `lane_mode=TOO_CLOSE` 但车不动：另一侧距离可能已小于安全层
   `tofReserveM`，SafetySupervisor 会直接把该方向速度压到 0。这是安全层
   的保守行为，不应通过绕过安全层解决。
5. `corr` 方向反了：优先检查 S1/S3 的物理左右映射和 `axisRad`，不要先改
   PD 符号；方向符号错误会让车越修越偏。
6. 目标附近来回抖：增大 `standstillDeadbandMm`，或降低
   `standstillMinCorrectionMps`；不要用增大 Kp 来掩盖静摩擦问题。
   主机闭环仿真显示默认 3 mm 死区会稳定极限环，6–10 mm 才消失（§12.4）。

### 12.4 主机闭环仿真（v2.1，未上机）

`sim/lane_keeping_sim.cpp` 直接编译本文件的 `LaneKeeper`，用
`maze_model.json` 的 45 cm 走廊、`chassis_config.hpp` 的底盘参数和 50 Hz
ToF 做闭环；运行 `bash sim/run_lane_keeping.sh`，完整结果见 `sim/README.md`。

默认参数、σ=1 mm 下的结论（均为仿真，需按 §11 实车验证）：

- 前进（B 组）巡航 20 mm 初偏能回到 0.6 mm，RMS 2.1 mm；
- **停车 0.08/3 mm 会形成 ±9 mm、17 mm 峰峰值的极限环**（3 s 内 9 次换向；
  噪声降到 0.3 mm 仍有 10 次）。0.08 m/s 的停车距离约 13 mm，大于 3 mm
  死区；仿真中死区 6 mm 才停止振荡，10 mm 最终 ≤3 mm；
- **航向误差**：P 控制稳态偏移 ≈ `v·tanθ/Kp`；前进 4°/0.4 m/s 就压到
  TOO_CLOSE，6°/8° 车架角点理论侵入墙面；Kp 0.002–0.003 可降到约 14 mm，
  但 8° 在 45 cm 走廊仍无解；
- **平移转弯（A 组，主测试）**：车头朝北、沿东西走廊平移时，20 mm 初偏
  在 1.67 s 短程内回到 3.6 mm，0.2 m/s 跑 3 s 后到 0.9 mm；停车同样有 ±9 mm 抖动
  （与 A/B 无关），死区 10 mm 后消失；4°/0.3 m/s 为 17.2 mm（角点 9.3 mm），
  0.4 m/s 为 22 mm（角点 4.6 mm），稳态公式仍为 `v·tanθ/Kp`；
- **横穿走廊的横向平移后停车不会重新居中**：最后方向与所在走廊轴垂直时
  （例如在南北走廊里向右横穿），A 组前后墙不可信，且 `pair.valid` 仍为
  true，`fallbackPair` 不会执行。沿走廊平移（平移转弯）A 组可信，不受影响；
- 左右/前后偏置 ±10 mm 直接变成约 8–12 mm 停车偏移（共模偏置互相抵消）；
- 实际能进入 TRACKING 的走廊约 **0.40–0.66 m**，与配置/文档的
  0.37–0.70 m 不一致；0.66 m 的 150 mm 端点被 `mm < detectMm` 排除；
- `FAR -> MID` 不能恢复：0.70 m 收窄到 0.60 m 仍为 NO_CORRIDOR，只有
  重新 `<100 mm`（NEAR）才能重置；
- 20% ToF 丢帧、驶出走廊端都能安全保持/退出，不是问题。

---

## 13. 待标定清单

| 参数 | 位置 | 方法 | 当前值/状态 |
| --- | --- | --- | --- |
| `sensors[0..3].xM/yM` | `lane_config.hpp` | 量测距出射点到车体中心 | 0.1803 顶点占位，未验证 |
| `sensors[0..3].axisRad` | `lane_config.hpp` | 量实际朝向 | 0/±π/2/π 占位，未验证 |
| `sensors[0..3].rangeSource` | `lane_config.hpp` | 确认逻辑通道↔快照索引 | 0/2/1/3，未验证 |
| 左右测距固定偏置（差值） | 无配置项 | 同一墙面互换/标定板比对 | 未标定；仿真 ±10 mm → ~10 mm 居中偏移 |
| `layoutVerified` | `lane_config.hpp` | 完成上述后置 true | false |
| `channel.enterMm/exitMm/detectMm` | `lane_config.hpp` | 近壁分类 | 100/150/150 |
| `channel.maxAgeMs/freshWindowMs` | `lane_config.hpp` | 实测刷新率/丢帧 | 180/42 ms |
| `channel.lowPassAlpha/jumpMm/jumpConfirm` | `lane_config.hpp` | 噪声/跳变 | 0.45/60 mm/2 |
| `state.min/maxCorridorWidthM` | `lane_config.hpp` | 实际通道宽度 | 0.37–0.70 m 占位（仿真实际 0.40–0.66，见 §12.4） |
| `pd.kp/kd` | `lane_config.hpp` | 低速直线整定，先 Ki=0 | 0.0010/0.000025 占位 |
| `state.maxCorrectionMps` | `lane_config.hpp` | 横向限幅 | 0.14 m/s 占位 |
| `state.maxLateralAccelMps2` | `lane_config.hpp` | 横向变化率 | 0.70 m/s² 占位 |
| `state.standstillMinCorrectionMps` | `lane_config.hpp` | 停车静摩擦补偿 | 0.08 m/s 占位 |
| `state.standstillDeadbandMm` | `lane_config.hpp` | 停车居中死区 | 3 mm 占位（仿真会抖，见 §12.4） |
| `state.repulseMps/tooCloseMm` | `lane_config.hpp` | 过近排斥 | 0.10 m/s / 20 mm |
| `state.degradedHoldMs` | `lane_config.hpp` | 丢墙缓释 | 150 ms |
| `state.minAxisAlignment` | `lane_config.hpp` | 斜向选对门限 | 0.82 |
| `SafetyConfig` | `safety_config.hpp` | 实测制动距离/延迟 | 0.12 s/1.2 m/s²/22 mm 占位 |
| `enableLaneAssist/enableSafety` | `robot_config.hpp` | 标定完成后决定 | true/true（测试临时） |

> 标定后同步更新 `docs/车辆配置.md`、`docs/TEST_PLAN.md` 和本表，
> 并把“未验证”改成实际测试条件与结果。
