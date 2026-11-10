# 四距离传感器自适应车道保持（LaneKeeper）

> 状态：**主机测试通过；旧版双墙已在实车短时触发；TOO_CLOSE/新 UI 尚未上机验证**。
> 范围：局部运动纠偏，不替代全局路径规划、不做定位/EKF、不直接控制四个电机。
> 相关需求：`docs/chassis_requirements.md` §LANE；接线：`docs/车辆配置.md`；
> 数据流：`docs/architecture.md`。

---

## 1. 目标与边界

默认 `corridorOnly=true`：**只有物理左/右两侧原始距离都 <150 mm** 时才启用墙距控制；
单侧/无墙不纠偏。进入双墙走廊后，即使停车也保持双墙模式/PID，并以不超过
`standstillMaxCorrectionMps=0.08 m/s` 的速度自动居中/远离过近侧；长时间
运行不会因为速度=0 退回“识别”。门控关闭
（`corridorOnly=false`）时保留旧的单墙/无墙回退，仅用于主机测试/特殊场地。

在双墙走廊内，利用四路 V5 Distance Sensor 中与运动方向垂直的一组相对传感器：

- 两侧都能看到近墙 -> **双墙居中 PID**；
- 只有一侧能看到近墙、另一侧已确认无墙 -> **单墙跟随 PID**；
- 两侧都确认无墙 -> **NO_WALL**，不做横向纠偏；
- 运动方向与两组测距轴都近似 45° -> **不选组、进入 ACQUIRING**；
- 单侧/双侧原始距离 `<20 mm` -> **TOO_CLOSE**，主动向远离近墙方向修正，不卡住；
- 读取失败/未安装持续异常 -> **SENSOR_FAULT**，按配置减速/停止。

本模块只输出车体平移修正；旋转仍由 Chassis 的既有 IMU 航向 PID 产生：

```
Radio / 未来自主规划
        │ core::MotionCommand(base, 车体坐标)
        ▼
   LaneKeeper ──(vx, vy, omega 原样透传)──► Chassis
        ▲                                     ├─ IMU 航向 PID（既有 HeadingController）
        │ core::RangeSnapshot                 ├─ X-Drive IK（既有 kinematics）
   DistanceArray                              └─ ChassisDrive → 四电机
```

因此：

- **没有重写 X-Drive IK**；
- **没有重写 IMU 航向 PID**：`omega` 由 Chassis 的既有航向环产生，
  LaneKeeper 只改 `vx/vy`；
- LaneKeeper 不访问任何硬件，可在主机端完整测试。

> `vx` 来自上游基础命令（遥控/全局规划），LaneKeeper **不生成巡航速度**；
> 它只按状态/前向安全做限速，再叠加横向纠偏。

---

## 2. 坐标、单位与传感器几何

### 2.1 坐标约定（沿用现有工程）

- 车体：`+X = 车头`，`+Y = 车体左侧`，`+Z = 竖直向上`；
- 角度：逆时针为正（从 +X 转向 +Y）；
- `MotionCommand`：`vx [m/s]` 车头、`vy [m/s]` 车体左侧、`omega [rad/s]` 逆时针；
- 内部墙距/阈值/滤波统一 **mm**，安装位置/参考垂距用 **m**。

> 任务书 §2.1 建议 `+vy = 车体右方`；现有工程已确认 `+vy = 车体左侧`
> （`docs/车辆配置.md` §1，CS-007）。本实现**沿用现有约定**，没有改 IK 符号。
> LaneKeeper 内部墙距误差采用“**向右为正**”的约定（与任务书 §7/8 一致），
> 最后通过右法向 `r=(dY,-dX)` 转回车体 `vx/vy`，不修改底盘符号。

### 2.2 传感器在四个顶点，不在轮子 45° 边中点

根据用户提供的俯视图，4 个 ToF 在正方形车架的**四个顶点**：

- 车头、车尾、左侧、右侧；
- 出射轴沿车体 ±X / ±Y；
- 顶点半径 = 框架边长/√2 = 0.255/1.414 ≈ **0.1803 m**；
- 轮子在四条边的中点（45° 位置），与测距安装点不是同一位置。

默认映射（**安装点/支架外伸未实测**）：

| 逻辑 | 默认物理侧 | Smart Port | rangeSource | 车体位置 `(x,y)` [m] | 出射轴 [rad] | 组 |
| --- | --- | ---: | ---: | --- | ---: | --- |
| S0 | 前 | 7 | 0 (前) | `(+0.1803, 0)` | `0` | A |
| S1 | 左 | 9 | 2 (左) | `(0, +0.1803)` | `+π/2` | B |
| S2 | 后 | 8 | 1 (后) | `(-0.1803, 0)` | `π` | A |
| S3 | 右 | 10 | 3 (右) | `(0, -0.1803)` | `-π/2` | B |

- **Group A = S0+S2 = 前+后**；
- **Group B = S1+S3 = 左+右**；
- 这与示意图“蓝色 A 组为前后、绿色左右为当前左右组”一致。
- 实际朝向/位置如果不同，只改 `SensorConfig` 的 `rangeSource/xM/yM/axisRad`，
  算法不改；被轮子/结构件遮挡可置 `enabled=false`。

### 2.3 单墙目标

`LaneStateConfig::targetWallMm = 45.0` 是**原始传感器距离**目标，不是车体中心到墙：

- 45 cm 通道、顶点半径 0.1803 m 时，居中原始读数约
  `225 - 180.3 = 44.7 mm`；
- 任务书示例 `target_wall_mm = 45.0f` 与此一致；
- 必须按实际通道净宽、车体轮廓、ToF 窗口、机械安全间隙标定；
- **不得把 100 mm 的墙体识别阈值当作目标墙距**。

---

## 3. 墙体识别：10/15 cm 滞回状态机

已确认参数（不得改成 5 cm）：

```cpp
inline constexpr std::int32_t kWallEnterMm = 100;      // 严格 <100 才累计有墙
inline constexpr std::int32_t kWallExitMm = 150;       // 严格 >150 才累计无墙
inline constexpr std::int32_t kWallConfirmCount = 3;   // 连续有效确认样本
inline constexpr std::uint32_t kWallStaleTimeoutMs = 200;
inline constexpr std::uint32_t kWallMinSampleIntervalMs = 15;
```

每个 ToF 独立维护状态与连续计数（`lane::WallTracker`）：

| 样本类型 | 近墙计数 | 远墙计数 | 状态变化 |
| --- | --- | --- | --- |
| `kOk` 且 `d < 100` | +1 | 清 0 | 连续 3 次 -> Present |
| `kOk` 且 `100 <= d <= 150` | 清 0 | 清 0 | 保持原状态（滞回带，含端点） |
| `kOk` 且 `d > 150` | 清 0 | +1 | 连续 3 次 -> Absent |
| `kNoTarget` (9999) | 清 0 | 清 0 | `wall_present` 不置 false；超时转 Unknown；状态机另有 no-target 证据计时 |
| `kTooFar` (>2000) | 清 0 | 清 0 | 同上（不当作有效远墙距离） |
| `kTooClose` (<20) | 清 0 | 清 0 | 墙体状态不确认；保留原始距离，状态机连续确认后进入 TOO_CLOSE 主动远离 |
| `kReadError` (PROS_ERR) | 清 0 | 清 0 | **无效样本**；超时转 Unknown |
| `kNotInstalled` | 清 0 | 清 0 | **无效样本**；超时转 Unknown |

关键语义（完整任务书 §5.4）：

1. **9999 不是“远墙”**：它是无效样本，不确认 `wall_present=false`；
   持续无效超过 `staleTimeoutMs` 转 `Unknown`。
   状态机另有一个**独立的 no-target 计时**（`noWallTimeoutMs=120`）：
   若某侧持续只返回 9999/>2000，则把该侧视为“无可观测近墙”，
   可进入 `NO_WALL` 或单墙模式；**墙体状态仍是 Unknown**，不是 Absent。
   这解决了“墙结束/路口返回 9999 时不能一直当作有墙”的问题，同时没有
   把 9999 直接写成 `wall_present=false`。
2. **PROS_ERR / <20 / >2000 / 未安装** 同样不确认无墙。
   `<20 mm` 不是丢弃：过近侧原始值进入 TOO_CLOSE 排斥控制，主动远离；
   只有读取失败/未安装才按故障处理。
3. 100–150 mm（含端点）保持原状态，并打断进入/退出连续序列。
4. 同一时刻/过短间隔的重复样本不重复计数（`minSampleIntervalMs=15`），
   防止一个控制周期内重复读取快速凑满 3 次。
5. 静止墙连续返回相同数值是有效的；不能因“数值相同”判无效。

状态机是纯逻辑，`tests/native/test_wall_tracker.cpp` 覆盖 99/100/101、
149/150/151、9999、PROS_ERR、过近、超量程、重复时间戳和 stale。

---

## 4. 测距组选择

### 4.1 前后=A、左右=B，按运动方向映射左右

车体坐标里的“左/右传感器”是**车体左右**，不是**运动方向左右**：

- 前进时：运动左侧 = 车体左侧（S1），右侧 = 车体右侧（S3） -> **Group B**；
- 左移时：运动左侧 = 车体后方（S2），右侧 = 车体前方（S0） -> **Group A**；
- 右移时：运动左侧 = 车体前方（S0），右侧 = 车体后方（S2） -> **Group A**；
- 倒车时：运动左侧 = 车体右侧（S3），右侧 = 车体左侧（S1） -> **Group B 但左右对调**。

因此必须由“当前目标平移速度方向 + 已标定传感器轴”共同决定，而不是转弯次数。

### 4.2 选择算法（`SensorGroupSelector`）

设车体目标平移方向 `d=(vx,vy)/|v|`，其左法向 `t=(-d_y,d_x)`。对每组两根轴 `a_i`：

1. 两根轴必须近似反向：`a_0·a_1 <= -cos(15°)`；
2. 两根轴在 `t` 上的投影必须一正一负；
3. 组得分 `score = min(|a_0·t|, |a_1·t|)`；
4. 要求 `score >= minAxisAlignment=0.90`（约 25.8°）；
5. 取两组中得分更高者；带 100 ms 驻留和 0.10 分数回差。

与两组轴都近似 45° 的斜向运动（例如 `(1,1)/√2`）两组得分都约 0.707 < 0.90
-> 不选组，状态机进入 ACQUIRING（限速），不硬选一组。

### 4.3 车体坐标 vs 世界坐标

LaneKeeper 的输入**明确是车体坐标**。如果上游是全局路径规划：

- 世界方向 `w` 必须先用 `lane::worldVelocityToBody(w, headingRad)` 转到车体；
- 车体原地转 90° 后仍沿车头前进：`d` 在车体坐标中不变 -> **组不变**；
- 车体转 90°、但世界目标方向不变：`d` 在车体坐标中转了 90° -> **组切换**。

`tests/native/test_sensor_group_selector.cpp` 与 `test_lane_keeper.cpp` 都有断言。

---

## 5. 状态机

`lane::LaneState`：

```cpp
DISABLED, ACQUIRING, DUAL_WALL, LEFT_WALL, RIGHT_WALL,
NO_WALL, TURNING, SENSOR_FAULT, TOO_CLOSE
```

| 状态 | 进入条件 | 控制权限 / 速度 |
| --- | --- | --- |
| DISABLED | 模块关闭 / 配置非法 / 输入非有限 | 透传基础命令（配置非法/非有限时更保守） |
| ACQUIRING | 启动、组切换、转弯后、方向不适用、测量状态不足 | 无横向纠偏，基础速度 ×0.5（可配） |
| ACQUIRING（走廊门控未通过） | 两侧物理距离不是都 <150 mm | 仅监控，不纠偏，基础速度 ×1；reason=18 |
| DUAL_WALL | 左右均 Present、可用、通道宽度合理 | 双墙 PID，速度 ×1 |
| LEFT_WALL | 左 Present/可用，右 Absent 或“持续无目标”超时 | 左墙 PID，速度 ×1 |
| RIGHT_WALL | 右 Present/可用，左 Absent 或“持续无目标”超时 | 右墙 PID，速度 ×1 |
| NO_WALL | 左右均 Absent，或两侧都“持续无目标”超时 | 无横向纠偏，速度 ×1 |
| TURNING | 基础 omega 或实测 gz 超阈值 | 暂停墙距 PID，清零积分，速度 ×1（可配） |
| TOO_CLOSE | 单侧/双侧 `<20 mm` 连续确认 | 墙距 PID 旁路；单侧向远离该侧、双侧按差居中；速度 ×0.35（可配） |
| SENSOR_FAULT | 持续读取失败/未安装，或 Present 但测量长期不可用 | 无横向纠偏，速度 ×0（可配） |

- 状态机是**唯一**墙体模式来源；LaneKeeper 不另维护“当前模式”。
- 状态切换、组切换 -> `resetPid=true`，清积分/D 历史。
- 主动转弯结束且航向稳定 -> `ACQUIRING`，等待重新确认墙体后再恢复。
- 单侧/双侧 `<20 mm`：进入 **TOO_CLOSE**，不进入 ACQUIRING/FAULT：
  - 单侧过近：向远离该侧修正；双侧过近：远离更近的一侧；
  - 最小/最大排斥速度、速度比例、确认时间可配；
  - 过近侧恢复到 `>=20 mm` 后，按正常双墙/单墙/无墙重新判定，不额外停车。
- 单侧有墙但另一侧 **Unknown**：
  - 若该侧是持续 9999/超量程的“无目标”证据，超时后可进入单墙/NO_WALL；
  - 若是读取失败/未安装，或 Unknown 但没有 no-target 证据，则不把
    未知侧当成无墙，先 ACQUIRING，超过 `sensorFaultTimeoutMs` 进入
    SENSOR_FAULT；过近不算读取失败。
- 双墙都 Present/可用但通道宽度不合理 -> ACQUIRING（不危险纠偏）。
- `NO_WALL` 只表示“没有可观测近墙”，不生成定位/里程计；横向位置不确定性
  由上游路径规划/后续定位模块管理。没有可靠里程计时不能保证长期精确居中。
- IMU 无效时航向稳定判断只能退回基础 `omega` 指令；此时无法保证“实际旋转中
  暂停墙距 PID”，应先把 IMU 校准好。

---

## 6. 控制律、积分规则与限幅

### 6.1 双墙误差

默认四顶点几何下左右对称；为了兼容安装不对称，先用参考垂距：

```
L_left  = p_left·t  + d_left ·(a_left·t)
L_right = p_right·(-t) + d_right·(a_right·(-t))
error   = (L_right - L_left) / 2      // 向右为正
```

如果左右基准已对称校准，`L_right-L_left` 等价于原始 `dR-dL`，传感器半径项抵消。
通道宽度合理性：`L_left + L_right` 必须在 `[minCorridorWidthM, maxCorridorWidthM]`
内，否则视为可能来自墙角/不同墙段/物块，不进入双墙。

### 6.2 单墙误差

单墙目标使用**原始传感器距离** `targetWallMm=45`：

```
左墙：error = targetWallMm - leftDistanceMm     // 左太近 -> 正 -> 向右
右墙：error = rightDistanceMm - targetWallMm     // 右太近 -> 负 -> 向左
```

三种模式统一使用“向右为正”的误差和同一个 PID 实现，避免复制三套代码。

### 6.2b 过近排斥（TOO_CLOSE）

`d < 20 mm` 时墙距已超出有效量程，不能继续用普通 PID 的目标误差，也不能
直接停车。此时状态机进入 `TOO_CLOSE`，LaneKeeper 旁路墙距 PID，按最近墙
产生排斥速度：

```
单侧左过近：error = +(tooCloseMm - d_left)，corr 向行程右
单侧右过近：error = -(tooCloseMm - d_right)，corr 向行程左
双侧过近：  error = (d_right - d_left)/2，按两侧差居中
```

只有单侧过近才强制最小排斥量；双侧都 <20 mm 时不再给“最小远离”，
否则窄通道中心会反复左右抖。双侧等距时目标为 0，保持位置。

- 排斥量 = `clamp(gap * tooCloseGain, min, max)`，默认最小 0.06 m/s、最大 0.20 m/s；
- 方向仍按“向右为正”和运动右法向映射到 `vx/vy`；
- `tooCloseSpeedScale=0.35`，只减速，不把速度置 0；
- 过近侧恢复到 `>=20 mm` 后，状态机按正常双墙/单墙/无墙重新判定；
- 读取失败/未安装仍按 SENSOR_FAULT，不拿“过近”掩盖硬件故障。

### 6.3 最终命令

```
corr = PID(error)                    // m/s，向右为正
r    = (dY, -dX)                    // 运动右法向
command.vx = (base.vx + corr*r.x) * speedScale
command.vy = (base.vy + corr*r.y) * speedScale
command.omega = base.omega          // 航向 PID 在 Chassis
```

- 纠偏不一定落在 `vy`：横移时可能落在 `vx`；
- `speedScale` 综合状态机与前向安全，故障/前向障碍为 0 时整车停；
- 横向纠偏有总限幅 `maxLateralCorrectionMps` 和变化率限幅
  `maxLateralAccelMps2`；
- 基础速度比例有 `maxSpeedScaleRatePerSec` 变化率限幅（故障/障碍除外）。

### 6.4 积分门控（严格按任务书 §9）

只有以下条件全部满足才累计积分：

1. 状态是 DUAL_WALL / LEFT_WALL / RIGHT_WALL；
2. 对应测量可用（Present + 滤波有效）；
3. 不处于转弯/故障/无墙/ACQUIRING/TOO_CLOSE；
4. `Ki != 0`；
5. 参与控制的**原始**墙距严格 `<100 mm`。

- 双墙：左右原始距离都 <100；
- 左墙：左原始距离 <100；
- 右墙：右原始距离 <100；
- 100–150 mm 滞回带：**冻结**积分（保留当前 I，不累计）；
- 模式/组切换、进入无墙/转弯/故障/TOO_CLOSE：`reset()` **清零**积分和 D 历史；
- `Ki=0`：积分状态清 0，不允许偷偷累积；
- TOO_CLOSE 时墙距 PID 被旁路，P/I/D 为 0，横向输出是排斥量；
- 输出饱和时条件积分抗饱和，撤销本帧积分增量。

> 墙距 PID 与 IMU 航向 PID 的积分状态完全独立；侧墙模式变化不会清航向积分。

### 6.5 前向安全

四路 ToF 在前后左右顶点。对当前运动方向，选择轴对齐度最高的一路：

- 仅当 `a·d >= motionSafetyMinAxisAlignment=0.80` 时启用；
- 原始有效距离 <= `motionStopMm=120`：立即停（速度比例 0）；
- 120–300 mm：线性减速；>=300 mm：不限速；
- 45° 斜向、该方向传感器未安装/失效/被遮挡时：**明确是感知盲区**，
  本模块不限速，必须由全局规划/地图/速度策略补足。

---

## 7. 滤波与测量管理

- `RangeFilter`：3 点中值 + 一阶低通 + 跳变确认 + 无距离超时复位；
- 中值抑制单点孤立异常；连续 `jumpConfirmSamples=2` 次跳变才接受新稳态；
- 9999/无效值不进入中值/均值；
- 相同数值连续出现有效（静止墙）；
- 墙体存在性用**原始有效样本**判定；PID 用**滤波后距离**；
- 短时丢数可用 hold 窗口内最近有效值；超过 `measurementHoldMs` 不可用；
- 不允许用过期的墙距无限期继续驾驶。

`LaneKeeperState` 同时暴露 raw / filtered / filterValid / usable / wall status，
便于区分“测距失败、无墙、超时、正常离开”。

### 7.1 实时分析 UI

`test/lane_plot.py` 读取 `$D`，同时显示两侧物理原始距离、控制侧滤波距离、
墙距/过近误差、P/I/D、控制目标和实际横向修正，并写 CSV。TOO_CLOSE 时
界面会提示“正在主动远离；不是故障停车”。三个曲线窗口都支持缩放/拖动/游标。

```bash
python3 test/lane_plot.py /dev/ttyACM1 15 --csv lane.csv
python3 test/lane_plot.py /dev/ttyACM1 --headless 10 --csv lane.csv
```

---

## 8. 在整车中的接入

`include/config/robot_config.hpp`：

```cpp
bool enableLaneAssist = true;   // 本次测试临时打开；量产建议标定后按需打开
```

`src/main.cpp` 每个 20 ms 周期：

```cpp
base = radio.valid ? radio.getDriveCommand() : zero;
g_lane.update(base, g_distance.toRangeSnapshot(),
              g_imu.toHeadingState(), dt);
g_chassis.setCommand(g_lane.getState().command);
g_chassis.update(dt);
```

- LaneKeeper 的 `enable` 由 `syncVehicleConfig()` 从整车开关同步；
- Chassis 的 `enableHeadingHold` 不受影响：航向 PID 仍在 Chassis 内工作；
- 航向目标沿用 Chassis 既有“上电 5 s 锁存/主动旋转后重锁”策略；
- `disabled()` 调用 `lane.reset()`，清滤波/墙体/PID/组/状态机历史；
- 不新增线程、不直接写电机、不绕过 IK。

---

## 9. 迷宫数据的使用

`/home/jc/下载/maze_model.json`（SHA256
`5bed05050cc36a2813fd2360d4b2c390e2eb847d7f7b316e62c63cf0fe10b601`）
已复制到 `test/data/maze_model.json`，作为**唯一墙体真值**：

- `scripts/gen_maze_fixture.py` 读取 JSON，生成
  `tests/native/fixtures/maze_model.hpp`（16 条固定竖墙 + 11 条固定横墙 + 3 条活动隔板）；
- `scripts/test_native.sh` 用 `--check` 阻止 JSON 与 fixture 漂移；
- `tests/native/test_maze_fixture.cpp` 用真实墙体做射线：
  - S 点四方向基准 `[front,right,back,left]=[0.675,0.225,0.225,0.675] m`；
  - 45 cm 走廊 cell (7,3) 中左右原始距离约 45 mm；
  - LaneKeeper 在真实走廊中完成双墙居中；
  - M1/M2/M3 安装/拆除会改变对应射线。

> 实时控制**不加载全局地图**：本模块是局部纠偏；全局地图/EKF/路径规划仍是后续独立模块。

---

## 10. 测试与验证状态

### 10.1 已执行（主机端）

```bash
bash scripts/test_native.sh
```

| 套件 | 内容 |
| --- | --- |
| `test_distance_status` | PROS_ERR / 9999 / <20 / >2000 / 未安装分类 |
| `test_wall_tracker` | 99/100/101、149/150/151、连续 3 次、滞回、9999、重复采样、stale |
| `test_range_filter` | 中值、低通、跳变确认、9999 不混入、超时复位 |
| `test_sensor_group_selector` | 前后=A/左右=B、前进/后退/横移/斜向/零速/转弯 |
| `test_lane_state_machine` | 双/单/无墙、过近 TOO_CLOSE、组切换、转弯、故障恢复、积分权限、速度比例 |
| `test_lane_pid` | P/I/D、积分门控、Ki=0、限幅、抗饱和、D 低通、复位 |
| `test_lane_config` | 顶点布局、目标 45 mm、阈值/滤波/PID/状态机校验 |
| `test_lane_keeper` | 双/单墙符号、映射、短墙、过近主动远离、故障、积分、限速、前向安全 |
| `test_maze_fixture` | JSON 数据、S 点射线、真实 45 cm 走廊、活动隔板 |

### 10.2 未执行/未验证（必须标注）

- 旧版（26 字段、未含 TOO_CLOSE）已烧录并短时采集到双墙模式；
- **本版 TOO_CLOSE 连续远离、双墙走廊门控、30 字段遥测、`lane_plot.py` UI 尚未上机验证**；
- 传感器实际安装位置/轴向/遮挡未实测；
- PID 未整定；
- 实际墙厚、反光、斜入射、轮子遮挡对测距的影响未测；
- 双墙直行持续时间短，单墙/无墙/过近恢复的实车连续行为未验证；
- 因此**不能**声称“实车车道保持通过”。首次上机按 §11 步骤执行，并把结果补入
  `docs/TEST_PLAN.md`。

---

## 11. 待标定清单（TODO(CALIBRATE)）

| 参数 | 位置 | 方法 | 当前值/状态 |
| --- | --- | --- | --- |
| `sensors[0..3].xM/yM` | `lane_config.hpp` | 量测距出射点到车体中心 | 0.1803 顶点占位，未验证 |
| `sensors[0..3].axisRad` | `lane_config.hpp` | 量实际朝向 | 0 / ±π/2 / π 占位，未验证 |
| `sensors[0..3].rangeSource` | `lane_config.hpp` | 确认逻辑通道↔快照索引 | 0/2/1/3（前/左/后/右），未验证 |
| `layoutVerified` | `lane_config.hpp` | 完成上述后置 true | false |
| `targetWallMm` | `lane_config.hpp` | 45 cm 通道/期望偏置 | 45 mm 占位 |
| `wallPid` Kp/Kd | `lane_config.hpp` | 低速直线整定，先 Ki=0 | 0.0015/0.0003 占位 |
| `min/maxCorridorWidthM` | `lane_config.hpp` | 实际通道宽度/墙角容忍 | 0.30–0.60 m 占位 |
| `motionStopMm/motionSlowMm` | `lane_config.hpp` | 前向安全距离 | 120/300 mm 占位 |
| `noWallTimeoutMs` / `sensorFaultTimeoutMs` | `lane_config.hpp` | 无目标确认 / 故障降级时间 | 120/500 ms 占位 |
| `corridorOnly/corridorEnableMm` | `lane_config.hpp` | 仅双墙走廊启用 / 双侧距离门限 | true / 150 mm 占位 |
| `standstillCenter/standstillMaxCorrectionMps` | `lane_config.hpp` | 停车自动居中 / 最大自动横移速度 | true / 0.08 m/s 占位 |
| `tooCloseMm/tooCloseConfirmMs` | `lane_config.hpp` | 过近阈值 / 连续确认 | 20 mm / 20 ms 占位 |
| `tooCloseGain/Min/Max/Scale` | `lane_config.hpp` | 过近排斥强度、最小/最大速度、速度比例 | 0.012 / 0.06 / 0.20 / 0.35 占位 |
| `enableLaneAssist` | `robot_config.hpp` | 标定完成后打开 | false |

> 标定后请同步更新 `docs/车辆配置.md`、`docs/TEST_PLAN.md` 和本表，
> 并把“未验证”改成实际的测试条件与结果。
