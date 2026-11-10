# 机器人软件架构

> 适用工程：`Car/`（VEX V5 + PROS Kernel 4.2.2）。
> 语言标准由 PROS 模板 `common.mk` 决定（当前 `gnu++26`）；工程代码只用到 C++17 特性，
> 纯逻辑头文件可用 `-std=c++17` 在主机端单独编译。
> 本文件描述**架构升级后**的模块划分、公开接口与数据流。
> 底盘运动学推导仍以 `docs/chassis_design.md` 为准；需求基准见 `docs/chassis_requirements.md`。
> 当前接线、轮位与车头朝向真值见 `docs/车辆配置.md`。
> Radio 调研见 `docs/radio_research.md`。
> 开发规范（强制，修改代码前必读）：`docs/development_rules.md`；规则的可执行检查：`scripts/check_rules.sh`。

---

## 1. 项目总体架构

重构目标：**main 负责组织，模块负责能力**。机器人被拆为三个控制模块
（Radio / IMU / Chassis）、一个测距读取模块（DistanceArray）和一个局部车道保持
模块（LaneKeeper）；每个模块拥有自己的配置、状态、初始化与更新接口；
`main.cpp` 只做调度。
遥测（`diag`）与台架调试（`bringup`）放在应用层，只在 main 里各占一个调用点。

```
                    main / Application
                           │ 调度 / 初始化 / 生命周期
        ┌──────────────────┼──────────────────┐
        ▼                  ▼                  ▼
   Radio 模块          IMU 模块          Chassis 模块
   (遥控输入)          (航向状态)        (航向 PID + IK + 四电机)
        │                  │                  ▲
        │ body MotionCommand│ HeadingState     │
        ▼                  │                  │
   LaneKeeper ◄── core::RangeSnapshot ── DistanceArray（四路测距）
   (WallTracker×4 / Group select / Dual+Single PID)
        │
        │ body MotionCommand：vx/vy 修正，omega 透传
        └──────────────────────────────────────────► Chassis

   应用层（不在控制链上）：
     diag::Telemetry  ← 只读各模块 *State 快照 ──► $P / $D / $S 串口帧
     bringup          ← 台架调试（静摩擦标定、点动自检），不参与比赛逻辑
```

- LaneKeeper 与 Chassis 都不直接碰对方内部：LaneKeeper 输出 `core::MotionCommand`，
  Chassis 消费它并叠加 IMU 航向 PID。
- 测距硬件仍只由 `DistanceArray` 读取；LaneKeeper 只消费 `core::RangeSnapshot`。

- 这里“模块/节点”是**逻辑模块**，不是线程、进程或 ROS2 节点。
- 更新为**单调度循环**：`main` 按固定顺序调用各模块，周期 20 ms。
- 模块之间只通过公共命令 / 状态快照通信，不直接操作对方内部变量。
- `diag` / `bringup` 只读模块状态或调用模块公开接口，不反向被模块依赖。

---

## 2. 模块职责表

| 模块 | 目录 / 文件 | 职责 | 直接碰硬件 |
| --- | --- | --- | --- |
| **Chassis** | `include/modules/chassis.hpp`、`src/modules/chassis.cpp` | 底盘配置、IK、四电机驱动、航向保持、状态 | 否（经 `ChassisDrive`） |
| ├ Kinematics | `include/chassis/kinematics.hpp`、`src/chassis/kinematics.cpp` | 纯数学 `(vx,vy,ω)` → 四轮 RPM | 否 |
| ├ ChassisDrive | `include/chassis/chassis_drive.hpp`、`src/chassis/chassis_drive.cpp` | 方向符号、饱和、slew、刹车 | **是（电机）** |
| └ HeadingController | `include/chassis/heading_controller.hpp`、`src/chassis/heading_controller.cpp` | 目标航向锁存、PID、主动旋转旁路 | 否 |
| **LaneKeeper** | `include/modules/lane_keeper.hpp`、`src/modules/lane_keeper.cpp` | 滤波、墙体状态、组选择、状态机、双墙/单墙 PID、过近排斥、限速/前向安全、vx/vy 纠偏 | 否 |
| ├ WallTracker | `include/lane/wall_state.hpp`、`src/lane/wall_state.cpp` | 单路 10/15 cm 滞回、连续确认、无效/超时 | 否 |
| ├ RangeFilter | `include/lane/range_filter.hpp`、`src/lane/range_filter.cpp` | 3 点中值 + 低通 + 跳变确认 + 复位 | 否 |
| ├ SensorGroupSelector | `include/lane/sensor_group_selector.hpp`、`src/lane/sensor_group_selector.cpp` | 前后 A/左右 B、左右映射、回差/驻留 | 否 |
| ├ LaneStateMachine | `include/lane/lane_state_machine.hpp`、`src/lane/lane_state_machine.cpp` | ACQUIRING/TURNING/FAULT/TOO_CLOSE、积分权限、速度比例 | 否 |
| ├ LanePid | `include/lane/pid.hpp`、`src/lane/pid.cpp` | 完整 P/I/D、积分门控、限幅、抗饱和、D 低通 | 否 |
| └ LaneConfig | `include/lane/lane_config.hpp`、`src/lane/lane_config.cpp` | 顶点布局、阈值、滤波/PID/状态机初值、静态校验 | 否 |
| **IMU** | `include/modules/imu.hpp`、`src/modules/imu.cpp` | IMU 初始化 / 校准 / 读取 / 单位与坐标归一 | **是（IMU）** |
| **Radio** | `include/modules/radio.hpp`、`src/modules/radio.cpp` | 遥控摇杆/按键读取、失联保护 | **是（Controller）** |
| └ 摇杆映射 | `include/radio/input_mapping.hpp`、`src/radio/input_mapping.cpp` | 纯函数：归一化/死区/摇杆→`MotionCommand` | 否 |
| 公共类型 | `include/core/types.hpp`、`include/core/math_util.hpp` | `MotionCommand`、`HeadingState`、角度工具 | 否 |
| 配置入口 | `include/config/robot_config.hpp` + 各模块 `*_config.hpp` | 真值参数与整车开关 | 否 |
| **DistanceArray** | `include/modules/distance_array.hpp`、`src/modules/distance_array.cpp` | 四路 V5 Distance Sensor 读取/状态，投影 `core::RangeSnapshot` 给 LaneKeeper/diag | **是（Distance）** |
| **Diag** | `include/diag/telemetry.hpp`、`src/diag/telemetry.cpp` | 只读状态快照 → `$P`/`$D`/`$S` 串口帧；端口扫描 | **是（仅端口扫描）** |
| **Bringup** | `include/bringup/friction_test.hpp`、`src/bringup/friction_test.cpp` | 台架调试：静摩擦标定、点动自检 | 否（经 Chassis / Imu 公开接口） |

> 一个硬件只有一个管理者：电机 → `ChassisDrive`；IMU → `Imu`；遥控 → `Radio`；
> 四路测距 → `DistanceArray`；Smart Port 设备枚举 → `diag::Telemetry`（诊断用途）。
> 测距数据经 `core::RangeSnapshot` 交给 `LaneKeeper`，后者只输出车体
> `MotionCommand` 的平移修正，不直接访问测距/电机。实时工具见
> `test/distance_monitor.py`，设计说明见 `docs/lane_keeping.md`。
>
> **依赖方向**：模块只依赖 `core` 的公共契约，不互相包含头文件——`chassis.hpp`
> 不再 include `modules/imu.hpp`，它消费的是 `core::HeadingState`。

---

## 3. 模块公开接口

### 3.1 Chassis（`chassis` 命名空间）

```cpp
struct ChassisConfig { /* 端口、几何、传动、方向、限幅、航向 PID */ };
struct ChassisState {
    core::MotionCommand target;        // 最近目标
    core::MotionCommand applied;       // 经航向修正 + omega 平滑后的命令
    double wheelTargetRpm[4];          // IK 输出（饱和前）
    double wheelCommandRpm[4];         // 最终下发（含方向/饱和/slew）
    double wheelActualRpm[4];          // 电机实测
    bool initialized, driveEnabled, headingActive, headingValid, error;
};

class Chassis {
public:
    void init(const ChassisConfig& config);
    void setCommand(const core::MotionCommand& command);
    void setHeadingState(const core::HeadingState& state);  // 航向保持用（默认关）
    void update(double dt);
    const ChassisState& getState() const;
    const ChassisConfig& config() const;            // 只读配置（离线标定换算）
    void stop();

    bool enableDrive = true;        // 运行时开关（ARCH-009）
    bool enableHeadingHold = false;
    bool enableSlewLimit = true;
};
```

- 外部只提交运动目标，**不计算四轮转速**。
- `update()` 内完成：航向修正 → omega 平滑 → IK → 饱和 → 方向符号 → 电机下发。

### 3.2 IMU（`imu` 命名空间）

```cpp
struct ImuConfig { std::int8_t port; double yawSign; double mountingYawOffsetRad; std::uint32_t calibrationTimeoutMs; double maxDeltaYawPerCycleRad; };
struct ImuState {
    double headingRad;                 // 归一化 (-π, π]
    double rotationRad;                // 连续累计
    double angularVelocityRadPerSec;   // 绕 Z
    bool calibrating, valid;
    std::uint32_t timestampMs;
};

class Imu {
public:
    void init(const ImuConfig& config);
    void startCalibration();
    void update();
    const ImuState& getState() const;
    core::HeadingState toHeadingState() const;   // 投影为跨模块公共契约
    bool isReady() const;
};
```

- 未校准 / 读取失败 / 单周期跳变 → `valid=false`，绝不返回“0 度”冒充有效。
- 不实现 EKF，不驱动底盘；**底盘不认识 IMU**，只消费 `core::HeadingState`。
- 换航向源（里程计 / GPS / 双天线）时只需产出同一个 `core::HeadingState`，无需改 Chassis。

### 3.3 Radio（`radio` 命名空间）

```cpp
struct RadioConfig {
    pros::controller_id_e_t masterId;
    double deadband, maxLinearSpeedMps, maxAngularSpeedRadPerSec;
    bool invertForward, invertStrafe, invertTurn;
    std::uint32_t commandTimeoutMs;
};
struct RadioState {
    bool connected;
    double leftX, leftY, rightX, rightY;
    bool buttonA,B,X,Y,L1,L2,R1,R2;
    core::MotionCommand driveCommand;
    bool valid;
    std::uint32_t timestampMs;
};

class Radio {
public:
    void init(const RadioConfig& config);
    void update();
    const RadioState& getState() const;
    core::MotionCommand getDriveCommand() const;
    bool isConnected() const;
    bool isCommandFresh() const;
};
```

- 只输出 `MotionCommand`，不知道底盘轮数与 IK。
- 失联 → `valid=false`、命令清零；命令超过 `commandTimeoutMs` 未刷新 → 不新鲜。
- 摇杆映射为纯函数 `radio::mapToMotionCommand()`（`radio/input_mapping.hpp`），可主机单测。

### 3.4 LaneKeeper（`lane` 命名空间）

```cpp
enum class LaneState { DISABLED, ACQUIRING, DUAL_WALL, LEFT_WALL,
                       RIGHT_WALL, NO_WALL, TURNING, SENSOR_FAULT, TOO_CLOSE };

struct LaneConfig {
    SensorConfig sensors[4];        // 顶点 S0..S3：rangeSource / x,y / 轴角
    WallConfig wall;                // enter<100, exit>150, confirm=3
    RangeFilterConfig filter;       // 中值 + 低通 + 跳变确认
    PidConfig wallPid;              // 一个墙距 PID（Ki 默认 0）
    LaneStateConfig state;          // 目标 45mm、状态/速度/前向安全参数
    double minAxisAlignment;        // 0.90
    // 组切换回差/驻留、测量保持、最大参考垂距 ...
};

struct LaneKeeperState {
    core::MotionCommand base, command;  // command 只改 vx/vy，omega 透传
    LaneState state; LaneReason reason;
    int8_t group;                       // 0=A(前后), 1=B(左右), -1=未选
    WallDebug walls[4];                 // raw/filtered/status/usable
    bool leftObserved, rightObserved;
    double leftDistanceMm, rightDistanceMm;   // 滤波原始距离
    double leftReferenceM, rightReferenceM;   // 中心->墙参考垂距
    double corridorWidthM; bool corridorWidthValid;
    double targetWallMm, lateralErrorMm;      // 误差向右为正
    PidDebug wallPid;
    double lateralTargetMps, lateralCorrectionMps;  // 向右为正
    double stateSpeedScale, targetSpeedScale, speedScale, forwardSafetyScale;
    bool turning, degraded, fault, stateChanged, groupChanged;
    std::uint32_t transitionCount, timestampMs;
};

class LaneKeeper {
public:
    void init(const LaneConfig& config);
    void update(const core::MotionCommand& bodyCommand,
                const core::RangeSnapshot& ranges,
                const core::HeadingState& heading, double dt);
    void reset();
    const LaneKeeperState& getState() const;
    bool enable = false;
};
```

- 输入是**车体坐标**基础命令 + `core::RangeSnapshot` + `core::HeadingState`；
  不读取 PROS 设备。
- 子模块：
  - `SensorGroupSelector`：前后 A / 左右 B、左右映射、回差/驻留；
  - `WallTracker`：10/15 cm 滞回、连续 3 次、无效样本/超时；
  - `RangeFilter`：3 点中值 + 低通 + 跳变确认；
  - `LaneStateMachine`：ACQUIRING/TURNING/FAULT/双/单/无墙、积分权限、速度比例；
  - `Pid`：一个墙距 PID，积分只在对应原始距离严格 <100 时累计。
- 组选择由目标平移方向与传感器轴决定，不依赖转弯次数；45° 斜向不硬选。
- 墙距误差统一“向右为正”；命令用右法向映射回车体 vx/vy。
- 默认 `corridorOnly=true`：物理左右原始距离都 `<150 mm` 才启用墙距控制；
  门外仅监控、不纠偏（`ACQUIRING` + `corridor-inactive`，速度×1）；
  门内即使基础速度为 0 也保持双墙模式/PID，并以 ≤80 mm/s 自动居中/远离；
- `9999`/超量程/PROS_ERR/未安装都是无效样本，不确认 `wall_present=false`；
  其中持续 9999/超量程作为“无可观测近墙”的独立证据（超时后 NO_WALL/单墙）；
  `<20 mm` 的过近样本不丢，而是进入 TOO_CLOSE 主动远离（不是停住/故障）；
  读取失败/未安装持续异常才进入 SENSOR_FAULT。
- **不实现航向 PID**：`omega` 原样透传，Chassis 的既有 `HeadingController`
  继续负责 IMU 航向保持。LaneKeeper 只读 `core::HeadingState` 的角速度用于
  TURNING 判定。详见 `docs/lane_keeping.md`。

---

## 4. 命令与状态结构体

| 结构体 | 生产者 | 消费者 | 含义 |
| --- | --- | --- | --- |
| `core::MotionCommand` | Radio / 自主逻辑 | Chassis | 车体三自由度运动目标（m/s, m/s, rad/s） |
| `core::HeadingState` | Imu（或未来里程计/GPS） | Chassis / LaneKeeper | 航向 / 角速度快照（rad, rad/s）；LaneKeeper 只用角速度判 TURNING |
| `core::RangeSnapshot` | DistanceArray | LaneKeeper / diag | 四路测距（前/后/左/右）+ 状态分类，近墙判断的公共契约 |
| `lane::LaneKeeperState` | LaneKeeper | main / diag | 基础/最终命令、模式、组、墙体状态、PID 内部量 |
| `imu::ImuState` | Imu | main / diag | 航向 + 校准/零偏/状态字等诊断量 |
| `chassis::ChassisState` | Chassis | main / diag | 目标、实际命令、四轮各级输出、健康标志 |
| `radio::RadioState` | Radio | main / diag | 摇杆、按键、映射命令、连接与新鲜度 |
| `distance::DistanceArrayState` | DistanceArray | main / diag | 四路测距原始值、置信度、大小、速度、有效位 |
| `*::*Config` | 配置入口 | 各模块 | 不可变硬件/控制真值 |

> 不设“大一统” `RobotState`；每个结构体用途单一。

---

## 5. 数据流关系

正常遥控/纠偏链路：

```
Radio ──► RadioState.driveCommand (body MotionCommand) ─┐
                                                       ▼
DistanceArray ──► core::RangeSnapshot ─────────► LaneKeeper (vx/vy 修正)
                                                       │ body MotionCommand
                                                       ▼
IMU ──► ImuState ──► core::HeadingState ─────► Chassis.setHeadingState()
                                                       │
                                                       ▼
                          Chassis.update(dt)：航向 PID → omega → IK → 四轮电机

*State 快照 ──► diag::Telemetry ──► $P / $D / $S 帧（main 只调用 update()）
```

> LaneKeeper 只修改平移分量；`omega` 由 Chassis 的既有航向 PID 产生。
> 未使能 LaneKeeper（`enableLaneAssist=false`）时，它原样透传基础命令。

安全链路（任一环节失效即归零）：

```
遥控失联 / 命令超时 / Radio 非法值  → main 下发 core::zeroMotion()
disabled()                          → Chassis.stop()
IMU 无效                            → 航向修正自动旁路（不阻塞底盘）
LaneKeeper 关闭/配置非法/无墙       → 原样透传基础命令（不虚构墙距）
LaneKeeper 非有限命令               → 归零并标记 invalid
LaneKeeper 测距读取失败             → 该侧转 Unknown/不可用，绝不当成“无墙”
```

未来可扩展为：

```
IMU + Encoders + Lidar ──► Localization ──► MotionController ──► Chassis
```

`MotionController` 只要产出 `core::MotionCommand`，即可替换 Radio，**无需改底盘**。

---

## 6. 坐标系与单位约定

- 右手车体坐标系：原点在几何中心，**+X 车头**，**+Y 车体左侧**，**+Z 竖直向上**。
- 车头朝向：在框架边长坐标（x/y 沿正方形两边）中为 **+x 向 +y 偏 45°**；
  控制与 IK 仍以车体 +X = 车头、+Y = 左侧为准。
- IMU 安装：绕竖直轴旋转，**车头 = IMU +x 向 +y 偏 45°**；
  `ImuConfig::mountingYawOffsetRad = +π/4` 只影响航向零点，不影响 `gyro.z`。
- `vx`：m/s，正方向 = 车头；`vy`：m/s，正方向 = 车体左侧；`omega`：rad/s，正方向 = 俯视逆时针。
- IMU 读数内部完成 度→弧度 与 顺时针→逆时针 变换（`ImuConfig::yawSign`）。
- 遥控摇杆：向上 / 向右为正（VEX 约定）；映射到车体命令的符号在 `radio.cpp` 内完成。
- LaneKeeper 的输入/输出都是**车体坐标**；世界坐标路径方向必须先经
  `lane::worldVelocityToBody(world, headingRad)` 旋转，不能把“世界方向”直接
  当“车体方向”做组选择。
- LaneKeeper 内部墙距误差采用“向右为正”，最终用右法向 `r=(dY,-dX)` 映射回
  车体 `vx/vy`；不修改 Chassis/IK 的 `+Y=左` 约定。

---

## 7. 初始化与执行顺序

```
initialize():
    chassis.init(makeDefaultChassisConfig())   // 建电机、复位航向
    imu.init(makeDefaultImuConfig())
    radio.init(makeDefaultRadioConfig())
    distance.init(makeDefaultDistanceConfig())
    lane.init(makeDefaultLaneConfig())
    if (enableImu) imu.startCalibration()
    syncVehicleConfig()
    telemetry.init(makeDefaultTelemetryConfig(), sources)  // 只绑定状态快照指针

opcontrol() 每 20 ms:
    imu.update()                       // 仅 enableImu 时
    distance.update()                  // 四路测距，独立于底盘控制
    radio.update()
    chassis.setHeadingState(imu.toHeadingState())
    base = radio.valid ? radio.getDriveCommand() : zero
    lane.update(base, distance.toRangeSnapshot(),
                imu.toHeadingState(), dt)               // 只改 vx/vy
    chassis.setCommand(lane.getState().command)
    chassis.update(dt)                 // 航向 PID + IK + 电机
    telemetry.update()                 // $P / $D / $S，内部按各自周期限频

disabled():
    chassis.stop()
    lane.reset()                       // 清 PID/组/墙体确认历史

autonomous():
    bringup::runStaticTestSequence(chassis)   // T-01~T-06 点动自检（无自主任务时）

opcontrol()（台架模式，kFrictionTestEnabled=true）:
    bringup::runFrictionTest(chassis, imu)    // 静摩擦标定，不进入遥控循环
```

- 模块实例在 `main.cpp` 文件作用域创建（稳定生命周期），初始化顺序显式、无动态注册。
- 所有耗时操作（IMU 校准）均非阻塞。
- `main` 只保留调用顺序与生命周期；帧格式在 `diag`，标定流程在 `bringup`。

---

## 8. 如何添加新模块（以 Lidar 为例）

1. 新建 `include/lidar/lidar_config.hpp`：`LidarConfig`。
2. 新建 `include/modules/lidar.hpp`：`LidarState` + `class Lidar { init / update / getState }`。
3. 新建 `src/modules/lidar.cpp`，在此文件内访问测距硬件。
4. 在 `include/config/robot_config.hpp` 增加 `VehicleConfig::enableLidar`（如需要）。
5. 在 `main.cpp`：声明实例、`initialize()` 中 `init()`、`opcontrol()` 循环中 `update()`、
   按需把 `LidarState` 传给未来的 `Localization` / `MotionController`。
6. 如需遥测：在 `diag/telemetry.cpp` 增加对应帧，并同步 `test/` 下的解析脚本；
   `main` 只多一行 `sources` 绑定，不出现格式串。

**不需要修改 Chassis / IMU / Radio 的内部实现。** 同理，用 `MotionController`
替换 Radio 时，只需向 `Chassis.setCommand()` 提交 `core::MotionCommand`。

---

## 9. 现有架构迁移说明

| 变更 | 说明 |
| --- | --- |
| **新增** `include/modules/*.hpp` + `src/modules/*.cpp` | 控制模块（Radio/IMU/Chassis）+ 测距模块 DistanceArray 的公开接口 |
| **新增** `include/core/types.hpp` | 公共 `core::MotionCommand` |
| **新增** `include/core/math_util.hpp` | 由 `chassis/math_util.hpp` 迁移并归入 `core` |
| **新增** `include/imu/imu_config.hpp`、`include/radio/radio_config.hpp` | 模块配置 |
| **新增** `include/config/robot_config.hpp` | 统一配置入口 + `VehicleConfig` |
| **新增** `include/modules/imu.hpp` + `src/modules/imu.cpp` | IMU 从底盘内部独立为顶层模块 |
| **移动** IMU 常量 | `kImuPort` / `kImuYawSign` / 跳变阈值 → `ImuConfig` |
| **改造** `Chassis` | `drive(vx,vy,ω)` → `setCommand(MotionCommand)` + `update(dt)` + `ChassisState` |
| **改造** IK / 驱动 / 航向 | 接受 `ChassisConfig`，配置真正驱动行为 |
| **重写** `src/main.cpp` | 去除演示中的硬件细节，只留调度、数据流与生命周期 |
| **新增** `include/core/types.hpp` 的 `core::HeadingState` | 航向的公共契约；`Chassis` 不再 include `modules/imu.hpp` |
| **新增** `include/diag/telemetry.hpp` + `src/diag/telemetry.cpp` | `$P`/`$D`/`$S` 帧格式与字段表集中一处；main 不再含 printf 格式串 |
| **新增** `include/bringup/friction_test.hpp` + `src/bringup/friction_test.cpp` | 静摩擦标定与点动自检移出 main；几何换算改用 `Chassis::config()` |
| **新增** `include/modules/lane_keeper.hpp` + `src/modules/lane_keeper.cpp`、`include/lane/*` | 四距离传感器自适应车道保持：滤波、墙体状态机、组选择、状态机、双墙/单墙 PID、过近排斥、限速/前向安全；只改 vx/vy |
| **新增** `include/core/range_types.hpp` + `include/distance/range_status.hpp` | 跨模块测距契约与读数分类；DistanceArray 增 `toRangeSnapshot()` |
| **删除** `chassis/chassis.hpp`、`chassis/imu_sensor.hpp`、`chassis/math_util.hpp` 及对应 `.cpp` | 职责已被顶层模块吸收 |

**保持不变**：IK 公式与几何参数、端口映射、方向符号约定、饱和/限幅/刹车行为、
航向 PID 与主动旋转旁路逻辑、20 ms 周期、`enabled/chassis` 等运行时开关语义。
如有明显 bug 与架构重构分离记录，见提交信息。

---

## 10. 验收对照

- 各模块均有独立接口与状态结构体；main 不含 IK / 电机写值 / IMU 读取 / 摇杆映射 / 测距读取 /
  遥测格式 / 标定流程，只保留调度与生命周期。
- 模块之间不互相 include 头文件：`chassis.hpp` 只依赖 `core`（`MotionCommand` / `HeadingState`）。
- 无公共可随意修改的硬件全局状态；配置有明确归属。
- 无额外线程、继承体系或插件机制。
- 车道保持不直接控制电机：只输出 `core::MotionCommand`，由 Chassis 的既有
  航向 PID + IK + 驱动链路执行。
- 编译：`pros make` 在 `-Wall -Wextra` 下无警告；单测：`bash scripts/test_native.sh`
  （IK + 航向控制器 + 摇杆映射 + 测距分类 + 墙体状态机 + 测距滤波 + 组选择器 +
  车道状态机 + Lane PID + LaneKeeper 模拟 + 真实迷宫射线 + 遥测字段契约）。
- 遥测契约：`$P` 的 38 个载荷字段由 `DIAG_TUNE_FIELDS` 单点定义，字段数由
  `static_assert` 守住，顺序与 `test/pid_tune.py` 的 `FIELDS[1:]` 一致；
  `$D` 追加的 30 个车道字段由 `kLaneFieldCount` 与
  `test/distance_monitor.py` 的 `LANE_FIELDS` 守住；
  `scripts/check_telemetry_schema.py` 在 `scripts/test_native.sh` 中自动比对两侧。
- 硬件行为验证见 `docs/TEST_PLAN.md`（当前仅完成理论 + 主机单测 + 编译验证）。
