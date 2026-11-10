# VEX V5 四全向轮 X-Drive 底盘 — 设计方案（待批准）

> 对应需求清单 `docs/chassis_requirements.md`（rev.5，已确认）。
> 当前接线、轮位与车头朝向真值见 `docs/车辆配置.md`。
> 本文件只做设计推导与接口设计，**不含可编译代码**。
> 所有参数来自已确认需求：轮心 a=0.1043 m、R=0.1475 m、轮半径 r=0.05 m、直驱、满速 200 rpm。

---

## 7.1 机械坐标模型

### 俯视示意（车头朝上，与 `docs/image/底盘数据/1792127597173.png` 一致）

```
                        车头 +X
                          ▲
             FL ●        ╱ ╲        ● FR
              (左上)    ╱   ╲    (右上)
                       ╱     ╲
                      ╱       ╲
          +Y ◄───────●    O    ●───────► −Y
           (左)       ╲       ╱        (右)
                       ╲     ╱
              (左下)    ╲   ╱    (右下)
             RL ●        ╲ ╱        ● RR
                          ▼
                        车尾 −X
```

- 框架为正方形，车头沿对角线的顶角方向。
- 四个全向轮分别位于四条边的中点外侧，轮子主动滚动方向平行于所在边。
- 车体坐标系：原点 O 在几何中心，+X 车头，+Y 车体左侧，+Z 竖直向上，omega 俯视逆时针为正。
- 车头方向也可描述为：在框架边长坐标（x/y 沿正方形两边）中，**+x 向 +y 偏 45°**；
  这只是同一朝向的另一种坐标表示，IK 仍使用车体 +X/+Y。

### 四轮位置向量 r_i 与主动滚动方向 d_i

| 轮 | 电机端口 | r_i = (x, y) [m] | 所在边 | d_i（滚动正方向单位向量） |
| --- | --- | --- | --- | --- |
| FL | 1 | (+0.1043, +0.1043) | 左上边 | (−1, +1)/√2 |
| FR | 3 | (+0.1043, −0.1043) | 右上边 | (+1, +1)/√2 |
| RL | 2 | (−0.1043, +0.1043) | 左下边 | (−1, −1)/√2 |
| RR | 4 | (−0.1043, −0.1043) | 右下边 | (+1, −1)/√2 |

- r_i 的 x、y 分量绝对值 = a = R/√2 ≈ 0.1043 m。
- 电机端口真值：**1=FL、2=RL、3=FR、4=RR**（`motorPorts` 数组顺序 FL,FR,RL,RR → `{1,3,2,4}`）。
- d_i 的符号是「约定」；真正让轮子朝正确方向转，由 `commandSign[4]`（模块内部配置）在驱动层解决，**不修改运动学公式**（KIN-008）。
- 轮子「正转」定义（KIN-004）：站在车头正前方看向车体，顺时针为正。此约定与 d_i 的符号配对使用。

> 测距传感器（已实现读取/测试，未接入控制）：**7=正前、8=正后、9=左、10=右**，
> 配置见 `include/distance/distance_config.hpp`、读取模块见 `include/modules/distance_array.hpp`，
> 实时工具见 `test/distance_monitor.py`。

---

## 7.2 逆运动学（IK）设计

### 一般关系

```
s_i = d_i · ( v + ω × r_i )        [m/s]   轮缘线速度
ω_wheel,i = s_i / r_wheel          [rad/s] 车轮角速度
motorRPM_i = ω_wheel,i · (60 / 2π) · gearRatio
```

其中 `v = (vx, vy)`，`ω × r_i = ω · (−y_i, x_i)`（二维叉乘），`gearRatio = 1`（直驱）。

### 逐轮推导

以 FL 为例：`ω×r_FL = ω(−a, a)`，`d_FL = (−1,1)/√2`

```
s_FL = (1/√2)·[ −(vx − ωa) + (vy + ωa) ] = (1/√2)(−vx + vy) + √2·ωa
     = (1/√2)(−vx + vy) + ωR          （因为 √2·a = R）
```

四轮结果：

```
s_FL = (1/√2)(−vx + vy) + R·ω
s_FR = (1/√2)( vx + vy) + R·ω
s_RL = (1/√2)(−vx − vy) + R·ω
s_RR = (1/√2)( vx − vy) + R·ω
```

### 4×3 逆运动学矩阵

令 `u = [vx, vy, ω]ᵀ`，轮缘速度 `s = M u`：

```
      ⎡ −1/√2   +1/√2   R ⎤
M =   ⎢ +1/√2   +1/√2   R ⎥
      ⎢ −1/√2   −1/√2   R ⎥
      ⎣ +1/√2   −1/√2   R ⎦
```

电机目标转速（1:1）：`RPM_i = (60 / (2π · r_wheel)) · s_i`，即 `RPM = K · M · u`，`K = 60/(2π·0.05) ≈ 190.99`。

### 用三个独立案例自检

| 输入 | s_FL | s_FR | s_RL | s_RR | 物理含义 |
| --- | --- | --- | --- | --- | --- |
| vx=1, vy=0, ω=0 | −0.707 | +0.707 | −0.707 | +0.707 | 前后两组反向 → 前进 |
| vy=1, vx=0, ω=0 | +0.707 | +0.707 | −0.707 | −0.707 | 前排同向、后排反向 → 左移 |
| ω=1, vx=vy=0 | +R | +R | +R | +R | 四轮同向同速 → 原地旋转 |
| vx=0.5, vy=0.2, ω=0.5 | 合成 | 合成 | 合成 | 合成 | 平移+旋转叠加 |

> 结论：IK 矩阵满足前进、横移、原地旋转三个正交情形，且纯旋转时四轮同向（X-Drive 特征）。

### 饱和与比例缩放（T-07）

1. 计算四轮 `RPM_i`。
2. `maxAbs = max(|RPM_i|)`。
3. 若 `maxAbs > 200`，全部按 `200 / maxAbs` 等比缩放（保留运动方向，只降速）。
4. 四轮独立保底限幅到 `[−200, +200]`。

---

## 7.3 IMU 航向控制设计

### 为什么需要

机械加工/装配/摩擦误差会让四轮理想运动学产生非预期偏航；用 IMU 测航向并闭环修正，可让平移时车头保持稳定（IMU-002 / T-08）。

### 数据流

```
IMU(端口6, 安装偏航 +45°) → 初始化/校准 → 读取 yaw → 归一化
                                   │
外锁存目标航向(target, 上电5s后存) ─┼→ 误差 e = wrapToPi(target − yaw)
                                   │
                          PID → 修正角速度 ω_corr
                                   │
外部 omega 指令 ──(≠0: 主动旋转，旁路航向保持)──► 融合 → omega_cmd
                                   │
                          omega_cmd → IK → 轮速 → 电机
```

### 关键点

- **目标航向**：上电后等 5 s（让 IMU 校准稳定），读取当前 yaw 锁存为 `targetYaw`；之后不再自动改变（IMU-011）。
- **安装偏航**：IMU 绕竖直轴旋转安装，车头 = IMU +x 向 +y 偏 45°。
  `ImuConfig::mountingYawOffsetRad = +π/4` 加到 `heading/rotation`；
  `gyro.z` 角速度不变，航向保持的相对闭环不受常值偏置影响。
- **静摩擦前馈**：实测旋转起步需要 `+0.361 rad/s`（逆时针）/`−0.221 rad/s`
  （顺时针）。仅当 `|gz|≤0.05 rad/s` 且 `|err|>0.0087 rad` 时把对应值加到 PID
  输出；车体转动后自动退出，PID 负责动态跟踪，避免静止摩擦吃掉小误差。
- **角度归一化**：`wrapToPi` 把任意角差折算到 `(−π, π]`，取最短角差，解决跨 0/360°（IMU-017）。
- **PID**：输入 `e`，输出 `ω_corr`；积分限幅、输出限幅，避免修正量过大（IMU-010）。
- **主动旋转 vs 航向保持**：`|外部 omega| > ε` 时判为主动旋转 → 旁路 PID（并重置积分），直接把外部 omega 作为 `omega_cmd`；`= 0` 时恢复保持，`omega_cmd = ω_corr`（IMU-015）。
- **平滑**：最终 `omega_cmd` 再过一次斜率限制（slew），避免切入/切出时速度跳变（IMU-016）。
- **失效保护**：IMU 未校准/掉线/跳变超阈值 → `ω_corr = 0`，航向修正自动停用，底盘仍可运动（IMU-009 / T-10）。
- **叠加位置**：`ω_corr` 直接加到 IK 的 omega 输入，不单独驱动电机（IMU-018）。

---

## 7.4 软件接口设计

### 模块划分（最小实现，ARCH-007 / ARCH-008）

| 模块 | 职责 | 是否直接碰硬件 |
| --- | --- | --- |
| `ChassisKinematics` | 纯数学：`(vx,vy,ω)` → 四轮目标 RPM | 否 |
| `ChassisDrive` | 电机对象、方向符号、限幅、slew、刹车函数 | 是 |
| `Imu`（顶层模块） | IMU 初始化/校准/读取/归一化，公开 `ImuState` + `toHeadingState()` | 是 |
| `HeadingController` | 目标航向锁存、PID、主动旋转旁路 | 否（只消费 `core::HeadingState` 的数值） |
| `Chassis`（顶层模块） | 对外 `MotionCommand` + `HeadingState`，串起上面四者 | 否 |
| `Radio`（顶层模块） | 遥控输入 → `MotionCommand` | 是（Controller） |
| `diag::Telemetry`（应用层） | 只读 `*State` → `$P`/`$D`/`$S` 串口帧 | 是（仅端口扫描） |
| `bringup`（应用层） | 台架调试：静摩擦标定、点动自检 | 否 |

> `PoseEstimator` 本阶段不做（EST-003/004）。
> **架构升级（后续）**：IMU 已从底盘内部拆为独立顶层模块 `modules/imu.hpp`；
> 底盘改为 `setCommand(MotionCommand)` + `update(dt)` 的调度式接口。
> 完整数据流与接口见 `docs/architecture.md`。

### 参数放置（ARCH-010）

- **模块内部配置**（端口、传动比、轮径、R、a、方向符号、PID 增益…）→ 各模块的 `*_config.hpp`。
- **`main` 的整车真值配置表** → 只放**模块开关**和整车级行为：

```cpp
// include/config/robot_config.hpp —— 整车真值配置（只放整车级内容）
struct VehicleConfig {
    bool enableChassisDrive = true;   // 底盘驱动模块
    bool enableImu          = true;   // IMU 模块：已装（端口 6），开启校准与读取
    bool enableImuHeading   = true;   // IMU 航向保持：链路已通，当前开启
    bool enableSlewLimit    = true;   // 加减速限制
    bool enableRadioControl = true;   // 遥控输入模块（opcontrol 中生效）
};
// main.cpp 文件作用域：config::VehicleConfig g_vehicle; 模块每周期同步，运行时可变
```

### 对外接口

```cpp
class Chassis {
public:
    void init(const ChassisConfig& config);           // 初始化电机 + 航向控制器
    void setCommand(const core::MotionCommand& cmd);   // 提交运动目标
    void setHeadingState(const core::HeadingState& s); // 提交航向快照（默认不用）
    const ChassisConfig& config() const;               // 只读配置（离线标定换算）
    void update(double dt);                            // 运行一次控制周期
    const ChassisState& getState() const;              // 只读状态
    void stop();                                       // 安全停止
};
```

- `update()` 内部流程：可选航向修正 → omega 平滑 → IK → 饱和缩放 → 方向符号 → `ChassisDrive`。
- 调用周期 **20 ms**（CTRL-007），由 `main` 的 `opcontrol()` 循环驱动。
- 刹车：`ChassisDrive::stop()` 独立函数（DRV-010），当前实现为 brake 模式（DRV-007），日后可替换。

### 模块单独启用的行为（ARCH-009 / PRJ-008）

- `enableChassisDrive=false`：不发任何电机指令。
- `enableImuHeading=false`：仅做 IK + 驱动，航向修正恒为 0（链路代码仍在，随时可开）。
- 开关为**运行时**，改 `g_vehicle` 即生效，不需重编译。

---

## 实现状态（已批准并完成）

用户已下达「开始按照计划进行」，实现已落地：

| 文件 | 内容 |
| --- | --- |
| `include/chassis/chassis_config.hpp` | 底盘模块配置 `ChassisConfig`（端口/几何/传动/符号/限幅/PID） |
| `include/core/math_util.hpp` | `core::wrapToPi` / `core::clampd` |
| `include/chassis/kinematics.hpp` + `src/chassis/kinematics.cpp` | 纯数学 IK（接受 `ChassisConfig`） |
| `include/chassis/chassis_drive.hpp` + `src/chassis/chassis_drive.cpp` | 四电机驱动、饱和、slew、刹车 |
| `include/modules/imu.hpp` + `src/modules/imu.cpp` | 顶层 IMU 模块 `init/update/getState` |
| `include/chassis/heading_controller.hpp` + `src/chassis/heading_controller.cpp` | 航向 PID、主动旋转旁路 |
| `include/modules/chassis.hpp` + `src/modules/chassis.cpp` | 顶层底盘模块 `setCommand/update/getState` |
| `include/modules/radio.hpp` + `src/modules/radio.cpp` | 顶层 Radio 模块（V5 Controller 遥控输入） |
| `include/config/robot_config.hpp` | 统一配置入口 + `VehicleConfig` |
| `src/main.cpp` | 纯调度：init / 数据流 / 生命周期 |
| `tests/native/test_kinematics.cpp` + `scripts/test_native.sh` | 主机端 IK 单测 |
| `docs/architecture.md` / `docs/radio_research.md` | 架构说明 / Radio 调研 |
| `docs/TEST_PLAN.md` | T-01～T-12 测试/标定记录模板（含 Radio） |

验证：`bash scripts/test_native.sh` 通过；`pros make` 无警告，生成 `bin/hot.package.bin`。

STATUS: IMPLEMENTED_PENDING_HARDWARE_TEST
