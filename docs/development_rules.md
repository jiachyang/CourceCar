# 开发规范（强制）

> 适用范围：本仓库所有代码与文档改动，**人类与 AI agent 同等适用**。
> 开工前先读第 1–3 节；交付前必须通过第 9 节的检查清单。
> 本文件与代码冲突时：先修代码或先修本文件，**不允许“照着旧代码继续写”**。
> 规则的可执行部分：`scripts/check_rules.sh`、`scripts/test_native.sh`。

---

## 0. 总则

**core 是契约，modules 是能力，main 是调度，diag/bringup 是应用层。**

```
        core/            纯数据与数学，不依赖 PROS，可主机单测
          ▲
      modules/           能力模块，唯一硬件管理者
          ▲
        main/            调度、数据传递、生命周期
          ▲
   diag/ + bringup/      应用层：遥测、台架调试
```

四条硬底线：

1. 依赖只能单向向下，**模块之间不互相 include**。
2. **一个硬件只有一个管理者**，任何模块不得越权访问别人的硬件。
3. **main 里不写功能**：只写“调用谁、传什么、按什么顺序”。
4. **改动必须可验证**：编译零警告、主机测试全绿、文档同步。

---

## 1. 分层与依赖

### R1.1 模块之间不得互相包含头文件

跨模块只允许两种通道：

1. `core/` 的公共契约：`core::MotionCommand`、`core::HeadingState`；
2. 对方公开的 `*State` 只读快照（由 main 转发，不直接 include 对方头文件）。

反例（曾经的写法）：

```cpp
// include/modules/chassis.hpp
#include "modules/imu.hpp"   // ✗ 底盘被迫认识 IMU 模块
```

正例（当前写法）：

```cpp
// include/modules/chassis.hpp
#include "core/types.hpp"    // ✓ 只认契约，换航向源不用改底盘
void setHeadingState(const core::HeadingState& state);
```

### R1.2 模块内部头文件不得依赖 `modules/`

`include/chassis/*`、`include/imu/*`、`include/radio/*`、`include/distance/*`
属于模块内部实现，允许依赖 `core/` 与自己的 config，**不允许** include `modules/`。

### R1.3 core 只依赖标准库

`include/core/` 只放纯数据与纯函数，禁止 include 任何工程头文件或 PROS 头文件。

### R1.4 一个硬件只有一个管理者

| 硬件 | 唯一管理者 | 文件 |
| --- | --- | --- |
| 四轮驱动电机 | `ChassisDrive` | `src/chassis/chassis_drive.cpp` |
| IMU | `Imu` | `src/modules/imu.cpp` |
| 遥控器 | `Radio` | `src/modules/radio.cpp` |
| 四路 Distance Sensor | `DistanceArray` | `src/modules/distance_array.cpp` |
| Smart Port 设备枚举 | `diag::Telemetry` | `src/diag/telemetry.cpp` |

新增硬件 = 新增模块。**不得**在 main 或其他模块里顺手读一个新设备。

### R1.5 应用层单向依赖

`diag/` 与 `bringup/` 可以依赖模块的公开接口；**模块不得依赖它们**。
应用层不得修改模块内部状态，只能读 `*State` 或调用公开方法。

### R1.6 main 只能用时间/任务 API

main 允许使用 `pros::millis()`、`pros::delay()`、`pros::Task::delay_until()`；
**禁止**使用任何设备 API（`Motor / Imu / Controller / Distance / Device / Optical / Vision / Serial / Link` 等）。

---

## 2. main 的边界

### 允许出现

- 模块实例（文件作用域，稳定生命周期）；
- `initialize()` 里的 `init()` 调用与 `syncVehicleConfig()`；
- 一个控制周期函数：读状态 → 传命令 → 调 `update()`；
- `disabled()` / `autonomous()` / `opcontrol()` 生命周期；
- 对 `diag::Telemetry` 与 `bringup` 的各一行调用。

### 禁止出现

- 逆运动学、轮速分配、饱和/限幅等控制算法；
- 硬件 API：电机写值、传感器读取（R2.2）；
- 设备探测，如 `get_plugged_type`（R2.4）；
- 遥测/串口帧的格式串与字段顺序（R2.3）；
- 摇杆映射、标定算法；
- 任何“临时 debug 打印”和测试流程。

### 规模上限（R2.1）

**`src/main.cpp` ≤ 200 行**（超过就往模块或应用层搬）。
当前约 149 行；`scripts/check_rules.sh` 会强制检查。

### 正例

```cpp
void runControlCycle(double dt) {
    if (g_vehicle.enableImu) { g_imu.update(); }
    g_distance.update();
    g_radio.update();

    g_chassis.setHeadingState(g_imu.toHeadingState());
    g_chassis.setCommand(pickCommand());
    g_chassis.update(dt);
}
```

反例：在 main 里 `printf("$P,...")`、`pros::Device::get_plugged_type()`、用 `chassis::kCenterToWheelM`
算换算——这些分别属于 `diag`、`diag`、`chassis.config()`。

---

## 3. 模块写法

### 统一接口

```cpp
namespace foo {
struct FooConfig { /* 端口、几何、标定、限幅 */ };
struct FooState  { /* 只读快照：数值 + 有效位 + 时间戳 */ };

class Foo {
public:
    void init(const FooConfig& config);
    void update();
    const FooState& getState() const { return state_; }

private:
    FooConfig config_{};
    struct Impl;                    // 隐藏 pros::Xxx 等实现细节
    std::unique_ptr<Impl> impl_;
    FooState state_{};
};
}  // namespace foo
```

- 命名空间名 = 目录名；每个头文件 `#pragma once`，且能独立编译。
- 状态用**只读快照**返回，外部不得拿到可变引用。
- 配置只在 `init()` 注入，不在运行中偷偷改模块内部成员。

### 公共头文件不暴露硬件对象

新模块的硬件对象必须放在 `.cpp` 的 `Impl` 里（PIMPL），公共头文件不出现 `pros/*`。

> 现有 `Imu` / `Radio` / `DistanceArray` / `ChassisDrive` 直接把 `pros::` 成员放在头文件里，
> 属于**已知债务**（见第 11 节）。不要照抄，也不要在无关改动里顺手改。

### 不要过度设计

本工程是单线程 20 ms 调度的小型固件：**不引入**虚接口、依赖注入容器、插件/动态注册、
额外线程、继承体系。解耦靠“公共契约 + 单向依赖”，不靠抽象层数量。

---

## 4. 配置与真值

- 模块硬件真值（端口、几何、传动、PID、限幅）→ `include/<module>/<module>_config.hpp`。
- 整车开关（模块启停、整车行为）→ `include/config/robot_config.hpp`，**只放开关**。
- **单一真值**：同一参数不得同时以“自由常量”和“结构体默认值”两种形式被业务代码使用。
  离线换算（标定、诊断）必须通过只读访问器取配置，例如 `chassis.config().centerToWheelM`。
- 配置结构体**不得依赖 PROS**，否则配置层无法主机测试（`radio_config.hpp` 是已知债务）。
- 称之为“运行时开关”的项，必须在运行时有修改入口；否则文档里只能写“编译期开关”。

---

## 5. 遥测与协议

### 唯一位置（R5.1）

所有 `$P` / `$D` / `$S` 帧只在 `src/diag/telemetry.cpp` 定义与格式化。
main 与各模块**不得**出现遥测帧格式串。

### 字段表单点定义

`$P` 的字段用 X 宏单点定义（字段名 + 取值表达式，顺序即输出顺序；示例省略行尾续行符）：

```cpp
#define DIAG_TUNE_FIELDS(X)
    X(tvx, static_cast<int>(c.applied.vx * 1000.0))
    X(tvy, static_cast<int>(c.applied.vy * 1000.0))
    /* ... */
    X(ff,  static_cast<int>(c.heading.ffTerm * 1000.0))
```

字段数由 `static_assert` 与 `kTuneFieldCount` 守住。

### 改字段的固定流程

1. 改 `DIAG_TUNE_FIELDS`（顺序即输出顺序）；
2. 改 `test/pid_tune.py` 的 `FIELDS`（`FIELDS[0]` 固定为 `ms`）；
3. 若字段数变化，更新 `kTuneFieldCount` 与文档中的字段数；
4. 跑 `bash scripts/check_telemetry_schema.py`（R5.2：`test_native.sh` 已自动调用）。

### 帧的工程约束

- **限频**：每帧有独立周期，禁止每周期无脑打印；
- **整行一次写出**：用 `FrameBuffer` 拼好再输出，禁止每字段一次 `printf`；
- **紧凑整数**：固件侧保持 `mm/s`、`mrad/s`、`mrad`、`milli-rpm`，单位换算放 Python；
- **格式稳定**：`test/pid_tune.py`、`test/distance_monitor.py`、`test/friction_test.py` 依赖帧格式，
  改格式必须同步脚本。

> `$F` 静摩擦标定帧归 `bringup`，不属于常规遥测。

---

## 6. 测试与构建

### 完成定义（必须全绿）

```bash
make -j4                      # 0 warning / 0 error
bash scripts/check_rules.sh   # 架构规则静态检查
bash scripts/test_native.sh   # 主机单测 + 遥测字段契约
```

### 纯逻辑必须可主机测试

以下文件与新增的同族文件**不得引入 PROS**（`scripts/check_rules.sh` 会检查）：

- `src/chassis/kinematics.cpp`、`src/chassis/heading_controller.cpp`、`src/radio/input_mapping.cpp`；
- `src/lane/*.cpp`、`src/modules/lane_keeper.cpp`、`include/lane/*.hpp`、
  `include/modules/lane_keeper.hpp`；
- `src/safety/*.cpp`、`src/modules/safety_supervisor.cpp`、`include/safety/*.hpp`、
  `include/modules/safety_supervisor.hpp`；
- `include/core/*`、`include/chassis/chassis_config.hpp`、`include/chassis/kinematics.hpp`、
  `include/chassis/heading_controller.hpp`、`include/imu/imu_config.hpp`、
  `include/distance/distance_config.hpp`、`include/distance/range_status.hpp`、
  `include/radio/input_mapping.hpp`。

新增纯逻辑必须：新建 `tests/native/test_*.cpp`，并在 `scripts/test_native.sh` 注册。

### 警告与标准

- 构建固定 `-Wall -Wextra`；**禁止**用 `-Wno-*` 压警告（R6.1；`common.mk` 由 PROS 模板生成，不在此列）。
- 主机测试用 `-std=c++17`；固件标准由 PROS 模板决定（当前 `gnu++26`）。
  纯逻辑代码必须保持 C++17 可编译，不要在纯逻辑头里用 C++20+ 特性。

### 构建系统

- `src/**/*.cpp` 自动参与构建，**不要手改** `common.mk` 的源文件列表。
- 新增模块不需要改构建脚本，但必须同步 `docs/architecture.md` 的模块表。
- 构建产物已在 `.gitignore`，不要提交 `bin/`、`.d/`、`*.o`。

### 硬件相关的诚实性

涉及真机行为（PID、方向、传感器）的改动，必须写明“已验证 / 未验证”：

- 未上机验证的，在提交说明与相关文档里标注**未验证**，并给出验证步骤；
- 不得把“理论推导 + 主机单测”写成“已验证”。

---

## 7. 文档同步

| 改动 | 必须同步 |
| --- | --- |
| 模块划分、公开接口、数据流、初始化顺序 | `docs/architecture.md` |
| 底盘控制/运动学细节 | `docs/chassis_design.md` |
| 遥测字段、调参工具用法 | `docs/debug_log/07_调参工具.md`、`test/README.md` |
| 硬件接线/端口 | `docs/车辆配置.md` |
| 需求状态 | `docs/chassis_requirements.md` |

- 文档里的**路径、字段数、函数签名**必须与代码一致；发现漂移顺手修正。
- 代码注释保留需求 ID（`ARCH-xxx` / `IMU-xxx` / `KIN-xxx` / `DRV-xxx` 等），便于回溯。
- 不确定的结论写“待验证”，不要写死。

---

## 8. 代码风格

- 注释用中文，讲**为什么**，不只讲“是什么”；单位与正方向写进字段注释。
- 命名空间与目录同名；类型用 `PascalCase`，函数用 `camelCase`，成员用 `xxx_`，常量用 `kXxx`。
- include 顺序：本文件对应头 → 项目头 → 平台/标准库；只 include 用到的。
- **禁止全局可变状态**；唯一例外是 `src/main.cpp` 匿名命名空间里的模块实例与 `g_vehicle`。
- 禁止单例、动态注册、隐藏的跨文件 `extern`。
- 不写“以后可能用”的代码；不为假想的扩展点加抽象。
- 单位换算、饱和限幅优先复用 `core/math_util.hpp` 与 config，不在局部写魔法数。

---

## 9. Agent 执行流程（Definition of Done）

### 开工前

1. 读 `docs/development_rules.md`（本文件）与 `docs/architecture.md`；
2. 判断改动属于哪一层：`core / modules / main / diag / bringup`；
   不属于任何一层 → 先补架构说明，再写代码；
3. 确认改动范围，**不要顺手重构无关代码**。

### 改动中

4. 行为不变优先；重构与修 bug 分开提交；
5. 新功能先想“归哪个模块”，而不是“在 main 里加几行”；
6. 需要新配置项时，先判断归属（模块真值 vs 整车开关）。

### 交付前（逐条打勾）

- [ ] `make -j4`：0 warning / 0 error
- [ ] `bash scripts/check_rules.sh`：ALL PASS
- [ ] `bash scripts/test_native.sh`：全部 PASS
- [ ] 没有在 main 里新增功能、格式串、硬件访问
- [ ] 没有新增模块间 include；跨模块只走 `core` 契约或公开 State
- [ ] 遥测字段变更已同步 Python，并通过 schema 检查
- [ ] 纯逻辑新增/修改已补主机单测
- [ ] 文档已同步（模块表、接口、路径、字段数）
- [ ] 交付说明写了：改了什么、为什么、验证命令与结果、遗留债务

### 交付说明模板

```
改了什么：
为什么：
验证：make / check_rules / test_native 的输出摘要
未验证/遗留：
```

---

## 10. 常见违规（本仓库实际出现过）

| 违规 | 正确做法 | 自动检查 |
| --- | --- | --- |
| main 里 `printf("$P,...")` | 放 `src/diag/telemetry.cpp` | check_rules R2.3 |
| main 里 `pros::Device::get_plugged_type()` | 放 `diag` | check_rules R2.4 |
| main 用 `chassis::kCenterToWheelM` 换算 | 用 `chassis.config()` | code review |
| 模块头 `#include "modules/imu.hpp"` | 用 `core::HeadingState` | check_rules R1.1 |
| 给 `$P` 加字段不改 Python | 同步 `FIELDS` + schema 检查 | test_native |
| 用 `-Wno-*` 压警告 | 修代码 | check_rules R6.1 |
| 把标定/测试流程写在 main | 放 `bringup/` | check_rules R2.1 |
| 改接口不更新 architecture.md | 同步文档 | code review |

---

## 11. 已知债务（不要照抄，也不要在无关改动里顺手改）

| 债务 | 位置 | 影响 |
| --- | --- | --- |
| 7/15 个工程头文件传递依赖 PROS | `modules/imu.hpp`、`modules/radio.hpp`、`modules/distance_array.hpp`、`chassis/chassis_drive.hpp`、`radio/radio_config.hpp`、`config/robot_config.hpp` | 这些头文件无法主机编译 |
| `ChassisState` 内嵌 `HeadingController::Debug` | `include/modules/chassis.hpp` | 公开快照绑定 PID 内部实现 |
| 常量 + 结构体默认值两套真值 | `include/chassis/chassis_config.hpp` | 改配置时易漏 |
| `radio_config.hpp` 使用 `pros::controller_id_e_t` | `include/radio/radio_config.hpp` | 配置层依赖平台 |
| ARCH-009 运行时开关无运行入口 | `src/main.cpp` 的 `g_vehicle` | 实际仍是编译期开关 |

规则：**债务不算违规，但新代码不得加重**；偿还债务要单独立项（PIMPL、类型下沉、配置合并），
不要混在功能提交里。

---

## 12. 规则修订

- 规则本身也要 review；变更写在提交说明里，并同步本文件。
- 新增规则时，优先把它变成 `scripts/check_rules.sh` 的一条检查——**可执行的规则才算规则**。
- 检查脚本是启发式的（例如按行首 `*` / `//` 跳过注释），
  对它的误报应通过改脚本解决，不应通过改代码注释绕过。
