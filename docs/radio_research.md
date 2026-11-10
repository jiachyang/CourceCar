# Radio 调研：V5 Robot Radio 与 PROS 遥控输入

> 对应架构文档 `docs/architecture.md` §3.3。
> 结论先行：本项目**没有**机器人间通信需求，采用**模式 A：V5 Controller 遥控输入**，
> 用 PROS `pros::Controller` 读取经过 VEXos 处理的手柄输入，**不启用 VEXlink 覆盖模式**。

---

## 1. V5 Robot Radio 的硬件角色

- V5 Robot Radio（276-4831）是 V5 Brain 与 V5 Controller（或另一台 V5 Brain）之间的
  **2.4 GHz 无线链路**设备，插在 V5 Brain 的 Smart Port 上。
- 对**遥控**场景：手柄摇杆/按键经 Radio 链路到达 Brain，**由 VEXos 固件解析**为
  标准的 Controller 输入。因此应用层读取的是 `pros::Controller`，
  **不需要也不应该自己去读无线原始数据**。
- 官方说明：
  <https://kb.vex.com/hc/en-us/articles/360061110671-Understanding-the-V5-Robot-Radio>

## 2. PROS Controller 与 VEXlink 的区别

| | 模式 A：Controller 遥控 | 模式 B：VEXlink 双机通信 |
| --- | --- | --- |
| 场景 | Brain ↔ V5 手柄 | Brain ↔ Brain（通过各自 Radio） |
| PROS API | `pros::Controller`（`misc.hpp`） | `pros::Link`（`link.hpp`） |
| 数据 | 摇杆 / 按键，VEXos 已解析 | 应用自定义原始字节帧 |
| 无线资源 | 由系统接管 | `Link(port, id, type, ov=true)` **覆盖**该端口的无线配置 |
| 风险 | 无 | 覆盖模式可能抢占/冲突遥控链路 |

`pros::Controller`（`pros/Controller` 类）关键 API：
`is_connected()`、`get_analog(controller_analog_e_t)`、`get_digital(controller_digital_e_t)`、
`get_digital_new_press(...)`、`get_battery_level()`。

`pros::Link`（VEXlink）关键 API：
`Link(port, link_id, link_type_e_t, bool ov)`、`transmit(...)`、`receive(...)`、
`link_init_override(...)`。使用它意味着接管该端口的无线链路，**会与手柄遥控争用**。

参考文档：
- PROS misc（Controller）：<https://pros.cs.purdue.edu/v5/api/cpp/misc.html>
- PROS link（VEXlink）：<https://pros.cs.purdue.edu/v5/api/cpp/link.html>
- PROS imu：<https://pros.cs.purdue.edu/v5/api/cpp/imu.html>

## 3. 本项目采用的方案

- 检查 `docs/` 与现有代码：**没有任何机器人间数据通信需求**，
  电机、测距、IMU 都是本地传感器。
- 因此选择 **模式 A**，`Radio` 模块只依赖 `pros::Controller`。
- 不实例化 `pros::Link`，不调用 `link_init_override`，**不存在与遥控冲突的风险**。
- 若未来确需双机通信，应在 `Radio` 之外单独设计一个 `Vexlink` 通信模块，
  定义独立的收发数据帧，不要与摇杆映射混为一谈。

## 4. 实际使用的 PROS API 与签名

环境：PROS Kernel **4.2.2**，PROS CLI 3.5.6。
头文件来源：`include/pros/misc.hpp`（Kernel 4.2.2 模板自带，本仓库已核对）。

```cpp
// 构造与连接
explicit pros::Controller(pros::controller_id_e_t id);   // E_CONTROLLER_MASTER / PARTNER
std::int32_t is_connected(void);

// 摇杆：返回 [-127,127]；未连接返回 0；出错 errno=EACCES
std::int32_t get_analog(pros::controller_analog_e_t channel);
//   E_CONTROLLER_ANALOG_LEFT_X / LEFT_Y / RIGHT_X / RIGHT_Y

// 按键：按下返回 1，否则 0；未连接返回 0
std::int32_t get_digital(pros::controller_digital_e_t button);
//   E_CONTROLLER_DIGITAL_{A,B,X,Y,L1,L2,R1,R2,UP,DOWN,LEFT,RIGHT}
```

`Radio` 模块（`src/modules/radio.cpp`）使用的即上述 API，未使用 VEXlink。

## 5. 通信能力与限制

- 摇杆分辨率：`get_analog` 为 8 位（-127…127）。
- 采样：与控制循环同频（本项目 20 ms）；VEXos 内部更高频刷新。
- 手柄本身自带电池与连接状态，可通过 `is_connected()`、`get_battery_level()` 检查。
- **限制**：Controller API 只提供摇杆/按键，不提供自定义载荷；
  自定义字节传输必须走 VEXlink（本项目不使用）。

## 6. 摇杆 → 运动命令映射

在 `radio::Radio::update()` 内完成，`main` 不参与：

| 输入 | 车体命令 | 正方向 |
| --- | --- | --- |
| 左摇杆 Y（上） | `vx` | 车头 |
| 左摇杆 X（右） | `vy = -leftX` | 右推 → 车体右移（vy<0） |
| 右摇杆 X（右） | `omega = -rightX` | 右推 → 顺时针（omega<0） |

- 死区 `deadband`：死区内归零，死区外线性重标定到 ±1。
- 左摇杆**正交优先**（`snapLeftStickToAxis`，默认 true）：只保留绝对值较大的轴，
  推杆直上/直下即纯 `vx`、直左/直右即纯 `vy`，**不做斜向平移**；右摇杆 `omega` 不受影响。
  这样"直上直下"可直接用于走直线测试。若日后需要斜向平移，置 false 即可。
- 满杆缩放：`maxLinearSpeedMps` / `maxAngularSpeedRadPerSec`。
- 若实机方向与预期相反，只改 `RadioConfig` 的 `invertForward / invertStrafe / invertTurn`，
  **不改任何底盘代码**。默认值为 `invertForward=true / invertStrafe=true`（实机 R-01/R-02 实测前后、左右相反）。
- ✅ 方向符号已在真机验证并修正（R-01/R-02）；若右摇杆转向相反，置 `invertTurn=true`（R-03）。

## 7. 失联行为与错误处理

| 情况 | 行为 |
| --- | --- |
| 手柄未连接 | `RadioState::connected=false`、`valid=false`、`driveCommand` 清零 |
| 读取 API 出错（未连接返回 0） | 同上，绝不以 0 摇杆冒充有效输入 |
| 命令超过 `commandTimeoutMs`（默认 100 ms）未刷新 | `isCommandFresh()==false`，`main` 下发零命令 |
| 遥控停止操作（摇杆居中） | 映射为 `MotionCommand{0,0,0}`，底盘安全停 |

安全原则：**无法确认有效控制输入时，调度层必须下发零命令**，
不得无限期沿用上一帧遥控指令。`main.cpp` 的 `runControlCycle()` 按此实现。

## 8. 结论

- 驱动方案：**PROS `pros::Controller`（模式 A）**。
- 未启用 VEXlink，不存在无线链路覆盖与遥控冲突。
- 使用的 API 均来自 Kernel 4.2.2 的 `pros/misc.hpp`，签名已在本仓库头文件中核对。
