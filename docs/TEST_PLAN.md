# 底盘测试与标定记录（模板）

> 对应需求清单 `docs/chassis_requirements.md` §K。**编译通过 ≠ 底盘能正确行驶。**
> 结论必须区分：理论计算验证 / 主机单元测试 / 编译验证 / 真实硬件验证。

---

## 1. 理论计算验证

- IK 推导见 `docs/chassis_design.md §7.2`，4×3 矩阵与三个正交案例已核对。
- 结果：☐ 通过 ☐ 不通过　备注：__________

## 2. 主机单元测试（不需要 V5）

```bash
bash scripts/test_native.sh
```

| 用例 | 内容 | 结果 |
| --- | --- | --- |
| KIN-01 | 前进时 FL/RL 与 FR/RR 反向且对称 | ☐ |
| KIN-02 | 左移时前排同号、后排同号、前后相反 | ☐ |
| KIN-03 | 原地逆时针旋转时四轮同向同速 | ☐ |
| KIN-04 | 前进 1 m/s 的转速量级 | ☐ |
| KIN-05 | 三输入叠加满足线性（T-06 理论） | ☐ |
| KIN-06 | 零命令 → 四轮目标为 0 | ☐ |

主机测试同时包含：

| 套件 | 内容 |
| --- | --- |
| `test_kinematics` | 上表 KIN-01～KIN-06 |
| `test_heading_controller` | 主动旋转旁路、航向保持、跨零最短角差、限幅、切换不突跳 |
| `test_radio_mapping` | 死区、重标定、三轴映射、invert 翻转、满杆缩放 |
| `test_distance_status` | PROS_ERR / 9999 / <20 / >2000 / 未安装分类 |
| `test_range_channel` | 99/100/150/151、MID 初始化、1.65 m 远距有效、丢帧保持/超时、跳变、滤波 |
| `test_sensor_group_selector` | 前后=A/左右=B、前进/后退/横移/斜向/换组 |
| `test_lane_state_machine` | TRACKING/DEGRADED/TOO_CLOSE/NO_CORRIDOR/INSUFFICIENT/NO_PAIR、hold 衰减 |
| `test_lane_pid` | P/D、限幅、D 低通、复位 |
| `test_lane_config` | 顶点布局、100/150/3、MID、走廊宽度、PD/状态参数校验 |
| `test_lane_keeper` | 横向符号、横移/倒车选对、远距回波、宽度、方向记忆、过近、丢墙缓释、无速度倍率 |
| `test_safety_supervisor` | 前向/侧向制动、45° 盲区、低置信度、航向门控、fail-closed |
| `test_maze_fixture` | 真实 maze_model.json 射线、S 点基准、45 cm 走廊双墙跟踪、M1/M2/M3 |

- 运行日期：__________　结果：__________

## 3. 编译验证

```bash
pros make        # 期望：无 warning / 无 error，生成 bin/hot.package.bin
```

| 项 | 结果 |
| --- | --- |
| 编译成功（T-12） | ☐ |
| 无警告 | ☐ |
| 生成 `bin/hot.package.bin` | ☐ |

- PROS CLI 版本：__________　Kernel：4.2.2　电池电压：__________ mV

---

## 4. 真实硬件测试

> **安全提示**：首次上电、方向测试（T-01~T-06、T-11）请把驱动轮**架空**。
> 每次测试前记录**电池电压**（docs-电机 已证明电压会显著影响转速，跨时段对比必须记录）。
> 上电前先按 `docs/车辆配置.md` 核对全部接线（1~10 号 Smart Port）。

### 4.1 方向与功能（架空车轮，低速）

| ID | 测试 | 期望 | 实测 | 结论 | 备注 |
| --- | --- | --- | --- | --- | --- |
| T-01 | `drive(+v, 0, 0)` | 向车头前进 | | ☐ | 日期/电压： |
| T-02 | `drive(-v, 0, 0)` | 向后 | | ☐ | |
| T-03 | `drive(0, +v, 0)` | 向左平移 | | ☐ | |
| T-04 | `drive(0, -v, 0)` | 向右平移 | | ☐ | |
| T-05 | `drive(0, 0, +w)` | 俯视逆时针 | | ☐ | |
| T-06 | `drive(+v,+v,+w)` | 平移+旋转同时 | | ☐ | |
| T-11 | 逐轮单轮微速 | 各轮符合运动学定义 | | ☐ | 不符则改 `ChassisConfig::commandSign[]` |

> T-11 记录表（哪轮反向就改 `include/chassis/chassis_config.hpp` 中 `ChassisConfig::commandSign`）：

| 轮 | 端口 | 指令 +RPM 时的实测转向 | 符号是否需翻转 |
| --- | --- | --- | --- |
| FL | 1 | | ☐ |
| FR | 3 | | ☐ |
| RL | 2 | | ☐ |
| RR | 4 | | ☐ |

### 4.2 限幅与加减速

| ID | 测试 | 期望 | 实测 | 结论 |
| --- | --- | --- | --- | --- |
| T-07 | 输入超过满速 | 四轮等比缩放、不超 200 rpm | | ☐ |
| DRV-006 | 阶跃输入 | 指令平滑、无猛冲 | | ☐ |

### 4.3 IMU 航向（需安装并校准 IMU 后）

> 当前 `g_vehicle.enableImuHeading = true`。若关闭，先把开关打开再测下列项。

| ID | 测试 | 期望 | 实测 | 结论 |
| --- | --- | --- | --- | --- |
| T-08 | 直线平移人为致偏 | PID 减小偏航 | | ☐ |
| T-09 | 主动旋转 | 航向保持不阻止旋转 | | ☐ |
| T-10 | 校准未完成/拔掉 IMU | 不使用无效航向、不修偏 | | ☐ |
| T-13 | IMU 45° 安装零点 | IMU +x 对准参考上电时 `rot≈+45°`；车头转到参考方向时 `rot≈0°` | | ☐ |
| T-14 | 静摩擦前馈 | 静止小误差时 `ff` 出现 ±0.361/−0.221 rad/s；车体转起来后 `ff` 归零 | | ☐ |

- IMU 端口：**6**　目标航向锁存延时：上电后 5 s
- IMU 安装：**车头 = IMU +x 向 +y 偏 45°**，`ImuConfig::mountingYawOffsetRad = +π/4`
  - 若 `rot` 符号相反（应 +45° 却显示 −45°），改 `−π/4`；
  - 若启动时以车头对准参考而非 IMU +x，改 `0`；
  - 航向保持是相对闭环，常值偏置不影响保持行为，只影响航向零点/遥测。
- 接线核对：IMU=6、Radio=5、测距 7/8/9/10（详见 `docs/车辆配置.md`）
- 若修正方向相反：改 `include/imu/imu_config.hpp` 的 `ImuConfig::yawSign`
- PID 初值为占位：`headingKp/Ki/Kd`，需整定后回填此表

### 4.4 Radio 遥控输入（需 V5 手柄，实测前架空车轮）

> 本项目采用 Controller 遥控输入模式（非 VEXlink），见 `docs/radio_research.md`。
> 方向符号未实机验证，若不符只改 `include/radio/radio_config.hpp` 的 `invert*` 开关。

| ID | 测试 | 期望 | 实测 | 结论 |
| --- | --- | --- | --- | --- |
| R-01 | 左摇杆前推 | 向车头前进（vx>0） | | ☐ |
| R-02 | 左摇杆右推 | 向右平移（vy<0） | | ☐ |
| R-03 | 右摇杆右推 | 顺时针旋转（omega<0） | | ☐ |
| R-04 | 摇杆居中（死区内） | 四轮目标为 0、不漂移 | | ☐ |
| R-05 | 关闭遥控器 / 拔无线 | `valid=false`，底盘停 | | ☐ |
| R-06 | 按键 A/B/X/Y | `RadioState` 对应位翻转 | | ☐ |
| R-07 | 左摇杆 45° 斜推 | 只走单一正交方向（前后优先）、不斜向平移 | | ☐ |

- 方向符号已按 R-01/R-02 实测修正（`invertForward=true`、`invertStrafe=true`）；
  若 R-03 转向相反，置 `invertTurn=true`。
- 左摇杆默认**正交优先**（`snapLeftStickToAxis=true`）：直上/直下为纯前后，便于走直线。

- 满杆速度：平移 `maxLinearSpeedMps`、旋转 `maxAngularSpeedRadPerSec`
- 命令保鲜期：`commandTimeoutMs`（默认 100 ms），超时 main 下发零命令

### 4.5 四路激光测距（前 7 / 后 8 / 左 9 / 右 10）

> 固件在 `opcontrol()` 中以 20 Hz 输出 `$D` 帧，工具：`test/distance_monitor.py`。
> 测距已可经 `LaneKeeper` 参与横向纠偏（`enableLaneAssist` 默认 false），
> 本节只验证原始读数；车道闭环见 4.6。

| ID | 测试 | 期望 | 实测 | 结论 |
| --- | --- | --- | --- | --- |
| D-01 | 四路都接好，运行监视脚本 | 四路均显示“有效”或“无目标”，无“未连接” | | ☐ |
| D-02 | 依次用挡板靠近 7/8/9/10 | 对应方向距离下降，其他方向不受影响 | | ☐ |
| D-03 | 移开挡板 | 显示“无目标”（9999）或超量程，不保持旧值 | | ☐ |
| D-04 | 拔掉任一路测距 | 仅该路“未连接”，其余三路正常 | | ☐ |
| D-05 | 挡板放在 <20 mm | 显示“过近”，不得当成有效距离 | | ☐ |

- 启动：`pros make && pros upload --after run`
- 监视：`python3 test/distance_monitor.py /dev/ttyACM1`
- 记录：`python3 test/distance_monitor.py /dev/ttyACM1 --csv distance.csv`
- 有效判据：`installed && 20 <= d <= 2000 mm`
- 连接诊断：固件 1 Hz 输出 `$S`（1～21 号端口类型）。脚本会显示
  “Distance 在 7/8/9/10”或具体插错端口；完全没有 Distance 则检查供电/智能线。

### 4.6 车道保持 v2.1（四路测距 + 双墙横向 PD + 安全层）

> **前置**：先完成 4.5 的四路测距有效性检查与 `LaneConfig` 安装几何标定。
> 当前 `enableLaneAssist=true`、`enableSafety=true` 是本次实车测试的临时值；标定完成前不要视为量产默认。
> 所有项目当前为**未验证**。

| ID | 测试 | 期望 | 实测 | 结论 |
| --- | --- | --- | --- | --- |
| L-01 | 四路测距原点位置/轴向量测 | 填入 `LaneConfig::sensors[]`，`layoutVerified=true` | | ☐ |
| L-02 | 前进进入 45 cm 通道 | 组 B（左右），左右约 45 mm，3 帧内 `NEAR_RETURN`，`TRACKING` | | ☐ |
| L-03 | 左移/右移 | 组 A（前后）；运动左侧=后/前，横向修正落在 `vx` | | ☐ |
| L-04 | 倒车 | 组 B 但左右墙映射对调，修正方向不反向 | | ☐ |
| L-05 | 车体原地转 90° 后仍沿车头前进 | 组不因旋转而交换（仍 B） | | ☐ |
| L-06 | 一侧墙结束（有效远墙 850 mm） | 该侧 `FAR_RETURN` 且 valid；不进入双墙，横向修正平滑回 0；原始距离仍为 850 | | ☐ |
| L-07 | 拔掉/遮挡任一路 | 该路 `valid=false/UNKNOWN`；不误判 FAR；不虚构双墙 | | ☐ |
| L-08 | 双墙低速直线 | 无持续振荡/过冲；P/D 遥测合理；先保持 Ki=0 | | ☐ |
| L-09 | 通道口持续 9999 | 不写 FAR/无墙；180 ms 后 valid=false/UNKNOWN | | ☐ |
| L-10 | 单侧/双侧 <20 mm 过近 | `TOO_CLOSE`，向远离最近墙方向修正；不判故障/不减速；恢复后回 TRACKING | | ☐ |
| L-11 | 上电就在 100–150 mm | 连续 3 帧进入 `MID_RETURN`，可 `TRACKING`，不永久 UNKNOWN | | ☐ |
| L-12 | 前/后 1.65 m / 0.98 m，左右 45 mm | Lane 保持 TRACKING；前后为 `FAR_RETURN` 且原始值不变 | | ☐ |
| L-13 | 正前方 60 mm、基础速度 0.55 m/s | SafetySupervisor 降低 vx，LaneKeeper 不改变巡航速度 | | ☐ |
| L-14 | 上电静止、无遥控、车偏左/偏右 10–20 mm | 无方向自动回退选对；`TRACKING/TOO_CLOSE`；`corr` 指向中心；车实际横移 | | ☐ |
| L-15 | 停车居中时误差 5 mm / 40 mm | 5 mm 给最小 0.08 m/s；误差 <3 mm 不再给最小速度；有平移速度时恢复 PD | | ☐ |

- 观察工具：`python3 test/distance_monitor.py` 或 `test/lane_plot.py` 的 `$D`
  车道字段（mode/group/near/valid/fresh/left/right/centerErr/P/D/corr/输出）。
- 首测务必低速（例如 `vx <= 0.15 m/s`），先让车在通道中心附近进入 TRACKING，再人为偏移。
- 标定后把横向 PD 的 Kp/Kd 回填到 `lane_config.hpp`，并更新本表。

---

## 5. 标定项清单（TODO(CALIBRATE)）

| 参数 | 位置 | 方法 | 标定值 |
| --- | --- | --- | --- |
| `commandSign[4]` | chassis_config.hpp | T-11 架空测转向 | |
| `yawSign` | imu_config.hpp | 打开航向、手转车头看修正方向 | 已由用户改为 +1.0（待实机确认修正方向） |
| `mountingYawOffsetRad` | imu_config.hpp | T-13：IMU +x 对准参考上电看 `rot`；应 +45°，反了改 −45° | +π/4（当前） |
| `headingFrictionFeedforwardPos/Neg` | chassis_config.hpp | T-14 / `friction_test.py` 实测 | +0.361 / −0.221 rad/s（2026-10-24） |
| `wheelRadiusM` | chassis_config.hpp | 卡尺/推车 1 m 反推 | 0.05 m（已填，可复核） |
| `headingKp/Ki/Kd` | chassis_config.hpp | 直线偏航整定 | |
| `slewRpmPerCycle` | chassis_config.hpp | 观察启停是否猛冲 | |
| `deadband` / `invertForward` / `invertStrafe` / `invertTurn` | radio_config.hpp | §4.4 遥控方向测试 | 三项 invert 均 true；正交优先 true |
| `gyroBiasDps` / `gyroDriftDps` | imu_config.hpp | 启动静置自动测一次（遥测 `bias`/`drift`） | bias≈-0.003 dps；drift < 0.02 dps（噪声级，视为 0；43 s 静置 rot 恒 0） |
| `sensors[0..3].xM/yM` | lane_config.hpp | 量测距出射点到车体中心（顶点半径约 0.1803 m） | 0.1803 占位，未验证 |
| `sensors[0..3].axisRad` | lane_config.hpp | 量实际出射轴（前 +X、左 +Y、后 π、右 −Y） | 0 / π/2 / π / −π/2 占位，未验证 |
| `sensors[0..3].rangeSource` | lane_config.hpp | 确认逻辑通道↔快照索引 | 0/2/1/3（前/左/后/右），未验证 |
| `channel.enterMm/exitMm/detectMm` | lane_config.hpp | 近壁分类/双墙门限 | 100/150/150 占位 |
| `channel.maxAgeMs/freshWindowMs` | lane_config.hpp | 实测刷新率/丢帧 | 180/42 ms 占位 |
| `pd.kp/kd` | lane_config.hpp | L-08 低速整定，先 Ki=0 | 0.0010/0.000025 占位 |
| `state.min/maxCorridorWidthM` | lane_config.hpp | 实际通道宽度/墙角容忍 | 0.37–0.70 m 占位 |
| `state.repulseMps` | lane_config.hpp | 过近排斥强度 | 0.10 m/s 占位 |
| `state.standstillMinCorrectionMps` | lane_config.hpp | L-15 停车静摩擦补偿 | 0.08 m/s 占位 |
| `state.standstillDeadbandMm` | lane_config.hpp | L-15 居中死区 | 3 mm 占位 |
| `state.maxCorrectionMps/maxLateralAccelMps2` | lane_config.hpp | 横向限幅/变化率 | 0.14 m/s / 0.70 m/s² 占位 |
| `SafetyConfig` | safety_config.hpp | 实测制动距离/延迟 | 0.12 s / 1.2 m/s² / 22 mm 占位 |
| `enableLaneAssist/enableSafety` | robot_config.hpp | L-01..L-13 完成后决定；本次测试临时 true | true/true |

---

## 6. 每次测试记录头

```
日期：
PROS Kernel：
电池电压（mV）：
环境温度：
测试人：
原始日志文件：
```
