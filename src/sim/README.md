# 迷宫传感器融合自主导航仿真

> **同步状态（2026-11）**：本仿真仍是 pre-v2.1 的 LaneAssist 原型（单墙/无墙/前向 120–300 mm
> 限速），**尚未同步固件 v2.1 的 RangeChannel / 只输出横向 delta / SafetySupervisor**。
> 它只用于迷宫/定位/规划闭环验证；固件车道语义以 `docs/lane_keeping.md` 为准。
>
> v2.1 固件车道保持的闭环验证见顶层 **`sim/README.md`**：那里直接编译
> `src/modules/lane_keeper.cpp`，不是本页的 JS 复刻。

`maze_nav_sim.html` 是一个**单文件、零依赖**的浏览器仿真，用来在写固件之前验证：

1. 传感器融合定位：四电机编码器 + IMU + 四路 ToF 距离/径向速度 + 已知固定墙的局部扫描匹配；
2. 局部车道保持：双墙走廊居中、过近排斥、前向障碍限速；
3. 全局任务：从 S 出发，经过 A/B/C 三个物块，最后到 F；转弯优先用 X-drive 平移，车头保持初始朝北；
4. 活动挡板：在相邻未知挡板处停车（不转向）用对应方向 ToF 观测，连续多帧确认 M1/M2/M3 后重规划。

直接打开：

```bash
xdg-open src/sim/maze_nav_sim.html
# 或浏览器打开 file:///.../src/sim/maze_nav_sim.html
```

## 数据与参数来源

| 内容 | 来源 |
| --- | --- |
| 8×8 迷宫、16 竖墙、11 横墙、M1/M2/M3、S/A/B/C/F | `test/data/maze_model.json`（SHA256 `5bed0505…10b601`） |
| 车架 0.255 m、轮半径 0.05 m、R=0.1475 m、200 rpm、40 rpm/20 ms、ω slew 20 rad/s² | `include/chassis/chassis_config.hpp` |
| 航向 PID 5.5/0/1.0、FF +0.361/−0.221、限幅 2 rad/s | `include/chassis/chassis_config.hpp`、`include/chassis/heading_controller.hpp` |
| 四 ToF 顶点安装（r=0.1803 m）、旧版 100/150 mm、3 次确认、PID、前向 120/300 mm | `include/distance/distance_config.hpp`；车道参数未同步 v2.1 |
| 量程 20–2000 mm、9999 = 无目标、回传物体径向速度 | `include/distance/distance_config.hpp`、`include/modules/distance_array.hpp` |
| IMU 5 ms 数据率、yawSign、安装偏置 +45°（相对闭环中抵消） | `include/imu/imu_config.hpp` |
| 遥控满杆 1.0 m/s、3 rad/s（仿真巡航低于此值） | `include/radio/radio_config.hpp` |
| 规划代价 | 平移 1.6 s/格、转弯 0 s（平移优先，车头保持）、物块 2.0 s、观测 2.5 s、终点 2.0 s（近似，非实测秒表） |

地图数据在 HTML 的 `MAP` 对象中逐项复刻 JSON；`test_sim.mjs` 会自动对比两者，防止漂移。

## 与参考 HTML 的主要差异

参考文件 `/home/jc/下载/vex_maze_sensor_closed_loop.html` 提供了很好的迷宫/信念规划框架，但它把
ToF 当成距中心 0.109 m 的简化炮塔，且车辆是瞬时速度模型。本仿真按本仓库真值重写了这些部分：

| 项目 | 参考文件 | 本仿真 |
| --- | --- | --- |
| ToF 安装 | 距中心 0.109 m，前/右/后/左 | 车架四顶点 `r=0.1803 m`，轴沿车体 ±X/±Y（`lane_config.hpp`） |
| 底盘 | 简化 X-drive 速度 | `kinematics.cpp` 的 45° IK + 200 rpm + 40 rpm/20 ms slew + 电机一阶惯性 |
| 航向 | 简化 P 控制 | `HeadingController` 的 P/D、单项/总限幅、静摩擦前馈 |
| 车道保持 | 侧向 P | LaneKeeper 的双墙参考垂距、100/150 mm、3 次确认、PID、过近排斥、前向 120/300 mm |
| ToF 速度 | 只显示 | 参与 vx/vy 融合（四顶点轴与位置共线，不提供 ω） |
| 定位 | 编码器 + IMU + 2D 墙距修正 | 增加 x/y/yaw 局部扫描匹配；纯编码器位姿单独对比 |
| 碰撞 | 圆形近似 | 沿对角线的正方形车架菱形包络（顶点半径 0.1803 + 2 mm） |
| 终点 | 直接停 | 格心微调；知道前方是固定墙时绕开 120 mm 保守限速，保留 25 mm 硬停止 |
| 失效 | 无 | 碰撞连续 15 帧后退脱困；6 次失败进入 STUCK，避免无限碰撞 |

## 仿真模型（明确边界）

- **X-drive IK**：正方形车架，车头沿对角线指向顶角；四个万向轮在四条边中点（向外偏 2 cm），ToF 在四个顶点；轮缘速度 → 电机 RPM（1:1 直驱），与 `src/chassis/kinematics.cpp` 相同。
- **电机**：200 rpm 饱和、40 rpm/20 ms slew、一阶惯性 τ=0.09 s、低速静摩擦死区。
- **航向**：与 `HeadingController` 相同的 P/D、每项限幅、总限幅、静摩擦前馈；导航器用 IMU 航向闭环。
- **ToF**：每 20 ms 读取（控制 50 Hz；`$D` 遥测 20 Hz）、射线求交、20–2000 mm、9999 无目标、高斯噪声、0.6° 角抖动、近距离置信度下降；速度来自真实车体速度在光束方向的投影（远离为正）。
- **编码器**：四轮累计角度，0.1° 量化，打滑以乘性噪声注入（只影响里程计，不改变真实运动）。
- **IMU**：航向噪声 + 可调慢漂移；默认漂移 0（与代码库 `kGyroDriftDps=0` / VEXos rotation 实测一致）。
- **碰撞**：正方形车架按车头对角线摆放的“菱形”包络（顶点半径 0.1803 m + 2 mm）对墙段做检测；不是轮地接触动力学。
- **车道保持**：按 `lane_keeper.cpp` 的要点移植——中值+低通、跳变确认、100/150 mm 滞回、3 次确认、双墙参考垂距、墙距 PID、过近排斥、横向加速度限制、前向 120/300 mm 限速。
- **终点微调**：导航知道目标格前方是已知道路/固定墙时，绕过 120 mm 保守前向限速，但仍保留 25 mm 硬停止；用于把车停到格心而不是提前 7.5 cm 停车。

**没有建模**：轮胎侧滑动力学、墙厚/反光/斜入射的真实 ToF 误差、电机电流/扭矩饱和、轮子遮挡、VEXos 任务调度抖动、真实墙体的弹性。因此仿真通过不等于实车通过；首次上机仍需按 `docs/lane_keeping.md` §11 标定。

## 操作

- `开始/暂停`、`单步 20 ms`、`重置`、`随机挡板`、`导出 CSV`。
- 真实挡板复选框只改仿真环境；导航器看不到 `sim.actual`。
- 关闭“显示真值”后，真值车体、真实挡板、真值射线都不再显示；ToF 射线改从估计位姿画，便于盲测。
- 参数滑块：ToF σ、编码器打滑、IMU 漂移、巡航速度、仿真倍速。
- 开关：车道保持、ToF 速度融合、地图测距校正，可单独关闭做消融对比。

调试 URL 参数（便于截图/复现）：

```text
?barriers=111&warp=120          # 三个挡板都装，快进 120 s 后暂停
?barriers=101&warp=200&run=1    # 快进后继续运行
```

## 信息边界（确认）

导航器在仿真中只读：

- 四路 ToF：距离、状态、置信度、物体径向速度；
- 四个电机编码器累计角度；
- IMU 航向 / gyro-z；
- **题目给定的固定墙地图**（16 竖墙 + 11 横墙 + 外框 + S/A/B/C/F 坐标）与起点 S（初始车头朝北）。

导航器**不读**：

- `sim.truePose` 真实位姿；
- `sim.actual[]` 真实挡板状态；
- 真实速度 / 真实碰撞位置。

3 个活动挡板 → **8 种迷宫配置**（2³）；未知时按 50% 先验，形成 **27 个三值信念**（3³）。
车不是物理上试 8 条路线，而是在 8 种配置的信念上规划；观测到相邻挡板后把对应信念收敛为 0/1，再重规划。
如果要求“连固定墙地图也不给、完全靠传感器探索/SLAM”，那是另一个层级的任务，当前仿真不是这个假设。

## 算法结构

- **定位**：编码器给出 vx/vy/ω；IMU 给出绝对航向；ToF 径向速度给出平移速度修正（四顶点轴与位置共线，r×a=0，因此不提供 ω）；每 0.2 s 用已知墙做局部 x/y/yaw 扫描匹配。
- **规划**：把 M1/M2/M3 的信念编码为 27 个三值状态；对每个信念在 `(格, 访问掩码, 朝向)` 上做反向 Dijkstra，得到期望最优代价；平移优先时 `turn90S=0`，朝向状态保留但不再惩罚转弯，路径由 vx/vy 平移执行；遇到相邻未知挡板先观测，再按 50% 先验展开。
- **执行**：平移优先——车头保持 `holdYaw`（默认初始朝北），到下一格中心用 vx/vy 分解，不原地旋转；到格后微调；航向 PID 只做保持；LaneKeeper 按**运动方向**选墙对（前进/后退=左右墙，横向=前后墙），修正沿运动法向叠加。
  直线续行：目标格不是新物块/未知挡板观测点/终点，且下一动作同方向时，距格心 5 cm 内直接提交并切到下一格，速度不低于 0.38 m/s，不再每格停车/微调；直角、180°、物块、观测、终点仍停车。
  与固件数据流一致：LaneKeeper 收到的 base omega=0，只改 vx/vy；航向 PID 的 omega 在 LaneKeeper **之后**叠加，再进 IK/slew，避免把航向修正误判成 TURNING。
- **挡板观测**：停车即可，不转向；用对应方向的顶点 ToF，分别按“挡板开/关”两种地图做射线预测，只使用首个命中确实是目标挡板的光束；连续 4 帧一致才更新信念；3 s 无法判定则保守按封闭处理。
- **安全**：碰撞连续 15 帧触发后退脱困；6 次脱困失败进入 STUCK，避免无限碰撞。

## 自检

```bash
node src/sim/test_sim.mjs
```

会检查：

- HTML 内嵌地图与 `test/data/maze_model.json` 完全一致；
- 27 个信念的规划值有限；
- 8 种挡板配置在标称工况（σ=8 mm、打滑 2%、漂移 0）全部完成；
- 8 种配置在压力工况（σ=20 mm、打滑 8%、漂移 0.02°/s）全部完成；
- 全封闭配置下 M1/M2/M3 都被识别为封闭；
- 全程无挡板误判、碰撞次数受限、融合位姿误差 < 15 cm。

该脚本是**可选的仿真工具测试**，不加入 `scripts/test_native.sh`（避免给固件 CI 引入 Node 依赖），也不参与 PROS 构建。按 `docs/development_rules.md`，真正移植 `path_planner` / `localization` 时必须新增 `tests/native/test_*.cpp` 并注册到 `scripts/test_native.sh`；当前 JS 自检不能替代固件单测。

## 8 种挡板配置的测试数据

运行：

```bash
node src/sim/generate_8routes.mjs
# 可选：seed noiseMm slipPct driftDps
node src/sim/generate_8routes.mjs 1 8 2 0
```

生成：

| 文件 | 内容 |
| --- | --- |
| `route_test_summary.csv` | 8 行汇总：挡板、完成、时间、碰撞、known、观测、误差、moves、route |
| `route_test_summary.md` | 可读表格 + 每个 case 的完整格路径 |
| `route_test_trace.csv` | 每个 case 的到达/观测/完成时间线 |
| `route_test_data.json` | 完整 route / obsLog / events |

页面里的“导出 8 条路测试”按钮会按当前噪声/打滑/漂移参数重跑 8 个 case 并下载汇总 CSV。

注意：8 个 case 是 8 种挡板配置（2³），不是 8 条必然不同的几何路径。若某挡板不在最优路径上，车不会专门绕路去观测它，`known` 里会保留 `?`；这是“只读传感器、按需观测”的正常结果。若要强制完整辨识，需要再加“巡检所有挡板”的任务层。

## 移植到固件的建议边界

按 `docs/development_rules.md` 的分层，建议后续拆成：

1. `core/nav_types.hpp`：`Pose2D`、`GridCell`、`NavAction`、`BarrierBelief`（纯数据）；
2. `modules/localization`：编码器 + IMU + ToF → `PoseState`（唯一读传感器的地方之一，输出只读快照）；
3. `modules/path_planner`：纯逻辑的 27 信念 DP，必须主机单测；
4. `modules/mission_nav`：状态机与格到格执行，只输出 `core::MotionCommand`；
5. `main` 只做调度：`localization.update()` → `planner/mission.update()` → `lane.update()` → `chassis.setCommand()`。

不要在 `main` 里写 IK、射线、规划或挡板判定；挡板信念和路径规划应能脱离 PROS 在主机端测试。
