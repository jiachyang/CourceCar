# 项目背景 —— VEX V5 四全向轮 X-Drive 底盘 + IMU 航向

## 1. 一句话

用 PROS（C++）做的 **四全向轮（X-Drive）遥控底盘**，四轮独立驱动、逆运动学分配，
带 **IMU 航向保持 PID**，可在运行时按模块启停，配套一套 **串口实时调参/诊断工具**。

## 2. 硬件

| 项 | 值 |
| --- | --- |
| 主控 | VEX V5 Brain |
| 框架 | PROS（Kernel 4.2.2，CLI 3.5.6） |
| 驱动电机 | 4 × V5 Smart Motor，Smart Port **1/2/3/4**，green / 200 rpm，直驱 1:1 |
| 电机↔轮映射（HW-003） | **1=FL(左前)、2=RL(左后)、3=FR(右前)、4=RR(右后)** |
| 姿态传感器 | VEX Inertial Sensor (IMU)，Smart Port **6**；安装：**车头 = IMU +x 向 +y 偏 45°** |
| 遥控 | V5 Controller（经 V5 Robot Radio，Smart Port **5**；用 `pros::Controller` 模式 A） |
| 测距 | 4 × V5 Distance Sensor：**7=前、8=后、9=左、10=右**（DistanceArray 读取，LaneKeeper 局部纠偏；默认关闭，几何待标定） |

> 完整接线表见 `docs/车辆配置.md`。

## 3. 坐标系与几何

- 车体坐标：**+X = 车头，+Y = 车体左侧，+Z = 竖直向上；omega 俯视逆时针为正**。
- 车头朝向：在框架边长坐标中为 **+x 向 +y 偏 45°**；控制仍使用车体 +X/+Y。
- 正方形外缘边长 `kFrameSideM = 0.255 m`；轮心外伸 `kWheelOffsetM = 0.02 m`；
  → 中心到轮心 `R = 0.255/2 + 0.02 = 0.1475 m`。
- 轮有效半径 `r = 0.05 m`（直径 10 cm）。
- 传动 `gearRatio = 1.0`（直驱）。
- 四轮索引顺序全工程统一：**FL, FR, RL, RR**（`enum WheelIndex`）。

**逆运动学（`src/chassis/kinematics.cpp`）**，`inv = 1/√2`，`rot = R·ω`：

```
s_FL = inv·(−vx + vy) + rot
s_FR = inv·( vx + vy) + rot
s_RL = inv·(−vx − vy) + rot
s_RR = inv·( vx − vy) + rot
rpm_i = s_i · (60/2π)/r · gearRatio · mixScale[i] + mixOffset[i]
```

## 4. 软件架构（分层）

```
config/robot_config.hpp        （整车开关 VehicleConfig）
        │  include
        ▼
src/main.cpp                   （Application：初始化 + 每周期调度，不做 IK/电机/IMU/映射）
   ├── modules/imu      ←→ imu/imu_config.hpp      （唯一访问 pros::Imu）
   ├── modules/radio    ←→ radio/radio_config.hpp  （唯一读遥控）
   └── modules/chassis  ←→ chassis/chassis_config.hpp
            ├── chassis/kinematics.cpp        （纯数学 IK）
            ├── chassis/heading_controller.cpp（航向 PID，纯逻辑）
            └── chassis/chassis_drive.cpp     （唯一访问电机）
core/types.hpp                 （MotionCommand{vx,vy,omega} 公共命令）
```

**数据流**：`Radio → core::MotionCommand → Chassis →（航向修正+omega平滑）→ IK → 4 电机`
；`IMU → imu::ImuState → Chassis（航向保持）`。

**配置真值分层（ARCH-010）**：
- 模块级硬件/控制真值 → 各模块 `*_config.hpp`；
- 整车级开关 → `include/config/robot_config.hpp` 的 `VehicleConfig`。

## 5. 控制周期

- `opcontrol()` 用 `pros::Task::delay_until(&tick, 20)`，**固定 20 ms** 周期。
- IMU 刷新实测 ~10 ms（VEXos 缓存），20 ms 每周期都能取到新样本。

## 6. 构建 / 烧录 / 测试

```bash
pros make                 # 编译
pros upload --after run   # 烧录并运行
pros v5 run 1             # 手动运行槽位 1
bash scripts/test_native.sh   # 主机端单测（IK / 航向控制器 / 遥控映射）
python3 test/pid_tune.py [串口] [窗口秒数] --csv run.csv   # 实时调参/诊断
```

## 7. 当前关键配置（真值快照）

| 位置 | 值 |
| --- | --- |
| 接线（`docs/车辆配置.md`） | 电机 **1=FL、2=RL、3=FR、4=RR**；Radio **5**；IMU **6**；测距 **7=前、8=后、9=左、10=右** |
| `chassis_config.hpp` | `kMaxWheelRpm=200`，`kMotorRpmToMillivolt=60`，`kSlewRpmPerCycle=40`，`kOmegaSlewRadPerSecPerSec=20` |
| 航向 PID | `Kp=5.5, Ki=0.0, Kd=1.0`；`∫限幅=5.0`；单项限幅 `P=10.0/I=5.0/D=5.0 rad/s`；总输出限幅 `2.0 rad/s`；`kHeadingLatchDelayMs=5000` |
| `imu_config.hpp` | 端口 **6**，`yawSign=1.0`，`mountingYawOffsetRad=+π/4`（车头相对 IMU +x），`dataRateMs=5`，漂移自动测量 2000 ms，角度补偿**关闭** |
| `radio_config.hpp` | `invertForward/Strafe/Turn=均 true`，`snapLeftStickToAxis=true`，`deadband=0.05` |
| `distance_config.hpp` | 端口 **7=前、8=后、9=左、10=右**；有效范围 20–2000 mm；20 Hz `$D` 遥测 |
| `lane_config.hpp` | 默认 S0=前/S1=左/S2=后/S3=右（顶点半径 0.1803 m）；A=前后、B=左右；近壁 100/150 mm、确认 3 帧、MID 可初始化；双墙宽度 0.37–0.70 m；横向 PD（Ki=0）；<20 mm 进 TOO_CLOSE 主动远离；`layoutVerified=false` |
| `robot_config.hpp` | `enableImu=true`，`enableImuHeading=true`，驱动/遥控/限速均开；`enableLaneAssist=false` |

## 8. 本 debug_log 索引

| 文件 | 主题 |
| --- | --- |
| `01_端口与遥控.md` | IMU/Radio 端口更新（历史：IMU 11）；遥控方向反了；左摇杆正交优先；右摇杆反转。当前接线以 `docs/车辆配置.md` 为准 |
| `02_IMU就绪判定.md` | `get_status()==ready` 判据失效 → 用 `is_calibrating()`；跳变保护基线 |
| `03_航向保持与重锁.md` | 开启航向 PID；主动旋转结束重锁目标，消除"转回旧目标" |
| `04_静置漂移补偿.md` | 漂移补偿引入 → "二次补偿"导致自转 → 关闭角度补偿 |
| `05_PID限幅与遥测.md` | 加入"死区"造成 0↔130 颤振 → 移除；P/I/D/输出四项限幅；PID 量回传 |
| `06_轮速量化死区.md` | `move_velocity` 整数 rpm 把小修正抹成 0 → 改用 `move_voltage` 支持小数 |
| `07_调参工具.md` | 串口 `$P` 帧 + Python 多窗口实时曲线（缩放/平移/CSV）；`$D` 帧末尾车道保持字段 |
| `08_经验总结.md` | 这次 debug 的通用教训清单 |
| [09_2026-11-11_居中与融合定位讨论归档.md](09_2026-11-11_居中与融合定位讨论归档.md) | 居中问题、初始航向、四 ToF 距离/速度与编码器/IMU 融合讨论；尚未授权修改控制代码 |
