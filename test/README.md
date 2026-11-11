# test/ — 调参 / 传感器测试工具

## 文件

- `pid_tune.py` — 纯标准库（Tk）实时曲线，读 V5 串口 `$P` 帧，画三自由度
  **目标 vs 实测**：`vx` 前后、`vy 左右平移`、航向角/角速度。
  航向角单位 **度（°）**，角速度/PID 修正单位 **度/秒（°/s）**；
  支持鼠标单击放置竖线游标，显示该时刻的目标/读取值。
- `distance_monitor.py` — 纯标准库终端实时表，读 V5 串口 `$D` 帧，
  显示四路 V5 Distance Sensor（前/后/左/右）的距离、置信度、大小、速度与状态。
- `lane_plot.py` — 纯标准库 Tk 实时分析 UI，读 `$D`，同时画两侧距离、
  墙距/过近误差、横向控制输出，并写 CSV；不依赖 matplotlib。

---

## 1. PID 调参（`pid_tune.py`）

### 前置（固件）

固件在 `src/diag/telemetry.cpp` 的 `printTuneFrame()` 里以 **50 Hz** 发出：

```
$P,ms,tvx,tvy,tw,wraw,rot,gz,ht,herr,...,outLim,ff
```

完整字段定义见 `src/diag/telemetry.cpp` 的 `DIAG_TUNE_FIELDS`（唯一真源）。

固件原始单位：`mm/s`、`mrad/s`、`mrad`、`rpm`。  
`pid_tune.py` 在解析后把旋转量换算成 **deg / deg/s**，再显示和写 CSV。

- `tvx/tvy` = 遥控下发目标平移速度
- `tw` = 下发目标自转（**含航向 PID 修正**）
- `rot` = 实测偏航角；`gz` = 陀螺 Z 角速度（逆时针为正）
- `a*` = 四轮实测转速，Python 用正运动学反算实测 `vx/vy`

编译并烧录：

```bash
pros make && pros upload --after run
```

### 运行

```bash
python3 test/pid_tune.py                  # 自动找 V5 user 口，窗口 10s
python3 test/pid_tune.py /dev/ttyACM2 15  # 指定串口 / 窗口秒数
python3 test/pid_tune.py --csv run1.csv   # 同时存 CSV（角度已换算成 deg）
python3 test/pid_tune.py --headless 10    # 无 GUI，只采集 10s
```

窗口操作：

- **单击曲线**：放置一条平行于 Y 轴的竖线游标，显示该时刻的
  `目标 / 读取 / 差`；
- 滚轮：缩放时间轴；
- 左键拖动：平移；
- 双击：回到“跟随最新”并清除游标。

曲线单位：

| 曲线 | 单位 |
| --- | --- |
| 前后 vx | mm/s |
| 平移 vy | mm/s |
| 自转 ω | °/s |
| 目标航向 vs 实测偏航 | ° |

操作：烧录后先**保持静止约 5 s**（IMU 校准 + 测零偏），然后用遥控器分别做
**前后、左右平移、原地旋转**，观察曲线。

### 怎么看曲线

| 现象 | 含义 | 调整 |
| --- | --- | --- |
| 实测滞后目标、跟不上 | 响应慢 | 提高 `kHeadingKp`（自转）；轮速看电机内置 PID |
| 实测过冲/来回振荡 | 增益过大 | 降低 `kHeadingKp`，或加 `kHeadingKd` 阻尼 |
| 稳定后有固定偏差 | 纯 P 稳态误差 | 加 `kHeadingKi`（配合 `kHeadingIntegralLimit`） |
| `omega` 目标≈0 时实测仍在漂 | 稳态偏置 | 加 `Ki`；仍是机械/标定问题 |

> 航向保持是**角度环**：PID 输入是航向角误差，输出才是 ω 指令。  
> “自转 ω”窗口只是把 PID 输出的 ω 指令和陀螺实测 ω 放在一起看，
> **不是独立的角速度闭环**。要看角度设定值/读数，用“航向角度环：目标 vs 实测”窗口。

> 角度显示按 **±180° 折叠**，误差使用**最短角差**。  
> 例如 `目标 -115.11°`、`实测 +244.25°`：两者相差约 359.36°，
> 但实际等价角度是 `-115.75°`，最短角差只有 `+0.64°`（控制周期采样会有几百分之一度差异）。
> 这是正确的，不是把 359° 当成误差。

### 静摩擦前馈（`ff`）

`chassis_config.hpp` 已加入 2026-10-24 实测的方向相关静摩擦前馈：

```text
err > 0（逆时针）：+0.361 rad/s（+20.68°/s）
err < 0（顺时针）：-0.221 rad/s（-12.66°/s）
```

启用条件：

```text
|实测 gz| <= 0.05 rad/s（接近静止）
且 |err| > 0.0087 rad（约 0.5°）
```

前馈直接加到 PID 输出上，`$P` 帧最后一项 `ff` 就是本周期实际生效值
（原始 mrad/s，上位机显示为 °/s）。一旦车体转起来，前馈自动关闭，
由 PID（尤其是 I 项）继续跟踪。

增益在 `include/chassis/chassis_config.hpp` 的 `kHeadingKp/Ki/Kd`。改完重新
`pros make && pros upload`，脚本不用重启也能继续看。

> `rot`（yaw）曲线用来单独确认航向环：静止时应贴着一条水平线不掉头。

---

## 2. 四路距离传感器实时监视（`distance_monitor.py`）

### 接线

| 方向 | Smart Port |
| --- | --- |
| 前 | 7 |
| 后 | 8 |
| 左 | 9 |
| 右 | 10 |

完整接线见 `docs/车辆配置.md`。

### 前置（固件）

固件 `src/main.cpp` 在 `opcontrol()` 中通过 `DistanceArray` 读取四路
`pros::Distance`，并以 **20 Hz** 发出：

```
$D,ms,instMask,validMask,
   dF,cF,sF,vF,tF,  dR,cR,sR,vR,tR,  dL,cL,sL,vL,tL,  dRt,cRt,sRt,vRt,tRt,
   laneMode,laneGroup,nearMask,validMask,
   leftMm,rightMm,centerErrMm,pMms,iMms,dMms,corrMms,outVxMms,outVyMms,outOmegaMradS,
   filterS0..S3,freshMask,leftClass,corridorWidthMm,laneReason,
   qualityMilli,rightClass,pairScoreMilli,laneFlags,
   sideLeftRawMm,sideRightRawMm,pidTargetMmS,tooCloseMask,
   safetyScaleMilli,safetyNearestMm,safetyFlags
```

方向顺序：前、后、左、右。末尾 **30 个**是车道保持字段，随后 **3 个**是安全层字段（旧固件没有，脚本会自动兼容）。

- `d`：距离 `mm`；`-1` = 读取失败，`9999` = 无目标
- `c`：置信度 `0..63`；`-1` = 读取失败
- `s`：物体大小 `0..400`；`-1` = 无效
- `v`：物体速度 `mm/s`
- `t`：VEXos 识别的设备类型；`7=Distance`，`0=无设备`，`2=Motor`，`6=IMU`
- `instMask` / `validMask`：`bit0=前、bit1=后、bit2=左、bit3=右`
- 有效范围：`20 <= d <= 2000 mm`；低于 20 mm 时传感器会返回小值而不是 9999，
  所以必须显式判为无效。

车道字段：

- `laneMode`：0=关 1=NO_PAIR 2=INSUFFICIENT_DATA 3=NO_CORRIDOR
  4=TRACKING 5=DEGRADED 6=TOO_CLOSE
- `laneGroup`：-1=未选，0=Group A(前后)，1=Group B(左右)
- `nearMask/validMask/freshMask`：逻辑 S0..S3，bit0=S0；
  分别表示 `NEAR_RETURN`、最近有效回波仍在 maxAge 内、处于新鲜窗口
- `leftMm/rightMm`：当前通道对运动左/右传感器的滤波距离；-1=不可用
- `centerErrMm`：`(右墙距-左墙距)/2`，**向右为正**
- `pMms/iMms/dMms/corrMms`：横向 PD 各项与最终纠偏 [mm/s]；v2.1 默认 Ki=0
- `filterS0..S3`：四路滤波距离，顺序为逻辑 S0..S3 = **前/左/后/右**
  （注意与前 20 个原始字段的“前/后/左/右”顺序不同）
- `leftClass/rightClass`：0=UNKNOWN 1=NEAR_RETURN 2=MID_RETURN 3=FAR_RETURN；
  `FAR_RETURN` 是**有效远距回波**，不是“无墙”
- `corridorWidthMm`：两侧滤波距离 + 传感器几何宽度；`laneReason`：模式原因码
- `qualityMilli`：1.0=TRACKING，0.5=DEGRADED/TOO_CLOSE
- `pairScoreMilli`：`max(|通道法向·传感器轴|)`
- `laneFlags`：bit0 active、bit1 degraded、bit2 tooClose、bit3 pairValid、
  bit4 modeChanged、bit5 groupChanged、bit6 headingValid、bit7 corridorWidthValid、
  bit8 corridor、bit9 decay、bit10 fallbackPair
- `sideLeftRawMm/sideRightRawMm`：物理左/右（逻辑 S1/S3）的原始距离，过近时仍有值
- `pidTargetMmS`：限幅后、平滑前的控制目标；TOO_CLOSE 时是过近排斥量
- `tooCloseMask`：bit0=左过近，bit1=右过近
- `outVxMms/outVyMms/outOmegaMradS`：LaneKeeper 输出命令；omega 为透传值，
  最终航向修正后的 omega 看 `$P` 的 `tw`
- 末尾 3 个安全层字段：`safetyScaleMilli`、`safetyNearestMm`、`safetyFlags`
  （bit0 enabled、bit1 configValid、bit2 limited、bit3 stopped、bit4 headingBlocked）

v2.1 的 LaneKeeper **只输出横向 delta**，不改变巡航速度；前向 ToF 限速由
`SafetySupervisor` 在合成命令之后统一执行。默认 `enableLaneAssist=true`
是本次实车测试的临时状态，安装几何/PID 未标定时不要长期打开。

车道保持说明见 `docs/lane_keeping.md`。

固件同时以 **1 Hz** 发出端口扫描帧，用于确认测距到底插在哪个端口：

```
$S,ms,t1,t2,...,t21
```

`tN` 是第 N 号 Smart Port 的 VEXos 设备类型（`0=无设备`、`2=Motor`、`6=IMU`、
`7=Distance`）。脚本会显示：

- `端口扫描：Distance 在 7/8/9/10，接线正确`
- `端口扫描：Distance 在 ...（期望 7/8/9/10，请检查接线）`
- `端口扫描：VEXos 未发现任何 Distance 设备（检查供电/智能线/端口）`

编译并烧录，然后运行 **opcontrol（driver）模式**：

```bash
pros make && pros upload --after run
```

### 运行

```bash
python3 test/distance_monitor.py                    # 自动找 /dev/ttyACM*
python3 test/distance_monitor.py /dev/ttyACM1       # 指定串口
python3 test/distance_monitor.py /dev/ttyACM1 --csv distance.csv
python3 test/distance_monitor.py /dev/ttyACM1 --headless 10   # 无界面采集 10s
```

终端表每收到一帧就刷新，约 20 Hz。  
> `$P`（PID 调参）与 `$D`（测距）共用 User 口。如果只测距且发现刷新变慢，
> 可临时把 `diag::TelemetryConfig::enableTune` 改为 `false`
> （`include/diag/telemetry.hpp` 的默认配置），减少串口流量。

如果连串口都打不开，先列出系统识别到的端口：

```bash
python3 test/distance_monitor.py --list
```

没有输出时检查：

1. V5 用 **USB 数据线**接电脑（不是充电线）；
2. USB 插在 Brain 的 User 口，V5 已开机；
3. Linux 权限：`sudo usermod -aG dialout $USER`，重新登录；
4. `lsusb` 能看到 VEX Robotics，`pros v5 ls` 能找到端口。

状态含义：

| 显示 | 含义 |
| --- | --- |
| 有效 | 已安装且 `20 <= d <= 2000 mm` |
| 无目标 | 原始读数为 `9999` |
| 过近 | 原始读数 `< 20 mm` |
| 超量程 | 原始读数 `> 2000 mm` |
| 未连接 | 该端口没有识别到设备（类型 0） |
| 类型X | 端口插了别的设备（例如类型 2=Motor） |
| 识别异常 | VEXos 报告类型 7，但代码未按 Distance 处理 |
| 读取失败 | PROS 返回错误 |

`--csv` 保存的列：

```
ms,inst_mask,valid_mask,
front_mm,front_conf,front_size,front_vel_mm_s,front_type,
rear_mm,rear_conf,rear_size,rear_vel_mm_s,rear_type,
left_mm,left_conf,left_size,left_vel_mm_s,left_type,
right_mm,right_conf,right_size,right_vel_mm_s,right_type,
lane_mode,lane_group,near_mask,valid_mask,
left_mm,right_mm,center_err_mm,
lane_p_mm_s,lane_i_mm_s,lane_d_mm_s,lane_corr_mm_s,
out_vx_mm_s,out_vy_mm_s,out_omega_mrad_s,
filter_s0_mm,filter_s1_mm,filter_s2_mm,filter_s3_mm,fresh_mask,
left_class,corridor_width_mm,lane_reason,
quality_milli,right_class,pair_score_milli,lane_flags,
safety_scale_milli,safety_nearest_mm,safety_flags
```

### 快速验收

1. 烧录后运行 `distance_monitor.py`，确认四路都显示“有效”或“无目标”。
2. 用手/挡板依次靠近 7、8、9、10 号传感器，对应行的距离应下降。
3. 移开挡板，应显示“无目标”（9999）或超量程，而不是保持旧值。
4. 拔掉某一颗传感器，该行应显示“未连接”，其他三路不受影响。
5. 打开 `enableLaneAssist` 后，终端底部会出现车道行（模式/组/墙掩码/
   左右垂距/误差/修正）；默认关闭，标定前不要打开。

---

## 2b. 车道保持实时分析（`lane_plot.py`）

`lane_plot.py` 读取 `$D`，同时显示：

- 物理左/右原始距离（过近时仍有值）；
- 当前运动组控制侧的滤波距离；
- 墙距/过近误差；
- 横向 PD 的 P/D、控制目标、实际横向修正；
- `TOO_CLOSE` 时自动提示“正在主动远离”，此时 PD 被旁路，
  输出的是过近排斥量；LaneKeeper 不改变巡航速度；
- 同时显示安全层 `scale` 和最近障碍，便于区分“车道纠偏”和“安全减速”。

```bash
python3 test/lane_plot.py                      # 自动找串口
python3 test/lane_plot.py /dev/ttyACM1 15      # 指定串口/窗口秒数
python3 test/lane_plot.py /dev/ttyACM1 --csv lane.csv
python3 test/lane_plot.py /dev/ttyACM1 --headless 10 --csv lane.csv
```

三个曲线窗口与 `pid_tune.py` 一样支持滚轮缩放、拖动、游标和双击跟随；
无图形环境用 `--headless`。

## 3. 静摩擦 / 旋转前馈测定（`friction_test.py`）

### 原理

固件测试模式关闭航向保持，只给纯旋转 `omega` 指令，按 `0.05 rad/s²`
慢速斜坡增加；用 IMU 的角度变化 + 陀螺 Z 角速度判断车体是否真正开始旋转。
记录“刚要动”的 `omega` 指令，即为静摩擦前馈量（rad/s）。

### 运行

1. 将 `src/main.cpp` 的 `kFrictionTestEnabled` 改为 `true`；
2. `pros make && pros upload --after run`
3. 另一个终端：

```bash
python3 test/friction_test.py --timeout 200
```

测试会自动正/反方向各做 3 次，结束输出：

```text
[FRICTION] POS_MEDIAN=... rad/s (... deg/s)
[FRICTION] NEG_MEDIAN=... rad/s (... deg/s)
[FRICTION] FF_SYMMETRIC=... rad/s (... deg/s, ... mV eq)
[FRICTION] DONE
```

测完把 `kFrictionTestEnabled` 改回 `false`，重新编译上传。

### 2026-10-24 实测结果

```text
正向 trials：0.374 / 0.361 / 0.360 rad/s
负向 trials：-0.222 / -0.221 / -0.221 rad/s

POS_MEDIAN = +0.361 rad/s  (+20.68 °/s)
NEG_MEDIAN = -0.221 rad/s  (-12.66 °/s)
对称建议值 = 0.361 rad/s   (20.68 °/s, 约 610 mV 等效)
```

正反方向静摩擦不对称，建议优先使用方向相关前馈：

```text
误差 > 0（需要逆时针转）：+0.361 rad/s
误差 < 0（需要顺时针转）：-0.221 rad/s
```

若只支持一个对称常量，先用 `0.361 rad/s` 保证两个方向都能起步，
但顺时针会偏大，后续可改成方向相关值。
