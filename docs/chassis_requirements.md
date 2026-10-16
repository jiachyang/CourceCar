# VEX V5 四全向轮 X-Drive 底盘 — 需求检查清单

> 状态：**需求已确认（rev.3）**，待用户批准进入设计/实现。
> 对齐基准：本文件是后续需求修改的**唯一基准**。修改时按 ID 更新，不要重述整套系统。
> 字段：`ID` / `需求` / `当前值` / `状态` / `来源` / `待确认`
> 状态定义：`CONFIRMED`（用户明确或文档可靠定义）/ `INFERRED`（推导或经验，未确认）/ `UNKNOWN`（缺信息）/ `CONFLICT`（来源冲突）

> **修订记录**
> - rev.1：初版需求清单。
> - rev.2：并入第一批回答（轮径 10 cm、直驱、200 rpm、IMU 端口 5、20 ms、PID…）。
> - rev.3：并入 §M 全部回答，**所有问题已闭环**。本阶段范围＝底盘驱动 + IMU 航向**框架**（链路打通、默认关闭、运行时开关）。

## 0. 资料与来源索引

| 代号 | 来源 |
| --- | --- |
| README | 本项目 `README`（任务书） |
| docs-底盘 | `docs/底盘数据.md` |
| docs-电机 | `docs/电机数据-P1空载-P3装轮.md` |
| docs-测距 | `docs/测距仪数据-P2.md` |
| 图片 | `docs/image/底盘数据/1792127597173.png`（俯视示意图，已确认即实物布局） |
| 用户 | 用户在 `待确认` 列与 §M 的直接回答 |
| 推导 | 由 README/docs/用户输入计算得出 |

---

## A. 工程与环境（PRJ）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| PRJ-001 | 主控为 VEX V5 Brain | V5 Brain | CONFIRMED | README | — |
| PRJ-002 | 开发框架为 PROS | PROS | CONFIRMED | README | — |
| PRJ-003 | 开发语言为 C++ | C++ | CONFIRMED | README | — |
| PRJ-004 | 开发/存放的 PROS 工程位置 | **在 `Car/` 下新建 PROS 工程** | CONFIRMED | 用户 Q-09 | — |
| PRJ-005 | 使用的 PROS Kernel / CLI 版本 | CLI 3.5.6；参考工程用 Kernel 4.2.2 | INFERRED | 现场检查 / docs-电机 | 目标是否固定 4.2.2 |
| PRJ-006 | 是否复用/参考现有 `mazecar` 工程 | **可参考，但代码须符合现要求** | CONFIRMED | 用户 Q-09 | — |
| PRJ-007 | 版本管理 | **本地 Git 仓库即可**（已在 `Car/` 初始化并提交） | CONFIRMED | 用户 Q-10 | — |
| PRJ-008 | 本阶段实现范围 | **底盘驱动 + IMU 航向框架**；IMU→底盘链路打通但默认关闭；每模块可单独启用 | CONFIRMED | 用户 Q-07 | — |

## B. 硬件（HW）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| HW-001 | 驱动电机数量 | 4 台 VEX 电机 | CONFIRMED | README | — |
| HW-002 | 驱动电机端口 | Smart Port 1～4 | CONFIRMED | README | — |
| HW-003 | 端口号与车轮编号的物理映射 | **1=FL(左上)、2=RL(左下)、3=FR(右上)、4=RR(右下)** | CONFIRMED | 用户 Q-01 | — |
| HW-004 | 姿态传感器 | VEX Inertial Sensor（IMU） | CONFIRMED | README | — |
| HW-005 | IMU 端口 | Smart Port 5 | CONFIRMED | 用户 | 安装朝向待实测，见 IMU-019 |
| HW-006 | 测距传感器 | P2 一个 V5 Distance Sensor | CONFIRMED | docs-测距 | 本阶段不使用，仅登记 |
| HW-007 | 电机齿箱配置 | 四台与 docs 台账一致 = green / 200 rpm | CONFIRMED | 用户 / docs-电机 | — |
| HW-008 | 齿箱「配置值」与「实物颜色」是否一致 | **暂按 docs 台账 green/200 计算**，实物颜色日后核对 | CONFIRMED | 用户 Q-02 | 日后可补 |
| HW-009 | 电机上报 gearing / reversed | 四台同一型号（gearing=1，reversed=0） | CONFIRMED | 用户 / docs-电机 | — |

## C. 机械结构（ME）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| ME-001 | 底盘为正方形金属框架 | 是 | CONFIRMED | README | — |
| ME-002 | 四条边各装一个普通全向轮 | 是 | CONFIRMED | README | — |
| ME-003 | 车轮近似位于所在边中点 | 先按**严格中点**计算 | CONFIRMED | 用户 | — |
| ME-004 | 框架尺寸 | 25.5 cm × 25.5 cm | CONFIRMED | docs-底盘 / README | — |
| ME-005 | 25.5 cm 指正方形外缘边长 | **外缘边长 25.5 cm** | CONFIRMED | 用户 Q-03 | — |
| ME-006 | 车轮中线相对框架边缘外伸 | 2 cm | CONFIRMED | docs-底盘 / README | — |
| ME-007 | 车轮实际直径 / 半径 | **直径 10 cm → 半径 5 cm = 0.05 m** | CONFIRMED | 用户 | — |
| ME-008 | 电机是否直驱车轮 / 传动比 | **直驱，传动比 1:1** | CONFIRMED | 用户 | — |
| ME-009 | 电机在框架内、轮在框架外 | 是 | CONFIRMED | README | — |
| ME-010 | 车轮主动滚动方向平行于所在框架边 | 是 | CONFIRMED | 用户 | — |
| ME-011 | 相邻车轮主动滚动方向互相垂直 | 是 | CONFIRMED | README | — |
| ME-012 | 全向轮被动滚子允许侧向被动滚动 | 是 | CONFIRMED | README | — |
| ME-013 | 车头方向 | 原始照片左上顶角 | CONFIRMED | README | — |
| ME-014 | 纵向轴 | 沿前后对角线（非某条边） | CONFIRMED | README | — |
| ME-015 | 底盘中心→轮心距离 R | 14.75 cm（=25.5/2+2，推导） | CONFIRMED | 推导 | — |
| ME-016 | 车体坐标下轮心横坐标 a | 10.43 cm（=R/√2，推导） | CONFIRMED | 推导 | — |
| ME-017 | 四轮理想轮心坐标 | **FL(+10.43,+10.43)/FR(+10.43,−10.43)/RL(−10.43,+10.43)/RR(−10.43,−10.43)** | CONFIRMED | 用户 Q-03 / 推导 | — |
| ME-018 | 轮子编号命名 | FL 左前 / FR 右前 / RL 左后 / RR 右后 | CONFIRMED | 用户 | — |
| ME-019 | 实物照片与示意图的关系 | 实物就是示意图的样子 | CONFIRMED | 用户 | — |
| ME-020 | 原始实物照片文件 | 仓库中缺失，以示意图为准 | CONFIRMED | 用户 | — |

## D. 坐标系（CS）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| CS-001 | 坐标系类型 | 右手车体坐标系 | CONFIRMED | README | — |
| CS-002 | 原点 | 底盘几何中心 | CONFIRMED | README | — |
| CS-003 | X 轴 | 沿车头对角线向前 | CONFIRMED | README | — |
| CS-004 | Y 轴 | 车体左侧 | CONFIRMED | README | — |
| CS-005 | Z 轴 | 竖直向上 | CONFIRMED | README | — |
| CS-006 | vx 正方向 | 车头方向（前） | CONFIRMED | README | — |
| CS-007 | vy 正方向 | 车体左侧（左） | CONFIRMED | README | — |
| CS-008 | omega 正方向 | 俯视逆时针 | CONFIRMED | README | — |
| CS-009 | 运动控制坐标系 | 仅车体坐标系，不要求世界坐标系 | CONFIRMED | README | — |

## E. 运动学（KIN）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| KIN-001 | 一般轮速关系 | `wheel_speed_i = d_i·(v + ω×r_i) / r_wheel` | CONFIRMED | README | — |
| KIN-002 | 必须给出四轮位置向量 r_i | 见设计文档 | CONFIRMED（设计产出） | README | — |
| KIN-003 | 必须给出四轮主动滚动方向 d_i | 见设计文档 | CONFIRMED（设计产出） | README | — |
| KIN-004 | 每轮「正转方向」定义 | **站在车头正前方看向车体，顺时针为正转** | CONFIRMED | 用户 Q-04 | — |
| KIN-005 | 必须给出 4×3 逆运动学矩阵 | 见设计文档 | CONFIRMED（设计产出） | README | — |
| KIN-006 | 轮角速度→电机目标转速换算 | 直驱 1:1 → 电机 RPM = 轮 RPM | CONFIRMED | 用户 / 推导 | — |
| KIN-007 | 轮速上限 | **200 rpm**（超出按比例缩放，见 T-07） | CONFIRMED | 用户 | — |
| KIN-008 | 几何模型与电机反向配置分离管理 | 要求 | CONFIRMED | README | — |
| KIN-009 | 实际安装误差处理方式 | 用可设定变量 + 偏置项解决 | CONFIRMED | 用户 | 标定项设计阶段定 |
| KIN-010 | 不得直接套用网上 X-Drive 公式 | 须按真实布置推导 | CONFIRMED | README | — |

## F. 控制接口（CTRL）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| CTRL-001 | 对外统一三自由度接口 | `drive(vx, vy, omega)` | CONFIRMED | README | — |
| CTRL-002 | 三分量可同时输入 | 是 | CONFIRMED | README | — |
| CTRL-003 | 支持前后 / 左右 / 原地旋转 / 平移+旋转 | 是 | CONFIRMED | README | — |
| CTRL-004 | 上层不关心四轮具体转速 | 是 | CONFIRMED | README | — |
| CTRL-005 | 接口物理单位 | **m/s、m/s、rad/s** | CONFIRMED | 用户 | — |
| CTRL-006 | 输入范围 | 上限与电机物理极限（200 rpm）一致 | CONFIRMED | 用户 | 换算见附录 |
| CTRL-007 | 控制/调用周期 | **20 ms**（50 Hz） | CONFIRMED | 用户 | — |

## G. 驱动与电机控制（DRV）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| DRV-001 | 优先使用 PROS 电机速度控制 API | 是 | CONFIRMED | 用户 | — |
| DRV-002 | 电机型号/档位 | green 200 rpm | CONFIRMED | 用户 / docs-电机 | — |
| DRV-003 | 是否直驱 / 是否有齿轮传动 | 直驱 | CONFIRMED | 用户 | — |
| DRV-004 | 目标轮速→电机 RPM 换算 | 1:1（电机 RPM = 轮 RPM） | CONFIRMED | 用户 / 推导 | — |
| DRV-005 | 电机最大转速 | 名义/指令上限 200 rpm；空载实测 12 V≈244 rpm，装轮≈231 rpm | CONFIRMED | 用户 / docs-电机 | 用于饱和限幅 |
| DRV-006 | 是否需要加减速限制 | 需要 | CONFIRMED | 用户 | 具体斜率设计阶段定 |
| DRV-007 | 停止时制动模式 | **先选 brake** | CONFIRMED | 用户 Q-05 | — |
| DRV-008 | 电机故障 / 掉线处理 | 给出错误日志并停止 | CONFIRMED | 用户 | — |
| DRV-009 | 四轮阻力/死区差异 | 需逐轮标定 | CONFIRMED | 用户 / docs-电机 | — |
| DRV-010 | 刹车逻辑**独立成函数** | 便于日后替换为自己实现的刹车 | CONFIRMED | 用户 Q-05 | — |

## H. IMU 与航向控制（IMU）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| IMU-001 | 用 VEX IMU 估计航向 | 是 | CONFIRMED | README | — |
| IMU-002 | 目标：减小平移时的非预期旋转 | 是 | CONFIRMED | README | — |
| IMU-003 | IMU 安装位置 | 底盘几何中心附近 | CONFIRMED | README | — |
| IMU-004 | IMU 初始化与校准 | 要求实现 | CONFIRMED | README | 校准超时/失败处理设计阶段定 |
| IMU-005 | 航向角读取 | 要求实现 | CONFIRMED | README | 读取周期设计阶段定 |
| IMU-006 | 角度归一化 | 要求实现 | CONFIRMED | README | — |
| IMU-007 | 航向误差计算 | 要求实现 | CONFIRMED | README | — |
| IMU-008 | 航向闭环控制 | 要求实现 | CONFIRMED | README | — |
| IMU-009 | 传感器异常处理 | 要求实现 | CONFIRMED | README | 无效/跳变判据设计阶段定 |
| IMU-010 | 控制器形式 | **直接使用 PID** | CONFIRMED | 用户 | 参数设计阶段定 |
| IMU-011 | 目标航向的产生方式 | 上电后等 5 s，读取当前航向并锁存为目标 | CONFIRMED | 用户 | — |
| IMU-012 | 情况 A：无主动旋转时保持航向 | 要求 | CONFIRMED | README | — |
| IMU-013 | 情况 B：主动旋转时不被航向锁定抵消 | 要求 | CONFIRMED | README | — |
| IMU-014 | 情况 C：平移+旋转同时生效 | 要求 | CONFIRMED | README | — |
| IMU-015 | 主动旋转与航向保持的切换策略 | **外部 omega 指令 ≠ 0 时暂停航向保持；= 0 时恢复** | CONFIRMED | 用户 Q-06 | — |
| IMU-016 | 切换时避免速度突变 | 要求 | CONFIRMED | README | 平滑方式设计阶段定 |
| IMU-017 | 角度跨越 0/360° 处理 | 用最短角差 | CONFIRMED | 用户 | — |
| IMU-018 | 修正角速度如何叠加到 IK 输入 | PID 输出的修正角速度叠加到 omega | CONFIRMED | 用户 | 限幅方式设计阶段定 |
| IMU-019 | IMU 端口与安装朝向 | 端口 5；安装朝向待实测（目前仅框架） | CONFIRMED（朝向待实测） | 用户 | 实测后补 |
| IMU-020 | 不要求 EKF、不要求 IMU 积分定位 | 是 | CONFIRMED | README | — |
| IMU-021 | 实现「IMU→航向修正→底盘」完整链路但默认关闭 | 因 IMU 未测未装，先打通链路、运行时开关默认关，不影响底盘 | CONFIRMED | 用户 Q-07 | — |

## I. 状态估计边界（EST）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| EST-001 | 明确区分航向/里程计/位置/多传感器融合 | 要求 | CONFIRMED | README | — |
| EST-002 | 当前仅实现 IMU 航向读取与修正 | 是 | CONFIRMED | README | — |
| EST-003 | 里程计（编码器位置） | 后续目标 | CONFIRMED | README | 本阶段不做 |
| EST-004 | 测距仪融合 / 位置估计 | 后续目标 | CONFIRMED | README | 本阶段不做 |

## J. 软件架构（ARCH）

| ID | 需求 | 当前值 | 状态 | 来源 | 待确认 |
| --- | --- | --- | --- | --- | --- |
| ARCH-001 | 建议模块：Kinematics/Drive/Imu/HeadingController/PoseEstimator | 建议，非强制 | CONFIRMED | README | 是否全部创建 |
| ARCH-002 | 运动学不直接访问硬件 | 原则 | CONFIRMED | README | — |
| ARCH-003 | 硬件控制不承担路径规划 | 原则 | CONFIRMED | README | — |
| ARCH-004 | 航向闭环与底盘速度指令关系明确 | 原则 | CONFIRMED | README | — |
| ARCH-005 | 硬件参数集中管理 | 原则 | CONFIRMED | README | — |
| ARCH-006 | 后续加激光测距不需重写 IK | 原则 | CONFIRMED | README | — |
| ARCH-007 | 在可扩展前提下尽量最小实现 | 偏好 | CONFIRMED | README | — |
| ARCH-008 | 是否真的需要拆分全部建议类 | 待分析 | UNKNOWN | README | 设计阶段论证 |
| ARCH-009 | 每个模块可**单独启用/停用** | **运行时开关**（不改代码、不重编译） | CONFIRMED | 用户 Q-07 | — |
| ARCH-010 | `main` 中提供集中的「真值配置表」 | **只放整车级内容**（模块开关等）；**端口、传动比等放各模块内部** | CONFIRMED | 用户 Q-08 | — |

## K. 测试与验收（TEST）

| ID | 测试 | 预期行为 | 状态 | 来源 |
| --- | --- | --- | --- | --- |
| T-01 | vx > 0 | 向车头方向前进 | CONFIRMED（目标） | README |
| T-02 | vx < 0 | 向后移动 | CONFIRMED（目标） | README |
| T-03 | vy > 0 | 向左平移 | CONFIRMED（目标） | README |
| T-04 | vy < 0 | 向右平移 | CONFIRMED（目标） | README |
| T-05 | omega > 0 | 逆时针旋转 | CONFIRMED（目标） | README |
| T-06 | 三输入同时非零 | 平移+旋转同时执行 | CONFIRMED（目标） | README |
| T-07 | 轮速达上限 | 合理限幅（200 rpm） | CONFIRMED（目标） | README / 用户 |
| T-08 | 直线运动出现偏航 | IMU 闭环减小偏航 | CONFIRMED（目标） | README |
| T-09 | 主动旋转 | 航向保持不阻止旋转 | CONFIRMED（目标） | README |
| T-10 | IMU 未完成校准 | 不使用无效航向数据 | CONFIRMED（目标） | README |
| T-11 | 电机方向测试 | 各轮符合运动学定义 | CONFIRMED（目标） | README |
| T-12 | PROS 编译 | 编译成功 | CONFIRMED（目标） | README |

> 验收须区分：理论计算验证 / 单元测试 / 编译验证 / 真实硬件验证。编译通过 ≠ 底盘能正确行驶。

## L. 后续扩展（EXT）

| ID | 需求 | 当前值 | 状态 | 来源 |
| --- | --- | --- | --- | --- |
| EXT-001 | 轮式里程计 | 后续 | CONFIRMED | README |
| EXT-002 | 测距仪融合 | 后续（P2 传感器已存在） | CONFIRMED | README / docs-测距 |
| EXT-003 | 位置估计（x/y） | 后续 | CONFIRMED | README |

---

## 附：本清单推导出的关键参数（供校对，不作为独立测量）

| 量 | 值 | 依据 |
| --- | --- | --- |
| 中心→轮心 R | 14.75 cm | R = 25.5/2 + 2 |
| 轮心横坐标 a | 10.43 cm | a = R/√2 |
| 轮心坐标 | FL(+a,+a)、FR(+a,−a)、RL(−a,+a)、RR(−a,−a) | ME-017 |
| 车轮半径 r_wheel | **0.05 m**（直径 10 cm） | ME-007 |
| 满速轮缘线速度 | 200 rpm × 2π/60 × 0.05 ≈ **1.047 m/s** | DRV/KIN-007 推导 |
| 纯平移最大速度 | ≈ **1.05 m/s**（单轮先饱和） | 推导 |
| 纯旋转最大角速度 | ≈ 1.047 / 0.1475 ≈ **7.10 rad/s** | 推导 |

## 附录：识别到的冲突与缺口（汇总）

| 编号 | 类型 | 内容 | 关联 ID | 状态 |
| --- | --- | --- | --- | --- |
| X-01 | 缺口 | 原始照片缺失 → 以示意图为准 | ME-019 / ME-020 | **已解决** |
| X-02 | 未知 | 齿箱配置值与实物颜色 → 暂按 docs green/200 | HW-008 | **已解决（暂定）** |
| X-03 | 未知 | 端口↔轮、轮子正转方向 → 已定；IMU 朝向待实测 | HW-003 / KIN-004 / IMU-019 | **基本解决** |
| X-04 | 未知 | 轮半径与传动比 → 0.05 m、直驱 | ME-007 / ME-008 | **已解决** |
| X-05 | 待确认 | 本阶段是否含 IMU → 含框架，默认关闭 | PRJ-008 / IMU-021 | **已解决** |

---

# M. 问答记录（已完成，rev.3 全部闭环）

> 以下保留你的原始回答，便于追溯。上表已并入结论。

### Q-01 端口↔轮的读法确认（HW-003 / X-03）
我的回答：对 → **1=左上/F、2=左下/RL、3=右上/FR、4=右下/RR**

### Q-02 齿箱「配置值」与「实物颜色」（HW-008 / X-02）
我的回答：先按目前实测的 docs 中的电机计算 → **暂用 green/200**

### Q-03 几何与单位细节确认（ME-005 / ME-017 / CTRL-005）
我的回答：对 轮心坐标表 → **外缘边长、坐标表、rad/s 均确认**

### Q-04 轮子「正转」的定义（KIN-004）
我的回答：站在车的正面 顺时针为正 → **以车头正前方视角，顺时针为正转**

### Q-05 停止时的制动模式（DRV-007）
我的回答：先选择 brake；注意这个单独写函数，以后可能替换为自己的刹车 → **brake + DRV-010**

### Q-06 主动旋转与航向保持的切换依据（IMU-015）
我的回答：同意 → **外部 omega≠0 暂停航向保持**

### Q-07 模块单独启用的方式 & 本阶段是否含 IMU（ARCH-009 / PRJ-008 / X-05）
我的回答：底盘驱动 + IMU 框架；方位后续改参数；IMU→底盘链路要打通但当前关掉（IMU 未测未装）；模块化不影响；开关在运行时 → **PRJ-008 / ARCH-009 / IMU-021**

### Q-08 真值配置表包含哪些字段（ARCH-010）
我的回答：端口、传动比属模块内部，不放 main；模块开关等整车级内容放 main → **ARCH-010**

### Q-09 PROS 工程落点与是否复用 mazecar（PRJ-004 / PRJ-006）
我的回答：新建 PROS 工程；可参考 mazecar 但要符合现要求 → **PRJ-004 / PRJ-006**

### Q-10 GitHub 仓库信息（PRJ-007）
我的回答：本地就好 → **仅本地 Git，不推远程**

---

### rev.3 变更摘要

- 并入 §M 全部回答，**所有 UNKNOWN/CONFLICT 均已闭环**。
- 新增需求：**DRV-010**（刹车独立成函数）、**IMU-021**（IMU→底盘链路打通但默认关闭）。
- 确认：端口映射、轮心坐标、外缘边长、正转定义、brake、切换判据、运行时开关、真值表字段划分、新建 PROS 工程、仅本地 Git。
- 下一阶段：**方案设计**（机械坐标模型 / IK 推导 / IMU 控制 / 软件接口）→ 用户批准 → 代码实现。
