# 05 PID 限幅、内部量回传、以及一次"死区"误改

## 5.1 误改：硬误差死区造成 0↔130 颤振（已回退）

**背景**：静止时航向 PID 输出很小但持续（`tw≈−45 mrad/s`），想让它归零。

**做法**：加了硬死区——`|err| < 0.02 rad` 时输出 0 并泄放积分。

**后果**：输出在死区边界**硬切**：
- 死区内 `tw=0`；
- 一越过 `|err|=0.02`，输出瞬间跳到 `Kp·err + Ki·integ ≈ 130 mrad/s`。

出现 **0 ↔ 130 的跳变/颤振**（典型的硬死区限环）。

**结论**：**已删除**。若要死区应做"软死区"（`err_eff = err − clamp(err,±db)`，
边界连续），或干脆不用死区、靠整定。

---

## 5.2 四项限幅（P / I / D / 总输出）

**需求**：给 PID 三项和总输出都加限幅。

`include/chassis/chassis_config.hpp`：

```cpp
inline constexpr double kHeadingIntegralLimit = 0.5;   // 积分“状态”限幅 [rad·s]
inline constexpr double kHeadingPLimit = 2.0;          // P 项输出限幅 [rad/s]
inline constexpr double kHeadingILimit = 1.0;          // I 项输出限幅 [rad/s]
inline constexpr double kHeadingDLimit = 1.0;          // D 项输出限幅 [rad/s]
inline constexpr double kMaxHeadingCorrectionRadPerSec = 2.0;  // 总输出限幅
```

`HeadingController::update()` 里逐项 clamp：

```cpp
p = clamp(Kp*err,    ±pLimit_);
i = clamp(Ki*integ_, ±iLimit_);
d = clamp(Kd*deriv,  ±dLimit_);
corr = clamp(p + i + d, ±maxCorrection_);
```

`Chassis::init` 用 `heading_.setTermLimits(...)` 注入；`HeadingController::Debug`
把 **Kp/Ki/Kd、各限幅、err/P/I/D/corr** 一并快照，供回传。

**测试**：新增"P/I/D 单项限幅 + 输出限幅"用例。

---

## 5.3 PID 全量回传

`ChassisState` 里放 `HeadingController::Debug heading;`，`Chassis::update` 填
`state_.heading = heading_.debug();`。

`$P` 新增字段：`kp,ki,kd,intLim,pLim,iLim,dLim,outLim`，
配合已有的 `hP,hI,hD,hcorr,herr`，可**同屏对照"实时值 vs 各自限幅"**。

---

## 5.4 关于"积分死区"

**结论：当前 I 项没有积分死区。** 积分 `integ_ += err·dt`（限幅 ±0.5）——
只要有误差就累加，没有"小误差不积分"的门限。全工程的 `deadband` 只在**输入侧**
（摇杆死区）与 IMU 诊断（漂移门限）出现，与积分无关。

---

## 5.5 观察到的真实问题

用 Kp=5 / Ki=1 / Kd=0.5 时，数据里出现**持续 ~2Hz、±90 mrad 的振荡**，
`hI` 恒在 ±500（积分饱和）、`hD` 峰值 ±600、`hcorr` 顶到 ±2000。
→ 增益过激 + 积分饱和 → **极限环**。后把参数降为 **Kp=0.5 / Ki=0 / Kd=1.0**。
