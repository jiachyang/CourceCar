# 02 IMU 就绪判定（关键坑）

## 现象

开了 IMU 与航向保持后，遥测里 **`imu=0` 一直无效**，
航向环因为"IMU 无效"永远不进 PID，标定完也不锁目标。

## 定位

`src/modules/imu.cpp` 原来用：

```cpp
if (status != pros::ImuStatus::ready) { state_.valid = false; return; }
```

但实测 `device_->get_status()` 的返回值是 **19（校准中）/ 1（校准中）/ 18（校准后）**，
而头文件枚举是 `ready=0 / calibrating=19 / error=0xFF`。
**校准后返回的是 18，永远不等于 0** → 永远判为"未就绪"。

（C API 里 `E_IMU_STATUS_CALIBRATING = 1`，是按位用的，和 C++ 枚举不一致。）

## 解决

参考工程 `/home/jc/VEX/moudleTest/test-gyro-port3` 的同内核结论：
**就绪判定以 `is_calibrating()` 为准**。

```cpp
state_.calibrating = device_->is_calibrating();
if (!device_->is_installed() || raw == kImuStatusError || state_.calibrating) {
    state_.valid = false; ...
}
```

同时修了两个相关问题：

1. **跳变保护基线复位**：只要 `valid=false`（未校准/未插/错误）就把 `hasLastRotation_=false`，
   避免校准复位后基线过期、永久卡在"跳变"无效。跳变那一帧也把基线移到新值，防止死锁。
2. **刷新率**：就绪后 `device_->set_data_rate(5)`（设备侧最小 5ms；VEXos 缓存仍 ~10ms）。

## 验证

遥测状态流转：`st=0 → st=1(校准中) → st=18(就绪)`，`imu=1` 持续，5s 后 `hdg=1`。

## 备注

- IMU 刷新实测 ~10ms，20ms 控制周期够用；`get_rotation()` 连续、无漂移，适合做航向。
- 读回 `angularVelocityRadPerSec` 在本工程**不参与控制**（只做诊断）。
