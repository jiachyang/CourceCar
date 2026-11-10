/**
 * @file core/types.hpp
 * @brief 跨模块交换的公共数据类型（ARCH：公共命令结构体）。
 *
 * 这里只放**纯数据**，不依赖 PROS，也不携带电机 / 传感器对象。
 * 各模块之间的数据流通过本文件的类型 + 各自模块的状态结构体完成。
 */
#pragma once

namespace core {

/**
 * @brief 车体坐标系三自由度速度命令。
 *
 * 单位与正方向（与 docs 坐标系约定一致，CS-001~CS-008）：
 *   - vx    ：m/s，正方向 = 车头（+X）
 *   - vy    ：m/s，正方向 = 车体左侧（+Y）
 *   - omega ：rad/s，正方向 = 俯视逆时针（绕 +Z）
 *
 * 该结构体只表达“运动目标”，不包含任何底盘实现信息（轮数、布置、IK）。
 * Radio 负责产生它，Chassis 负责消费它，二者互不认识对方。
 */
struct MotionCommand {
    double vx = 0.0;     ///< 前后速度 [m/s]
    double vy = 0.0;     ///< 左右速度 [m/s]
    double omega = 0.0;  ///< 自转角速度 [rad/s]

    /// 是否为零命令（安全停机判据用）。
    bool isZero() const { return vx == 0.0 && vy == 0.0 && omega == 0.0; }
};

/// 零运动命令：失联 / 超时 / disabled 时统一下发它。
inline MotionCommand zeroMotion() { return MotionCommand{}; }

/**
 * @brief 航向状态（任意航向源的公共契约）。
 *
 * 生产者：IMU（当前）、未来的里程计 / GPS / 双天线。
 * 消费者：Chassis 的航向保持。
 *
 * 只含控制所需的量，不含设备诊断字段；因此底盘不需要知道航向从哪来，
 * 换航向源时不必改 Chassis，也不必包含 imu 模块的头文件。
 *
 * 单位：rad / rad/s，逆时针为正；rotationRad 为连续累计角（不归一化）。
 */
struct HeadingState {
    double rotationRad = 0.0;                    ///< 连续累计旋转角 [rad]
    double angularVelocityRadPerSec = 0.0;       ///< 绕 Z 轴角速度 [rad/s]
    bool valid = false;                          ///< 本帧数据是否有效
};

}  // namespace core
