/**
 * @file lane/geometry.hpp
 * @brief 车道保持的纯几何适配：世界坐标方向 <-> 车体坐标方向。
 *
 * 为什么需要它：
 *   LaneKeeper 的组选择以**车体坐标运动方向**为输入，传感器轴也在车体坐标中定义。
 *   若全局路径规划给出的是世界坐标方向，必须先按当前车体朝向转到车体坐标，
 *   否则“车头前进”和“世界向北”会被混为一谈。
 *
 * 约定：
 *   - `headingRad` 是**车体 +X 在世界坐标中的方向角**，逆时针为正；
 *   - 世界坐标与 IMU 的零点必须已对齐（例如上电时车头对准世界 +X）；
 *   - 本文件不读取 IMU，只做旋转，便于主机测试。
 */
#pragma once

#include <cmath>

#include "core/types.hpp"

namespace lane {

/// 世界速度（vx,vy,omega）-> 车体速度（vx,vy,omega）。
inline core::MotionCommand worldVelocityToBody(
    const core::MotionCommand& world, double headingRad) {
    const double c = std::cos(headingRad);
    const double s = std::sin(headingRad);
    core::MotionCommand body{};
    body.vx = world.vx * c + world.vy * s;
    body.vy = -world.vx * s + world.vy * c;
    body.omega = world.omega;  // 绕 Z 的角速度在旋转下不变
    return body;
}

/// 车体速度 -> 世界速度（与上式互逆，供测试/诊断）。
inline core::MotionCommand bodyVelocityToWorld(
    const core::MotionCommand& body, double headingRad) {
    const double c = std::cos(headingRad);
    const double s = std::sin(headingRad);
    core::MotionCommand world{};
    world.vx = body.vx * c - body.vy * s;
    world.vy = body.vx * s + body.vy * c;
    world.omega = body.omega;
    return world;
}

}  // namespace lane
