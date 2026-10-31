/**
 * gravity.h — 静态重力矩（关节侧）
 *
 * 与 ros2_gong/tools/gravity_comp.py 的 Pinocchio computeGeneralizedGravity 同源：
 * 参数由 tools/gen_gravity_model.py 从 URDF 生成，算法在 test/test_gravity.c 里
 * 用 Pinocchio 的 40 组采样对拍过。
 */
#ifndef GRAVITY_H
#define GRAVITY_H

#include "gravity_model.h"

/* 输入关节角 q（rad，URDF 关节侧），输出各关节需要施加的重力矩（N·m，关节侧） */
void gravity_torque(const float q[GM_JOINTS], float tau[GM_JOINTS]);

/* 电机侧直通版：q_joint = dir*q_motor + offset，tau_motor = dir*tau_joint */
void gravity_torque_motor(const float q_motor[GM_JOINTS], float tau_motor[GM_JOINTS]);

/* 标定换算：q_joint = dir*q_motor + offset；q_motor = dir*(q_joint − offset) */
void gravity_motor_to_joint(const float q_motor[GM_JOINTS], float q_joint[GM_JOINTS]);
void gravity_joint_to_motor(const float q_joint[GM_JOINTS], float q_motor[GM_JOINTS]);

#endif /* GRAVITY_H */
