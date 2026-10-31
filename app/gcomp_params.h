/**
 * gcomp_params.h — 重力补偿参数（**手改**，改完重新编译烧录）
 *
 * 结构和 ros2_gong 的 gravity_comp.yaml 一致：一行一个轴，改哪个动哪个。
 *
 *   τ_motor = dir · g(q) · GCOMP_GAIN[i] · GCOMP_SCALE · ramp
 *   MIT 帧  : p_des = 使能瞬间锁存的角度（只有 kp>0 时有意义）
 *             v_des = 0，kp = GCOMP_KP[i]，kd = GCOMP_KD[i]，t_ff = τ_motor
 *
 * 改完执行：bash tools/flash.sh
 */
#ifndef GCOMP_PARAMS_H
#define GCOMP_PARAMS_H

#include <stdbool.h>

/* ---- 全局 ---- */
#define GCOMP_SCALE 1.0f        /* 重力补偿总量，先别动，用 GAIN 调单轴 */
#define GCOMP_LIMIT_NM 6.0f     /* 单轴力矩上限 N·m（实测全臂峰值约 3.6，必须大于它） */
#define GCOMP_RAMP_MS 4000u     /* 力矩从 0 渐入的毫秒数，避免上电"猛地一撑" */
#define GCOMP_PERIOD_MS 10u     /* 力矩重算周期：10 ms = 100 Hz（与 ros2_gong 一致） */

/* ---- 重力补偿系数 ----
 *   某轴「拖动时下坠」→ 加大（1.1 / 1.2 / 1.3 …）
 *   某轴「自己往上飘」→ 减小（0.9 / 0.8 …）
 *   顺序固定：J0 J1 J2 J3 J4 J5 */
static const float GCOMP_GAIN[6] = {1.0f, 1.45f, 1.0f, 1.0f, 1.0f, 1.0f};

/* ---- 位置刚度 N·m/rad ----
 *   0    = 不做位置控制（零重力示教，推哪停哪）
 *   3~10 = 松手会回到 p_des（= 使能瞬间锁存的角度） */
static const float GCOMP_KP[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* ---- 阻尼 N·m/(rad/s) ----
 *   0    = 无阻尼（会缓慢漂）
 *   0.5~1.5 = 能刹住，拖动手感适中 */
static const float GCOMP_KD[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

/* ---- 参与补偿的轴 ----
 *   false 的轴既不补偿也不保持（力矩恒 0） */
static const bool GCOMP_ACTIVE[6] = {true, true, true, true, true, true};

#endif /* GCOMP_PARAMS_H */
