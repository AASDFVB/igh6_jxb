/**
 * gravity.c — 静态重力矩，纯 C，不依赖 HAL（可在 PC 上编译测试）
 *
 * 串链 + 六个转动关节，重力是唯一外力：
 *
 *   τ_i = − Σ_{k≥i} axis_i · ( (p_k − p_i) × m_k·g )
 *
 * 推导：势能 V = Σ m_k·9.81·z_k，τ_i = ∂V/∂q_i。负号来自"力矩需要抵消重力"。
 * 与 Pinocchio 的 computeGeneralizedGravity 同号（已对拍，见 test/test_gravity.c）。
 */
#include "gravity.h"

#include <math.h>
#include <string.h>

static void mul3(const float a[9], const float b[9], float out[9])
{
    float t[9];
    for (int r = 0; r < 3; r++) {
        for (int c = 0; c < 3; c++) {
            t[r * 3 + c] = a[r * 3 + 0] * b[0 * 3 + c] +
                           a[r * 3 + 1] * b[1 * 3 + c] +
                           a[r * 3 + 2] * b[2 * 3 + c];
        }
    }
    memcpy(out, t, sizeof(t));
}

static void mulv3(const float R[9], const float v[3], float out[3])
{
    const float x = R[0] * v[0] + R[1] * v[1] + R[2] * v[2];
    const float y = R[3] * v[0] + R[4] * v[1] + R[5] * v[2];
    const float z = R[6] * v[0] + R[7] * v[1] + R[8] * v[2];
    out[0] = x;
    out[1] = y;
    out[2] = z;
}

/* R = I + sin(q)·K + (1−cos q)·K²，K 是轴的单位反对称矩阵 */
static void rot_axis(const float axis[3], float q, float R[9])
{
    const float s = sinf(q);
    const float c = 1.0f - cosf(q);
    const float x = axis[0], y = axis[1], z = axis[2];
    const float K[9] = {0.0f, -z, y,
                        z, 0.0f, -x,
                        -y, x, 0.0f};
    float K2[9];
    mul3(K, K, K2);
    for (int i = 0; i < 9; i++) {
        R[i] = (i % 4 == 0 ? 1.0f : 0.0f) + s * K[i] + c * K2[i];
    }
}

void gravity_torque(const float q[GM_JOINTS], float tau[GM_JOINTS])
{
    float R[9] = {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f};
    float p[3] = {0.0f, 0.0f, 0.0f};
    float pj[GM_JOINTS][3];
    float axw[GM_JOINTS][3];
    float pc[GM_JOINTS][3];

    for (int i = 0; i < GM_JOINTS; i++) {
        float d[3], Rq[9], c[3];
        mulv3(R, GM_ORIGIN_T[i], d);
        p[0] += d[0];
        p[1] += d[1];
        p[2] += d[2];
        mul3(R, GM_ORIGIN_R[i], R);

        pj[i][0] = p[0];
        pj[i][1] = p[1];
        pj[i][2] = p[2];
        mulv3(R, GM_AXIS[i], axw[i]);

        rot_axis(GM_AXIS[i], q[i], Rq);
        mul3(R, Rq, R);

        mulv3(R, GM_COM[i], c);
        pc[i][0] = p[0] + c[0];
        pc[i][1] = p[1] + c[1];
        pc[i][2] = p[2] + c[2];
    }

    for (int i = 0; i < GM_JOINTS; i++) {
        float sum = 0.0f;
        for (int k = i; k < GM_JOINTS; k++) {
            const float r[3] = {pc[k][0] - pj[i][0],
                                pc[k][1] - pj[i][1],
                                pc[k][2] - pj[i][2]};
            const float F[3] = {0.0f, 0.0f, -GM_MASS[k] * GM_GRAVITY};
            const float m[3] = {r[1] * F[2] - r[2] * F[1],
                                r[2] * F[0] - r[0] * F[2],
                                r[0] * F[1] - r[1] * F[0]};
            sum -= axw[i][0] * m[0] + axw[i][1] * m[1] + axw[i][2] * m[2];
        }
        tau[i] = sum;
    }
}

void gravity_motor_to_joint(const float q_motor[GM_JOINTS], float q_joint[GM_JOINTS])
{
    for (int i = 0; i < GM_JOINTS; i++) {
        q_joint[i] = GM_DIR[i] * (q_motor[i] * GM_SCALE[i]) + GM_OFFSET[i];
    }
}

void gravity_joint_to_motor(const float q_joint[GM_JOINTS], float q_motor[GM_JOINTS])
{
    for (int i = 0; i < GM_JOINTS; i++) {
        q_motor[i] = GM_DIR[i] * ((q_joint[i] - GM_OFFSET[i]) / GM_SCALE[i]);
    }
}

void gravity_torque_motor(const float q_motor[GM_JOINTS], float tau_motor[GM_JOINTS])
{
    float q[GM_JOINTS], tau[GM_JOINTS];
    gravity_motor_to_joint(q_motor, q);
    gravity_torque(q, tau);
    for (int i = 0; i < GM_JOINTS; i++) {
        tau_motor[i] = GM_DIR[i] * tau[i];
    }
}
