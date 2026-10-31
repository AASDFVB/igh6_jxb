/**
 * test_gravity.c — 主机侧校验重力模型
 *
 *   cc app/gravity.c test/test_gravity.c -Iapp -lm -o /tmp/test_gravity && /tmp/test_gravity
 *
 * 对拍数据由 tools/gen_gravity_model.py 用 Pinocchio computeGeneralizedGravity 生成。
 * 同时校验电机侧封装：q_motor = (q_joint − offset)/dir，τ_motor = dir·τ_joint。
 */
#include <math.h>
#include <stdio.h>

#include "gravity.h"
#include "gravity_vectors.h"

int main(void)
{
    double worst_joint = 0.0;
    double worst_motor = 0.0;

    for (int v = 0; v < GRAVITY_VECTOR_COUNT; v++) {
        const gravity_vector_t *vec = &GRAVITY_VECTORS[v];
        float tau[GM_JOINTS];
        gravity_torque(vec->q, tau);
        for (int i = 0; i < GM_JOINTS; i++) {
            const double e = fabs((double)tau[i] - (double)vec->tau[i]);
            if (e > worst_joint) {
                worst_joint = e;
            }
        }

        float q_motor[GM_JOINTS], tau_motor[GM_JOINTS];
        for (int i = 0; i < GM_JOINTS; i++) {
            q_motor[i] = (vec->q[i] - GM_OFFSET[i]) / GM_DIR[i];
        }
        gravity_torque_motor(q_motor, tau_motor);
        for (int i = 0; i < GM_JOINTS; i++) {
            const double want = GM_DIR[i] * vec->tau[i];
            const double e = fabs((double)tau_motor[i] - want);
            if (e > worst_motor) {
                worst_motor = e;
            }
        }
    }

    printf("关节侧最大误差 %.3e N·m（%d 组）\n", worst_joint, GRAVITY_VECTOR_COUNT);
    printf("电机侧最大误差 %.3e N·m\n", worst_motor);

    const int ok = worst_joint < 1e-3 && worst_motor < 1e-3;
    printf("%s\n", ok ? "PASS" : "FAIL");
    return ok ? 0 : 1;
}
