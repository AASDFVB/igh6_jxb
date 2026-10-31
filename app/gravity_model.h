/* 自动生成，别手改：tools/gen_gravity_model.py
 *   源 URDF: /home/ubuntu-24-04/Desktop/ros2_gong/src/robotarm/urdf/robotarm.urdf
 *   算法: 静态重力矩（与 Pinocchio computeGeneralizedGravity 一致）
 */
#ifndef GRAVITY_MODEL_H
#define GRAVITY_MODEL_H

#define GM_JOINTS 6
#define GM_GRAVITY 9.81f

static const float GM_ORIGIN_R[GM_JOINTS][9] = {
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f},
};

static const float GM_ORIGIN_T[GM_JOINTS][3] = {
    {0.0f, 0.0f, 0.0f},
    {-0.023412f, 0.0019307f, 0.043729f},
    {-0.017f, 0.0f, 0.18f},
    {0.0405f, -0.026832f, 0.052475f},
    {0.016f, 0.0f, 0.04f},
    {-0.025f, 0.0f, 0.091f},
};

static const float GM_AXIS[GM_JOINTS][3] = {
    {0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f},
    {1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
    {1.0f, 0.0f, 0.0f},
    {0.0f, 0.0f, 1.0f},
};

static const float GM_COM[GM_JOINTS][3] = {
    {0.0027813f, 0.0019097f, 0.039364f},
    {-0.0046437f, 1.4323e-06f, 0.11295f},
    {0.042347f, -0.021222f, 0.027574f},
    {-0.0073786f, -1.1465e-06f, 0.034162f},
    {-0.020483f, 1.1422e-06f, 0.057393f},
    {-0.015996f, -3.518e-15f, 0.010859f},
};

static const float GM_MASS[GM_JOINTS] = {
    0.37f, 0.57f, 0.38f, 0.35f, 0.35f, 0.03f,
};

/* 标定：q_joint = dir * q_motor + offset（来自 host/joint_offsets.yaml）*/
static const float GM_DIR[GM_JOINTS] = {+1, -1, +1, +1, +1, +1};
static const float GM_OFFSET[GM_JOINTS] = {-5.244f, 0.1932f, -0.549f, -0.328f, -1.869f, -0.813f};
static const unsigned char GM_CAN_ID[GM_JOINTS] = {1, 2, 3, 4, 5, 6};

#endif /* GRAVITY_MODEL_H */
