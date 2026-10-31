/**
 * home_params.h — 上电回初始位（**手改**，改完重新编译烧录）
 *
 * 逻辑：六颗全部使能且反馈正常 → 等 HOME_DELAY_MS → 用 HOME_MOVE_MS 从当前姿态
 *       平滑插值到 HOME_TARGET → 之后用 HOME_KP/HOME_KD 保持在该姿态。
 *
 * 目标角是 **URDF 关节角**（rad）。全 0 = URDF 初始位（标定零点时摆的那个立姿）。
 *
 * ⚠️ 上电后手臂会自己动。确认运动路径上没有人和东西，且 HOME_MOVE_MS 够慢。
 */
#ifndef HOME_PARAMS_H
#define HOME_PARAMS_H


#define HOME_ENABLE 1           /* 0 = 关掉，上电不自动回位 */
#define HOME_DELAY_MS 3000u     /* 使能稳定后再等多久开始动 */
#define HOME_MOVE_MS 15000u      /* 从当前姿态走到目标用多久（越大越慢越安全） */
#define HOME_KP 3.0f           /* 移动/保持用的位置刚度 N·m/rad（6~40 比较温和） */
#define HOME_KD 1.0f            /* 阻尼 N·m/(rad/s) */
#define HOME_HOLD_AFTER 0       /* 1 = 到位后保持（推开会自己回来）；0 = 到位后放开（推哪停哪，只剩重力补偿） */

/* 目标姿态：URDF 关节角（rad），顺序 J0..J5。全 0 = URDF 初始位。 */
static const float HOME_TARGET[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

#endif /* HOME_PARAMS_H */
