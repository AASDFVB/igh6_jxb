/**
 * app_config.h — 板级配置：电机表、总线映射、时序参数
 *
 * 拓扑：CAN1 = 电机 1/2/3，CAN2 = 电机 4/5/6，各 1 kHz 轮询。
 * Master ID 约定 0x10 + 电机 ID（厂商文档明确禁止用默认 0：多机会全回在 0x000）。
 */
#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#define APP_MOTOR_COUNT 6
#define APP_BUS_COUNT 2

typedef struct {
    uint8_t id;  /* 电机 ID = ESC_ID，命令帧寻址用 */
    uint8_t bus; /* 0 = CAN1, 1 = CAN2 */
} app_motor_cfg_t;

static const app_motor_cfg_t APP_MOTORS[APP_MOTOR_COUNT] = {
    {1, 0}, {2, 0}, {3, 0},
    {4, 1}, {5, 1}, {6, 1},
};

#define APP_MASTER_ID(id) ((uint8_t)(0x10u + (uint8_t)(id)))

#define APP_TICK_HZ 1000u
#define APP_ENABLE_AT_MS 100u   /* 上电后多久开始发 clear-error / 使能帧 */
#define APP_HOLD_AT_MS 200u     /* 之后开始 1 kHz 保持帧（使能帧会一直补到全部使能） */
#define APP_REPORT_EVERY_MS 1000u
#define APP_STREAM_EVERY_MS 20u    /* 机器可读关节流周期：20 ms = 50 Hz，给上位机做 RViz 同步 */

/* ---- 重力补偿 ---- */
#define APP_GCOMP_ENABLE 1
#define APP_GCOMP_STALE_MS 100u      /* 任一轴反馈超过 100 ms 没更新 → 立刻零力矩 */
/* gains / kp / kd / scale / limit / ramp / rate 都在 app/gcomp_params.h（手改）。 */
#define APP_DEBUG_TORQUE 1           /* 10 Hz 打 G 行（力矩 mN·m），用于和 Pinocchio 对拍 */
#define APP_TORQUE_EVERY_MS 100u

/* ---- 串口下行（遥操作）---- */
#define APP_TELEOP_RAD_S 1.5f        /* 目标角的最大变化速度 rad/s；0.8 太拖、3 以上开始发冲 */
#define APP_TELEOP_RANGE 7.0f        /* 单轴目标角允许范围 ±rad，超出直接拒绝 */

#endif /* APP_CONFIG_H */
