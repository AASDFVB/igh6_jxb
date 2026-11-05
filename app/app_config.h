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
/* ---- 串口日志音量（唯一旋钮，改这一个数就行）----
 *   0 = 只打每秒那一行汇总（最安静）
 *   1 = 汇总 + 6 行电机状态        ← 当前
 *   2 = 再加 10 Hz 的 G 行（力矩 mN·m，和 Pinocchio 对拍用）
 *   3 = 再加 50 Hz 的 J 行（关节流；RViz 同步 / zero_offsets.py 需要）
 */
#define APP_LOG_LEVEL 0
#define APP_TORQUE_EVERY_MS 100u     /* G 行周期（仅 LEVEL>=2 生效） */

/* ---- 调试串口（C 板 J30「UART1」= USART1：PA9=TX / PB7=RX）---- */
#define APP_TX_UART 1                /* 1 = 除了 USB CDC，同样内容再从 UART1 打一份 */
#define APP_UART_BAUD 115200u        /* 必须和无线模块的波特率一致 */

/* ---- 串口下行（遥操作）---- */
#define APP_TELEOP_RAD_S 2.5f        /* 目标角的最大变化速度 rad/s；镜像遥操作要跟手，1.5 偏拖 */
#define APP_TELEOP_RANGE 7.0f        /* 单轴目标角允许范围 ±rad，超出直接拒绝 */
#define APP_TELEOP_TIMEOUT_MS 300u   /* 上位机停发超过这么久 → 等价 M 0，退回纯重力补偿 */
#define APP_TELEOP_ECHO 0            /* 1 = 每帧回 OK P（高频下会白吃掉上行带宽，默认关） */

/* ---- 二进制度帧（镜像遥操作的高速通道）----
 *   0xAA | q0(i16) q1(i16) ... q5(i16) | sum      = 14 字节
 *   角度单位 mrad，小端；sum = 前 13 字节累加和的低 8 位。
 *   0xAA 不在 ASCII 文本里出现，所以文本命令（M/K/P）和二进制帧可以混在一条流里。
 *   14 字节/帧 在实测 ~680 B/s 的空口上能跑到 ~48 帧/秒，比文本的 42 字节快 3 倍。 */
#define APP_BIN_HEADER 0xAAu
#define APP_BIN_FRAME_LEN 14u

#endif /* APP_CONFIG_H */
