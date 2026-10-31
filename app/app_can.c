/**
 * app_can.c — 双总线 CAN：滤波器、启动、保持帧、反馈分发
 *
 * 关键事实（见 docs/adr）：
 *  - 反馈是轮询式：不发命令就没有反馈，所以"接收"靠 1 kHz 保持帧驱动。
 *  - CAN2 是从控制器：滤波器 0–13 归 CAN1，14–27 归 CAN2，且必须在每次
 *    HAL_CAN_ConfigFilter 里显式给 SlaveStartFilterBank（DJI 官方例程漏了它）。
 *  - 反馈帧的 CAN ID 是 Master ID（0x10+电机ID），但若电机仍是出厂默认 0，
 *    则全部落在 0x000，只能靠 D[0] 低半字节区分；两条路都兼容。
 */
#include "app_can.h"
#include <stdbool.h>

#include "main.h"

extern CAN_HandleTypeDef hcan1;
extern CAN_HandleTypeDef hcan2;

app_motor_state_t g_motors[APP_MOTOR_COUNT];
volatile uint32_t g_rx_unmatched;
volatile uint32_t g_fifo_overruns;
volatile uint32_t g_tx_dropped;
volatile uint32_t g_busoff_recoveries;
volatile uint32_t g_busoff_events;

#define BUSOFF_RECOVER_INTERVAL_MS 500u
static bool s_in_busoff[APP_BUS_COUNT];
static uint32_t s_last_recover_ms[APP_BUS_COUNT];
static uint32_t s_now_ms;   /* app_can_poll 传进来的时刻，给反馈打时间戳 */

#define SLAVE_START_FILTER_BANK 14u

static CAN_HandleTypeDef *bus_handle(uint8_t bus)
{
    return (bus == 0) ? &hcan1 : &hcan2;
}

static uint8_t handle_bus(const CAN_HandleTypeDef *h)
{
    return (h->Instance == CAN1) ? 0u : 1u;
}

/* 32 位掩码滤波器：标准 ID 在 31:21，低半区留 0 */
static void config_filter(CAN_HandleTypeDef *h, uint32_t bank,
                          uint8_t id_base, uint32_t id_mask, uint32_t fifo)
{
    CAN_FilterTypeDef f = {0};
    f.FilterBank = bank;
    f.FilterMode = CAN_FILTERMODE_IDMASK;
    f.FilterScale = CAN_FILTERSCALE_32BIT;
    f.FilterIdHigh = (uint32_t)(id_base << 5);
    f.FilterIdLow = 0;
    f.FilterMaskIdHigh = (uint32_t)(id_mask << 5);
    f.FilterMaskIdLow = 0;
    f.FilterFIFOAssignment = fifo;
    f.FilterActivation = ENABLE;
    f.SlaveStartFilterBank = SLAVE_START_FILTER_BANK;
    if (HAL_CAN_ConfigFilter(h, &f) != HAL_OK) {
        Error_Handler();
    }
}

void app_can_init(void)
{
    /* CAN1 拥有 bank 0–13：0x10–0x13 → FIFO0，0x14–0x17 → FIFO1，0x00–0x0F → FIFO0 */
    config_filter(&hcan1, 0, 0x10, 0x7FC, CAN_FILTER_FIFO0);
    config_filter(&hcan1, 1, 0x14, 0x7FC, CAN_FILTER_FIFO1);
    config_filter(&hcan1, 2, 0x00, 0x7F0, CAN_FILTER_FIFO0);

    /* CAN2 拥有 bank 14–27：同样的三段，bank 号从 14 起 */
    config_filter(&hcan2, 14, 0x10, 0x7FC, CAN_FILTER_FIFO0);
    config_filter(&hcan2, 15, 0x14, 0x7FC, CAN_FILTER_FIFO1);
    config_filter(&hcan2, 16, 0x00, 0x7F0, CAN_FILTER_FIFO0);

    for (uint8_t b = 0; b < APP_BUS_COUNT; b++) {
        CAN_HandleTypeDef *h = bus_handle(b);
        if (HAL_CAN_Start(h) != HAL_OK) {
            Error_Handler();
        }
    }
}

static void send_8(CAN_HandleTypeDef *h, uint8_t std_id, const uint8_t data[8])
{
    CAN_TxHeaderTypeDef tx = {0};
    uint32_t mailbox;
    tx.StdId = std_id;
    tx.IDE = CAN_ID_STD;
    tx.RTR = CAN_RTR_DATA;
    tx.DLC = 8;
    tx.TransmitGlobalTime = DISABLE;
    if (HAL_CAN_AddTxMessage(h, &tx, (uint8_t *)data, &mailbox) != HAL_OK) {
        g_tx_dropped++;
    }
}

void app_can_send_mit(int i, float p_des, float v_des, float kp, float kd, float t_ff)
{
    uint8_t buf[8];
    dm_encode_mit(buf, p_des, v_des, kp, kd, t_ff);
    send_8(bus_handle(APP_MOTORS[i].bus), APP_MOTORS[i].id, buf);
}

void app_can_send_hold(int i)
{
    app_can_send_mit(i, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
}

void app_can_send_special(int i, uint8_t cmd)
{
    uint8_t buf[8];
    dm_encode_special(buf, cmd);
    send_8(bus_handle(APP_MOTORS[i].bus), APP_MOTORS[i].id, buf);
}

uint32_t app_can_close_window(int i)
{
    uint32_t n = g_motors[i].rx_in_window;
    g_motors[i].rx_in_window = 0;
    return n;
}

/* 反馈分发：先按 CAN ID = Master ID 匹配，再退回 payload 低半字节 */
static void dispatch_feedback(uint8_t bus, const CAN_RxHeaderTypeDef *rx, const uint8_t data[8])
{
    int idx = -1;

    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        if (APP_MOTORS[i].bus == bus && APP_MASTER_ID(APP_MOTORS[i].id) == rx->StdId) {
            idx = i;
            break;
        }
    }
    const uint8_t payload_id = (uint8_t)(data[0] & 0x0Fu);
    if (idx < 0) {
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            if (APP_MOTORS[i].bus == bus && APP_MOTORS[i].id == payload_id) {
                idx = i;
                break;
            }
        }
    }
    if (idx < 0) {
        g_rx_unmatched++;
        return;
    }

    dm_decode_feedback(data, &g_motors[idx].last);
    g_motors[idx].rx_frames++;
    g_motors[idx].rx_in_window++;
    g_motors[idx].last_rx_ms = s_now_ms;
}

static void drain_fifo(CAN_HandleTypeDef *h, uint32_t fifo)
{
    const uint8_t bus = handle_bus(h);
    while (HAL_CAN_GetRxFifoFillLevel(h, fifo) > 0) {
        CAN_RxHeaderTypeDef rx = {0};
        uint8_t data[8] = {0};
        if (HAL_CAN_GetRxMessage(h, fifo, &rx, data) != HAL_OK) {
            return;
        }
        if (rx.IDE != CAN_ID_STD || rx.DLC != 8) {
            g_rx_unmatched++;
            continue;
        }
        dispatch_feedback(bus, &rx, data);
    }
}

void app_can_poll(uint32_t now_ms)
{
    s_now_ms = now_ms;
    /* 全部 CAN 收发都在主循环里做：HAL 的 GetRxMessage/AddTxMessage 都带
     * __HAL_LOCK，混在中断里会互相抢锁。轮询只要主循环比 1 ms 快，就不会丢。 */
    for (uint8_t b = 0; b < APP_BUS_COUNT; b++) {
        CAN_HandleTypeDef *h = bus_handle(b);
        drain_fifo(h, CAN_RX_FIFO0);
        drain_fifo(h, CAN_RX_FIFO1);

        /* F4 HAL 没有 BUS_OFF 状态枚举，直接读 ESR 的 BOFF 位。
         * 总线上没有应答节点时它会反复进 bus-off，所以只统计"进入"次数，
         * 且恢复动作限速到 500 ms 一次，不然主循环会把 CPU 全耗在重启上。 */
        if ((h->Instance->ESR & CAN_ESR_BOFF) != 0u) {
            if (!s_in_busoff[b]) {
                s_in_busoff[b] = true;
                g_busoff_events++;
            }
            if (now_ms - s_last_recover_ms[b] >= BUSOFF_RECOVER_INTERVAL_MS) {
                s_last_recover_ms[b] = now_ms;
                g_busoff_recoveries++;
                (void)HAL_CAN_Stop(h);
                (void)HAL_CAN_Start(h);
            }
        } else {
            s_in_busoff[b] = false;
        }
    }
}

/* 错误标志按需轮询，不做中断 */
void app_can_poll_errors(void)
{
    for (uint8_t b = 0; b < APP_BUS_COUNT; b++) {
        CAN_HandleTypeDef *h = bus_handle(b);
        const uint32_t err = HAL_CAN_GetError(h);
        if ((err & HAL_CAN_ERROR_RX_FOV0) || (err & HAL_CAN_ERROR_RX_FOV1)) {
            g_fifo_overruns++;
        }
    }
}
