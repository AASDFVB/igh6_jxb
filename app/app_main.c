/**
 * app_main.c — bring-up 主逻辑
 *
 * 时序（以 1 kHz 的 SysTick 计数 s_ms 为准）：
 *   100 ms     : 给 6 个电机各发一帧 clear-error（0xFB）
 *   101..106 ms: 每 ms 给一个电机发 enable（0xFC）
 *   150 ms 起  : 每 ms 给 6 个电机各发一帧保持帧，反馈随即流回
 *   每 1000 ms : 通过 USB CDC 打一行汇总 + 6 行电机状态
 *
 * 所有 CAN 收发都在主循环里做（HAL 的收发函数带锁，放中断里会互相抢锁），
 * SysTick 只负责数毫秒。主循环比 1 ms 快，所以每个 ms 都能被处理到；
 * 万一落后，跳过的 ms 会计进 skipped 计数。
 */
#include "app_main.h"

#include <stdio.h>

#include "app_can.h"
#include "app_led.h"
#include "app_config.h"
#include "dm_motor.h"
#include "usbd_cdc_if.h"

static volatile uint32_t s_ms;
static uint32_t s_served_ms;
static uint32_t s_skipped_ticks;

void app_init(void)
{
    app_can_init();
    app_led_init();
    s_ms = 0;
    s_served_ms = 0;
    s_skipped_ticks = 0;
}

void app_tick_1khz(void)
{
    s_ms++;
}

static int fsat_round(float x)
{
    if (x >= 0.0f) {
        return (int)(x + 0.5f);
    }
    return (int)(x - 0.5f);
}

static void send_for_ms(uint32_t ms)
{
    if (ms == APP_ENABLE_AT_MS) {
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            app_can_send_special(i, DM_CMD_CLEAR_ERROR);
        }
        return;
    }

    const uint32_t enable_end = APP_ENABLE_AT_MS + APP_ENABLE_REPEATS;
    if (ms > APP_ENABLE_AT_MS && ms <= enable_end) {
        const int i = (int)(ms - APP_ENABLE_AT_MS) - 1;
        if (i >= 0 && i < APP_MOTOR_COUNT) {
            app_can_send_special(i, DM_CMD_ENABLE);
        }
        return;
    }

    if (ms >= APP_HOLD_AT_MS) {
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            app_can_send_hold(i);
        }
    }
}

static void report(void)
{
    char buf[256];
    int n = 0;

    n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                  "t=%lus rx=", (unsigned long)(s_ms / 1000u));
    for (int i = 0; i < APP_MOTOR_COUNT && n < (int)sizeof(buf); i++) {
        const uint32_t rx = app_can_close_window(i);
        n += snprintf(buf + n, sizeof(buf) - (size_t)n, "%s%lu",
                      (i == 0) ? "" : "/", (unsigned long)rx);
    }
    if (n < (int)sizeof(buf)) {
        n += snprintf(buf + n, sizeof(buf) - (size_t)n,
                      " ovf=%lu un=%lu drop=%lu bof=%lu skip=%lu\r\n",
                      (unsigned long)g_fifo_overruns,
                      (unsigned long)g_rx_unmatched,
                      (unsigned long)g_tx_dropped,
                      (unsigned long)g_busoff_recoveries,
                      (unsigned long)s_skipped_ticks);
    }
    if (n > 0 && n < (int)sizeof(buf)) {
        (void)CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
    }

    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        const dm_feedback_t *f = &g_motors[i].last;
        n = snprintf(buf, sizeof(buf),
                     "[%u] e=%u p=%+d v=%+d t=%+d %u/%u\r\n",
                     (unsigned)APP_MOTORS[i].id, (unsigned)f->err,
                     fsat_round(f->pos * 1000.0f),
                     fsat_round(f->vel * 100.0f),
                     fsat_round(f->tor * 1000.0f),
                     (unsigned)f->t_mos, (unsigned)f->t_rotor);
        if (n > 0 && n < (int)sizeof(buf)) {
            (void)CDC_Transmit_FS((uint8_t *)buf, (uint16_t)n);
        }
    }
}

void app_loop(void)
{
    app_can_poll();
    app_can_poll_errors();

    const uint32_t now = s_ms;
    app_led_update(now);

    if (now == s_served_ms) {
        return;
    }
    if (now - s_served_ms > 1u) {
        s_skipped_ticks += now - s_served_ms - 1u;
    }
    s_served_ms = now;

    send_for_ms(now);

    if (now % APP_REPORT_EVERY_MS == 0u) {
        report();
    }
}
