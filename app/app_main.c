/**
 * app_main.c — bring-up 主逻辑
 *
 * 时序（以 1 kHz 的 SysTick 计数 s_ms 为准）：
 *   100 ms     : 给 6 个电机各发一帧 clear-error（0xFB）
 *   101 ms 起  : 每 ms 给一颗「还没回已使能」的电机补一帧 enable（0xFC），
 *                循环轮询直到六颗全部使能；这一拍用使能帧替掉该电机的保持帧
 *   200 ms 起  : 每 ms 给 6 个电机各发一帧保持帧，反馈随即流回
 *   每 1000 ms : 通过 USB CDC 打一行汇总 + 6 行电机状态
 *
 * 所有 CAN 收发都在主循环里做（HAL 的收发函数带锁，放中断里会互相抢锁），
 * SysTick 只负责数毫秒。USB 输出走一个小队列：CDC_Transmit_FS 在上一个包没
 * 发完时返回 BUSY，直接丢弃会丢行，所以先入队再逐包排空。
 */
#include "app_main.h"

#include <math.h>
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "app_can.h"
#include "app_config.h"
#include "app_led.h"
#include "dm_motor.h"
#include "gcomp_params.h"
#include "gravity.h"
#include "home_params.h"
#include "usbd_cdc_if.h"

static volatile uint32_t s_ms;
static uint32_t s_served_ms;
static uint32_t s_skipped_ticks;
static bool s_banner_sent;

/* ---- 上电回初始位 ---- */
enum { HOME_WAIT = 0, HOME_MOVE, HOME_HOLD, HOME_OFF };
static int s_home_state;
static uint32_t s_home_t0;
static float s_home_from[APP_MOTOR_COUNT];   /* 起步时的关节角 */
static float s_home_cmd[APP_MOTOR_COUNT];    /* 当前生效的目标（关节角，带限速） */
static float s_target_req[APP_MOTOR_COUNT];  /* 上位机请求的目标（关节角） */
static float s_kp = HOME_KP;                 /* 运行时可改的位置刚度 */
static float s_kd = HOME_KD;
static bool s_hold_enabled = HOME_HOLD_AFTER;

/* ---- 串口下行：环形缓冲 + 行缓冲 ---- */
#define RX_RING 256u
static volatile uint8_t s_rx[RX_RING];
static volatile uint16_t s_rx_head;
static volatile uint16_t s_rx_tail;
static char s_line[96];
static uint16_t s_line_len;

/* ---- 重力补偿状态 ---- */
static float s_tau[APP_MOTOR_COUNT];
static float s_hold[APP_MOTOR_COUNT];   /* 使能瞬间锁存的角度，kp>0 时作保持目标 */
static uint32_t s_gcomp_t0;
static uint32_t s_gcomp_last_calc;
static bool s_gcomp_active;

/* ---- USB CDC 发送队列 ---- */
static uint8_t s_tx[1024];
static uint16_t s_tx_len;
static uint16_t s_tx_off;

static void tx_push(const char *s, int n)
{
    if (n <= 0) {
        return;
    }
    if ((size_t)n > sizeof(s_tx) - s_tx_len) {
        return; /* 队列满就丢这一行，绝不覆盖未发完的数据 */
    }
    memcpy(&s_tx[s_tx_len], s, (size_t)n);
    s_tx_len += (uint16_t)n;
}

static void tx_pushf(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

static void tx_pushf(const char *fmt, ...)
{
    char line[192];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(line, sizeof(line), fmt, ap);
    va_end(ap);
    if (n >= (int)sizeof(line)) {
        n = (int)sizeof(line) - 1; /* 截断时 vsnprintf 返回“本来要写多少”，按缓冲区长度收口 */
    }
    tx_push(line, n);
}

static void tx_drain(void)
{
    if (s_tx_off >= s_tx_len) {
        s_tx_off = 0;
        s_tx_len = 0;
        return;
    }

    uint16_t n = (uint16_t)(s_tx_len - s_tx_off);
    if (n > 64u) {
        n = 64u;
        /* 优先在行尾断开，免得终端里出现半行 */
        for (uint16_t i = 0; i < 64u; i++) {
            if (s_tx[s_tx_off + i] == '\n') {
                n = (uint16_t)(i + 1u);
                break;
            }
        }
    }
    if (CDC_Transmit_FS(&s_tx[s_tx_off], n) == 0u) { /* USBD_OK */
        s_tx_off = (uint16_t)(s_tx_off + n);
    }
}

/* ---- 主逻辑 ---- */

void app_init(void)
{
    app_can_init();
    app_led_init();
    s_ms = 0;
    s_served_ms = 0;
    s_skipped_ticks = 0;
    s_banner_sent = false;
    memcpy(s_target_req, HOME_TARGET, sizeof(s_target_req));
    memcpy(s_home_cmd, HOME_TARGET, sizeof(s_home_cmd));
    s_line_len = 0;
}

void app_tick_1khz(void)
{
    s_ms++;
}

/* ---------------- 串口下行（遥操作） ---------------- */

void app_serial_input(const uint8_t *buf, uint32_t len)
{
    for (uint32_t i = 0; i < len; i++) {
        const uint16_t next = (uint16_t)((s_rx_head + 1u) % RX_RING);
        if (next != s_rx_tail) {
            s_rx[s_rx_head] = buf[i];
            s_rx_head = next;
        }
    }
}

/* 从 s 里连续解析 n 个浮点，成功返回 0 */
static int parse_floats(const char *s, float *out, int n)
{
    for (int i = 0; i < n; i++) {
        char *end = NULL;
        const float v = strtof(s, &end);
        if (end == s) {
            return -1;
        }
        out[i] = v;
        s = end;
    }
    return 0;
}

static void handle_command(char *line)
{
    if (line[0] == 'P' && line[1] == ' ') {
        /* P q0 q1 ... q5 —— 关节角目标（rad） */
        float q[APP_MOTOR_COUNT];
        if (parse_floats(line + 2, q, APP_MOTOR_COUNT) != 0) {
            tx_pushf("ERR P parse\r\n");
            return;
        }
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            if (!isfinite(q[i]) || q[i] < -APP_TELEOP_RANGE || q[i] > APP_TELEOP_RANGE) {
                tx_pushf("ERR P range\r\n");
                return;
            }
        }
        if (s_home_state != HOME_HOLD) {
            /* 还没接管（等待中 / 已放弃 / 正在回零）：立刻从当前姿态接管，
             * 交给 HOLD 阶段的限速去追新目标，避免插值中途改目标造成跳变。 */
            float q_motor[APP_MOTOR_COUNT];
            for (int i = 0; i < APP_MOTOR_COUNT; i++) {
                q_motor[i] = g_motors[i].last.pos;
            }
            gravity_motor_to_joint(q_motor, s_home_cmd);
            s_home_state = HOME_HOLD;
        }
        memcpy(s_target_req, q, sizeof(s_target_req));
        tx_pushf("OK P\r\n");
        return;
    }
    if (line[0] == 'M' && line[1] == ' ') {
        /* M 0 = 放开（只剩重力补偿）；M 1 = 位置保持 */
        char *end = NULL;
        const long m = strtol(line + 2, &end, 10);
        if (end == line + 2) {
            tx_pushf("ERR M\r\n");
            return;
        }
        s_hold_enabled = (m != 0);
        if (!s_hold_enabled && s_home_state == HOME_MOVE) {
            s_home_state = HOME_OFF;   /* 回零途中发 M 0 = 放弃回零，别再硬走 */
        }
        tx_pushf("OK M %d\r\n", s_hold_enabled ? 1 : 0);
        return;
    }
    if (line[0] == 'K' && line[1] == ' ') {
        /* K kp kd —— 运行时改位置刚度/阻尼 */
        float k[2];
        if (parse_floats(line + 2, k, 2) != 0 || !isfinite(k[0]) || !isfinite(k[1]) ||
            k[0] < 0.0f || k[0] > 50.0f || k[1] < 0.0f || k[1] > 10.0f) {
            tx_pushf("ERR K\r\n");
            return;
        }
        s_kp = k[0];
        s_kd = k[1];
        tx_pushf("OK K %.2f %.2f\r\n", (double)s_kp, (double)s_kd);
        return;
    }
    tx_pushf("ERR ?\r\n");
}

/* 主循环里反复调用：把收到的字节切成行，逐行执行 */
static void serial_poll(void)
{
    while (s_rx_tail != s_rx_head) {
        const char c = (char)s_rx[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1u) % RX_RING);
        if (c == '\n' || c == '\r') {
            if (s_line_len > 0) {
                s_line[s_line_len] = '\0';
                handle_command(s_line);
                s_line_len = 0;
            }
        } else if (s_line_len < sizeof(s_line) - 1u) {
            s_line[s_line_len++] = c;
        } else {
            s_line_len = 0;   /* 超长行整行丢弃 */
        }
    }
}

static int fsat_round(float x)
{
    if (x >= 0.0f) {
        return (int)(x + 0.5f);
    }
    return (int)(x - 0.5f);
}

/* 六颗都回过「已使能」了吗 */
static bool all_enabled(void)
{
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        if (g_motors[i].last.err != 0x1) {
            return false;
        }
    }
    return true;
}

/* 六颗都健康（已使能 + 反馈没超时）才允许出力 */
static bool gcomp_ready(uint32_t now)
{
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        if (g_motors[i].last.err != 0x1) {
            return false;
        }
        if ((uint32_t)(now - g_motors[i].last_rx_ms) > APP_GCOMP_STALE_MS) {
            return false;
        }
    }
    return true;
}

/* 维护 s_tau：100 Hz 重算、4 秒渐入、单轴限幅；条件一破立刻归零 */
static void gcomp_update(uint32_t now)
{
#if APP_GCOMP_ENABLE
    if (!gcomp_ready(now)) {
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            s_tau[i] = 0.0f;
        }
        s_gcomp_active = false;
        return;
    }
    if (!s_gcomp_active) {
        s_gcomp_active = true;
        s_gcomp_t0 = now;
        s_gcomp_last_calc = 0;      /* 强制立刻算一次 */
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            s_hold[i] = g_motors[i].last.pos;   /* 锁存：kp>0 时就是保持目标 */
        }
    }
    if (s_gcomp_last_calc != 0u && now - s_gcomp_last_calc < GCOMP_PERIOD_MS) {
        return;
    }
    s_gcomp_last_calc = now;

    float q_motor[APP_MOTOR_COUNT];
    float tau[APP_MOTOR_COUNT];
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        q_motor[i] = g_motors[i].last.pos;   /* 电机侧角度，rad */
    }
    gravity_torque_motor(q_motor, tau);

    float ramp = 1.0f;
    const uint32_t elapsed = now - s_gcomp_t0;
    if (elapsed < GCOMP_RAMP_MS) {
        ramp = (float)elapsed / (float)GCOMP_RAMP_MS;
    }
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        float t = GCOMP_ACTIVE[i] ? tau[i] * GCOMP_GAIN[i] * GCOMP_SCALE * ramp : 0.0f;
        if (t > GCOMP_LIMIT_NM) {
            t = GCOMP_LIMIT_NM;
        }
        if (t < -GCOMP_LIMIT_NM) {
            t = -GCOMP_LIMIT_NM;
        }
        s_tau[i] = t;
    }
#else
    (void)now;
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        s_tau[i] = 0.0f;
    }
#endif
}

/* 上电回初始位：等六轴就绪 → HOME_MOVE_MS 内平滑插值到 HOME_TARGET → 停在那儿 */
static void home_update(uint32_t now)
{
#if HOME_ENABLE
    if (s_home_state == HOME_OFF) {
        return;
    }
    if (!gcomp_ready(now)) {
        /* 中途失效（掉使能/丢帧）就放弃回位，退回纯重力补偿，别硬顶 */
        if (s_home_state == HOME_MOVE || s_home_state == HOME_HOLD) {
            s_home_state = HOME_OFF;
        }
        return;
    }
    if (s_home_state == HOME_WAIT) {
        if (now < HOME_DELAY_MS) {
            return;
        }
        float q_motor[APP_MOTOR_COUNT];
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            q_motor[i] = g_motors[i].last.pos;
        }
        gravity_motor_to_joint(q_motor, s_home_from);
        memcpy(s_home_cmd, s_home_from, sizeof(s_home_cmd));
        s_home_t0 = now;
        s_home_state = HOME_MOVE;
        return;
    }
    if (s_home_state == HOME_MOVE) {
        const uint32_t elapsed = now - s_home_t0;
        float s = (elapsed >= HOME_MOVE_MS) ? 1.0f : (float)elapsed / (float)HOME_MOVE_MS;
        s = s * s * (3.0f - 2.0f * s);   /* smoothstep：起步和到位都不顿 */
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            s_home_cmd[i] = s_home_from[i] + (s_target_req[i] - s_home_from[i]) * s;
        }
        if (elapsed >= HOME_MOVE_MS) {
            s_home_state = HOME_HOLD;
        }
    }
    if (s_home_state == HOME_HOLD) {
        /* 保持阶段：目标可能被上位机改，按 APP_TELEOP_RAD_S 限速追过去，
         * 避免一帧指令把目标拉太远（kp 会把大偏差变成一个大力矩）。 */
        const float step = APP_TELEOP_RAD_S * 0.001f;   /* 每毫秒最多走这么多 rad */
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            float d = s_target_req[i] - s_home_cmd[i];
            if (d > step) {
                d = step;
            }
            if (d < -step) {
                d = -step;
            }
            s_home_cmd[i] += d;
        }
    }
#else
    (void)now;
#endif
}

#if APP_DEBUG_TORQUE
/* G <ms> <t1..t6>：实际下发的力矩（mN·m），用来和 Pinocchio 对拍 */
static void stream_torque(uint32_t ms)
{
    char line[96];
    int n = snprintf(line, sizeof(line), "G %lu", (unsigned long)ms);
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        if (n < 0 || n >= (int)sizeof(line) - 10) {
            return;
        }
        const int w = snprintf(line + n, sizeof(line) - (size_t)n, " %d",
                               fsat_round(s_tau[i] * 1000.0f));
        if (w < 0) {
            return;
        }
        n += w;
    }
    line[n++] = '\r';
    line[n++] = '\n';
    tx_push(line, n);
}
#endif

static void send_for_ms(uint32_t ms)
{
    if (ms == APP_ENABLE_AT_MS) {
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            app_can_send_special(i, DM_CMD_CLEAR_ERROR);
        }
        return;
    }

    /* 使能可能打空：电机 24V 上电比 MCU 慢，且 4310 一掉电就回到未使能。
     * 所以每拍补一颗「还没回已使能」的电机，循环轮询，直到全部使能。
     * 这一拍用使能帧替掉该电机的保持帧，保证每条总线每毫秒仍是 3 帧，
     * 否则第 4 帧会因三个发送邮箱都占着而丢掉（drop 计数会涨）。 */
    int retry = -1;
    if (ms > APP_ENABLE_AT_MS && !all_enabled()) {
        retry = (int)((ms - APP_ENABLE_AT_MS) % APP_MOTOR_COUNT);
        if (g_motors[retry].last.err == 0x1) {
            retry = -1;
        }
    }

    if (ms >= APP_HOLD_AT_MS) {
        for (int i = 0; i < APP_MOTOR_COUNT; i++) {
            if (i != retry) {
                /* 默认：p_des 是使能瞬间锁存的角度（kp=0 时不起作用，出力全靠 t_ff）。
                 * 回初始位期间：p_des 跟着插值目标走，kp/kd 用 HOME_*。
                 * 条件不满足（s_gcomp_active=false）时一律给 0，绝不在失联时还拿 kp 顶。 */
                float p_des = s_hold[i];
                float kp = GCOMP_ACTIVE[i] ? GCOMP_KP[i] : 0.0f;
                float kd = GCOMP_ACTIVE[i] ? GCOMP_KD[i] : 0.0f;
                if (s_home_state == HOME_MOVE) {
                    p_des = GM_DIR[i] * (s_home_cmd[i] - GM_OFFSET[i]);
                    kp = s_kp;
                    kd = s_kd;
                } else if (s_home_state == HOME_HOLD) {
                    /* 保持/遥操作：M 0 放开（只剩重力补偿），M 1 钉住 */
                    p_des = GM_DIR[i] * (s_home_cmd[i] - GM_OFFSET[i]);
                    kp = s_hold_enabled ? s_kp : 0.0f;
                    kd = s_hold_enabled ? s_kd : 0.0f;
                }
                if (!s_gcomp_active) {
                    kp = 0.0f;
                    kd = 0.0f;
                }
                app_can_send_mit(i, p_des, 0.0f, kp, kd, s_tau[i]);
            }
        }
    }

    if (retry >= 0) {
        app_can_send_special(retry, DM_CMD_ENABLE);
    }
}

/* 错误码 → 中文名，串口直接可读 */
static const char *err_cn(uint8_t e)
{
    switch (e) {
    case 0x0: return "未使能";
    case 0x1: return "已使能";
    case 0x8: return "过压";
    case 0x9: return "欠压";
    case 0xA: return "过流";
    case 0xB: return "MOS过温";
    case 0xC: return "转子过温";
    case 0xD: return "通信丢失";
    case 0xE: return "过载";
    default:  return "未知";
    }
}

static void report(void)
{
    tx_pushf("第%lu秒 收帧=", (unsigned long)(s_ms / 1000u));
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        const uint32_t rx = app_can_close_window(i);
        tx_pushf("%s%lu", (i == 0) ? "" : "/", (unsigned long)rx);
    }
    tx_pushf(" 溢出=%lu 未匹配=%lu 发送失败=%lu 总线关闭=%lu/%lu 丢拍=%lu\r\n",
             (unsigned long)g_fifo_overruns,
             (unsigned long)g_rx_unmatched,
             (unsigned long)g_tx_dropped,
             (unsigned long)g_busoff_events,
             (unsigned long)g_busoff_recoveries,
             (unsigned long)s_skipped_ticks);

    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        const dm_feedback_t *f = &g_motors[i].last;
        tx_pushf("[%u] 状态=%s 位置=%+dmrad 速度=%+dmrad/s 力矩=%+dmN·m 温度=%u/%u℃\r\n",
                 (unsigned)APP_MOTORS[i].id, err_cn(f->err),
                 fsat_round(f->pos * 1000.0f),
                 fsat_round(f->vel * 1000.0f),
                 fsat_round(f->tor * 1000.0f),
                 (unsigned)f->t_mos, (unsigned)f->t_rotor);
    }
}

/* 机器可读的关节流，一行六个数，单位 mrad，顺序 = 电机 ID 1..6：
 *   J <t_ms> <p1> <p2> <p3> <p4> <p5> <p6>
 * 人工看的中文报告仍是 1 Hz，这行是给串口桥节点解析的。 */
static void stream_joints(uint32_t ms)
{
    char line[96];
    int n = snprintf(line, sizeof(line), "J %lu", (unsigned long)ms);
    for (int i = 0; i < APP_MOTOR_COUNT; i++) {
        if (n < 0 || n >= (int)sizeof(line) - 10) {
            return;
        }
        const int w = snprintf(line + n, sizeof(line) - (size_t)n, " %d",
                               fsat_round(g_motors[i].last.pos * 1000.0f));
        if (w < 0) {
            return;
        }
        n += w;
    }
    line[n++] = '\r';
    line[n++] = '\n';
    tx_push(line, n);
}

void app_loop(void)
{
    const uint32_t now = s_ms;
    serial_poll();              /* 先收指令：一圈可能来好几条 */
    app_can_poll(now);
    app_can_poll_errors();
    app_led_update(now);

    if (!s_banner_sent && now >= 10u) {
        s_banner_sent = true;
        tx_pushf("\r\nigh6_jxb 固件 %s %s\r\n6 个 DM4310，MIT 模式，CAN1=ID1-3，CAN2=ID4-6，1 kHz 轮询\r\n",
                 __DATE__, __TIME__);
    }

    if (now != s_served_ms) {
        if (now - s_served_ms > 1u) {
            s_skipped_ticks += now - s_served_ms - 1u;
        }
        s_served_ms = now;
        gcomp_update(now);          /* 先更新力矩，再发这一拍的 MIT 帧 */
        home_update(now);           /* 上电回初始位（未使能时不动） */
        send_for_ms(now);
        if (now % APP_STREAM_EVERY_MS == 0u) {
            stream_joints(now);
        }
#if APP_DEBUG_TORQUE
        if (now % APP_TORQUE_EVERY_MS == 0u) {
            stream_torque(now);
        }
#endif
        if (now % APP_REPORT_EVERY_MS == 0u) {
            report();
        }
    }

    tx_drain();
}
