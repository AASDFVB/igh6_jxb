/**
 * app_uart.c — USART1 调试输出（C 板 J30 那个 4-pin「UART1」接口）
 *
 * 引脚：PA9 = USART1_TX（AF7）、PB7 = USART1_RX（AF7）。
 *   STM32F407 的 USART1 有两组可选引脚（PA9/PA10 与 PB6/PB7），本板按原理图
 *   走的是 PA9 + PB7 这种混搭组合——两者都在 AF7 上，是合法组合。
 *   PB5/PB6 已被 CAN2 占用，所以不能用完整的 PB6/PB7 一组。
 *
 * 为什么不用 CubeMX 的 USART 外设：
 *   .ioc 里没有任何 USART 项、HAL_UART_MODULE_ENABLED 也是注释掉的。为了一路
 *   调试输出去改 .ioc 重新生成，会整份重写 Makefile / Src/main.c，风险大于收益
 *   （同 ADR-0005 的取舍），所以这里直接配寄存器，放在 app/ 里自持。
 *
 * 为什么必须有发送队列：
 *   主循环是 1 kHz 的硬实时节拍，一行 40 字节在 115200 下要占 3.5 ms 总线时间。
 *   如果在这里死等 TXE，每行都会吃掉 3~4 个 1 ms 拍，实测直接体现为「丢拍」飙到
 *   13%。所以这里只往环形队列里拷，真正的发送交给每拍调用的 app_uart_drain()，
 *   一次只填满硬件 FIFO 就走，绝不等待。
 *
 * 和 USB CDC 完全独立：没插 USB 时 CDC 那条路会一直 BUSY，不能让它连带拖死 UART。
 */
#include "app_uart.h"

#include "stm32f4xx_hal.h"

#include "app_config.h"

/* PA9 = USART1_TX */
#define UART_TX_PORT GPIOA
#define UART_TX_PIN  9u

/* PB7 = USART1_RX */
#define UART_RX_PORT GPIOB
#define UART_RX_PIN  7u

/* 发送环形队列：115200 下约 11.5 KB/s，固件峰值约 2.5 KB/s，
 * 1 KB 足够吸收 400 ms 的抖动，且远小于 128 KB RAM 的压力线。 */
#define UART_TXQ_SIZE 1024u

static uint8_t s_q[UART_TXQ_SIZE];
static uint16_t s_q_head; /* 写入位置 */
static uint16_t s_q_tail; /* 读出位置 */

/* 接收环形队列：**必须靠中断填**，不能靠主循环轮询。
 * STM32F4 的 USART 只有一个字节的 DR，115200 下每 87 µs 到一个字节；
 * 而主循环一圈（CAN 收发 + 重力补偿 + 打日志）经常超过 100 µs，
 * 轮询方式必然 ORE 丢字节——实测丢了 73%（2.4 KB/s 灌进来只收到 642 B/s）。
 * 中断里每来一个字节就搬进环形缓冲，主循环再慢慢取。 */
#define UART_RXQ_SIZE 512u

static volatile uint8_t s_rxq[UART_RXQ_SIZE];
static volatile uint16_t s_rxq_head; /* ISR 写 */
static volatile uint16_t s_rxq_tail; /* 主循环读 */
static volatile uint32_t s_rx_overruns;

static uint16_t q_used(void)
{
    return (uint16_t)((s_q_head + UART_TXQ_SIZE - s_q_tail) % UART_TXQ_SIZE);
}

void app_uart_init(void)
{
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

    /* --- TX: PA9，复用推挽，高速 --- */
    UART_TX_PORT->MODER &= ~(3u << (UART_TX_PIN * 2u));
    UART_TX_PORT->MODER |= (2u << (UART_TX_PIN * 2u));
    UART_TX_PORT->OTYPER &= ~(1u << UART_TX_PIN);
    UART_TX_PORT->OSPEEDR |= (3u << (UART_TX_PIN * 2u));
    UART_TX_PORT->PUPDR &= ~(3u << (UART_TX_PIN * 2u));
    /* PA9 → AFR[1]，位域编号从 pin 8 起算 */
    UART_TX_PORT->AFR[1] &= ~(0xFu << ((UART_TX_PIN - 8u) * 4u));
    UART_TX_PORT->AFR[1] |= (7u << ((UART_TX_PIN - 8u) * 4u));

    /* --- RX: PB7，复用 + 上拉（模块不驱动时别浮空） --- */
    UART_RX_PORT->MODER &= ~(3u << (UART_RX_PIN * 2u));
    UART_RX_PORT->MODER |= (2u << (UART_RX_PIN * 2u));
    UART_RX_PORT->PUPDR &= ~(3u << (UART_RX_PIN * 2u));
    UART_RX_PORT->PUPDR |= (1u << (UART_RX_PIN * 2u));
    /* PB7 → AFR[0] */
    UART_RX_PORT->AFR[0] &= ~(0xFu << (UART_RX_PIN * 4u));
    UART_RX_PORT->AFR[0] |= (7u << (UART_RX_PIN * 4u));

    /* --- 波特率：BRR = fCK/baud，高 12 位整数 + 低 4 位 1/16 小数 --- */
    uint32_t pclk2 = HAL_RCC_GetPCLK2Freq();
    uint32_t div_x100 = (uint32_t)(((uint64_t)pclk2 * 100u) / (16u * (uint64_t)APP_UART_BAUD));
    uint32_t mantissa = div_x100 / 100u;
    uint32_t fraction = (((div_x100 % 100u) * 16u) + 50u) / 100u;
    USART1->BRR = (mantissa << 4) | (fraction & 0x0Fu);

    /* 8N1 是默认值，不用动 CR2/CR3。RX 收遥操作指令，必须开中断，见上面注释 */
    s_rxq_head = 0;
    s_rxq_tail = 0;
    s_rx_overruns = 0;

    USART1->CR1 = USART_CR1_TE | USART_CR1_RE | USART_CR1_RXNEIE | USART_CR1_UE;

    NVIC_SetPriority(USART1_IRQn, 5);
    NVIC_EnableIRQ(USART1_IRQn);

    s_q_head = 0;
    s_q_tail = 0;
}

/* 每来一个字节进一次中断：115200 下约 1.15 万次/秒，M4 上只有约 1% CPU。
 * 这就是不丢字节的关键——主循环再慢也影响不到接收。 */
void USART1_IRQHandler(void)
{
    const uint32_t sr = USART1->SR;
    if ((sr & USART_SR_RXNE) != 0u) {
        const uint8_t b = (uint8_t)USART1->DR; /* 读 DR 同时清 RXNE 与 ORE */
        const uint16_t next = (uint16_t)((s_rxq_head + 1u) % UART_RXQ_SIZE);
        if (next != s_rxq_tail) {
            s_rxq[s_rxq_head] = b;
            s_rxq_head = next;
        } else {
            s_rx_overruns++; /* 环形缓冲也满了（主循环太久没取） */
        }
    } else if ((sr & USART_SR_ORE) != 0u) {
        (void)USART1->DR; /* 只清 ORE */
        s_rx_overruns++;
    }
}

void app_uart_put(const char *s, size_t n)
{
    for (size_t i = 0; i < n; i++) {
        uint16_t next = (uint16_t)((s_q_head + 1u) % UART_TXQ_SIZE);
        if (next == s_q_tail) {
            return; /* 队列满：丢剩下的（宁可少打几行，也不阻塞 1 kHz 节拍） */
        }
        s_q[s_q_head] = (uint8_t)s[i];
        s_q_head = next;
    }
}

void app_uart_drain(void)
{
    while (s_q_tail != s_q_head) {
        if ((USART1->SR & USART_SR_TXE) == 0u) {
            return; /* 硬件还忙，下一拍再来，绝不等 */
        }
        USART1->DR = s_q[s_q_tail];
        s_q_tail = (uint16_t)((s_q_tail + 1u) % UART_TXQ_SIZE);
    }
}

uint16_t app_uart_pending(void)
{
    return q_used();
}

size_t app_uart_read(uint8_t *buf, size_t max)
{
    size_t n = 0;
    /* 单生产者（ISR 写 head）/ 单消费者（主循环写 tail），无需关中断 */
    while (n < max && s_rxq_tail != s_rxq_head) {
        buf[n++] = s_rxq[s_rxq_tail];
        s_rxq_tail = (uint16_t)((s_rxq_tail + 1u) % UART_RXQ_SIZE);
    }
    return n;
}

uint32_t app_uart_rx_overruns(void)
{
    return s_rx_overruns;
}
