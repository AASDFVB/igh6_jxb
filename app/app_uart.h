/**
 * app_uart.h — USART1 调试输出（C 板 J30 那个 4-pin「UART1」接口）
 *
 * 引脚：PA9 = USART1_TX（AF7），PB7 = USART1_RX（AF7）。
 * 波特率 APP_UART_BAUD（app_config.h），帧格式 8N1，挂在 APB2 上。
 *
 * 全部非阻塞：app_uart_put() 只往环形队列里拷，app_uart_drain() 每一拍最多
 * 填满硬件 FIFO 就走，绝不等待——1 kHz 主循环里不能有任何死等。
 */
#ifndef APP_UART_H
#define APP_UART_H

#include <stddef.h>
#include <stdint.h>

/* 配好 PA9/PB7 + USART1（在 app_init 里调用一次） */
void app_uart_init(void);

/* 投递到发送队列，满了就丢尾巴；不阻塞 */
void app_uart_put(const char *s, size_t n);

/* 在主循环里每一拍调用一次：把队列里的字节尽可能塞进 USART */
void app_uart_drain(void);

/* 取出已收到的字节（不含阻塞）；返回实际取到的个数。
 * 顺带清 ORE：读 SR 再读 DR 才能把溢出标志清掉，否则溢出一次以后 RXNE 不再置位。 */
size_t app_uart_read(uint8_t *buf, size_t max);

/* 还没发出去的字节数（调试用） */
uint16_t app_uart_pending(void);

/* 接收侧丢字节次数：硬件 ORE（主循环读太慢）或软件环形缓冲满。
 * 正常应为 0；不为 0 说明有字节被丢在链路上。 */
uint32_t app_uart_rx_overruns(void);

#endif /* APP_UART_H */
