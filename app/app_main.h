/**
 * app_main.h — 应用入口：1 kHz 轮询、使能序列、USB 上报/下行
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

#include <stdint.h>

void app_init(void);      /* main() 的 USER CODE 2 里调用 */
void app_loop(void);      /* 主循环里调用，负责上报 */
void app_tick_1khz(void); /* SysTick 中断里调用 */

/* USB CDC 收到的字节（在 USB 中断里调用，只入队，不做解析） */
void app_serial_input(const uint8_t *buf, uint32_t len);

#endif /* APP_MAIN_H */
