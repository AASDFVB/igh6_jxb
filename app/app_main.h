/**
 * app_main.h — 应用入口：1 kHz 轮询、使能序列、1 Hz USB 上报
 */
#ifndef APP_MAIN_H
#define APP_MAIN_H

void app_init(void);      /* main() 的 USER CODE 2 里调用 */
void app_loop(void);      /* 主循环里调用，负责上报 */
void app_tick_1khz(void); /* SysTick 中断里调用 */

#endif /* APP_MAIN_H */
