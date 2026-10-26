/**
 * app_led.h — 三颗用户 LED 的呼吸灯循环
 *
 * R=PH12(TIM5_CH3)、G=PH11(TIM5_CH2)、B=PH10(TIM5_CH1)，PWM 自持在 app_led.c 里。
 */
#ifndef APP_LED_H
#define APP_LED_H

#include <stdint.h>

/* 配好 TIM5 的三路 PWM（在 app_init 里调用一次）*/
void app_led_init(void);

/* 在主循环里反复调用，参数为开机以来的毫秒数 */
void app_led_update(uint32_t ms);

#endif /* APP_LED_H */
