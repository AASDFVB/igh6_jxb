/**
 * app_led.c — 三颗用户 LED 的呼吸灯循环（R=PH12, G=PH11, B=PH10）
 *
 * 引脚是 TIM5 的 CH3/CH2/CH1（AF2）。这里直接写 TIM5 寄存器而不是用 HAL 的
 * TIM 驱动：CubeMX 无 UI 模式加不上 TIM5（会在 ProjectSettingsPanel 崩），
 * 而 app/ 下面的代码不会被重新生成覆盖，所以把 PWM 初始化放在这里自持。
 *
 * PWM：84 MHz / 84 = 1 MHz 计数，ARR=999 → 1 kHz，10 位分辨率。
 * CH1=蓝、CH2=绿、CH3=红（与官方 bsp_led.c 的顺序一致）。
 */
#include "app_led.h"

#include <stdint.h>

#include "main.h"

#define LED_MAX_DUTY 800u  /* 别拉满，呼吸到 100% 有点晃眼 */

static uint32_t tri(uint32_t ms, uint32_t phase, uint32_t period)
{
    /* 0 → 满 → 0 的三角波 */
    const uint32_t half = period / 2u;
    const uint32_t t = (ms + phase) % period;
    const uint32_t up = (t < half) ? t : (period - t);
    return (up * LED_MAX_DUTY) / half;
}

static void led_pwm_init(void)
{
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();

    GPIO_InitTypeDef gpio = {0};
    gpio.Pin = GPIO_PIN_10 | GPIO_PIN_11 | GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_HIGH;
    gpio.Alternate = GPIO_AF2_TIM5;
    HAL_GPIO_Init(GPIOH, &gpio);

    TIM5->PSC = 84u - 1u;    /* 1 MHz */
    TIM5->ARR = 1000u - 1u;  /* 1 kHz PWM */
    TIM5->CCR1 = 0;
    TIM5->CCR2 = 0;
    TIM5->CCR3 = 0;
    /* 三个通道都设为 PWM mode 1，并打开预装载（改 CCR 不会出现毛刺） */
    TIM5->CCMR1 = (6u << 4) | (1u << 3) | (6u << 12) | (1u << 11);
    TIM5->CCMR2 = (6u << 4) | (1u << 3);
    TIM5->CCER = TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E;
    TIM5->CR1 = TIM_CR1_ARPE | TIM_CR1_CEN;
}

void app_led_init(void)
{
    led_pwm_init();
}

void app_led_update(uint32_t ms)
{
    /* 一整圈 3 s：三个通道依次呼吸，相位各差 1/3 圈 */
    const uint32_t period = 3000u;
    const uint32_t phase = period / 3u;

    TIM5->CCR1 = tri(ms, 0u, period);         /* 蓝 */
    TIM5->CCR2 = tri(ms, phase, period);      /* 绿 */
    TIM5->CCR3 = tri(ms, phase * 2u, period); /* 红 */
}
