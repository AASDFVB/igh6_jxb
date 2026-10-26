# LED 的 PWM 自持在 app_led.c，不走 CubeMX

三颗用户 LED（R=PH12 / G=PH11 / B=PH10）是 TIM5 的 CH3/CH2/CH1。按常规应在 CubeMX 里把这三个引脚配成 `S_TIM5_CHn`，但这台机器的 CubeMX 在**无 UI 模式**下加载带 TIM5 的 `.ioc` 会崩（`ProjectSettingsPanel` 空指针），生成不出 `MX_TIM5_Init`，也就没有 HAL TIM 驱动。所以改为在 `app_led.c` 里直接配置 TIM5 的寄存器（PSC/ARR/CCMR/CCER/CR1）并写 CCR 调占空比：只需要 RCC 与 GPIO 两个 HAL 模块，不引入 `stm32f4xx_hal_tim.c`，也不改 CubeMX 生成的任何文件。

## Consequences

- `igh6_jxb.ioc` 里看不到这三颗 LED——改 `.ioc` 重新生成不会碰它们，`app_led.c` 也不会被覆盖。
- 若将来在 CubeMX 里给 LED 配了别的复用功能，会与这里的 AF2 配置打架：谁后初始化谁生效（`app_init` 在 `MX_GPIO_Init` 之后，所以实际是这里赢）。
- 呼吸周期 3 s、相位各差 1/3 圈、上限 80% 占空比，改 `app_led.c` 顶部三个常量即可。
