# 手写 CMakeLists，CubeMX 只负责生成 HAL 初始化

CubeMX 6.18 的 `Toolchains` 里虽然列了 CMake，但它只对 STM32MP1 系列开放；STM32F4 只有 MDK-ARM / EWARM / STM32CubeIDE 三种选择。所以工程分成两层：CubeMX 生成 `Src/Inc/Drivers/Middlewares/Startup/*.ld`（工具链选 STM32CubeIDE，这样拿到的是 GNU 语法的启动文件和链接脚本），构建系统由仓库里的 `CMakeLists.txt` + `cmake/arm-none-eabi.cmake` 提供。这样既保住 CubeMX 的图形化配置，又能用 CMake + VSCode 而不用装 STM32CubeIDE。

## Consequences

- 重新生成后必须保留 `Src/main.c`、`Src/stm32f4xx_it.c` 里 USER CODE 段的现有内容；`app/` 与 `test/` 不在生成范围内，不会被覆盖。
- 工具链装在 `~/.local/opt`（不需要 sudo）：版本变化时改 `cmake/arm-none-eabi.cmake` 的 `TOOLCHAIN_ROOT`。
