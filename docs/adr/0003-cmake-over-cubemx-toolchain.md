# 手写 CMakeLists，而不是用 CubeMX 生成的 Makefile

CubeMX 6.18 对 STM32F4 其实能生成 **Makefile**（也能生成 STM32CubeIDE；只有 CMake 是 STM32MP1 专属）。但生成的 Makefile 有两个硬伤：每次 GENERATE CODE 都整份重写，而且源文件清单是写死的——它不认识 `app/`，实测生成物里 `app/` 出现 0 次。也就是说每动一次 `.ioc` 都要手工把 5 个 app 源文件补回去，漏了就是"编译通过但行为还是旧的"。所以构建系统留给手写的 CMake：它 glob `app/*.c`，文件名也不与 CubeMX 的任何生成物冲突（生成物是 `Makefile`，我们是 `CMakeLists.txt`）。

## Consequences

- CubeMX 生成代码这一步仍然要做，但只产出 `Src/Inc/Drivers/Middlewares/Startup/*.ld`；构建由 `CMakeLists.txt` + `cmake/arm-none-eabi.cmake` 负责。
- 重新生成后必须保留 `Src/main.c`、`Src/stm32f4xx_it.c` 里 USER CODE 段的现有内容；`app/` 与 `test/` 不在生成范围内。
- 想敲 `make` 也可以：`make -C build/Debug`，CMake 生成的就是 Makefile。
- 工具链装在 `~/.local/opt`（不需要 sudo）：版本变化时改 `cmake/arm-none-eabi.cmake` 的 `TOOLCHAIN_ROOT`。
