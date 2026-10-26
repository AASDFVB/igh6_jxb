# 改用 CubeMX 原生 Makefile，app/ 源文件由补丁脚本补回

上一版用了手写 CMake，理由是"CubeMX 生成的构建文件会被整份重写"。实际把工具链切成 `Makefile` 之后（CubeMX 6.18 对 STM32F4 **是**支持 Makefile 的），代价只有一个：生成的 `Makefile` 不认识 `app/`，每次 GENERATE CODE 都要把三个源文件和 `-Iapp` 补回去。与其为了躲这一点维护一套 CMake，不如把"补回去"这件事做成一个幂等脚本 `tools/patch_makefile.py`，并把它串进 VSCode 的 `generate` 任务——构建系统从此只有一套，和 CubeMX 完全同源。

## Consequences

- 每次重新生成代码后必须跑 `python3 tools/patch_makefile.py`；不跑不会报错，只会静默地用旧目标文件，症状是"改了代码但行为没变"。脚本已做成幂等，重复跑无副作用。
- `Makefile`、`startup_stm32f407xx.s`、`STM32F407xx_FLASH.ld` 都是生成物，不要手改。
- 编译用 `make GCC_PATH=<xpack gcc bin> -j`，不需要把工具链塞进系统 PATH。
