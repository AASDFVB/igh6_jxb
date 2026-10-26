#!/usr/bin/env bash
# 一条命令：编译 + 烧录（+ 校验 + 复位）。
#
#   bash tools/flash.sh            # 编译并烧录
#   bash tools/flash.sh --erase    # 先全片擦除再烧
#
# 需要：ST-Link 插好、板子上电。工具链默认在 ~/.local/opt。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
GCC_BIN="${GCC_BIN:-$HOME/.local/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin}"
OPENOCD="${OPENOCD:-$HOME/.local/opt/xpack-openocd-0.12.0-7/bin/openocd}"
ELF="$ROOT/build/igh6_jxb.elf"

make -C "$ROOT" -j"$(nproc)" GCC_PATH="$GCC_BIN"

COMMANDS="program $ELF verify reset exit"
if [ "${1:-}" = "--erase" ]; then
    COMMANDS="init; reset halt; stm32f4x mass_erase 0; program $ELF verify reset exit"
fi

"$OPENOCD" -f interface/stlink.cfg -f target/stm32f4x.cfg -c "$COMMANDS"
