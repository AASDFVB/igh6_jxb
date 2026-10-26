#!/usr/bin/env bash
# 重新生成 CubeMX 代码，并把 app/ 源文件补回 Makefile。
#
# 为什么需要这个脚本：CubeMX 的 -q 脚本模式不认相对路径（脚本里的
# `config load igh6_jxb.ioc` 会找不到文件，然后静默什么都不做），所以这里
# 用绝对路径现写脚本；生成完必须补 Makefile，否则 app/*.c 会从构建里消失，
# 症状是"编译通过但烧进去还是旧固件"。
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
CUBEMX="${CUBEMX:-$HOME/.local/opt/stm32cubemx/MX/STM32CubeMX}"

if [ ! -x "$CUBEMX" ]; then
    echo "找不到 CubeMX：$CUBEMX" >&2
    exit 1
fi

TMP="$(mktemp)"
trap 'rm -f "$TMP"' EXIT

cat > "$TMP" <<EOF
config load $ROOT/igh6_jxb.ioc
project name igh6_jxb
project path $ROOT
project toolchain Makefile
project generate
exit
EOF

"$CUBEMX" -q "$TMP"
python3 "$ROOT/tools/patch_makefile.py"
echo "生成完成。接下来：make GCC_PATH=\$HOME/.local/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin -j"
