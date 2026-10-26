#!/usr/bin/env python3
"""往 CubeMX 生成的 Makefile 里补回 app/ 的源文件和 include 路径。

CubeMX 每次 GENERATE CODE 都会整份重写 Makefile，而它不认识 app/ —— 生成物里
app/ 出现 0 次。本脚本把那几行加回去，把"重新生成 → 忘了补 → 烧的还是旧固件"
这个坑堵死。

用法（每次 CubeMX 生成代码之后跑一次）：
    python3 tools/patch_makefile.py

幂等：已经补过就直接退出。
"""
from __future__ import annotations

import pathlib
import re
import sys

ROOT = pathlib.Path(__file__).resolve().parent.parent
MAKEFILE = ROOT / "Makefile"


def app_sources() -> list[str]:
    return sorted(p.relative_to(ROOT).as_posix() for p in ROOT.glob("app/*.c"))


def main() -> int:
    if not MAKEFILE.exists():
        print(f"找不到 {MAKEFILE}：先用 CubeMX 生成代码（toolchain 选 Makefile）", file=sys.stderr)
        return 1

    text = MAKEFILE.read_text(encoding="utf-8")
    if "app/dm_motor.c" in text:
        print("Makefile 里已有 app 源文件，跳过")
        return 0

    sources = app_sources()
    if not sources:
        print("app/ 下没有 .c 文件，跳过", file=sys.stderr)
        return 1

    src_block = "".join(f"{s} \\\n" for s in sources)
    text, n_src = re.subn(r"^C_SOURCES\s*=\s*\\\n", lambda _: f"C_SOURCES =  \\\n{src_block}", text, count=1, flags=re.M)
    text, n_inc = re.subn(r"^(-IInc\s*\\\n)", r"-Iapp \\\n\1", text, count=1, flags=re.M)

    if n_src != 1 or n_inc != 1:
        print(f"没找到 C_SOURCES / -IInc 锚点（src={n_src}, inc={n_inc}），Makefile 结构变了？", file=sys.stderr)
        return 1

    MAKEFILE.write_text(text, encoding="utf-8")
    print(f"已插入 {len(sources)} 个 app 源文件 + -Iapp")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
