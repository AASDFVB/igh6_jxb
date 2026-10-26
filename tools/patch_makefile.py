#!/usr/bin/env python3
"""把 app/ 的源文件和 -Iapp 同步进 CubeMX 生成的 Makefile。

CubeMX 每次 GENERATE CODE 都会整份重写 Makefile，而它不认识 app/ —— 生成物里
app/ 出现 0 次。本脚本把缺的补上、已有的保留，把"重新生成 → 忘了补 → 烧的还是
旧固件"这个坑堵死。

用法（每次 CubeMX 生成代码之后跑一次）：
    python3 tools/patch_makefile.py

幂等：重复跑不会重复插入。新增 app/*.c 也只需要重跑一次。
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
    sources = app_sources()
    if not sources:
        print("app/ 下没有 .c 文件，跳过", file=sys.stderr)
        return 1

    match = re.search(r"^C_SOURCES\s*=\s*\\\n", text, flags=re.M)
    if not match:
        print("没找到 C_SOURCES 锚点，Makefile 结构变了？", file=sys.stderr)
        return 1

    missing = [s for s in sources if f"{s} " not in text and f"{s}\\\n" not in text]
    if missing:
        block = "".join(f"{s} \\\n" for s in missing)
        text = text[: match.end()] + block + text[match.end():]

    inc_added = "Iapp" not in text
    if inc_added:
        text, n = re.subn(r"^(-IInc\s*\\\n)", r"-Iapp \\\n\1", text, count=1, flags=re.M)
        if n != 1:
            print("没找到 -IInc 锚点，Makefile 结构变了？", file=sys.stderr)
            return 1

    if not missing and not inc_added:
        print("Makefile 已是最新，无需改动")
        return 0

    MAKEFILE.write_text(text, encoding="utf-8")
    if missing:
        print(f"已补 {len(missing)} 个源文件：" + ", ".join(missing))
    if inc_added:
        print("已补 -Iapp")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
