#!/usr/bin/env python3
"""用 igh6_jxb 的关节流重新标定软件零点，写 host/joint_offsets.yaml。

原理
----
    q_joint = dir * q_encoder + offset

把实物摆到 URDF 全零位（对应 ros2_gong 的 SolidWorks 装配姿态）后，令 q_joint = 0：

    offset_J = -dir_J * q_encoder_J

q_encoder 直接取固件的 J 行（mrad → rad），和电机 RID 0x51 输出轴位置同源。
目标角可以用 --target 改成非零（比如你只想要"当前姿态 = 某个已知角"）。

安全
----
* 只读串口，不发任何 CAN 帧，不使能、不出力、不写电机 flash 零位
* 采样期间要求每轴稳定（极差 < --tol），否则拒绝写入，除非 --force
* 写之前打印 diff，--write 才会落盘

用法
----
    python3 host/zero_offsets.py --measure              # 只打印，不动文件
    python3 host/zero_offsets.py --write                # 以当前姿态为零位
    python3 host/zero_offsets.py --apply                # 用现有 offset 打印关节角
    python3 host/zero_offsets.py --target 0,0,0,0,0,0 --write
    python3 host/zero_offsets.py --dirs 1,-1,1,1,1,1 --write   # 重定方向
"""

from __future__ import annotations

import argparse
import pathlib
import statistics
import sys
import time

import yaml

from igh6_serial import FrameReader, find_port, open_port

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_OUT = HERE / "joint_offsets.yaml"

FRAMES = 50          # 50 帧 @50Hz = 1 s
HEADER = """\
# 6 轴 robotarm 零点标定结果
#   q_joint = dir * q_encoder + offset      (rad)
#   由 igh6_jxb/host/zero_offsets.py 经 USB CDC 关节流生成
#   同步到 ros2_gong 需手工拷贝（本工具不写那个仓库）
"""


def load_cal(path: pathlib.Path):
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    offsets = raw.get("offsets") or {}
    dirs = raw.get("directions") or {}
    ids = raw.get("can_ids") or {}
    joints = [f"J{i}" for i in range(len(offsets))]
    return joints, ids, dirs, offsets


def dump_cal(joints, ids, dirs, offsets) -> str:
    lines = HEADER.splitlines() + ["", "offsets:"]
    lines += ["  %-4s: %+.6f" % (j, offsets[j]) for j in joints]
    lines += ["", "directions:"]
    lines += ["  %-4s: %+d" % (j, dirs[j]) for j in joints]
    lines += ["", "can_ids:"]
    lines += ["  %-4s: %d" % (j, ids[j]) for j in joints]
    return "\n".join(lines) + "\n"


def sample(port: str, count: int, timeout_s: float = 20.0):
    """读 count 个关节帧，返回每轴的 (均值 mrad, 极差 mrad)。"""
    ser = open_port(port)
    reader = FrameReader(ser)
    samples: list[list[int]] = []
    deadline = time.monotonic() + timeout_s
    while len(samples) < count and time.monotonic() < deadline:
        frame = reader.read()
        if frame is None:
            time.sleep(0.01)
            continue
        samples.append(frame)
    if len(samples) < count:
        raise SystemExit(f"只收到 {len(samples)}/{count} 帧——板子在跑吗？固件烧了吗？")

    means, spreads = [], []
    for i in range(len(samples[0])):
        col = [s[i] for s in samples]
        means.append(statistics.fmean(col))
        spreads.append(max(col) - min(col))
    return means, spreads


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="默认自动找 0483:5740")
    ap.add_argument("--cal", default=str(DEFAULT_OUT), help="读哪个 yaml 取 dir/ids")
    ap.add_argument("--out", default=None, help="写到哪，默认同 --cal")
    ap.add_argument("--target", default="0,0,0,0,0,0", help="各关节目标角(rad)")
    ap.add_argument("--dirs", default=None, help="覆盖 directions，例如 1,-1,1,1,1,1")
    ap.add_argument("--tol", type=float, default=0.005, help="稳定判据：各轴极差上限(rad)")
    ap.add_argument("--frames", type=int, default=FRAMES)
    ap.add_argument("--encoders", default=None,
                    help="跳过采样，直接用给定的 6 个编码器角(rad)，逗号分隔；"
                         "用于复现已经测准、但手臂已移位的标定")
    ap.add_argument("--measure", action="store_true", help="只打印编码器角")
    ap.add_argument("--apply", action="store_true", help="用现有 offset 打印关节角")
    ap.add_argument("--write", action="store_true", help="写入 yaml")
    ap.add_argument("--force", action="store_true", help="不稳定也写")
    args = ap.parse_args()

    cal = pathlib.Path(args.cal)
    out = pathlib.Path(args.out) if args.out else cal
    joints, ids, dirs_saved, offsets = load_cal(cal)

    if args.dirs:
        dirs = {j: int(d) for j, d in zip(joints, args.dirs.split(","))}
    else:
        dirs = {j: int(dirs_saved.get(j, 1)) for j in joints}

    targets = [float(x) for x in args.target.split(",")]
    if len(targets) != len(joints):
        raise SystemExit(f"--target 需要 {len(joints)} 个数")

    if args.encoders:
        given = [float(x) for x in args.encoders.split(",")]
        if len(given) != len(joints):
            raise SystemExit(f"--encoders 需要 {len(joints)} 个数")
        means_mrad = [v * 1000.0 for v in given]
        spreads_mrad = [0.0] * len(given)
        print("（使用 --encoders 给定值，不采样、不检查稳定性）")
    else:
        port = args.port or find_port()
        if not port:
            raise SystemExit("找不到 /dev/ttyACM*；板子插好了吗？")
        means_mrad, spreads_mrad = sample(port, args.frames)

    q_enc = {j: means_mrad[int(ids.get(j, i + 1)) - 1] / 1000.0
             for i, j in enumerate(joints)}
    spread = {j: spreads_mrad[int(ids.get(j, i + 1)) - 1] / 1000.0
              for i, j in enumerate(joints)}

    print(f"{'关节':<6}{'编码器(rad)':>14}{'当前offset':>14}{'新offset':>14}{'Δ':>10}{'极差(mrad)':>12}")
    new_offsets = {}
    unstable = []
    for j, tgt in zip(joints, targets):
        new = tgt - dirs[j] * q_enc[j]
        new_offsets[j] = new
        noise = spread[j] * 1000.0
        flag = ""
        if spread[j] > args.tol:
            unstable.append(j)
            flag = " ← 不稳"
        print(f"{j:<6}{q_enc[j]:>+14.4f}{offsets.get(j, 0.0):>+14.4f}"
              f"{new:>+14.4f}{new - offsets.get(j, 0.0):>+10.4f}{noise:>12.1f}{flag}")

    if unstable and not args.force:
        print(f"\n这些轴在 {args.frames} 帧内极差超过 {args.tol*1000:.0f} mrad："
              f"{', '.join(unstable)}", file=sys.stderr)
        print("手扶着/settling 时重测，或加 --force 强行写入。", file=sys.stderr)
        return 2

    if args.apply:
        print("\n现有 offset 下的关节角：")
        for j in joints:
            print(f"  {j}: {dirs[j] * q_enc[j] + offsets.get(j, 0.0):+.4f} rad")

    if args.write:
        out.write_text(dump_cal(joints, ids, dirs, new_offsets), encoding="utf-8")
        print(f"\n已写入 {out}")
        print("RViz 用的是这份；要给 ros2_gong 自己也用，手工拷过去。")

    if not (args.write or args.apply or args.measure):
        print("\n（只打印，未写入；加 --write 落盘）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
