"""igh6_jxb USB CDC 协议层：找设备、分帧、拆 J 行。

固件每 20 ms 打一行（见 app/app_main.c: stream_joints）：

    J <t_ms> <p1> <p2> <p3> <p4> <p5> <p6>     ← mrad，顺序 = 电机 ID 1..6

两个坑，这里统一处理：

1. **打开端口前内核 tty 会积压历史数据**——不清掉的话，第一帧可能是几秒前的旧值
   （甚至是上电初期的全 0）。`open_port()` 里 reset_input_buffer 解决。
2. **一次可能读到半行**——按字节流累积、遇到 `\\n` 才切帧，避免解析半行。
"""

from __future__ import annotations

import glob
import os
import pathlib

import serial

VID = "0483"
PID = "5740"
BAUD = 115200
MAX_BUF = 4096


def find_port() -> str | None:
    """优先找 0483:5740 的 CDC，退化到第一个 /dev/ttyACM*。"""
    for path in sorted(glob.glob("/dev/ttyACM*")):
        name = os.path.basename(path)
        try:
            usb = pathlib.Path(f"/sys/class/tty/{name}/device").resolve().parent
            vid = (usb / "idVendor").read_text().strip()
            pid = (usb / "idProduct").read_text().strip()
        except OSError:
            continue
        if vid == VID and pid == PID:
            return path
    acm = sorted(glob.glob("/dev/ttyACM*"))
    return acm[0] if acm else None


def open_port(port: str | None = None) -> serial.Serial:
    path = port or find_port()
    if not path:
        raise RuntimeError("找不到 /dev/ttyACM*：板子插好了吗？上电了吗？")
    ser = serial.Serial(path, BAUD, timeout=0)
    ser.reset_input_buffer()   # 丢掉打开之前积压的旧数据
    return ser


def parse_joint_frame(line: str) -> list[int] | None:
    """`J 12345 141 -1642 ...` → [141, -1642, ...]；不是关节帧返回 None。"""
    if not line.startswith("J "):
        return None
    parts = line.split()
    if len(parts) != 8:
        return None
    try:
        return [int(v) for v in parts[2:]]
    except ValueError:
        return None


class FrameReader:
    """把串口字节流切成完整的行，交给调用方按前缀分类。

    一次 read 会返回这一瞬间缓冲里所有完整的行——J（关节流）、G（力矩）、
    OK/ERR（命令回执）、中文报告都在里面。别只挑 J，否则没法统计命令丢没丢。
    """

    def __init__(self, ser: serial.Serial):
        self.ser = ser
        self.buf = b""

    def read_lines(self) -> list[str]:
        n = self.ser.in_waiting
        if not n:
            return []
        self.buf += self.ser.read(n)
        if len(self.buf) > MAX_BUF:      # 协议跑飞时别把内存吃光
            self.buf = b""
            return []
        *chunks, self.buf = self.buf.split(b"\n")
        return [c.decode("utf-8", "ignore").strip() for c in chunks]

    def read(self) -> list[int] | None:
        """只取关节帧，返回最新一帧（旧接口，RViz 桥用）。"""
        latest = None
        for line in self.read_lines():
            frame = parse_joint_frame(line)
            if frame is not None:
                latest = frame
        return latest
