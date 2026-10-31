#!/usr/bin/env python3
"""把 igh6_jxb 的 USB CDC 关节流喂给 /joint_states，让 RViz 里的模型跟着实物动。

    q_joint = dir * (p_mrad / 1000) + offset

关节名 / 方向 / 零点读自本目录的 joint_offsets.yaml（不碰 ros2_gong）。

用法:
    python3 host/rviz_joint_states.py
    python3 host/rviz_joint_states.py --port /dev/ttyACM1
    python3 host/rviz_joint_states.py --invert J1     # 临时翻转某轴
    python3 host/rviz_joint_states.py --once          # 打一帧就退出（不开 ROS）
"""

from __future__ import annotations

import argparse
import pathlib
import sys
import time

import yaml

HERE = pathlib.Path(__file__).resolve().parent
DEFAULT_CAL = HERE / "joint_offsets.yaml"
STALL_WARN_S = 1.0

from igh6_serial import FrameReader, find_port, open_port


def load_calibration(path: pathlib.Path):
    """读 offsets / directions / can_ids，返回按 can_id 排序的关节表。"""
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    offsets = raw.get("offsets") or {}
    dirs = raw.get("directions") or {}
    can_ids = raw.get("can_ids") or {}

    joints = []
    for name in sorted(offsets.keys(), key=lambda j: int(j[1:])):
        joints.append({
            "name": name,
            "can_id": int(can_ids.get(name, int(name[1:]) + 1)),
            "dir": float(dirs.get(name, 1)),
            "offset": float(offsets[name]),
        })
    joints.sort(key=lambda j: j["can_id"])
    return joints


def joint_positions(joints, values, invert=frozenset()):
    """电机侧的 mrad → 关节角（rad），按 can_id 查表。"""
    by_id = {j["can_id"]: j for j in joints}
    out = {}
    for motor_id, mrad in enumerate(values, start=1):
        joint = by_id.get(motor_id)
        if joint is None:
            continue
        sign = -1.0 if joint["name"] in invert else 1.0
        out[joint["name"]] = sign * joint["dir"] * (mrad / 1000.0) + joint["offset"]
    return out


def run_once(args, joints):
    """不开 ROS，只打一帧，方便先验证串口和换算。"""
    ser = open_port(args.port)
    reader = FrameReader(ser)
    deadline = time.monotonic() + 5.0
    while time.monotonic() < deadline:
        values = reader.read()
        if values is None:
            time.sleep(0.01)
            continue
        for name, q in joint_positions(joints, values, set(args.invert)).items():
            print(f"{name}: {q:+.4f} rad")
        return 0
    print("5 秒内没收到 J 行——固件是不是还没烧带关节流的版本？", file=sys.stderr)
    return 1


def run_ros(args, joints):
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    class SerialJointStates(Node):
        def __init__(self):
            super().__init__("igh6_serial_joint_states")
            self.pub = self.create_publisher(JointState, "joint_states", 10)
            self.ser = None
            self.frames = 0
            self.last_rx = 0.0
            self.started = time.monotonic()
            self.open_serial()
            self.create_timer(0.02, self.poll)
            self.create_timer(5.0, self.status)

        def open_serial(self):
            try:
                self.ser = open_port(args.port)
                self.reader = FrameReader(self.ser)
                self.get_logger().info(f"已打开 {self.ser.port}")
            except Exception as e:  # noqa: BLE001 - 串口异常种类多，统一重试
                self.ser = None
                self.get_logger().warn(f"打开失败（{e}），稍后重试", throttle_duration_sec=5.0)

        def poll(self):
            if self.ser is None:
                self.open_serial()
                return
            try:
                values = self.reader.read()
            except Exception:  # noqa: BLE001
                self.get_logger().warn("串口断开，重连")
                self.ser = None
                return

            if values is None:
                return

            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            for name, q in joint_positions(joints, values, set(args.invert)).items():
                msg.name.append(name)
                msg.position.append(q)
            if not msg.name:
                return
            self.pub.publish(msg)
            self.frames += 1
            self.last_rx = time.monotonic()

        def status(self):
            if not self.frames:
                self.get_logger().warn("还没收到 J 行——固件是不是没烧新版本？")
                return
            rate = self.frames / max(1e-6, time.monotonic() - self.started)
            self.get_logger().info(f"{self.frames} 帧，平均 {rate:.1f} Hz")
            if time.monotonic() - self.last_rx > STALL_WARN_S:
                self.get_logger().warn("超过 1 s 没有关节数据")

    rclpy.init()
    node = SerialJointStates()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", default=None, help="串口设备，默认自动找 0483:5740")
    ap.add_argument("--calib", default=str(DEFAULT_CAL), help="joint_offsets.yaml")
    ap.add_argument("--invert", nargs="*", default=[], help="临时翻转方向的关节名")
    ap.add_argument("--once", action="store_true", help="只打一帧，不启动 ROS")
    args = ap.parse_args()

    if not args.port:
        args.port = find_port()
    if not args.port:
        sys.exit("找不到 /dev/ttyACM*；板子插好了吗？")

    joints = load_calibration(pathlib.Path(args.calib))
    return run_once(args, joints) if args.once else run_ros(args, joints)


if __name__ == "__main__":
    sys.exit(main())
