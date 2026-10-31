#!/usr/bin/env python3
"""串口 ↔ viser 双向桥：模型跟随实物，滑块反过来驱动实物。

    MCU ──J 行(50 Hz)──▶ viser 模型
    viser 滑块 ──P q0..q5──▶ MCU（固件内按 0.8 rad/s 限速，不会被一帧拉飞）

固件侧命令（换行结尾）：
    P q0 q1 q2 q3 q4 q5    关节角目标，rad
    M 0 / M 1              放开（只剩重力补偿）/ 位置保持
    K kp kd                运行时改刚度、阻尼

用法:
    python3 host/viser_bridge.py
    python3 host/viser_bridge.py --port 8081 --serial /dev/ttyACM1
    python3 host/viser_bridge.py --urdf ~/Desktop/ros2_gong/src/robotarm/urdf/robotarm.urdf
"""

from __future__ import annotations

import argparse
import os
import pathlib
import re
import sys
import time

import serial                     # 系统 pyserial，先于 viser-libs 导入，避免被遮蔽
import yaml

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent
DEFAULT_URDF = pathlib.Path.home() / "Desktop/ros2_gong/src/robotarm/urdf/robotarm.urdf"
DEFAULT_CAL = HERE / "joint_offsets.yaml"
VENDOR = pathlib.Path.home() / "Documents/deepseek-harness/default-workspace/viser-libs"

if VENDOR.exists():
    sys.path.insert(0, str(VENDOR))

import numpy as np          # noqa: E402
import trimesh              # noqa: E402
import viser                # noqa: E402
import yourdfpy             # noqa: E402

from igh6_serial import FrameReader, find_port, open_port, parse_joint_frame   # noqa: E402
from plan_path import compute_plan as plan_compute   # noqa: E402


# ---------------------------------------------------------------- URDF 工具
def mat_to_pos_wxyz(T):
    pos = np.asarray(T[:3, 3], dtype=np.float64)
    R = np.asarray(T[:3, :3], dtype=np.float64)
    tr = np.trace(R)
    if tr > 0:
        s = np.sqrt(tr + 1.0) * 2
        w, x = 0.25 * s, (R[2, 1] - R[1, 2]) / s
        y, z = (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w, x = (R[2, 1] - R[1, 2]) / s, 0.25 * s
        y, z = (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w, x = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s
        y, z = 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w, x = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s
        y, z = (R[1, 2] + R[2, 1]) / s, 0.25 * s
    q = np.array([w, x, y, z], dtype=np.float64)
    return pos, q / np.linalg.norm(q)


def resolve_mesh(filename: str, mesh_dir: str) -> str:
    fn = filename
    if fn.startswith("package://"):
        fn = fn[len("package://"):]
        fn = fn.split("/", 1)[1] if "/" in fn else fn
    cand = os.path.join(mesh_dir, os.path.basename(fn))
    return cand if os.path.exists(cand) else os.path.abspath(fn)


def read_teleop_rate(default: float = 1.5) -> float:
    """从 app/app_config.h 读 APP_TELEOP_RAD_S，读不到就用默认值。"""
    try:
        text = (REPO / "app" / "app_config.h").read_text(encoding="utf-8")
    except OSError:
        return default
    m = re.search(r"#define\s+APP_TELEOP_RAD_S\s+([0-9.]+)f", text)
    return float(m.group(1)) if m else default


def load_cal(path: pathlib.Path):
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    names = sorted(raw["offsets"], key=lambda j: int(j[1:]))
    return [dict(name=n, can_id=int(raw["can_ids"][n]),
                 dir=float(raw["directions"][n]), offset=float(raw["offsets"][n]))
            for n in names]


def _val(event, fallback):
    """viser 不同版本的回调参数不一样：event.target.value 或直接 value。"""
    tgt = getattr(event, "target", event)
    return getattr(tgt, "value", fallback)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=str(DEFAULT_URDF))
    ap.add_argument("--cal", default=str(DEFAULT_CAL))
    ap.add_argument("--serial", default=None)
    ap.add_argument("--port", type=int, default=8081)
    ap.add_argument("--host", default="0.0.0.0")
    args = ap.parse_args()

    urdf_path = pathlib.Path(args.urdf).expanduser()
    mesh_dir = str(urdf_path.parent.parent / "meshes")
    urdf = yourdfpy.URDF.load(str(urdf_path), mesh_dir=mesh_dir, build_scene_graph=True)
    actuated = [j.name for j in urdf.robot.joints
                if j.type in ("revolute", "prismatic", "continuous")]
    joints = load_cal(pathlib.Path(args.cal))
    cfg = {name: 0.0 for name in actuated}
    urdf.update_cfg(cfg)

    # 收集网格
    cache, items = {}, []
    for link in urdf.robot.links:
        for vi, vis in enumerate(link.visuals):
            if vis.geometry.mesh is None:
                continue
            path = resolve_mesh(vis.geometry.mesh.filename, mesh_dir)
            if not os.path.exists(path):
                print(f"[warn] 缺网格 {path}", file=sys.stderr)
                continue
            if path not in cache:
                cache[path] = trimesh.load(path, force="mesh", process=False)
            items.append(dict(name=f"/robot/{link.name}/v{vi}", link=link.name,
                              origin=np.asarray(vis.origin, dtype=np.float64),
                              mesh=cache[path],
                              scale=vis.geometry.mesh.scale if vis.geometry.mesh.scale else 1.0))

    server = viser.ViserServer(host=args.host, port=args.port, label="igh6_jxb")
    try:
        server.scene.set_up_direction("+z")
    except Exception:
        pass
    server.scene.add_grid("/grid", width=2.0, height=2.0, plane="xy", cell_size=0.1)

    for it in items:
        T = urdf.get_transform(it["link"]) @ it["origin"]
        pos, wxyz = mat_to_pos_wxyz(T)
        it["handle"] = server.scene.add_mesh_trimesh(
            it["name"], it["mesh"], scale=it["scale"], position=pos, wxyz=wxyz)

    def refresh():
        urdf.update_cfg(cfg)
        for it in items:
            T = urdf.get_transform(it["link"]) @ it["origin"]
            pos, wxyz = mat_to_pos_wxyz(T)
            it["handle"].position = pos
            it["handle"].wxyz = wxyz

    refresh()

    # ---------------- 规划：末端路径预演 ----------------
    # 固件把目标角按 APP_TELEOP_RAD_S 限速"直线"追过去，所以预演就是照这个规则走一遍 FK。
    # 直接读固件头文件，避免两边各写一份、改了忘了同步。
    TELEOP_RAD_S = read_teleop_rate()
    tool_link = next((l.name for l in urdf.robot.links if l.name == "tool0"),
                     urdf.robot.links[-1].name)
    plan_target = {n: 0.0 for n in actuated}
    plan_handles = {"path": None, "target": None}

    def tool_point(qdict):
        urdf.update_cfg(qdict)
        return np.asarray(urdf.get_transform(tool_link)[:3, 3], dtype=float)

    def compute_plan(q_from, q_to):
        # 规则在 host/plan_path.py 里，test/test_plan.py 有单测
        return plan_compute(tool_point, q_from, q_to, actuated, TELEOP_RAD_S)

    def clear_plan():
        for k, h in plan_handles.items():
            if h is not None:
                h.remove()
                plan_handles[k] = None

    def redraw_plan():
        clear_plan()
        pts, dur = compute_plan(dict(cfg), plan_target)
        refresh()                      # compute_plan 改过 urdf 的 cfg，恢复成真实姿态
        if len(pts) < 2:
            plan_info.value = "已在目标位"
            return
        plan_handles["path"] = server.scene.add_spline_catmull_rom(
            "/plan/path", pts, color=(255, 170, 40), thickness=0.004)
        plan_handles["target"] = server.scene.add_icosphere(
            "/plan/target", radius=0.015, color=(255, 170, 40), opacity=0.5,
            position=pts[-1])
        length = float(np.sum(np.linalg.norm(np.diff(pts, axis=0), axis=1)))
        plan_info.value = f"{length * 100:.1f} cm · {dur:.1f} s"

    # ---------------- GUI ----------------
    state = {"mode": "跟随", "teleop": False, "syncing": False, "dirty": False,
             "rate": 0.0, "frames": 0, "t0": time.time(), "ser": None, "reader": None,
             "sent": 0, "acked": 0, "rejected": 0}

    with server.gui.add_folder("连接"):
        status = server.gui.add_text("状态", initial_value="等待串口…")
        torque = server.gui.add_text("力矩 mN·m", initial_value="-")

    with server.gui.add_folder("模式"):
        mode = server.gui.add_dropdown(
            "模式", ["跟随", "遥操作", "规划"], initial_value="跟随")
        plan_info = server.gui.add_text("规划", initial_value="-")
        b_exec = server.gui.add_button("执行规划")
        b_clear = server.gui.add_button("清空规划")
        b_home = server.gui.add_button("回初始位（全 0）")
        b_free = server.gui.add_button("放开（只剩重力补偿）")
        b_hold = server.gui.add_button("保持（按当前刚度）")

    sliders = {}
    with server.gui.add_folder("关节"):
        for name in actuated:
            lim = next((j.limit for j in urdf.robot.joints if j.name == name), None)
            lo = float(lim.lower) if lim and lim.lower is not None else -np.pi
            hi = float(lim.upper) if lim and lim.upper is not None else np.pi
            if hi <= lo:
                hi = lo + 1.0
            h = server.gui.add_slider(name, min=lo, max=hi, step=0.005, initial_value=0.0)
            sliders[name] = h

            def _cb(event, n=name):
                if state["syncing"]:
                    return
                v = float(_val(event, cfg[n]))
                if mode.value == "遥操作":
                    cfg[n] = v
                    refresh()
                    state["dirty"] = True
                elif mode.value == "规划":
                    plan_target[n] = v
                    redraw_plan()
            h.on_update(_cb)

    @mode.on_update
    def _(event):
        m = _val(event, "跟随")
        state["mode"] = m
        state["teleop"] = (m == "遥操作")
        state["dirty"] = True
        # 切模式时把滑块对齐到实物当前姿态，避免第一下拖拽就跳一大步
        state["syncing"] = True
        for n in actuated:
            sliders[n].value = float(cfg[n])
        state["syncing"] = False
        if m == "规划":
            plan_target.update(dict(cfg))
            redraw_plan()
        else:
            clear_plan()
            plan_info.value = "-"

    @b_exec.on_click
    def _(_):
        send_target([plan_target[n] for n in actuated])

    @b_clear.on_click
    def _(_):
        clear_plan()
        plan_info.value = "-"

    @b_home.on_click
    def _(_):
        send_target([0.0] * len(actuated))

    @b_free.on_click
    def _(_):
        send_line("M 0")

    @b_hold.on_click
    def _(_):
        send_line("M 1")

    # ---------------- 串口 ----------------
    def ensure_serial():
        if state["ser"] is not None:
            return True
        try:
            state["ser"] = open_port(args.serial)
            state["reader"] = FrameReader(state["ser"])
            return True
        except Exception as exc:  # noqa: BLE001
            status.value = f"串口打不开：{exc}"
            state["ser"] = None
            return False

    def send_line(line: str):
        if not ensure_serial():
            return
        try:
            state["ser"].write(line.encode() + b"\n")
        except Exception:  # noqa: BLE001
            state["ser"] = None

    def send_target(q_joint):
        # 固件收的是关节角（rad），不用换电机侧
        state["sent"] += 1
        send_line("P " + " ".join(f"{q:.4f}" for q in q_joint))

    print(f"[ok] {len(items)} 个网格，{len(actuated)} 个关节，http://127.0.0.1:{args.port}", flush=True)

    last_send = 0.0
    last_ui = 0.0
    while True:
        # 每轮直接读控件，不依赖 on_update 回调有没有触发
        state["mode"] = mode.value
        state["teleop"] = (mode.value == "遥操作")
        if ensure_serial():
            try:
                lines = state["reader"].read_lines()
            except Exception:  # noqa: BLE001
                state["ser"] = None
                lines = []
            for line in lines:
                if line.startswith("J "):
                    frame = parse_joint_frame(line)
                    if not frame:
                        continue
                    q_motor = [v / 1000.0 for v in frame]
                    for j in joints:
                        if j["name"] in cfg and j["can_id"] - 1 < len(q_motor):
                            cfg[j["name"]] = j["dir"] * q_motor[j["can_id"] - 1] + j["offset"]
                    refresh()
                    state["frames"] += 1
                elif line.startswith("OK P"):
                    state["acked"] += 1
                elif line.startswith("ERR"):
                    state["rejected"] += 1
                elif line.startswith("G "):
                    torque.value = " ".join(line.split()[2:])

        # 50 Hz 下发；dirty 由滑块事件置位
        if state["teleop"] and state["dirty"] and time.time() - last_send > 0.02:
            last_send = time.time()
            state["dirty"] = False
            send_target([cfg[n] for n in actuated])

        if time.time() - last_ui > 0.5:
            last_ui = time.time()
            dt = time.time() - state["t0"]
            rate = state["frames"] / dt if dt > 0 else 0.0
            if state["ser"]:
                status.value = (f"[{state['mode']}] {getattr(state['ser'], 'port', '-')} · "
                                f"{rate:.0f} Hz · 发 {state['sent']} / 回 {state['acked']} · "
                                f"拒 {state['rejected']}")
            else:
                status.value = "串口未连接"
            if state["mode"] == "跟随":
                state["syncing"] = True
                for name in actuated:
                    if abs(sliders[name].value - cfg[name]) > 1e-3:
                        sliders[name].value = float(cfg[name])
                state["syncing"] = False

        time.sleep(0.01)


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        sys.exit(0)
