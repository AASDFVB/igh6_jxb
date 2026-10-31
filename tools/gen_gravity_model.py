#!/usr/bin/env python3
"""从 URDF 生成固件用的重力模型，并生成主机侧对拍向量。

生成两样东西：

    app/gravity_model.h      每关节的原点/轴/质量/质心 + 零点标定（编译进固件）
    test/gravity_vectors.h   若干组 (q, g(q))，用来在主机上校验 C 实现

数据流：
    ros2_gong/src/robotarm/urdf/robotarm.urdf   ← 只读
    host/joint_offsets.yaml                     ← dir / offset
                    ↓
    app/gravity_model.h + test/gravity_vectors.h

算法与 Pinocchio `computeGeneralizedGravity` 对齐（默认会现场校验，误差 >1e-9 就报错）。

用法:
    python3 tools/gen_gravity_model.py
    python3 tools/gen_gravity_model.py --urdf ~/Desktop/ros2_gong/src/robotarm/urdf/robotarm.urdf
"""

from __future__ import annotations

import argparse
import pathlib
import xml.etree.ElementTree as ET

import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
REPO = HERE.parent
DEFAULT_URDF = (pathlib.Path.home() /
                "Desktop/ros2_gong/src/robotarm/urdf/robotarm.urdf")
DEFAULT_CAL = REPO / "host/joint_offsets.yaml"
DEFAULT_OUT = REPO / "app/gravity_model.h"
DEFAULT_VEC = REPO / "test/gravity_vectors.h"
JOINTS = [f"J{i}" for i in range(6)]
GRAVITY = 9.81


def rpy2R(rpy):
    r, p, y = rpy
    Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
    Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
    Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def rot_axis(axis, q):
    K = np.array([[0, -axis[2], axis[1]],
                  [axis[2], 0, -axis[0]],
                  [-axis[1], axis[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * (K @ K)


def load_chain(urdf: pathlib.Path):
    root = ET.parse(urdf).getroot()
    links = {l.get("name"): l for l in root.findall("link")}
    by_parent = {}
    for j in root.findall("joint"):
        by_parent.setdefault(j.find("parent").get("link"), []).append(j)

    chain, child = [], "base_link"
    while True:
        rev = [j for j in by_parent.get(child, []) if j.get("type") == "revolute"]
        if not rev:
            break
        j = rev[0]
        o, a = j.find("origin"), j.find("axis")
        if a is None:
            raise SystemExit(f"{j.get('name')} 没有 axis")
        axis = np.array([float(x) for x in a.get("xyz").split()])
        axis /= np.linalg.norm(axis)
        link = links[j.find("child").get("link")]
        ine = link.find("inertial")
        if ine is None:
            raise SystemExit(f"{link.get('name')} 没有 inertial")
        io = ine.find("origin")
        chain.append({
            "name": j.get("name"),
            "t": np.array([float(x) for x in (o.get("xyz") or "0 0 0").split()]),
            "R": rpy2R([float(x) for x in (o.get("rpy") or "0 0 0").split()]),
            "axis": axis,
            "mass": float(ine.find("mass").get("value")),
            "com": np.array([float(x) for x in (io.get("xyz") or "0 0 0").split()]),
        })
        child = j.find("child").get("link")

    if [c["name"] for c in chain] != JOINTS:
        raise SystemExit(f"关节链不符：{[c['name'] for c in chain]} != {JOINTS}")
    return chain


def gravity(chain, q):
    """静态重力矩（关节侧）。与 Pinocchio computeGeneralizedGravity 同号。"""
    tau = np.zeros(len(chain))
    R = np.eye(3)
    p = np.zeros(3)
    pj, axw, pc = [], [], []
    for i, c in enumerate(chain):
        p = p + R @ c["t"]
        R = R @ c["R"]
        pj.append(p.copy())
        axw.append(R @ c["axis"])
        R = R @ rot_axis(c["axis"], q[i])
        pc.append(p + R @ c["com"])
    for i in range(len(chain)):
        for k in range(i, len(chain)):
            F = np.array([0.0, 0.0, -chain[k]["mass"] * GRAVITY])
            tau[i] -= axw[i] @ np.cross(pc[k] - pj[i], F)
    return tau


def load_cal(path: pathlib.Path):
    import yaml
    raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    dirs = raw.get("directions") or {}
    offs = raw.get("offsets") or {}
    ids = raw.get("can_ids") or {}
    for j in JOINTS:
        if j not in offs or j not in dirs:
            raise SystemExit(f"{path} 缺 {j}")
    return ([int(ids.get(j, i + 1)) for i, j in enumerate(JOINTS)],
            [int(dirs[j]) for j in JOINTS],
            [float(offs[j]) for j in JOINTS])


def fmt_val(x):
    s = "%.9g" % float(x)
    if "." not in s and "e" not in s and "E" not in s:
        s += ".0"          # C 里 1f 非法，必须 1.0f
    return s + "f"


def fmt(v):
    return ", ".join(fmt_val(x) for x in np.ravel(v))


def emit_model(chain, ids, dirs, offs, path: pathlib.Path, urdf: pathlib.Path):
    L = []
    L.append("/* 自动生成，别手改：tools/gen_gravity_model.py")
    L.append(" *   源 URDF: %s" % urdf)
    L.append(" *   算法: 静态重力矩（与 Pinocchio computeGeneralizedGravity 一致）")
    L.append(" */")
    L.append("#ifndef GRAVITY_MODEL_H")
    L.append("#define GRAVITY_MODEL_H")
    L.append("")
    L.append("#define GM_JOINTS 6")
    L.append("#define GM_GRAVITY %.9gf" % GRAVITY)
    L.append("")
    for name, key, shape in (("GM_ORIGIN_R", "R", "[GM_JOINTS][9]"),
                             ("GM_ORIGIN_T", "t", "[GM_JOINTS][3]"),
                             ("GM_AXIS", "axis", "[GM_JOINTS][3]"),
                             ("GM_COM", "com", "[GM_JOINTS][3]")):
        L.append(f"static const float {name}{shape} = {{")
        for c in chain:
            L.append("    {%s}," % fmt(c[key]))
        L.append("};")
        L.append("")
    L.append("static const float GM_MASS[GM_JOINTS] = {")
    L.append("    %s," % fmt([c["mass"] for c in chain]))
    L.append("};")
    L.append("")
    L.append("/* 标定：q_joint = dir * q_motor + offset（来自 host/joint_offsets.yaml）*/")
    L.append("static const float GM_DIR[GM_JOINTS] = {%s};"
             % ", ".join("%+d" % d for d in dirs))
    L.append("static const float GM_OFFSET[GM_JOINTS] = {%s};"
             % ", ".join("%.9gf" % o for o in offs))
    L.append("static const unsigned char GM_CAN_ID[GM_JOINTS] = {%s};"
             % ", ".join(str(i) for i in ids))
    L.append("")
    L.append("#endif /* GRAVITY_MODEL_H */")
    path.write_text("\n".join(L) + "\n", encoding="utf-8")


def emit_vectors(chain, path: pathlib.Path, count=40, seed=7):
    rng = np.random.default_rng(seed)
    poses = [np.zeros(len(chain))]
    poses += [rng.uniform(-2.8, 2.8, len(chain)) for _ in range(count - 1)]
    L = ["/* 自动生成，别手改：tools/gen_gravity_model.py",
         " * 每组 (q, tau)：tau 是 Pinocchio computeGeneralizedGravity 的输出。",
         " */",
         "#ifndef GRAVITY_VECTORS_H",
         "#define GRAVITY_VECTORS_H",
         "",
         "typedef struct { float q[6]; float tau[6]; } gravity_vector_t;",
         "",
         "static const gravity_vector_t GRAVITY_VECTORS[] = {"]
    worst = 0.0
    for q in poses:
        ref = pin_gravity(q)
        mine = gravity(chain, q)
        worst = max(worst, float(np.abs(ref - mine).max()))
        L.append("    {{%s}, {%s}}," % (fmt(q), fmt(ref)))
    L += ["};", f"#define GRAVITY_VECTOR_COUNT {len(poses)}", "", "#endif /* GRAVITY_VECTORS_H */"]
    path.write_text("\n".join(L) + "\n", encoding="utf-8")
    return worst


_PIN = {}


def pin_gravity(q):
    if "model" not in _PIN:
        import pinocchio as pin
        model = pin.buildModelFromUrdf(str(_PIN["urdf"]))
        _PIN["pin"] = pin
        _PIN["model"] = model
        _PIN["data"] = model.createData()
    pin, model, data = _PIN["pin"], _PIN["model"], _PIN["data"]
    return np.array(pin.computeGeneralizedGravity(model, data, np.asarray(q, dtype=float)))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--urdf", default=str(DEFAULT_URDF))
    ap.add_argument("--cal", default=str(DEFAULT_CAL))
    ap.add_argument("--out", default=str(DEFAULT_OUT))
    ap.add_argument("--vectors", default=str(DEFAULT_VEC))
    args = ap.parse_args()

    urdf = pathlib.Path(args.urdf).expanduser()
    if not urdf.exists():
        raise SystemExit(f"找不到 URDF: {urdf}")

    chain = load_chain(urdf)
    ids, dirs, offs = load_cal(pathlib.Path(args.cal))

    _PIN["urdf"] = urdf
    try:
        worst = emit_vectors(chain, pathlib.Path(args.vectors))
        print(f"对拍向量：40 组，与 Pinocchio 最大误差 {worst:.2e} N·m")
        if worst > 1e-9:
            raise SystemExit("❌ 算法与 Pinocchio 不一致，拒绝生成")
        emit_model(chain, ids, dirs, offs, pathlib.Path(args.out), urdf)
    except ImportError:
        print("⚠️ 没装 pinocchio，跳过对拍（只生成模型）")
        emit_model(chain, ids, dirs, offs, pathlib.Path(args.out), urdf)

    total = sum(c["mass"] for c in chain)
    print(f"模型：6 关节，总质量 {total:.2f} kg")
    print(f"  → {args.out}")
    print(f"  → {args.vectors}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
