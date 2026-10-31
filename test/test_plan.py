"""test_plan.py — 校验 host/plan_path.py 的路径预演

    python3 test/test_plan.py
"""

import pathlib
import sys

import numpy as np

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "host"))
from plan_path import compute_plan  # noqa: E402


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}  {detail}")
    return ok


def main() -> int:
    ok = True

    # 1) 单轴：末端沿 x 走，终点必须命中，耗时 = 距离/速度
    fk = lambda q: (q["a"], 0.0, 0.0)  # noqa: E731
    pts, dur = compute_plan(fk, {"a": 0.0}, {"a": 1.0}, ["a"], vmax=1.5)
    ok &= check("单轴终点", abs(pts[-1][0] - 1.0) < 1e-3, f"末端={pts[-1][0]:.4f}")
    ok &= check("单轴耗时", abs(dur - 1.0 / 1.5) < 0.05, f"{dur:.2f}s")
    ok &= check("单轴单调", bool(np.all(np.diff(pts[:, 0]) >= 0)), "")

    # 2) 两轴同时到：不动的轴不该被拖走
    fk2 = lambda q: (q["a"], q["b"], 0.0)  # noqa: E731
    pts2, dur2 = compute_plan(fk2, {"a": 0.0, "b": 0.0}, {"a": 0.3, "b": 0.0},
                              ["a", "b"], vmax=1.5)
    ok &= check("静止轴不动", float(np.max(np.abs(pts2[:, 1]))) < 1e-9, f"max|b|={np.max(np.abs(pts2[:, 1])):.2e}")
    ok &= check("短行程耗时", abs(dur2 - 0.3 / 1.5) < 0.05, f"{dur2:.2f}s")

    # 3) 已在目标：单点、零耗时
    pts3, dur3 = compute_plan(fk, {"a": 0.5}, {"a": 0.5}, ["a"], vmax=1.5)
    ok &= check("零行程", len(pts3) == 1 and dur3 == 0.0, f"{len(pts3)} 点, {dur3:.2f}s")

    # 4) 不同轴不同速度：先到的轴停下，其它继续 —— 路径应是直线到角点再折
    pts4, _ = compute_plan(fk2, {"a": 0.0, "b": 0.0}, {"a": 1.0, "b": 0.5},
                           ["a", "b"], vmax=1.0, dt=0.1)
    ok &= check("同步到达", abs(pts4[-1][0] - 1.0) < 1e-3 and abs(pts4[-1][1] - 0.5) < 1e-3,
                f"末端={pts4[-1][:2]}")

    print("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
