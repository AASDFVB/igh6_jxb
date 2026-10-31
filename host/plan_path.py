"""关节直线 + 单轴限速的路径预演。

和固件的追目标规则一致（`app/app_config.h: APP_TELEOP_RAD_S`）：
每个轴独立按 `vmax` 走，谁先到谁先停——所以末端轨迹不是直线，是条弧。

`fk(q) -> (x, y, z)` 由调用方提供（viser 里是 yourdfpy 的正解）。
"""

from __future__ import annotations

import numpy as np


def compute_plan(fk, q_from: dict, q_to: dict, names, vmax: float,
                 dt: float = 0.02, max_steps: int = 4000):
    """返回 (末端路径点 (N,3), 预计耗时秒)。

    q_from / q_to 是关节名到角度的字典；names 决定遍历顺序。
    """
    if vmax <= 0:
        raise ValueError("vmax 必须为正")
    q = dict(q_from)
    pts, t = [], 0.0
    step = vmax * dt
    for _ in range(max_steps):
        pts.append(np.asarray(fk(q), dtype=float))
        d = {n: q_to[n] - q[n] for n in names}
        if max(abs(v) for v in d.values()) < 1e-4:
            break
        for n in names:
            q[n] += max(-step, min(step, d[n]))
        t += dt
    return np.asarray(pts, dtype=float), t
