# 重力补偿跑在 MCU 上，模型从 URDF 生成

ros2_gong 的重力补偿是 PC 侧脚本（`tools/gravity_comp.py`）：用 Pinocchio 的
`computeGeneralizedGravity(q)` 算关节力矩，再打包成达妙 MIT 帧的 `t_ff` 发到 CAN。
把这件事搬到 C 板上，等于把"动力学模型 + 标定"也搬进固件。

## 决定

**在固件里直接算静态重力矩，模型参数由脚本从 URDF 生成，不手抄。**

- `tools/gen_gravity_model.py` 读 `ros2_gong/src/robotarm/urdf/robotarm.urdf`（只读）
  和 `host/joint_offsets.yaml`，生成 `app/gravity_model.h`（每关节的原点/轴/质量/质心 +
  `dir`/`offset`）与 `test/gravity_vectors.h`（40 组 Pinocchio 采样）。
- `app/gravity.c` 实现静态重力矩：串链、六个转动关节、重力是唯一外力，

      τ_i = − Σ_{k≥i} axis_i · ( (p_k − p_i) × m_k·g )

  与 `computeGeneralizedGravity` 同号同量纲。
- 主循环 100 Hz 重算（与 ros2_gong 的 `rate: 100` 一致），MIT 帧 `kp=kd=0`、
  `t_ff = dir·g(q)·gain·scale·ramp`，单轴限幅 6 N·m，4 秒渐入。

## 为什么不用近似

重力矩对质量与质心很敏感，多项式拟合或"每轴一条正弦"在六轴串联臂上误差会随姿态
放大；而 RNEA 的静态项只是一个 6×6 的前向递推，Cortex-M4F 上几十微秒，没有省的必要。

## 为什么参数要生成而不是手抄

URDF 一旦改（换件、改质量），手抄的常数就悄悄过期，而症状是"手臂慢慢下坠"这种
很容易被归咎于别处的问题。生成 + 对拍把这个坑堵死：

- 生成时现场用 Pinocchio 校验，误差 > 1e-9 N·m 直接拒绝生成；
- `test/test_gravity.c` 在主机上跑 40 组采样，C 实现与 Pinocchio 差 < 1e-3 N·m（实测 3e-7）；
- 固件 `APP_DEBUG_TORQUE` 打开时以 10 Hz 打 `G` 行，可在实机上再对一次。

## Consequences

- **固件第一次真的出力**。之前所有版本 `kp=kd=t_ff=0`，电机永远自由；这一版上电
  4 秒后手臂会被"扶住"。安全条款：六轴全部 `已使能` 且反馈未超时（100 ms）才出力，
  任一条件破坏立刻零力矩并重新渐入。
- `dir` / `offset` 被编进固件（`GM_DIR`/`GM_OFFSET`），**重新定零点后必须重新生成并重烧**，
  否则 RViz 和力矩会用两套不同的零点。
- `host/joint_offsets.yaml` 仍是标定的唯一来源；它也是 ros2_gong 那份的副本，
  两边要手工同步。
- 模型只含重力，没有科氏力/惯性项——当前控制是纯力矩前馈、`kp=kd=0`，够用；
  将来做轨迹跟踪时再补完整 RNEA。
