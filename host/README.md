# host/ — 上位机工具

固件跑在 C 板上，这一目录是把它的 USB CDC 流接到 ROS 2 / RViz 的脚本。
**不修改 ros2_gong**，只读它的 URDF 和 RViz 配置。

## 协议

固件每 20 ms（50 Hz）打一行：

```
J <t_ms> <p1> <p2> <p3> <p4> <p5> <p6>
```

- `<p1..p6>`：六个电机的位置，单位 **mrad**，顺序 = 电机 **ID 1..6**
- 另外 1 Hz 打一版中文报告（给人看），工具忽略它

`APP_DEBUG_TORQUE=1` 时还有一行 10 Hz 的力矩（见 `app/app_config.h`）：

```
G <t_ms> <t1> <t2> <t3> <t4> <t5> <t6>      ← 实际下发的重力矩，mN·m
```

用它和 Pinocchio 对拍：同一姿态下 `gravity_torque_motor()` 的输出应与
`computeGeneralizedGravity` 一致（差 < 1 mN·m）。

## 下行命令（遥操作）

串口是双向的，固件按行解析：

| 命令 | 含义 |
|---|---|
| `P q0 q1 q2 q3 q4 q5` | 关节角目标（rad），六个数 |
| `M 0` / `M 1` | 放开（只剩重力补偿）/ 位置保持 |
| `K kp kd` | 运行时改位置刚度、阻尼 |

固件回 `OK ...` 或 `ERR ...`。目标角按 `APP_TELEOP_RAD_S`（0.8 rad/s）限速追过去，
超范围（±7 rad）或 NaN 直接拒绝——不会因为一帧指令把目标拉飞。

## viser 双向

```bash
python3 host/viser_bridge.py            # 默认 8081
python3 host/viser_bridge.py --port 8081 --serial /dev/ttyACM1
```

打开 `http://127.0.0.1:8081`：

- **跟随**（默认）：模型跟着实物动，滑块也跟着变
- **遥操作**：拖滑块 → 发 `P ...` 驱动实物（50 Hz）
- **规划**：拖滑块只改目标，不动实物；场景里画出**末端路径预演**（橙色曲线 + 目标球）
  和 `长度 · 预计耗时`，点「执行规划」才发出去
- 按钮：执行规划、清空规划、回初始位（全 0）、放开、保持

规划不是 IK 规划器：它只是照固件的追目标规则（每轴 1.5 rad/s 直线逼近）走一遍正解，
所以画出来的就是**实物接下来会走的轨迹**。规则在 [plan_path.py](plan_path.py)，
单测 `python3 test/test_plan.py`。

## 看 RViz

```bash
source /opt/ros/jazzy/setup.bash
source ~/Desktop/ros2_gong/install/setup.bash     # 为了 robotarm 的 URDF
ros2 launch ~/Desktop/igh6_jxb/host/display.launch.py
```

起来后用手掰关节，RViz 里的模型应该跟着动。

只验证串口和换算（不开 ROS）：

```bash
python3 host/rviz_joint_states.py --once
```

## 调重力补偿

参数在固件仓的 [app/gcomp_params.h](../app/gcomp_params.h)，**手改**（结构和 ros2_gong
的 `gravity_comp.yaml` 一样，一行一个轴）：

```c
GCOMP_GAIN[6]   // 下坠→加大，上飘→减小
GCOMP_KP[6]     // 0 = 推哪停哪；3~10 = 松手回到锁存位置
GCOMP_KD[6]     // 0 = 无阻尼（会漂）；0.5~1.5 = 能刹住
GCOMP_ACTIVE[6] // false 的轴既不补偿也不保持
```

改完直接烧：

```bash
bash tools/flash.sh
```

## 重新定零点

`joint_offsets.yaml` 是**本仓的副本**，工具只写这一份。

```bash
# 1) 把实物摆成 URDF 全零位（= ros2_gong 的 SolidWorks 装配姿态）
python3 host/zero_offsets.py                 # 先看：编码器角 / 现在的 offset / 新 offset / 稳定性
# 2) 确认「极差」很小（手松开、别碰），再落盘
python3 host/zero_offsets.py --write
# 3) 校验
python3 host/zero_offsets.py --apply
```

- 极差超过 `--tol`（默认 5 mrad）会拒绝写入，防止在抖动时标定
- `--target 0,0,0,0.1,0,0` 可以把某个非零姿态定义成目标角
- `--dirs 1,-1,1,1,1,1` 可以在标定的同时重定方向

**只读安全**：这些脚本都只读串口，不发 CAN、不使能、不出力、不写电机 flash 零位。

## 文件

| 文件 | 作用 |
|---|---|
| `igh6_serial.py` | 找 `0483:5740`、拆 `J` 行（共享协议层） |
| `rviz_joint_states.py` | 串口 → `/joint_states`（rclpy） |
| `zero_offsets.py` | 用关节流重算 `offset` |
| `joint_offsets.yaml` | 本仓的零点/方向/ID 映射，初值抄自 ros2_gong |
| `display.launch.py` | rsp + rviz2 + 桥，不启动 `joint_state_publisher` |
