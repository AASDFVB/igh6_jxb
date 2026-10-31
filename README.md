# igh6_jxb

STM32F407IGH6（大疆 RoboMaster C 板）用双路 CAN 以 1 kHz 轮询 6 个达妙 DM4310，解析 MIT 模式反馈帧，经 USB CDC 输出；并**在板上实时算静态重力矩**，作为 MIT 帧的 `t_ff` 下发，让手臂"被扶住"。

- 反馈与使能：只读 + 保持帧（`kp=kd=0`）
- 出力：重力补偿（100 Hz 重算、4 秒渐入、单轴限幅 6 N·m），模型从 ros2_gong 的 URDF 生成
- 不做：目标位置跟踪、轨迹、闭环增益整定

## 接线

| 项 | 接法 |
|---|---|
| 电机 1/2/3 | C 板 **CAN1**（J22/J23，2-pin：CANH/CANL） |
| 电机 4/5/6 | C 板 **CAN2**（J20/J21，4-pin：第 3 脚 CANH、第 4 脚 CANL） |
| 电机电源 | **外部 24 V**；C 板 CAN2 接头上的 5 V 不要接电机 |
| 共地 | 24 V 电源 GND 与 C 板 GND 相连（CAN1 的 2-pin 没有 GND） |
| 终端电阻 | 断电量 CANH–CANL：≈120 Ω 说明已接；开路则需在总线两端各加 120 Ω |
| 调试 | ST-Link 接 SWD 排针（PA13/PA14），板载无调试器 |
| 输出 | C 板 Micro-USB 接电脑，作为 CDC 虚拟串口（115200 8N1，实际速率不受限） |

## 构建 / 烧录

```bash
make GCC_PATH=$HOME/.local/opt/xpack-arm-none-eabi-gcc-15.2.1-1.1/bin -j
```

VSCode：直接 **F5**（Cortex-Debug，ST-Link + OpenOCD，自动先执行 build）；只烧录用 **Terminal → Run Task → flash**。

`Makefile` 是 CubeMX 生成的，`app/*.c` 那几行由 `tools/patch_makefile.py` 补进去（见下）。

工具链全在 `~/.local/opt`，不需要 sudo：
`xpack-arm-none-eabi-gcc-15.2.1-1.1`、`xpack-openocd-0.12.0-7`。

## 看串口

```bash
stty -F /dev/ttyACM0 115200 raw -echo   # 必须 raw，否则 \r\n 会被终端再转成空行
cat /dev/ttyACM0
```

或交互式：`python3 -m serial.tools.miniterm /dev/ttyACM0 115200`（`Ctrl-]` 退出）。
udev 规则 `99-stm32cdc.rules` 已把设备设成 0666，不需要 sudo。

## 预期输出

上电约 200 ms 后开始轮询，每秒一行汇总 + 6 行电机状态（UTF-8）：

```
第12秒 收帧=1000/1000/1000/1000/1000/1000 溢出=0 未匹配=0 发送失败=0 总线关闭=0/0 丢拍=0
[1] 状态=已使能 位置=-1338mrad 速度=+7mrad/s 力矩=-7mN·m 温度=32/30℃
```

- `收帧` 每秒每机应收 **1000** 帧（1 kHz 轮询，一问一答）；明显低于 1000 = 丢帧
- `状态`：`已使能` / `未使能` / `过压` / `欠压` / `过流` / `MOS过温` / `转子过温` / `通信丢失` / `过载`
- `位置` mrad、`速度` mrad/s、`力矩` mN·m、`温度` = MOS/转子 ℃
- `溢出` FIFO 溢出、`未匹配` 无法归属的帧、`发送失败` 命令帧没发出去、
  `总线关闭` 进入次数/恢复次数、`丢拍` 主循环错过的毫秒

## 指示灯

三颗用户 LED（R=PH12 / G=PH11 / B=PH10，TIM5 的 CH3/CH2/CH1）跑 3 秒一圈的呼吸灯：红 → 绿 → 蓝依次亮起，各差 1/3 相位，上限 80% 占空比。PWM 配置在 `app/app_led.c` 里自持（原因见 ADR-0005），改周期/亮度就改那三个常量。

## 改参数

`app/gcomp_params.h`：**重力补偿的全部可调项**（手改）——`GCOMP_GAIN` / `GCOMP_KP` /
`GCOMP_KD` 逐轴一行，外加 `GCOMP_SCALE` / `GCOMP_LIMIT_NM` / `GCOMP_RAMP_MS` /
`GCOMP_PERIOD_MS` / `GCOMP_ACTIVE`。语义与 ros2_gong 的 `gravity_comp.yaml` 一致。
改完 `bash tools/flash.sh`。

`app/app_config.h`：电机表（ID + 挂在哪条总线）、使能延时、保持帧起始时间、上报周期、关节流周期、重力补偿开关/看门狗。
`app/dm_motor.h`：缩放常数（4310 为 12.5 rad / 30 rad/s / 10 N·m / kp 500 / kd 5）。

## 重力补偿

固件用静态重力矩抵消手臂自重，模型与 ros2_gong 的 Pinocchio 实现同源：

```
τ_i = − Σ_{k≥i} axis_i · ( (p_k − p_i) × m_k·g )      // 关节侧
t_ff = dir · τ_i · gain_i · scale · ramp              // 电机侧，MIT 帧的 t_ff
```

- 模型参数**从 URDF 生成**，不手抄：`python3 tools/gen_gravity_model.py`
  → `app/gravity_model.h`（含 `dir`/`offset`）+ `test/gravity_vectors.h`
- 主机侧对拍：`cc app/gravity.c test/test_gravity.c -Iapp -lm && ./a.out`（实测 3e-7 N·m）
- 实机对拍：`APP_DEBUG_TORQUE=1` 时 10 Hz 打 `G <ms> <t1..t6>`（mN·m），与 Pinocchio 比
- 安全：六轴全部「已使能」且反馈未超时 100 ms 才出力；条件破坏立刻零力矩并重新渐入
- **重新定零点后必须重新生成并重烧**——`dir`/`offset` 已经编进固件

## 上电回初始位

`app/home_params.h` 控制：六轴就绪后等 `HOME_DELAY_MS`，用 `HOME_MOVE_MS` 从当前姿态
**smoothstep 插值**到 `HOME_TARGET`（默认全 0 = URDF 初始位，也就是标定零点时的立姿），
之后用 `HOME_KP`/`HOME_KD` 停在那儿。

- 目标角是 **URDF 关节角**（rad），不是电机侧；`HOME_ENABLE 0` 可整体关掉
- 中途掉使能/丢帧 → 立刻放弃回位，退回纯重力补偿（不会再硬顶）
- 收尾保持靠 `HOME_KP`（默认 30），想回到"推哪停哪"就把它改小或关掉回位

⚠️ 开了这个，**上电后手臂会自己动**。慢一点（`HOME_MOVE_MS` 调大）比快安全。

## 上位机

除了 1 Hz 的中文报告，固件每 20 ms（50 Hz）还打一行机器可读的关节流：

```
J <t_ms> <p1> <p2> <p3> <p4> <p5> <p6>      ← mrad，顺序 = 电机 ID 1..6
```

`host/` 里是消费它的脚本：RViz 实时同步、用串口流重新定零点、viser 双向遥操作。见 [host/README.md](host/README.md)。

串口也是双向的，固件按行接收：

```
P q0 q1 q2 q3 q4 q5     关节角目标（rad），固件内按 0.8 rad/s 限速追过去
M 0 | M 1               放开（只剩重力补偿）/ 位置保持
K kp kd                 运行时改位置刚度、阻尼
```

## 目录

```
Makefile    CubeMX 生成（每次重新生成会被整份覆盖，别手改）
tools/
  generate.sh         CubeMX 无 UI 生成 + 补 Makefile（一条命令）
  patch_makefile.py   把 app/*.c 和 -Iapp 补回 Makefile（幂等）
  flash.sh            编译 + 烧录 + 校验 + 复位
app/        业务代码（不在 CubeMX 生成范围内）
  dm_motor.*    MIT 帧编解码，纯 C，可在 PC 上测
  app_can.*     双总线滤波器、使能/保持帧、反馈分发
  app_main.*    1 kHz 时序、1 Hz 报告 + 50 Hz 关节流
host/       上位机工具（Python，只读串口）
  igh6_serial.py       协议：找设备、拆 J 行
  rviz_joint_states.py 串口 → /joint_states
  zero_offsets.py      用关节流重算零点
  display.launch.py    robot_state_publisher + RViz + 桥
test/       主机侧测试：cc app/dm_motor.c test/test_dm_motor.c -lm && ./a.out
            python3 test/xcheck.py  ← 与 Python 驱动逐字节对拍
Src/ Inc/    CubeMX 生成（USER CODE 段可以手改，重新生成会保留）
```

## 重新生成 CubeMX 代码

```bash
bash tools/generate.sh          # CubeMX 生成 + 补 Makefile，一步到位
# 或 VSCode：Terminal → Run Task → generate (CubeMX + patch)
```

**第二步不能省**：CubeMX 重写 Makefile 时不认识 `app/`，不补回去就是"编译通过、烧进去还是旧固件"。
注意 `.ioc` 里有两个容易丢的项：**PH0-OSC_IN / PH1-OSC_OUT 振荡器引脚**（丢了 HSE 会是 `RCC_HSE_OFF`，PLL 锁不上）和 **`USB_DEVICE` 的虚拟引脚 `VP_USB_DEVICE_VS_USB_DEVICE_CDC_FS`**（丢了 CubeMX 会静默不生成 `usbd_*` 中间件，只剩 PCD）。手写 `.ioc` 时若 PLL 源没接上，还要补 `RCC.PLLSourceVirtual=RCC_PLLSOURCE_HSE`，否则会退回内部 HSI、CAN 波特率全错——生成一次后 CubeMX 会把该键规范化掉，属正常。

## 故障排查

| 现象 | 查什么 |
|---|---|
| 一条回帧都没有 | 电机是否使能（`e=0`）、Master ID 是否与固件一致、CANH/CANL 是否接反、终端电阻是否 120 Ω、波特率是否 1 Mbit/s |
| `rx` 稳定但远小于 1000 | 总线负载过重（两路是否都在跑）、`drop` 是否增长 |
| `ovf` 增长 | FIFO 溢出，ISR 被别的中断拖住 |
| 电机嗡嗡/自走 | 说明发了非零 kp/kd；本工程恒为 0，检查是否烧错固件 |
