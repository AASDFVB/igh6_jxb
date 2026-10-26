# igh6_jxb

STM32F407IGH6（大疆 RoboMaster C 板）用双路 CAN 以 1 kHz 轮询 6 个达妙 DM4310，解析 MIT 模式反馈帧，并经 USB CDC 打印。这是 **bring-up 验证件**：只发保持帧、只读反馈，不做闭环控制。

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

## 预期输出

上电 ~150 ms 后开始轮询，每秒一行汇总 + 6 行电机状态：

```
t=12 rx=1000/1000/1000/1000/1000/1000 ovf=0 un=0 drop=0
[1] e=1 p=+0000 v=+0000 t=+0000 32/30
```

- `rx` 每秒每机应收 **1000** 帧（1 kHz 轮询，一问一答）；明显低于 1000 = 丢帧
- `e` = 错误码：`1` 已使能、`0` 未使能、`8/9` 过压/欠压、`A` 过流、`B/C` 过温、`D` 通信丢失、`E` 过载
- `p` 单位 mrad，`v` 0.01 rad/s，`t` mN·m；`32/30` = MOS/转子温度 ℃
- `ovf` FIFO 溢出、`un` 无法归属的帧、`drop` 命令帧未发出

## 指示灯

三颗用户 LED（R=PH12 / G=PH11 / B=PH10，TIM5 的 CH3/CH2/CH1）跑 3 秒一圈的呼吸灯：红 → 绿 → 蓝依次亮起，各差 1/3 相位，上限 80% 占空比。PWM 配置在 `app/app_led.c` 里自持（原因见 ADR-0005），改周期/亮度就改那三个常量。

## 改参数

`app/app_config.h`：电机表（ID + 挂在哪条总线）、使能延时、保持帧起始时间、上报周期。
`app/dm_motor.h`：缩放常数（4310 为 12.5 rad / 30 rad/s / 10 N·m / kp 500 / kd 5）。

## 目录

```
Makefile    CubeMX 生成（每次重新生成会被整份覆盖，别手改）
tools/
  generate.sh         CubeMX 无 UI 生成 + 补 Makefile（一条命令）
  patch_makefile.py   把 app/*.c 和 -Iapp 补回 Makefile（幂等）
app/        业务代码（不在 CubeMX 生成范围内）
  dm_motor.*    MIT 帧编解码，纯 C，可在 PC 上测
  app_can.*     双总线滤波器、使能/保持帧、反馈分发
  app_main.*    1 kHz 时序、1 Hz USB 上报
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
