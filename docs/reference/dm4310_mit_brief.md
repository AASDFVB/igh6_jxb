# DM4310 (DM-J4310-2EC) — MIT mode over classic CAN

Primary sources: 达妙 protocol V1.4, the **DM-J4310-2EC manual V1.4 (2026-09-14)**, official `motor-sdk` Python/C++. The J4310-specific manual wins where they differ.

## 1. CAN IDs

| Frame | 11-bit ID |
|---|---|
| MIT command | `0x000 + CAN_ID` (MIT offset is **0**; pos-vel `0x100+ID`, vel `0x200+ID`, force-pos `0x300+ID`) |
| Feedback | `Master ID` (reg 0x07), **default 0** |
| Param read/write/save/refresh | **`0x7FF`** broadcast, target ID inside payload |

Motor ID = `ESC_ID` (reg 0x08), usually `0x01..0x0A`; Master ID is a *separate* register, conventionally motor ID + 0x10. **`0xFC..0xFF` are not CAN IDs** — they are the **D[7] command byte** sent on the motor's own CAN ID (`control_cmd(SlaveID, 0xFC)` in both SDKs; manual: "帧 ID 如前所述"). The only broadcast ID is `0x7FF`.

| Function | ID | Payload |
|---|---|---|
| Enable | motor CAN_ID | `FF FF FF FF FF FF FF FC` |
| Disable | motor CAN_ID | `… FD` |
| Save zero | motor CAN_ID | `… FE` |
| Clear error | motor CAN_ID | `… FB` |
| Save params to flash | `0x7FF` | `ID_L,ID_H,0xAA,0x01,…` |

**Default Master ID = 0 is a trap:** all motors then answer on ID `0x000`, so the only demux key is D[0]'s low nibble — usable only for IDs ≤ 0x0F. The SDK special-cases `CANID==0` (`MasterID = data[0] & 0x0F`) and its docs repeat "不要把 MasterID 设为 0x00". Assign unique Master IDs.

## 2. Frame layouts (MSB-first / big-endian bit order)

**(a) MIT command — ID = CAN_ID**

| Byte | Bits | Field |
|---|---|---|
| D0 / D1 | `p_des[15:8]` / `p_des[7:0]` | position, uint16 |
| D2 | `v_des[11:4]` | velocity, uint12 |
| D3 | `v_des[3:0]<<4 \| kp[11:8]` | |
| D4 | `kp[7:0]` | kp, uint12 |
| D5 | `kd[11:4]` | kd, uint12 |
| D6 | `kd[3:0]<<4 \| t_ff[11:8]` | |
| D7 | `t_ff[7:0]` | torque, uint12 |

**(b) MIT feedback — ID = Master ID** (identical in all modes)

| Byte | Bits | Field |
|---|---|---|
| D0 | `ERR<<4 \| ID[3:0]` | state nibble + motor ID |
| D1 / D2 | `POS[15:8]` / `POS[7:0]` | uint16 |
| D3 | `VEL[11:4]` | uint12 |
| D4 | `VEL[3:0]<<4 \| T[11:8]` | |
| D5 | `T[7:0]` | uint12 |
| D6 / D7 | `T_MOS` / `T_Rotor` | int8 °C |

All fields are **unsigned**; sign comes from the linear map. ERR: `0` disabled, `1` enabled, `3/4/5` cal faults, `8` over-V, `9` under-V, `A` over-current, `B` MOS over-temp, `C` coil over-temp, `D` **comm loss**, `E` overload.

## 3. Scaling constants (DM4310, 24 V)

| Const | P_MAX | V_MAX | T_MAX | KP_MAX | KD_MAX |
|---|---|---|---|---|---|
| Value | **12.5 rad** | **30 rad/s** | **10 N·m** | **500** | **5** |
| Min | −12.5 | −30 | −10 | 0 | 0 |

```
float_to_uint(x,min,max,bits) = (clamp(x,min,max)-min) * ((1<<bits)-1)/(max-min)
uint_to_float(x,min,max,bits) = x * (max-min)/((1<<bits)-1) + min
```
Full scale is `2^bits − 1`, not `2^bits`. Resolution: 0.38 mrad, 14.6 mrad/s, 4.9 mN·m, kp 0.12, kd 0.0012.

⚠️ Protocol doc V1.4 §4.2 quotes ±12.5/**±45**/**±18** as a *generic example*; the J4310 manual and both SDKs use **±30/±10**. Mismatched ranges silently rescale commands. PMAX/VMAX/TMAX are writable (0x15/0x16/0x17) — read and match them.

## 4. Feedback is **request/response only**

The manual says feedback behaviour is **问询式 (polled)**: the drive emits state *when it receives a frame whose ID matches its CAN ID*. No periodic streaming, **no feedback-rate setting**. SDK: "达妙电机是一发一收模式，只有发送指令电机才会返回当前状态". Vendor load test confirms 1:1 (10,763,791 commands ↔ 10,763,791 feedbacks).

**To keep 6 motors' feedback flowing you must keep transmitting** — MIT commands (each returns fresh state) or a status poll: SDK `refresh_motor_status()` sends `[ID_L,ID_H,0xCC,0…]` to `0x7FF` (SDK-only; not in the manuals' frame tables). Silence ⇒ no feedback *and* the timeout fires.

## 5. Bus speed and frame budget

Default **1 Mbit/s**; reg 0x23 selects 125k/200k/250k/500k/1M/2M/2.5M/3.2M/4M/5M. **Stay ≤1 Mbit/s for classic CAN:** the manual states any rate >1 Mbit/s auto-switches the drive to **CAN FD**; it still accepts 2.0B frames but *sends feedback as FD*, so a classic controller gets nothing and the drive keeps erroring.

11-bit ID, 8 data bytes, no extended format, 1 Mbit/s:

| Case | Bits | Frames/s |
|---|---|---|
| No stuffing (`1+12+6+64+16+2+7+3`) | **111** | **≈9,000** |
| Worst-case stuffing (98 stuffable + 24 stuff bits → 135) | **135** | **≈7,400** |
| Vendor-measured sustained (DM4310) | — | **6,000** |

6 motors × (command + feedback):

| Rate | Frames/s | Verdict |
|---|---|---|
| 500 Hz | 12 × 500 = **6,000** | At the vendor ceiling (~81 % of 7,400). The vendor's "一控六" test runs exactly this: 2 ms tick, 200 µs gap per frame pair; ≈1.6 ms bus + 0.4 ms gaps ≈ 2 ms. |
| 1 kHz | 12 × 1,000 = **12,000** | **Infeasible** — 1.6–2× over capacity. Vendor: "1kHz…可以有效的控制三个电机". Needs 2 buses of 3, or CAN FD. |

## 6. Gotchas

- **Enable first.** Drive boots **disabled** (ERR=0); send `FF…FC`, then wait ~100 ms (`enable()` sleeps 0.1 s). Enable state is *not* persisted — re-enable every power-up.
- **Mode is a register (0x0A), not a frame:** 1=MIT, 2=pos-vel, 3=vel, 4=force-pos, written via `0x7FF`/`0x55`. Switching **zeroes p_des, v_des, t_ff, kp, kd**. Not saved unless `0xAA` — and save works **only while disabled**, takes ≤30 ms, ~10k flash cycles.
- **Zero position:** `FF…FE` sets the current output position as zero and forces p_des=0.
- **Timeout:** reg 0x09, unit **50 µs**, active **only while enabled**; on expiry ERR=`0xD` and the drive drops out of enable. Set it above your loop period.
- **Lost frames:** no protocol ACK — a dropped command simply means a dropped feedback sample. Watch ERR and D[6]/D[7].
- **kp=0, kd=0 ⇒ `T_ref = t_ff`**, i.e. genuinely free at `t_ff=0`; with `t_ff` set and no load it runs to max speed (manual §二.2). **kp≠0 with kd=0 oscillates / can run away** (§二.3) — never do it.
- Position limited to **[−π, π] rad** after power-up.
- Keep CAN ID **< 16** — it shares D[0] with the error nibble.

## Sources

- [DM-J4310-2EC V1.2 使用说明书 V1.4](https://github.com/dmBots/DM-J4310-2EC/blob/master/manual/DM-J4310-2EC%20V1.2%20%E5%87%8F%E9%80%9F%E7%94%B5%E6%9C%BA%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%20V1.4%202026-09-14.pdf) — §CAN 通信, 使能/失能/零点, 寄存器列表, 波特率 *(primary, 4310-specific)*
- [达妙驱动控制协议 V1.4](https://gitee.com/kit-miao/damiao-document/blob/master/%E8%B0%83%E8%AF%95%E5%8A%A9%E6%89%8B%E4%BD%BF%E7%94%A8%E8%AF%B4%E6%98%8E%E4%B9%A6%EF%BC%88%E8%BE%BE%E5%A6%99%E9%A9%B1%E5%8A%A8%E6%8E%A7%E5%88%B6%E5%8D%8F%E8%AE%AE%EF%BC%89V1.4.pdf) — ch. 4 *(primary)*
- [MIT 模式说明](https://gitee.com/kit-miao/damiao-document/blob/master/3.%20%E7%94%B5%E6%9C%BA%E4%B8%8A%E6%89%8B%E6%B5%81%E7%A8%8B\(%E4%B8%89\)-MIT%E6%A8%A1%E5%BC%8F%E8%AF%B4%E6%98%8E.pdf) — ±12.5/±30/±10, kp/kd rules *(primary)*
- [DM4310-CAN 测试报告](https://gitee.com/kit-miao/damiao-document/blob/master/DM4310-CAN%E6%B5%8B%E8%AF%95%E6%8A%A5%E5%91%8A.pdf) — 1:1 frames, 6000 fps, 一控六/500 Hz *(primary)*
- [DM_CAN.py](https://gitee.com/kit-miao/motor-sdk/blob/master/Python%E4%BE%8B%E7%A8%8B/u2can/DM_CAN.py) · [damiao.h](https://gitee.com/kit-miao/motor-sdk/blob/master/C%2B%2B%E4%BE%8B%E7%A8%8B/u2can/include/damiao.h) · [USAGE.md](https://gitee.com/kit-miao/motor-sdk/blob/master/Python%E4%BE%8B%E7%A8%8B/u2can/USAGE.md) *(primary SDK)*

**Flagged / unconfirmed:** the manual calls the feedback ID field "CAN_ID 的低 8 位" but D[0] only carries 4 ID bits — it separately advises CAN ID < 16, so 4 bits is the real limit. The `0xCC` refresh frame appears only in the SDK, not the manuals. Whether a timeout latches a hard fault or merely reports ERR=`0xD` is not stated explicitly.
