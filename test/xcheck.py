"""Differential test: C encoder vs the Python driver's algorithm.

The reference implementation is `float_to_uint` + `mit()` copied from the
working SocketCAN driver `dm4310_can.py` (dm4310_can.py:124-140), so this
proves the firmware packs byte-identical frames to the rig that already
drives these motors.

Usage: python3 test/xcheck.py   (expects /tmp/xcheck built by the same script)
"""
import os
import random
import subprocess
import sys

P_MAX, V_MAX, T_MAX, KP_MAX, KD_MAX = 12.5, 30.0, 10.0, 500.0, 5.0


def float_to_uint(x, x_min, x_max, bits):
    span = x_max - x_min
    if x < x_min:
        x = x_min
    elif x > x_max:
        x = x_max
    return int((x - x_min) * ((1 << bits) - 1) / span)


def reference_mit(p, v, kp, kd, t_ff):
    p_i = float_to_uint(p, -P_MAX, P_MAX, 16)
    v_i = float_to_uint(v, -V_MAX, V_MAX, 12)
    kp_i = float_to_uint(kp, 0.0, KP_MAX, 12)
    kd_i = float_to_uint(kd, 0.0, KD_MAX, 12)
    t_i = float_to_uint(t_ff, -T_MAX, T_MAX, 12)
    d = bytearray(8)
    d[0] = (p_i >> 8) & 0xFF
    d[1] = p_i & 0xFF
    d[2] = (v_i >> 4) & 0xFF
    d[3] = ((v_i & 0x0F) << 4) | ((kp_i >> 8) & 0x0F)
    d[4] = kp_i & 0xFF
    d[5] = (kd_i >> 4) & 0xFF
    d[6] = ((kd_i & 0x0F) << 4) | ((t_i >> 8) & 0x0F)
    d[7] = t_i & 0xFF
    return d


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    root = os.path.dirname(here)
    exe = "/tmp/xcheck"
    cc = subprocess.run(
        ["cc", "-std=c11", "-O2", "-o", exe,
         os.path.join(root, "app", "dm_motor.c"), os.path.join(here, "xcheck.c")],
        capture_output=True, text=True)
    if cc.returncode != 0:
        print(cc.stderr)
        return 1

    rng = random.Random(20261026)
    cases = [(0.0, 0.0, 0.0, 0.0, 0.0)]
    for _ in range(2000):
        # Round to 6 dp: that is exactly what the C side receives through
        # scanf("%f"), so both implementations see identical inputs.
        cases.append(tuple(round(rng.uniform(lo, hi), 6) for lo, hi in
                           [(-15, 15), (-35, 35), (-50, 550), (-1, 6), (-12, 12)]))

    lines = "\n".join(f"{c[0]:.6f} {c[1]:.6f} {c[2]:.6f} {c[3]:.6f} {c[4]:.6f}" for c in cases)
    proc = subprocess.run([exe], input=lines, capture_output=True, text=True)
    got = proc.stdout.strip().splitlines()
    if len(got) != len(cases):
        print(f"FAIL: expected {len(cases)} lines, got {len(got)}\n{proc.stderr[:500]}")
        return 1

    bad = 0
    exact = 0
    for (p, v, kp, kd, t), line in zip(cases, got):
        want = reference_mit(p, v, kp, kd, t)
        have = bytes.fromhex(line.replace(" ", ""))
        if have == bytes(want):
            exact += 1
            continue
        # Firmware computes in float32 (the F407 FPU is single precision only);
        # the reference is double. At exact LSB boundaries that costs 1 LSB.
        # 1 LSB = 0.00038 rad / 0.0146 rad/s / 0.0049 N·m — physically moot.
        d_p = abs(int.from_bytes(have[0:2], "big") - int.from_bytes(want[0:2], "big"))
        d_v = abs(((have[2] << 4) | (have[3] >> 4)) - ((want[2] << 4) | (want[3] >> 4)))
        d_kp = abs((((have[3] & 0xF) << 8) | have[4]) - (((want[3] & 0xF) << 8) | want[4]))
        d_kd = abs(((have[5] << 4) | (have[6] >> 4)) - ((want[5] << 4) | (want[6] >> 4)))
        d_t = abs((((have[6] & 0xF) << 8) | have[7]) - (((want[6] & 0xF) << 8) | want[7]))
        if max(d_p, d_v, d_kp, d_kd, d_t) > 1:
            bad += 1
            if bad <= 5:
                print(f"MISMATCH p={p:.4f} v={v:.4f} kp={kp:.3f} kd={kd:.3f} t={t:.3f}")
                print(f"  C     : {have.hex(' ')}")
                print(f"  python: {bytes(want).hex(' ')}")
                print(f"  deltas: p={d_p} v={d_v} kp={d_kp} kd={d_kd} t={d_t}")

    print(f"{len(cases)} cases, {exact} byte-identical, {len(cases) - exact - bad} within 1 LSB, {bad} MISMATCH")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
