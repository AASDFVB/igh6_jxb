/* Host-side tests for the DM4310 MIT codec. Built and run on the PC:
 *   cc -std=c11 -Wall -Wextra -O2 -o /tmp/test_dm app/dm_motor.c test/test_dm_motor.c && /tmp/test_dm
 */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include "../app/dm_motor.h"

static int failures;

static void expect_bytes(const char *what, const uint8_t *got, const uint8_t *want)
{
    if (memcmp(got, want, 8) != 0) {
        printf("FAIL %-28s got %02X %02X %02X %02X %02X %02X %02X %02X\n", what,
               got[0], got[1], got[2], got[3], got[4], got[5], got[6], got[7]);
        failures++;
    } else {
        printf("ok   %s\n", what);
    }
}

static void expect_near(const char *what, float got, float want, float tol)
{
    if (fabsf(got - want) > tol) {
        printf("FAIL %-28s got %.6f want %.6f\n", what, got, want);
        failures++;
    } else {
        printf("ok   %s (%.6f)\n", what, got);
    }
}

int main(void)
{
    uint8_t buf[8];

    /* 1. Zero command. Real frame captured from the working Python driver:
     *    (000.000847) can0 001 [8] 7F FF 7F F0 00 00 07 FF  */
    static const uint8_t zero_want[8] = {0x7F, 0xFF, 0x7F, 0xF0, 0x00, 0x00, 0x07, 0xFF};
    dm_encode_mit(buf, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
    expect_bytes("mit zero = captured frame", buf, zero_want);

    /* 2. Clamping: far out of range pins to full scale, never wraps. */
    dm_encode_mit(buf, 99.0f, 99.0f, 999.0f, 99.0f, 99.0f);
    static const uint8_t max_want[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
    expect_bytes("mit clamp high", buf, max_want);
    dm_encode_mit(buf, -99.0f, -99.0f, -99.0f, -99.0f, -99.0f);
    static const uint8_t min_want[8] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
    expect_bytes("mit clamp low", buf, min_want);

    /* 3. Special command frames: 7x FF + D[7]. */
    dm_encode_special(buf, DM_CMD_ENABLE);
    static const uint8_t en[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    expect_bytes("enable frame", buf, en);
    dm_encode_special(buf, DM_CMD_CLEAR_ERROR);
    static const uint8_t ce[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFB};
    expect_bytes("clear-error frame", buf, ce);

    /* 4. Feedback decode: MID position, zero velocity, zero torque.
     *    POS raw 0x7FFF, VEL raw 0x7FF (D3[7:0]=0x7F, D4[7:4]=0xF),
     *    TOR raw 0x7FF (D4[3:0]=0x7, D5=0xFF). */
    static const uint8_t fb[8] = {0x11, 0x7F, 0xFF, 0x7F, 0xF7, 0xFF, 0x2A, 0x1E};
    dm_feedback_t f;
    dm_decode_feedback(fb, &f);
    if (f.id != 0x01 || f.err != 0x01) {
        printf("FAIL feedback id/err got id=%u err=%u\n", f.id, f.err);
        failures++;
    } else {
        printf("ok   feedback id=%u err=%u (%s)\n", f.id, f.err, dm_err_name(f.err));
    }
    /* Exact half-LSB offsets, not zero: full scale is 2^bits-1, so raw 0x7FFF
     * (32767) sits half an LSB below the mathematical midpoint (32767.5). */
    expect_near("feedback pos mid", f.pos, -0.0001907f, 1e-6f);
    expect_near("feedback vel mid", f.vel, -0.0073260f, 1e-6f);
    expect_near("feedback tor mid", f.tor, -0.0024420f, 1e-6f);
    if (f.t_mos != 0x2A || f.t_rotor != 0x1E) {
        printf("FAIL temperatures\n");
        failures++;
    } else {
        printf("ok   temperatures %u/%u\n", f.t_mos, f.t_rotor);
    }

    /* 5. Round-trip: raw 16-bit/12-bit fields survive encode->decode within 1 LSB. */
    const float cases[][5] = {
        {0.5f, 1.0f, 10.0f, 0.5f, 0.2f},
        {-3.25f, -12.0f, 300.0f, 2.0f, -4.5f},
        {12.0f, 29.0f, 499.0f, 4.9f, 9.9f},
        {-12.0f, -29.0f, 1.0f, 0.01f, -9.9f},
    };
    for (unsigned i = 0; i < sizeof(cases) / sizeof(cases[0]); i++) {
        const float *c = cases[i];
        dm_encode_mit(buf, c[0], c[1], c[2], c[3], c[4]);
        /* 参考值来自同一套公式的独立实现（Python 驱动），见 test/xcheck.py */
        dm_feedback_t r;
        uint8_t loop[8];
        loop[0] = 0x01;
        loop[1] = buf[0];
        loop[2] = buf[1];
        loop[3] = buf[2];
        loop[4] = buf[3];
        loop[5] = buf[4];
        loop[6] = 0;
        loop[7] = 0;
        dm_decode_feedback(loop, &r);
        expect_near("round-trip pos", r.pos, c[0], 25.0f / 65535.0f);
        expect_near("round-trip vel", r.vel, c[1], 60.0f / 4095.0f);
    }

    /* 6. Torque lives in a different field layout on the way back:
     *    command D6/D7 vs feedback D4[3:0]/D5. Build the feedback frame by hand. */
    {
        const uint16_t tor_raw = dm_float_to_uint(3.5f, -10.0f, 10.0f, 12);
        uint8_t fb2[8] = {0x01, 0x7F, 0xFF, 0x7F,
                          (uint8_t)((0x0Fu << 4) | ((tor_raw >> 8) & 0x0Fu)),
                          (uint8_t)(tor_raw & 0xFFu), 0, 0};
        dm_feedback_t t;
        dm_decode_feedback(fb2, &t);
        expect_near("round-trip torque", t.tor, 3.5f, 20.0f / 4095.0f);
    }

    printf(failures == 0 ? "\nALL PASS\n" : "\n%d FAILURE(S)\n", failures);
    return failures == 0 ? 0 : 1;
}
