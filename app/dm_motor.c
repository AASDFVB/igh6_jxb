#include "dm_motor.h"

#define DM_P_MIN (-DM4310_P_MAX)
#define DM_V_MIN (-DM4310_V_MAX)
#define DM_T_MIN (-DM4310_T_MAX)

uint16_t dm_float_to_uint(float x, float x_min, float x_max, unsigned bits)
{
    if (x < x_min) {
        x = x_min;
    }
    if (x > x_max) {
        x = x_max;
    }
    const float maxv = (float)((1u << bits) - 1u);
    return (uint16_t)((x - x_min) * maxv / (x_max - x_min));
}

float dm_uint_to_float(uint16_t x, float x_min, float x_max, unsigned bits)
{
    const float maxv = (float)((1u << bits) - 1u);
    return (float)x * (x_max - x_min) / maxv + x_min;
}

void dm_encode_mit_r(uint8_t out[8], float p_des, float v_des, float kp, float kd, float t_ff,
                     const dm_range_t *r)
{
    const uint16_t p = dm_float_to_uint(p_des, -r->p_max, r->p_max, 16);
    const uint16_t v = dm_float_to_uint(v_des, -r->v_max, r->v_max, 12);
    const uint16_t kpi = dm_float_to_uint(kp, 0.0f, DM4310_KP_MAX, 12);
    const uint16_t kdi = dm_float_to_uint(kd, 0.0f, DM4310_KD_MAX, 12);
    const uint16_t t = dm_float_to_uint(t_ff, -r->t_max, r->t_max, 12);

    out[0] = (uint8_t)(p >> 8);
    out[1] = (uint8_t)(p & 0xFFu);
    out[2] = (uint8_t)(v >> 4);
    out[3] = (uint8_t)(((v & 0x0Fu) << 4) | (kpi >> 8));
    out[4] = (uint8_t)(kpi & 0xFFu);
    out[5] = (uint8_t)(kdi >> 4);
    out[6] = (uint8_t)(((kdi & 0x0Fu) << 4) | (t >> 8));
    out[7] = (uint8_t)(t & 0xFFu);
}

void dm_encode_mit(uint8_t out[8], float p_des, float v_des, float kp, float kd, float t_ff)
{
    static const dm_range_t def = {DM4310_P_MAX, DM4310_V_MAX, DM4310_T_MAX};
    dm_encode_mit_r(out, p_des, v_des, kp, kd, t_ff, &def);
}

void dm_decode_feedback_r(const uint8_t in[8], dm_feedback_t *fb, const dm_range_t *r)
{
    fb->id = (uint8_t)(in[0] & 0x0Fu);
    fb->err = (uint8_t)(in[0] >> 4);
    fb->pos = dm_uint_to_float((uint16_t)(((uint16_t)in[1] << 8) | in[2]), -r->p_max, r->p_max, 16);
    fb->vel = dm_uint_to_float((uint16_t)(((uint16_t)in[3] << 4) | (in[4] >> 4)), -r->v_max, r->v_max, 12);
    fb->tor = dm_uint_to_float((uint16_t)((((uint16_t)in[4] & 0x0Fu) << 8) | in[5]), -r->t_max, r->t_max, 12);
    fb->t_mos = in[6];
    fb->t_rotor = in[7];
}

void dm_decode_feedback(const uint8_t in[8], dm_feedback_t *fb)
{
    static const dm_range_t def = {DM4310_P_MAX, DM4310_V_MAX, DM4310_T_MAX};
    dm_decode_feedback_r(in, fb, &def);
}

void dm_encode_special(uint8_t out[8], uint8_t cmd)
{
    for (int i = 0; i < 7; i++) {
        out[i] = 0xFFu;
    }
    out[7] = cmd;
}

const char *dm_err_name(uint8_t err)
{
    switch (err) {
    case 0x0: return "disabled";
    case 0x1: return "enabled";
    case 0x8: return "over-volt";
    case 0x9: return "under-volt";
    case 0xA: return "over-current";
    case 0xB: return "mos-over-temp";
    case 0xC: return "rotor-over-temp";
    case 0xD: return "comm-loss";
    case 0xE: return "overload";
    default:  return "?";
    }
}
