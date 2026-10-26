/**
 * dm_motor.h — 达妙 DM4310 MIT 模式帧编解码（纯 C，不依赖 HAL）
 *
 * 帧格式以厂商 DM-J4310-2EC 手册 V1.4 与官方 motor-sdk 为准，并与
 * 本机已验证工作的 Python 驱动 dm4310_can.py 逐字节对齐。
 *
 * MIT 命令帧（8 字节，MSB-first）：
 *   D0/D1 p_des[15:8]/[7:0]   D2 v_des[11:4]
 *   D3 v_des[3:0]<<4|kp[11:8] D4 kp[7:0]
 *   D5 kd[11:4]               D6 kd[3:0]<<4|t_ff[11:8]   D7 t_ff[7:0]
 *
 * 反馈帧（8 字节，MSB-first，CAN ID = Master ID）：
 *   D0 ERR<<4|ID[3:0]         D1/D2 POS[15:0]
 *   D3 VEL[11:4]              D4 VEL[3:0]<<4|T[11:8]     D5 T[7:0]
 *   D6 T_MOS                  D7 T_ROTOR
 */
#ifndef DM_MOTOR_H
#define DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

/* DM4310 缩放上限（手册与 SDK 一致；协议 V1.4 §4.2 的 ±45/±18 是通用示例） */
#define DM4310_P_MAX 12.5f
#define DM4310_V_MAX 30.0f
#define DM4310_T_MAX 10.0f
#define DM4310_KP_MAX 500.0f
#define DM4310_KD_MAX 5.0f

/* 特殊命令字节：发在电机自己的 CAN ID 上，D[7] 落这几个值 */
#define DM_CMD_ENABLE 0xFCu
#define DM_CMD_DISABLE 0xFDu
#define DM_CMD_SAVE_ZERO 0xFEu
#define DM_CMD_CLEAR_ERROR 0xFBu

typedef struct {
    uint8_t id;    /* D[0] 低半字节 = 电机 ESC_ID */
    uint8_t err;   /* D[0] 高半字节 = 错误码 */
    float pos;     /* rad,   [-12.5, 12.5] */
    float vel;     /* rad/s, [-30, 30] */
    float tor;     /* N·m,   [-10, 10] */
    uint8_t t_mos; /* ℃ */
    uint8_t t_rotor;
} dm_feedback_t;

/* 把 MIT 五个浮点量编码成 8 字节命令帧。越界值先钳位再缩放。 */
void dm_encode_mit(uint8_t out[8], float p_des, float v_des, float kp, float kd, float t_ff);

/* 解析 8 字节反馈帧。非法帧（长度由调用方保证）无额外校验，err 原样带出。 */
void dm_decode_feedback(const uint8_t in[8], dm_feedback_t *fb);

/* 特殊帧：全部 0xFF 填充，最后一字节为 DM_CMD_xxx */
void dm_encode_special(uint8_t out[8], uint8_t cmd);

/* 原始字段访问：便于测试与日志 */
uint16_t dm_float_to_uint(float x, float x_min, float x_max, unsigned bits);
float dm_uint_to_float(uint16_t x, float x_min, float x_max, unsigned bits);

/* 错误码 → 可读名（未知返回 "?"） */
const char *dm_err_name(uint8_t err);

#endif /* DM_MOTOR_H */
