/**
 * app_can.h — 双总线 CAN 收发：滤波器、启动、保持帧、反馈分发
 */
#ifndef APP_CAN_H
#define APP_CAN_H

#include <stdint.h>
#include "app_config.h"
#include "dm_motor.h"

typedef struct {
    dm_feedback_t last;
    uint32_t rx_frames;   /* 该电机收到的反馈帧总数 */
    uint32_t rx_in_window;/* 最近一个上报窗口内收到多少帧 */
    uint32_t bad_frames;  /* 帧内容可用但 ID 对不上的次数 */
    uint32_t last_rx_ms;  /* 最近一次收到反馈的时刻（ms），给重力补偿做看门狗 */
} app_motor_state_t;

extern app_motor_state_t g_motors[APP_MOTOR_COUNT];
extern volatile uint32_t g_rx_unmatched;  /* 无法归属的反馈帧 */
extern volatile uint32_t g_fifo_overruns;
extern volatile uint32_t g_tx_dropped;    /* 三个邮箱都满、命令帧没发出去 */
extern volatile uint32_t g_busoff_recoveries;
extern volatile uint32_t g_busoff_events;

/* 配滤波器、启动两路 CAN、打开 RX0/RX1 中断 */
void app_can_init(void);

/* 给第 i 个电机发一帧保持帧（kp=kd=0, p_des=v_des=t_ff=0）*/
void app_can_send_hold(int i);

/* 给第 i 个电机发一帧任意 MIT 指令（p_des 用电机侧角度，t_ff 单位 N·m）*/
void app_can_send_mit(int i, float p_des, float v_des, float kp, float kd, float t_ff);

/* 给第 i 个电机发特殊帧（DM_CMD_ENABLE / DISABLE / SAVE_ZERO / CLEAR_ERROR）*/
void app_can_send_special(int i, uint8_t cmd);

/* 在主循环里反复调用：排空 FIFO、处理 bus-off */
void app_can_poll(uint32_t now_ms);

/* 轮询错误标志（FIFO 溢出计数）*/
void app_can_poll_errors(void);

/* 上报窗口结算：把 rx_in_window 清零，返回上一窗口的收帧数 */
uint32_t app_can_close_window(int i);

#endif /* APP_CAN_H */
