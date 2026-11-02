/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_VOFA_H
#define MOTOR_VOFA_H

#include <stdint.h>

/* JustFloat 固定两通道：CH0=编码器计数，CH1=编码器轴机械角度（度）。 */
#define MOTOR_VOFA_CHANNEL_COUNT 2u
/* 上传频率，单位 Hz；100 Hz 对应每 10 ms 一帧。 */
#define MOTOR_VOFA_SAMPLE_HZ 100u

/* board_init() 后调用一次；0=成功，负数=初始化失败。 */
int motor_vofa_init(void);
/* 主循环持续调用，负责串口收发和按周期上传；不可在中断内调用。 */
void motor_vofa_poll(void);
/* 编码器模块获取数据后调用；可在主循环或本核中断中调用。
 * position_count：编码器模块提供的计数，可为单圈或累计计数。
 * angle_deg：与该计数同一次采样得到的编码器轴机械角度，单位度。
 * VOFA 不进行减速比、电角度或每圈计数换算，由编码器模块确定角度语义。
 * 返回 0 表示更新成功；非有限角度返回负数并保留上一组有效数据。
 * 未接入编码器前两通道均为 0，不会自动读取编码器硬件。
 * CH0 通过 float 上传，计数绝对值超过 16777216 后可能不能逐计数精确显示。 */
int motor_vofa_set_encoder(int32_t position_count, float angle_deg);

/* 调试器中查看：0=成功，负数=最近一次串口/初始化或入队操作失败。 */
extern volatile int motor_vofa_io_result;
extern volatile int motor_vofa_publish_result;

#endif /* MOTOR_VOFA_H */
