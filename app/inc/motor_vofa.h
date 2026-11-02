/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_VOFA_H
#define MOTOR_VOFA_H
#include "encoder.h"

/* JustFloat 五通道：CH0=累计计数；CH1=累计机械角度；CH2=电角度；
 * CH3=编码器轴原始转速 rpm；CH4=编码器轴滤波转速 rpm。 */
#define MOTOR_VOFA_CHANNEL_COUNT 5u
/* 上传频率，100 Hz 对应每 10 ms 一帧，与约 1 ms 的测速更新相互独立。 */
#define MOTOR_VOFA_SAMPLE_HZ 100u

/* board_init() 后调用一次；0=成功，负数=初始化失败。 */
int motor_vofa_init(void);
/* 主循环持续调用，不可在中断内调用。 */
void motor_vofa_poll(void);
/* 复制一次 encoder_read() 返回的完整采样，可从本核前台或中断调用。
 * 不保留 data 指针；NULL 或非有限浮点值返回负数，上一组有效数据保持不变。
 * 初始未更新时所有通道为零。CH0 为 float，计数绝对值超过 16777216 后
 * 可能不能逐计数精确显示；发送数据不会乘减速比或重新计算电角度。 */
int motor_vofa_set_encoder(const encoder_data_t *data);
extern volatile int motor_vofa_io_result;
extern volatile int motor_vofa_publish_result;
#endif /* MOTOR_VOFA_H */
