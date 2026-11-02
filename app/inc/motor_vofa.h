/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_VOFA_H
#define MOTOR_VOFA_H
#include "encoder.h"

/* JustFloat 三通道，全部为减速器输出轴数据：
 * CH0=累计角度（度）；CH1=原始转速（rpm）；CH2=滤波转速（rpm）。 */
#define MOTOR_VOFA_CHANNEL_COUNT 3u
/* 上传频率，100 Hz 对应每 10 ms 一帧，与约 1 ms 的测速更新相互独立。 */
#define MOTOR_VOFA_SAMPLE_HZ 100u

/* board_init() 后调用一次；0=成功，负数=初始化失败。 */
int motor_vofa_init(void);
/* 主循环持续调用，按配置减速比换算输出轴数据并上传；不可在中断内调用。 */
void motor_vofa_poll(void);
/* 复制一次 encoder_read() 返回的电机轴采样，可从本核前台或中断调用。
 * 不保留 data 指针；NULL 或非有限浮点值返回负数，上一组有效数据保持不变。
 * 主循环上传时调用 encoder_to_output_axis()，使用 motor_config.h 中的
 * MOTOR_SPEED_REDUCTION_RATIO（当前 7.75），对角度和两种转速统一换算。
 * 初始未更新时为零；CH0 累计角度可为负值、可超过 360 度，不上传计数或电角度。 */
int motor_vofa_set_encoder(const encoder_data_t *data);
extern volatile int motor_vofa_io_result;
extern volatile int motor_vofa_publish_result;
#endif /* MOTOR_VOFA_H */
