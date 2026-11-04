/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_VOFA_H
#define MOTOR_VOFA_H
#include "encoder.h"

/* 闭环CH0~7：Id目标/实际、Iq目标/实际(mA)、Ud/Uq(计数)、输出轴rpm、故障位。 */
/* CH8/9追加速度环目标/滤波反馈，减速器输出轴rpm。 */
/* CH10/11位置目标/反馈，减速器输出轴累计绝对角度。 */
#define MOTOR_VOFA_CHANNEL_COUNT 12u
/* 上传频率，100 Hz 对应每 10 ms 一帧，与约 1 ms 的测速更新相互独立。 */
#define MOTOR_VOFA_SAMPLE_HZ MOTOR_VOFA_UPLOAD_HZ

/* board_init() 后调用一次；0=成功，负数=初始化失败。 */
int motor_vofa_init(void);
/* 主循环持续调用，准备完成后ENABLE,1直接启动电流闭环，ENABLE,0/STOP停机。
 * 未准备完成的ENABLE被拒绝，不排队；STOP还会取消未完成的上电准备。
 * SET,SPEED_RPM选择速度模式，SET,CURRENT_MA选择电流模式；CLEAR_FAULT清除故障。
 * SET,POSITION_DEG选择位置模式，单位为输出轴累计绝对角度，360度为输出轴一圈。
 * 命令以CR/LF结束；启动执行ADC/过流门禁，失败不排队，不在中断内调用。 */
void motor_vofa_poll(void);
/* 复制一次 encoder_read() 返回的电机轴采样，可从本核前台或中断调用。
 * 不保留 data 指针；NULL 或非有限浮点值返回负数，上一组有效数据保持不变。
 * 本模块在接收/上传时使用MOTOR_ABZ_COUNTS_PER_REV和MOTOR_SPEED_REDUCTION_RATIO，
 * 统一换算计数/秒与输出轴rpm；编码器和控制器不保存显示单位副本。
 * 初始未更新时为零；当前仅上传输出轴转速到CH6。 */
int motor_vofa_set_encoder(const encoder_data_t *data);
extern volatile int motor_vofa_io_result;
extern volatile int motor_vofa_publish_result;
/* 应用回调结果；语法错误等协议层拒绝看motor_vofa_context.stats.last_command_result。 */
extern volatile int motor_vofa_command_result;
#endif /* MOTOR_VOFA_H */
