/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>
#include "motor_config.h"

/* 编码器数值统一在 motor/inc/motor_config.h 修改。
 * 以下仅兼容模块内部宏名，不再保存独立初值；编译参数仍可覆盖这些兼容名。 */
#ifndef ENCODER_COUNTS_PER_REV
#define ENCODER_COUNTS_PER_REV MOTOR_ABZ_COUNTS_PER_REV
#endif
#ifndef ENCODER_DIRECTION
#define ENCODER_DIRECTION MOTOR_ENCODER_DIRECTION
#endif
#ifndef ENCODER_POLE_PAIRS
#define ENCODER_POLE_PAIRS MOTOR_POLE_PAIRS
#endif
#ifndef ENCODER_SPEED_SAMPLE_HZ
#define ENCODER_SPEED_SAMPLE_HZ MOTOR_SPEED_ESTIMATE_HZ
#endif
#ifndef ENCODER_SPEED_FILTER_ALPHA
#define ENCODER_SPEED_FILTER_ALPHA MOTOR_SPEED_FILTER_ALPHA
#endif

#if MOTOR_ENCODER_USE_SPI != 0
#error "Current encoder implementation only supports QEI A/B"
#endif
#if ENCODER_COUNTS_PER_REV == 0 || ENCODER_COUNTS_PER_REV > 0x200000u
#error "Encoder counts per revolution must fit the phase counter"
#endif
#if ENCODER_DIRECTION != 1 && ENCODER_DIRECTION != -1
#error "Encoder direction must be 1 or -1"
#endif
#if ENCODER_POLE_PAIRS == 0 || ENCODER_SPEED_SAMPLE_HZ == 0
#error "Encoder pole pairs and speed sample frequency must be positive"
#endif

typedef struct
{
    int32_t position_count;            /* 编码器轴累计计数，上电/清零位置为原点。 */
    float mechanical_angle_deg;        /* 累计机械角度，可为负值、可超过 360 度。 */
    float electrical_angle_deg;        /* 电角度，[0,360)，参考零点尚需 FOC 对齐标定。 */
    float speed_counts_per_s;          /* 原始测速，编码器计数/秒，供后续速度环使用。 */
    float speed_filtered_counts_per_s; /* 滤波测速，编码器计数/秒。 */
    float speed_rpm;                   /* 原始转速，编码器/电机轴 rpm，不除减速比。 */
    float speed_filtered_rpm;          /* 滤波转速，编码器/电机轴 rpm。 */
} encoder_data_t;

/* 减速器输出轴数据；角度仍是相对于编码器清零位置的累计角度。 */
typedef struct
{
    float angle_deg;          /* 输出轴累计机械角度，单位度，不在 360 度回绕。 */
    float speed_rpm;          /* 输出轴原始转速，单位转/分钟。 */
    float speed_filtered_rpm; /* 输出轴滤波转速，单位转/分钟。 */
} encoder_output_data_t;

/* board_init() 后调用；QEI0：A=PB07、B=PB06，不使用物理 Z。
 * 0=成功，-1=QEI/CPU 时钟无效，-4=滤波系数无效。初始化清零计数及测速历史。
 * 不依赖电机 PWM 或 QEI 中断。 */
int encoder_init(void);
/* 读取硬件一致快照，每次更新位置和电角度，约每 1 ms 更新测速。
 * 前台忙时不补算多拍，直接按两次有效测速之间的实际时间计算速度。
 * 第一次读取仅建立测速基准，速度为零；之后未到更新周期时返回最近速度。
 * 0=成功，-1=未初始化，-2=空指针/相位越界，-3=累计计数超出 int32_t。
 * 失败不改写 data。当前仅在主循环调用，不要由前台/中断并发调用。 */
int encoder_read(encoder_data_t *data);
/* 把同一次 encoder_read() 的结果换算到减速器输出轴。
 * 三个值分别除以 motor_config.h 的 MOTOR_SPEED_REDUCTION_RATIO，默认 7.75。
 * 纯数据换算，不再次读取硬件，不改变原采样、测速历史或电角度。
 * 不要求先初始化编码器，也可换算调用方提供的有效电机轴数据。
 * 0=成功，-1=空指针，-2=减速比非正/非有限，-3=输入或结果非有限。
 * 失败不改写 output。若减速器未机械归零，这里也不表示绝对机械零位。 */
int encoder_to_output_axis(const encoder_data_t *motor, encoder_output_data_t *output);
/* 将当前位置设为零，同时清除速度和滤波历史；不是寻找物理 Z 零位。
 * 电角度零参考也随清零改变；后续 FOC 对齐应确定它与转子磁极的关系。 */
int encoder_reset(void);
#endif /* ENCODER_H */
