/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef ENCODER_H
#define ENCODER_H
#include <stdint.h>

/* A/B 正交解码后的每圈计数，包含四倍频：1000 线对应 4000 计数。 */
#ifndef ENCODER_COUNTS_PER_REV
#define ENCODER_COUNTS_PER_REV 4000u
#endif
/* 沿用原调通工程方向：-1=反向，1=硬件计数方向。 */
#ifndef ENCODER_DIRECTION
#define ENCODER_DIRECTION (-1)
#endif
/* 电机极对数：机械角度每变化一圈，电角度变化这么多圈。 */
#ifndef ENCODER_POLE_PAIRS
#define ENCODER_POLE_PAIRS 14u
#endif
/* 主循环测速的标称更新频率，单位 Hz；不是 VOFA 上传频率。 */
#ifndef ENCODER_SPEED_SAMPLE_HZ
#define ENCODER_SPEED_SAMPLE_HZ 1000u
#endif
/* 标称测速周期下的一阶低通系数，范围 (0,1]；1=不滤波。
 * 沿用原工程 0.1，主循环延迟时按实际时间修正有效系数。 */
#ifndef ENCODER_SPEED_FILTER_ALPHA
#define ENCODER_SPEED_FILTER_ALPHA 0.1f
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

typedef struct {
    int32_t position_count;        /* 编码器轴累计计数，上电/清零位置为原点。 */
    float mechanical_angle_deg;   /* 累计机械角度，可为负值、可超过 360 度。 */
    float electrical_angle_deg;   /* 电角度，[0,360)，参考零点尚需 FOC 对齐标定。 */
    float speed_counts_per_s;     /* 原始测速，编码器计数/秒，供后续速度环使用。 */
    float speed_filtered_counts_per_s; /* 滤波测速，编码器计数/秒。 */
    float speed_rpm;              /* 原始转速，编码器/电机轴 rpm，不除减速比。 */
    float speed_filtered_rpm;     /* 滤波转速，编码器/电机轴 rpm。 */
} encoder_data_t;

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
/* 将当前位置设为零，同时清除速度和滤波历史；不是寻找物理 Z 零位。
 * 电角度零参考也随清零改变；后续 FOC 对齐应确定它与转子磁极的关系。 */
int encoder_reset(void);
#endif /* ENCODER_H */
