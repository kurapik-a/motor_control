/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

/* A/B 正交解码后的每圈计数，已经包含四倍频：1000 线对应 4000 计数。 */
#ifndef ENCODER_COUNTS_PER_REV
#define ENCODER_COUNTS_PER_REV 4000u
#endif
/* 沿用原调通工程的方向：-1=反向，1=使用硬件计数方向。 */
#ifndef ENCODER_DIRECTION
#define ENCODER_DIRECTION (-1)
#endif

#if ENCODER_COUNTS_PER_REV == 0 || ENCODER_COUNTS_PER_REV > 0x200000u
#error "Encoder counts per revolution must fit the phase counter"
#endif
#if ENCODER_DIRECTION != 1 && ENCODER_DIRECTION != -1
#error "Encoder direction must be 1 or -1"
#endif

/* 当前两项均以上电/清零位置为原点，可为负数；角度连续累加，不在 360 度回绕。 */
typedef struct {
    int32_t position_count;       /* 编码器轴累计计数，不乘减速比。 */
    float mechanical_angle_deg;  /* 累计机械角度，计数 * 360 / 每圈计数。 */
} encoder_data_t;

/* 当前适配 QEI0：A=PB07、B=PB06；不使用物理 Z 输入。
 * board_init() 后调用；0=成功，-1=QEI 时钟不可用。初始化会把当前位置置零。
 * 使用硬件正交计数和软件触发快照，不需要 QEI 中断或电机 PWM。 */
int encoder_init(void);
/* 获取同一次硬件快照的圈计数和相位计数。
 * 0=成功；-1=未初始化；-2=空指针/相位越界；-3=累计计数超出 int32_t。
 * 失败时不改写 data。当前在主循环调用，同一模块不要被前台/中断并发读取。 */
int encoder_read(encoder_data_t *data);
/* 手动把当前位置设为零，不是寻找物理 Z 零位；未初始化返回 -1。 */
int encoder_reset(void);

#endif /* ENCODER_H */
