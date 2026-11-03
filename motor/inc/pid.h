/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_PID_H
#define MOTOR_PID_H
#include "pmsm_math.h"

/* 独立控制器模块，当前保留原工程PI，没有D项。
 * Ki已包含每拍周期，不再次乘Ts；每个控制轴使用独立的状态实例。
 * 电流环：target/feedback=mA，out/limit=PWM计数。 */
typedef struct {
    HPM_MOTOR_MATH_TYPE kp, ki, limit; /* 比例、每拍积分增益、输出绝对值上限。 */
    HPM_MOTOR_MATH_TYPE mem, out;     /* 积分状态、最近输出；首次使用初始化为0。 */
} pmsm_pi_t;

/* 清零积分和输出，保留增益与限幅；空指针不操作。 */
void pmsm_pi_reset(pmsm_pi_t *pi);
/* 候选积分+比例，输出对称限幅；饱和时冻结积分。
 * 0=成功，-1=参数或中间值非有限/配置无效；失败不改写状态。
 * 纯算法，不读写外设；调用频率由上层控制环决定。 */
int pmsm_pi_step(pmsm_pi_t *pi, HPM_MOTOR_MATH_TYPE target,
                 HPM_MOTOR_MATH_TYPE feedback);
#endif
