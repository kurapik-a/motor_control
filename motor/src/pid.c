#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
/* Copyright (c) 2025 hpmicro. SPDX-License-Identifier: BSD-3-Clause */
/* 保留原工程PI公式和饱和冻结积分行为，与SVPWM和外设无依赖。 */
#include "pid.h"
#include <float.h>
#if HPM_MOTOR_MATH_MOD != HPM_MOTOR_MATH_MOD_FP
#error "This port currently supports floating-point motor math only"
#endif
MOTOR_INLINE int pid_finite(float value) { return value >= -FLT_MAX && value <= FLT_MAX; }

void pmsm_pi_reset(pmsm_pi_t *pi)
{
    if (pi) pi->mem = pi->out = 0.0f;
}

int pmsm_pi_step(pmsm_pi_t *pi, HPM_MOTOR_MATH_TYPE target, HPM_MOTOR_MATH_TYPE feedback)
{
    float error, integral, result;
    if (!pi || !pid_finite(target) || !pid_finite(feedback) ||
        !pid_finite(pi->kp) || !pid_finite(pi->ki) ||
        !pid_finite(pi->limit) || !pid_finite(pi->mem) ||
        pi->kp < 0 || pi->ki < 0 || pi->limit <= 0) return -1;
    error = target - feedback;
    integral = HPM_MOTOR_MATH_MUL(error, pi->ki) + pi->mem;
    result = integral + HPM_MOTOR_MATH_MUL(error, pi->kp);
    if (!pid_finite(error) || !pid_finite(integral) || !pid_finite(result)) return -1;
    if (result < -pi->limit) result = -pi->limit;
    else if (result > pi->limit) result = pi->limit;
    else pi->mem = integral;
    pi->out = result;
    return 0;
}
