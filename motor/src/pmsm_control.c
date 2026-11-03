/* Copyright (c) 2025 hpmicro. SPDX-License-Identifier: BSD-3-Clause */
/* 从原工程 pmsm_currentctrl_svpwm.c 拆出纯算法；不依赖旧 MOTOR_PARA。 */
#include "pmsm_control.h"
#include <float.h>
#include <math.h>
#if HPM_MOTOR_MATH_MOD != HPM_MOTOR_MATH_MOD_FP
#error "This port currently supports floating-point motor math only"
#endif
static int control_finite(float value) { return value >= -FLT_MAX && value <= FLT_MAX; }

int pmsm_svpwm(float alpha, float beta, uint32_t reload, pmsm_pwm_t *output)
{
    pmsm_pwm_t result;
    if (!output || reload < 4u || reload > 0xffffffu ||
        !control_finite(alpha) || !control_finite(beta) ||
        fabsf(alpha) > 2.0f * reload || fabsf(beta) > 2.0f * reload) return -1;
    int32_t ualpha_60, ubeta_30;
    uint32_t pwm_reload;
    int32_t uref1, uref2, uref3;
    int32_t tx, ty, t0;
    int32_t tuon, tvon, twon = 0;
    int8_t sector = 0;

    uref1 = HPM_MOTOR_MATH_MDF_FL(beta);
    ualpha_60 = HPM_MOTOR_MATH_MDF_FL(alpha);
    ubeta_30 = HPM_MOTOR_MATH_MDF_FL(beta);
    ualpha_60 = (int32_t)(((int64_t)ualpha_60 * 14189) >> 14);
    ubeta_30 = ubeta_30 >> 1;
    uref2 = HPM_MOTOR_MATH_MDF_FL(ualpha_60 - ubeta_30);
    uref3 = HPM_MOTOR_MATH_MDF_FL(-ualpha_60 - ubeta_30);
    pwm_reload = reload;

    if (uref1 >= 0)
        sector = 1;
    if (uref2 >= 0)
        sector = sector + 2;
    if (uref3 >= 0)
        sector = sector + 4;

    switch (sector) {
    case 1: /* sector2  000 010 110 111 110 010 000 */
        tx = -uref2;
        ty = -uref3;
        t0 = pwm_reload - tx - ty;
        /* tx + ty > T */
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        twon = t0 >> 1;
        tuon = ty + twon;
        tvon = tx + tuon;
        break;
    case 2: /* sector6  000   100 101 111 101 100 000 */
        tx = -uref3;
        ty = -uref1;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tvon = t0 >> 1;
        twon = ty + tvon;
        tuon = tx + twon;
        break;
    case 3: /* sector1  000 100 110 111 110 100 000 */
        tx = uref2;
        ty = uref1;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        twon = t0 >> 1;
        tvon = ty + twon;
        tuon = tx + tvon;
        break;
    case 4: /* sector4  000 001 011 111 011 001 000 */
        tx = -uref1;
        ty = -uref2;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tuon = t0 >> 1;
        tvon = ty + tuon;
        twon = tx + tvon;
        break;
    case 5: /* sector5 000 010 011 111 011 010 000 */
        tx = uref1;
        ty = uref3;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tuon = t0 >> 1;
        twon = ty + tuon;
        tvon = tx + twon;
        break;
    case 6: /* sector6 000 001 101 111 101 001 000 */
        tx = uref3;
        ty = uref2;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tvon = t0 >> 1;
        tuon = ty + tvon;
        twon = tx + tuon;
        break;

    default:
        tuon = (int)(pwm_reload/2);
        tvon = (int)(pwm_reload/2);
        twon = (int)(pwm_reload/2);
    }
    result.sector = sector;

    if (tuon < 0)
        tuon = 0;
    if (tvon < 0)
        tvon = 0;
    if (twon < 0)
        twon = 0;
    if (tuon > (int32_t)pwm_reload) tuon = pwm_reload;
    if (tvon > (int32_t)pwm_reload) tvon = pwm_reload;
    if (twon > (int32_t)pwm_reload) twon = pwm_reload;

    result.u = tuon;
    result.v = tvon;
    result.w = twon;
    *output = result;
    return 0;
}


int pmsm_voltage_to_pwm(float ud, float uq, float angle_deg,
                        uint32_t reload, pmsm_pwm_t *output)
{
    float radians, sine, cosine, alpha, beta;
    if (!control_finite(ud) || !control_finite(uq) || !control_finite(angle_deg)) return -1;
    radians = fmodf(angle_deg, 360.0f) * 0.01745329251994329577f;
    sine = sinf(radians);
    cosine = cosf(radians);
    /* 反Park（dq电压→静止alpha/beta）；正Park/Clarke属于电流反馈路径，尚未接入。 */
    alpha = HPM_MOTOR_MATH_MUL(cosine, ud) - HPM_MOTOR_MATH_MUL(sine, uq);
    beta = HPM_MOTOR_MATH_MUL(sine, ud) + HPM_MOTOR_MATH_MUL(cosine, uq);
    return pmsm_svpwm(alpha, beta, reload, output);
}
