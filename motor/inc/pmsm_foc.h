/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_FOC_H
#define PMSM_FOC_H
#include "pmsm_math.h"
#include "motor_rt.h"
typedef struct { float alpha, beta; } pmsm_ab_t;
typedef struct { float d, q; } pmsm_dq_t;
typedef struct { float sine, cosine; } pmsm_trig_t;
/* 电角度单位度；查表插值，范围±360000度。失败返回-1，成功0。 */
int pmsm_sincos(float angle_deg, pmsm_trig_t *trig) MOTOR_RAMFUNC;
/* 两相采样Clarke，假定Iu+Iv+Iw=0；Park/反Park使用同一sin/cos。 */
int pmsm_clarke(float iu, float iv, pmsm_ab_t *ab) MOTOR_RAMFUNC;
int pmsm_park(const pmsm_ab_t *ab, const pmsm_trig_t *trig, pmsm_dq_t *dq) MOTOR_RAMFUNC;
int pmsm_inv_park(const pmsm_dq_t *dq, const pmsm_trig_t *trig, pmsm_ab_t *ab) MOTOR_RAMFUNC;

typedef struct {
    uint32_t u, v, w;
    uint8_t sector; /* 原工程扇区编码1~6；零矢量可能为7。 */
} pmsm_pwm_t;

/* 原工程六扇区算法及过调制等比例压缩；输入是原电压PWM计数尺度，非V。
 * alpha/beta幅值在此尺度下约为相占空比峰值*reload*sqrt(3)。
 * 0=成功，-1=非法输入；失败不改写输出。reload允许4~0xffffff。 */
int pmsm_svpwm(float alpha, float beta, uint32_t reload, pmsm_pwm_t *output) MOTOR_RAMFUNC;
/* 给定Ud/Uq（同上计数尺度）、电角度（度）→反Park→SVPWM。
 * 纯算法，不读编码器/ADC、不写PWM、不使能桥臂。 */
int pmsm_voltage_to_pwm(float ud, float uq, float angle_deg,
                        uint32_t reload, pmsm_pwm_t *output) MOTOR_RAMFUNC;
#endif
