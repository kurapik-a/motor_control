/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_CONTROL_H
#define PMSM_CONTROL_H
#include "pmsm_math.h"

typedef struct {
    uint32_t u, v, w;
    uint8_t sector; /* 原工程扇区编码1~6；零矢量可能为7。 */
} pmsm_pwm_t;

/* 原工程六扇区算法及过调制等比例压缩；输入是原电压PWM计数尺度，非V。
 * alpha/beta幅值在此尺度下约为相占空比峰值*reload*sqrt(3)。
 * 0=成功，-1=非法输入；失败不改写输出。reload允许4~0xffffff。 */
int pmsm_svpwm(float alpha, float beta, uint32_t reload, pmsm_pwm_t *output);
/* 给定Ud/Uq（同上计数尺度）、电角度（度）→反Park→SVPWM。
 * 纯算法，不读编码器/ADC、不写PWM、不使能桥臂。 */
int pmsm_voltage_to_pwm(float ud, float uq, float angle_deg,
                        uint32_t reload, pmsm_pwm_t *output);
#endif
