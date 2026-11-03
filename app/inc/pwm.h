/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_PWM_H
#define MOTOR_PWM_H
#include <stdbool.h>
#include <stdint.h>

/* 原 single_motor_monitor_vofa 的 PWM0 独立驱动；PE00~PE05 对应 UH/UL/VH/VL/WH/WL。
 * 三相中心对齐互补输出；频率、死区使用 motor_config.h。
 * 返回值：0=成功，-1=未初始化，-2=时钟/周期配置无效，-3=占空比/计数越界。
 * 当前仅移植 PWM，不自动启动 FOC、不自动接管 ADC，不在此处发送串口数据。 */
int pwm_init(void);           /* board_init 后调用；首次初始化六路强制低电平，重复调用不改变现有状态。 */
uint32_t pwm_get_reload(void); /* 返回每周期计数；未初始化返回 0。 */
bool pwm_is_enabled(void);    /* 软件输出使能状态，不代表已经形成闭环控制。 */
int pwm_set_counts(uint32_t u, uint32_t v, uint32_t w); /* 三相宽度计数，范围 0~reload；兼容原 SVPWM 输出尺度。 */
int pwm_set_duty(float u, float v, float w); /* 三相占空比，范围 0~1；拒绝越界、NaN、Inf。 */
int pwm_enable(void);         /* 显式恢复三相输出；调用方负责标定状态、故障门禁及控制目标。 */
void pwm_disable(void);       /* 立即强制六路为低电平；计数器和预留触发点继续运行。 */

/* 后续 FOC 接口：回调在 PWM0 比较中断中以 PWM 频率调用，禁止阻塞操作。
 * NULL=关闭该中断并关闭桥臂；初始化默认 NULL，绑定回调不会自动使能桥臂。
 * ADC 触发比较点已准备在周期末，但未连接 TRGM→ADC；接入同步采样时须统一改造 adc.c。 */
typedef void (*pwm_current_loop_callback_t)(void);
int pwm_set_current_loop_callback(pwm_current_loop_callback_t callback);
#endif /* MOTOR_PWM_H */
