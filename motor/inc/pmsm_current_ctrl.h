/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_CURRENT_CTRL_H
#define PMSM_CURRENT_CTRL_H
#include "pmsm_foc.h"
#include "pmsm_protect.h"

typedef enum { PMSM_CURRENT_STOPPED, PMSM_CURRENT_ALIGNING,
    PMSM_CURRENT_RUNNING, PMSM_CURRENT_FAULT,
    PMSM_CURRENT_ALIGN_RELEASING, PMSM_CURRENT_ALIGN_COMPLETE } pmsm_current_state_t;
enum {
    PMSM_CURRENT_ADC_FAULT = 1u << 15,
    PMSM_CURRENT_TIMING_FAULT = 1u << 16,
    PMSM_CURRENT_DRIVER_FAULT = 1u << 17,
    PMSM_CURRENT_ALIGN_FAULT = 1u << 18,
    PMSM_CURRENT_ALIGN_CURRENT_FAULT = 1u << 19,
    PMSM_CURRENT_ALIGN_POSITION_FAULT = 1u << 20
};
enum {
    PMSM_ALIGN_ID_ERROR = 1u, PMSM_ALIGN_IQ_ERROR = 2u,
    PMSM_ALIGN_MOVING = 4u, PMSM_ALIGN_SWEEP = 8u, PMSM_ALIGN_WAIT_STABLE = 16u
};
typedef struct {
    pmsm_current_state_t state;
    uint32_t faults, first_faults, inhibit_faults, updates, alignment_ticks, max_exec_cycles;
    uint32_t alignment_stable_ticks, alignment_best_ticks, alignment_reason, alignment_last_reset;
    uint32_t alignment_current_resets, alignment_motion_resets;
    uint32_t alignment_release_ticks;
    bool aligned; /* 本次上电的编码器零点已由对齐建立；普通STOP不清除。 */
    int32_t alignment_phase_delta;
    float alignment_average_speed_counts_s;
    uint32_t alignment_window_good_ticks, alignment_window_bad_ticks, alignment_final_good_ticks;
    float alignment_window_good_percent, alignment_mean_id_error_ma, alignment_mean_abs_iq_ma;
    uint32_t alignment_last_bad_tick, alignment_last_bad_reason;
    float alignment_last_bad_id_ma, alignment_last_bad_iq_ma;
    uint16_t alignment_last_bad_raw_u, alignment_last_bad_raw_v;
    float id_ref_ma, iq_ref_ma, id_ma, iq_ma;
    float ud, uq, angle_deg, offset_deg, speed_counts_s;
    uint32_t pwm_u, pwm_v, pwm_w;
    bool id_voltage_limited, iq_voltage_limited, alignment_voltage_limited;
    bool voltage_limited; /* 上述三项取或；不包含SVPWM过调制压缩。 */
} pmsm_current_status_t;
extern volatile pmsm_current_status_t pmsm_current_monitor;

/* board/PWM/encoder/ADC初始化后调用；绑定新ADC帧回调，不使能桥臂。 */
int pmsm_current_ctrl_init(void);
/* 仅前台调用。0成功；-1参数/未初始化，-2忙，-3采样/保护未就绪，-4故障锁存。
 * start只启动正式运行；未建立对齐零点返回-3，不隐式启动对齐。 */
int pmsm_current_ctrl_start(void) MOTOR_RAMFUNC;
/* pmsm_init状态机使用：启动对齐；完成降流关桥后，由前台清零编码器。 */
int pmsm_current_ctrl_align(void) MOTOR_RAMFUNC;
int pmsm_current_ctrl_finish_alignment(void) MOTOR_RAMFUNC;
void pmsm_current_ctrl_stop(void) MOTOR_RAMFUNC;
int pmsm_current_ctrl_set_target(float iq_ma) MOTOR_RAMFUNC;
/* 电机轴计数/秒；选择速度模式，速度PI在ADC调度中生成Iq。 */
int pmsm_current_ctrl_set_speed_counts_s(float speed_counts_s) MOTOR_RAMFUNC;
/* 电机轴绝对累计计数；选择位置→速度→电流级联。 */
int pmsm_current_ctrl_set_position_counts(float position_count) MOTOR_RAMFUNC;
int pmsm_current_ctrl_clear_fault(void) MOTOR_RAMFUNC;
void pmsm_current_ctrl_poll(void) MOTOR_RAMFUNC;
void pmsm_current_ctrl_get_status(pmsm_current_status_t *status) MOTOR_RAMFUNC;
#endif
