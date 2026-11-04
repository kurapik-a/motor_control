/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_CURRENT_CTRL_H
#define PMSM_CURRENT_CTRL_H
#include "pmsm_foc.h"
#include "pmsm_protect.h"

typedef enum { PMSM_CURRENT_STOPPED, PMSM_CURRENT_ALIGNING,
    PMSM_CURRENT_RUNNING, PMSM_CURRENT_FAULT } pmsm_current_state_t;
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
 * 每次启动重新对齐；不会使用SPI偏移或直接把编码器清零当作磁极对齐。 */
int pmsm_current_ctrl_start(void) MOTOR_RAMFUNC;
void pmsm_current_ctrl_stop(void) MOTOR_RAMFUNC;
int pmsm_current_ctrl_set_target(float iq_ma) MOTOR_RAMFUNC;
int pmsm_current_ctrl_clear_fault(void) MOTOR_RAMFUNC;
void pmsm_current_ctrl_poll(void) MOTOR_RAMFUNC;
void pmsm_current_ctrl_get_status(pmsm_current_status_t *status) MOTOR_RAMFUNC;
#endif
