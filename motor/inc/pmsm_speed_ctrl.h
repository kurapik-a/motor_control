/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_SPEED_CTRL_H
#define PMSM_SPEED_CTRL_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_rt.h"

typedef struct {
    bool speed_mode, running, feedback_valid, current_limited;
    uint32_t updates;
    float target_counts_s;
    float raw_counts_s, feedback_counts_s;
    float iq_ref_ma;
} pmsm_speed_status_t;
extern volatile pmsm_speed_status_t pmsm_speed_monitor;
int pmsm_speed_ctrl_init(void);
/* 以下运行接口由电流控制层串行调用：ADC回调或前台短临界区。 */
void pmsm_speed_ctrl_observe(float speed_counts_s) MOTOR_RAMFUNC;
void pmsm_speed_ctrl_reset_feedback(void) MOTOR_RAMFUNC;
int pmsm_speed_ctrl_step(void) MOTOR_RAMFUNC;
/* 目标单位为电机轴计数/秒；切换模式时以当前Iq作为积分初值。 */
int pmsm_speed_ctrl_set_target(float target_counts_s, float initial_iq_ma) MOTOR_RAMFUNC;
void pmsm_speed_ctrl_select_current(void) MOTOR_RAMFUNC;
void pmsm_speed_ctrl_start(void) MOTOR_RAMFUNC;
void pmsm_speed_ctrl_stop(void) MOTOR_RAMFUNC;
void pmsm_speed_ctrl_get_status(pmsm_speed_status_t *status) MOTOR_RAMFUNC;
#endif
