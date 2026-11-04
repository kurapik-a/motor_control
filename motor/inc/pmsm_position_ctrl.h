/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_POSITION_CTRL_H
#define PMSM_POSITION_CTRL_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_rt.h"

typedef struct {
    bool position_mode, running, feedback_valid, target_pending, speed_limited;
    uint32_t updates;
    int32_t position_count;
    float target_count, error_count, speed_ref_counts_s;
} pmsm_position_status_t;
extern volatile pmsm_position_status_t pmsm_position_monitor;
int pmsm_position_ctrl_init(void);
/* 由电流调度层串行调用；内部累计位置为计数，不使用角度或减速比。 */
void pmsm_position_ctrl_observe(int32_t position_count) MOTOR_RAMFUNC;
void pmsm_position_ctrl_reset_feedback(void) MOTOR_RAMFUNC;
int pmsm_position_ctrl_set_target(float target_count) MOTOR_RAMFUNC;
int pmsm_position_ctrl_step(void) MOTOR_RAMFUNC;
void pmsm_position_ctrl_start(void) MOTOR_RAMFUNC;
void pmsm_position_ctrl_stop(void) MOTOR_RAMFUNC;
void pmsm_position_ctrl_select_direct(void) MOTOR_RAMFUNC;
void pmsm_position_ctrl_get_status(pmsm_position_status_t *status) MOTOR_RAMFUNC;
#endif
