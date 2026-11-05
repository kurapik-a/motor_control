/* SPDX-License-Identifier: BSD-3-Clause */
#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
#include "pmsm_position_ctrl.h"
#include "motor_config.h"
#include "pid.h"
#include "pmsm_trajectory.h"
#include "hpm_interrupt.h"
#include "hpm_csr_drv.h"
#include <float.h>
#include <string.h>

volatile pmsm_position_status_t pmsm_position_monitor;
static pmsm_pi_t position_pi;
MOTOR_INLINE bool position_finite(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }

int pmsm_position_ctrl_init(void)
{
    memset((void *)&pmsm_position_monitor, 0, sizeof(pmsm_position_monitor));
    if (!(MOTOR_POSITION_MAX_COMMAND_COUNTS > 0 && MOTOR_POSITION_MAX_COMMAND_COUNTS <= 16777216.0f) ||
        !(MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK > 0) ||
        MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK * MOTOR_SPEED_LOOP_HZ > MOTOR_SPEED_CTRL_MAX_COUNTS_S) return -1;
    position_pi.kp = MOTOR_POSITION_KP; position_pi.ki = MOTOR_POSITION_KI;
    position_pi.limit = MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK;
    pmsm_pi_reset(&position_pi);
    return pmsm_pi_step(&position_pi, 0, 0);
}

void pmsm_position_ctrl_observe(int32_t position_count)
{
    pmsm_position_monitor.position_count = position_count;
    pmsm_position_monitor.feedback_valid = true;
}

void pmsm_position_ctrl_reset_feedback(void)
{
    pmsm_position_monitor.feedback_valid = false;
}

int pmsm_position_ctrl_set_target(float target_count)
{
    if (!position_finite(target_count) || target_count > MOTOR_POSITION_MAX_COMMAND_COUNTS ||
        target_count < -MOTOR_POSITION_MAX_COMMAND_COUNTS) return -1;
    if (!pmsm_position_monitor.position_mode) pmsm_pi_reset(&position_pi);
    pmsm_position_monitor.position_mode = true;
    pmsm_position_monitor.target_pending = true;
    pmsm_position_monitor.target_count = target_count;
    return 0;
}

int pmsm_position_ctrl_step(void)
{
    float limit = MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK * MOTOR_SPEED_LOOP_HZ;
    float previous = position_pi.mem;
    if (!pmsm_position_monitor.position_mode || !pmsm_position_monitor.running || !pmsm_position_monitor.feedback_valid) return -1;
    pmsm_position_monitor.reference_count = pmsm_trajectory_monitor.position_count;
    pmsm_position_monitor.target_count = pmsm_trajectory_monitor.target_count;
    pmsm_position_monitor.feedforward_counts_s = pmsm_trajectory_monitor.speed_counts_s;
    if (pmsm_pi_step(&position_pi, pmsm_position_monitor.reference_count, (float)pmsm_position_monitor.position_count)) return -1;
    pmsm_position_monitor.error_count = pmsm_position_monitor.reference_count - (float)pmsm_position_monitor.position_count;
    /* 原位置PI输出为计数/外环拍；周期换算属于控制律，不是显示单位转换。 */
    pmsm_position_monitor.speed_ref_counts_s = position_pi.out * MOTOR_SPEED_LOOP_HZ + pmsm_position_monitor.feedforward_counts_s;
    pmsm_position_monitor.speed_limited = position_pi.limited;
    if (pmsm_position_monitor.speed_ref_counts_s > limit) {
        pmsm_position_monitor.speed_ref_counts_s = limit; pmsm_position_monitor.speed_limited = true; position_pi.mem = previous;
    } else if (pmsm_position_monitor.speed_ref_counts_s < -limit) {
        pmsm_position_monitor.speed_ref_counts_s = -limit; pmsm_position_monitor.speed_limited = true; position_pi.mem = previous;
    }
    ++pmsm_position_monitor.updates;
    return 0;
}

void pmsm_position_ctrl_start(void)
{
    pmsm_pi_reset(&position_pi);
    if (!pmsm_position_monitor.target_pending) pmsm_position_monitor.target_count = (float)pmsm_position_monitor.position_count;
    pmsm_position_monitor.speed_ref_counts_s = 0;
    pmsm_position_monitor.reference_count = (float)pmsm_position_monitor.position_count;
    pmsm_position_monitor.feedforward_counts_s = 0;
    pmsm_position_monitor.speed_limited = false;
    pmsm_position_monitor.running = true;
}

void pmsm_position_ctrl_stop(void)
{
    pmsm_pi_reset(&position_pi);
    pmsm_position_monitor.running = false;
    pmsm_position_monitor.target_pending = false;
    pmsm_position_monitor.target_count = (float)pmsm_position_monitor.position_count;
    pmsm_position_monitor.error_count = pmsm_position_monitor.speed_ref_counts_s = 0;
    pmsm_position_monitor.reference_count = (float)pmsm_position_monitor.position_count;
    pmsm_position_monitor.feedforward_counts_s = 0;
    pmsm_position_monitor.speed_limited = false;
}

void pmsm_position_ctrl_select_direct(void)
{
    pmsm_position_ctrl_stop();
    pmsm_position_monitor.position_mode = false;
}

void pmsm_position_ctrl_get_status(pmsm_position_status_t *status)
{
    uint32_t irq;
    if (!status) return;
    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    *status = pmsm_position_monitor;
    restore_global_irq(irq);
}
