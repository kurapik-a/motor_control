/* SPDX-License-Identifier: BSD-3-Clause */
#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
#include "pmsm_speed_ctrl.h"
#include "motor_config.h"
#include "pid.h"
#include "hpm_interrupt.h"
#include "hpm_csr_drv.h"
#include <float.h>
#include <string.h>

volatile pmsm_speed_status_t pmsm_speed_monitor;
static pmsm_pi_t speed_pi;
MOTOR_INLINE bool speed_finite(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }

int pmsm_speed_ctrl_init(void)
{
    memset((void *)&pmsm_speed_monitor, 0, sizeof(pmsm_speed_monitor));
    if (!(MOTOR_SPEED_FILTER_ALPHA > 0 && MOTOR_SPEED_FILTER_ALPHA <= 1) ||
        !(MOTOR_SPEED_CTRL_MAX_COUNTS_S > 0) || !speed_finite(MOTOR_SPEED_CTRL_MAX_COUNTS_S) ||
        MOTOR_SPEED_CTRL_MAX_COUNTS_S >= MOTOR_PROTECT_SPEED_LIMIT_COUNTS_S ||
        !(MOTOR_SPEED_CTRL_IQ_LIMIT_MA > 0 && MOTOR_SPEED_CTRL_IQ_LIMIT_MA <= MOTOR_CURRENT_CTRL_MAX_COMMAND_MA)) return -1;
    speed_pi.kp = MOTOR_SPEED_KP; speed_pi.ki = MOTOR_SPEED_KI;
    speed_pi.limit = MOTOR_SPEED_CTRL_IQ_LIMIT_MA;
    pmsm_pi_reset(&speed_pi);
    return pmsm_pi_step(&speed_pi, 0, 0);
}

void pmsm_speed_ctrl_observe(float speed_counts_s)
{
    if (!speed_finite(speed_counts_s)) { pmsm_speed_ctrl_reset_feedback(); return; }
    pmsm_speed_monitor.raw_counts_s = speed_counts_s;
    if (!pmsm_speed_monitor.feedback_valid) pmsm_speed_monitor.feedback_counts_s = speed_counts_s;
    else pmsm_speed_monitor.feedback_counts_s += MOTOR_SPEED_FILTER_ALPHA *
        (speed_counts_s - pmsm_speed_monitor.feedback_counts_s);
    pmsm_speed_monitor.feedback_valid = true;
}

void pmsm_speed_ctrl_reset_feedback(void)
{
    pmsm_speed_monitor.feedback_valid = false;
    pmsm_speed_monitor.raw_counts_s = pmsm_speed_monitor.feedback_counts_s = 0;
}

int pmsm_speed_ctrl_step(void)
{
    if (!pmsm_speed_monitor.speed_mode || !pmsm_speed_monitor.running || !pmsm_speed_monitor.feedback_valid) return -1;
    if (pmsm_pi_step(&speed_pi, pmsm_speed_monitor.target_counts_s, pmsm_speed_monitor.feedback_counts_s)) return -1;
    pmsm_speed_monitor.iq_ref_ma = speed_pi.out;
    pmsm_speed_monitor.current_limited = speed_pi.limited;
    ++pmsm_speed_monitor.updates;
    return 0;
}

int pmsm_speed_ctrl_set_target(float target_counts_s, float initial_iq_ma)
{
    if (!speed_finite(target_counts_s) || !speed_finite(initial_iq_ma) ||
        target_counts_s > MOTOR_SPEED_CTRL_MAX_COUNTS_S || target_counts_s < -MOTOR_SPEED_CTRL_MAX_COUNTS_S) return -1;
    if (!pmsm_speed_monitor.speed_mode) {
        pmsm_pi_reset(&speed_pi);
        if (initial_iq_ma > speed_pi.limit) initial_iq_ma = speed_pi.limit;
        if (initial_iq_ma < -speed_pi.limit) initial_iq_ma = -speed_pi.limit;
        speed_pi.mem = speed_pi.out = initial_iq_ma;
        pmsm_speed_monitor.iq_ref_ma = initial_iq_ma;
    }
    pmsm_speed_monitor.speed_mode = true;
    pmsm_speed_monitor.target_counts_s = target_counts_s;
    return 0;
}

void pmsm_speed_ctrl_start(void)
{
    pmsm_pi_reset(&speed_pi);
    pmsm_speed_monitor.iq_ref_ma = 0;
    pmsm_speed_monitor.current_limited = false;
    pmsm_speed_monitor.running = true;
}

void pmsm_speed_ctrl_stop(void)
{
    pmsm_pi_reset(&speed_pi);
    pmsm_speed_monitor.running = false;
    pmsm_speed_monitor.target_counts_s = 0;
    pmsm_speed_monitor.iq_ref_ma = 0;
    pmsm_speed_monitor.current_limited = false;
}

void pmsm_speed_ctrl_select_current(void)
{
    pmsm_speed_ctrl_stop();
    pmsm_speed_monitor.speed_mode = false;
}

void pmsm_speed_ctrl_get_status(pmsm_speed_status_t *status)
{
    uint32_t irq;
    if (!status) return;
    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    *status = pmsm_speed_monitor;
    restore_global_irq(irq);
}
