/* SPDX-License-Identifier: BSD-3-Clause */
#include "pmsm_init.h"
#include "pmsm_current_ctrl.h"
#include "pwm.h"
#include "encoder.h"
#include "adc.h"

volatile pmsm_init_status_t pmsm_init_monitor;
static bool initialized;

int pmsm_init(void)
{
    int result;
    if (initialized || pwm_is_enabled()) return -2;
    pmsm_init_monitor.state = PMSM_INIT_OFF;
    pmsm_init_monitor.faults = 0;
    pmsm_init_monitor.stage = PMSM_INIT_STAGE_PWM;
    result = pwm_init();
    if (result) goto failed;
    pmsm_init_monitor.stage = PMSM_INIT_STAGE_ENCODER;
    result = encoder_init();
    if (result) goto failed;
    pmsm_init_monitor.stage = PMSM_INIT_STAGE_ADC;
    result = adc_init();
    if (result < 0) goto failed;
    pmsm_init_monitor.stage = PMSM_INIT_STAGE_CURRENT;
    result = pmsm_current_ctrl_init();
    if (result) goto failed;
    initialized = true;
    return pmsm_prepare();
failed:
    pwm_disable();
    pmsm_init_monitor.state = PMSM_INIT_FAULT;
    pmsm_init_monitor.result = result;
    return result;
}

int pmsm_prepare(void)
{
    pmsm_current_status_t current;
    if (!initialized) return -1;
    pmsm_current_ctrl_get_status(&current);
    if (current.state == PMSM_CURRENT_FAULT) return -4;
    if (current.state != PMSM_CURRENT_STOPPED || pwm_is_enabled()) return -2;
    pmsm_init_monitor.result = 0;
    pmsm_init_monitor.faults = 0;
    pmsm_init_monitor.stage = PMSM_INIT_STAGE_PREPARE;
    pmsm_init_monitor.state = current.aligned ? PMSM_INIT_READY : PMSM_INIT_WAIT_SAMPLES;
    return 0;
}

void pmsm_init_poll(void)
{
    pmsm_current_status_t current;
    adc_data_t sample;
    int result;
    if (!initialized) return;
    pmsm_current_ctrl_poll();
    pmsm_current_ctrl_get_status(&current);
    if (current.state == PMSM_CURRENT_FAULT) {
        pmsm_init_monitor.state = PMSM_INIT_FAULT;
        pmsm_init_monitor.faults = current.faults;
        pmsm_init_monitor.result = -4;
        return;
    }
    /* 正式运行中的普通保护故障，清除后仅恢复就绪，不自动使能。
     * 对齐失败/编码器基准失效则须显式prepare，禁止自动重试对齐。 */
    if (pmsm_init_monitor.state == PMSM_INIT_FAULT) {
        if (pmsm_init_monitor.result != -4) return;
        pmsm_init_monitor.state = current.aligned ? PMSM_INIT_READY : PMSM_INIT_CANCELLED;
        pmsm_init_monitor.faults = 0;
        pmsm_init_monitor.result = 0;
    }
    if (pmsm_init_monitor.state == PMSM_INIT_READY) {
        if (!current.aligned) pmsm_init_monitor.state = PMSM_INIT_CANCELLED;
        return;
    }
    if (pmsm_init_monitor.state != PMSM_INIT_WAIT_SAMPLES && pmsm_init_monitor.state != PMSM_INIT_ALIGNING) return;
    result = adc_read(&sample);
    if (result < 0) {
        pmsm_init_cancel();
        pmsm_init_monitor.state = PMSM_INIT_FAULT;
        pmsm_init_monitor.result = result;
        return;
    }
    if (pmsm_init_monitor.state == PMSM_INIT_WAIT_SAMPLES) {
        result = pmsm_current_ctrl_align();
        pmsm_init_monitor.result = result;
        if (!result) pmsm_init_monitor.state = PMSM_INIT_ALIGNING;
        else if (result != -3) pmsm_init_monitor.state = PMSM_INIT_FAULT;
    } else if (current.state == PMSM_CURRENT_ALIGN_COMPLETE) {
        result = pmsm_current_ctrl_finish_alignment();
        pmsm_init_monitor.result = result;
        if (!result) pmsm_init_monitor.state = PMSM_INIT_READY;
        else if (result != -3) pmsm_init_monitor.state = PMSM_INIT_FAULT;
    } else if (current.state == PMSM_CURRENT_STOPPED) {
        pmsm_init_monitor.state = PMSM_INIT_CANCELLED;
    }
}

bool pmsm_is_ready(void)
{
    pmsm_current_status_t current;
    if (!initialized || pmsm_init_monitor.state != PMSM_INIT_READY) return false;
    pmsm_current_ctrl_get_status(&current);
    return current.aligned && current.state != PMSM_CURRENT_FAULT;
}

void pmsm_init_cancel(void)
{
    pmsm_current_ctrl_stop();
    if (!pmsm_is_ready() && pmsm_init_monitor.state != PMSM_INIT_FAULT)
        pmsm_init_monitor.state = PMSM_INIT_CANCELLED;
}
