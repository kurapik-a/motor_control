/* Copyright (c) 2025 hpmicro. SPDX-License-Identifier: BSD-3-Clause */
/* 参考原pmsm_protect.c的保护项目，独立于旧MOTOR_PARA及当前测试状态机。 */
#include "pmsm_protect.h"
#include "motor_config.h"
#include <float.h>

static bool finite_value(float value) { return value >= -FLT_MAX && value <= FLT_MAX; }
static bool positive(float value) { return finite_value(value) && value > 0.0f; }
static bool over_limit(float value, float limit) { return value > limit || value < -limit; }

static uint32_t evaluate(const pmsm_protect_input_t *input)
{
    uint32_t faults = 0;
    if (!input) return PMSM_FAULT_INPUT;
#if MOTOR_PROTECT_PHASE_ENABLE
    if (!positive(MOTOR_PROTECT_PHASE_LIMIT_MA) || MOTOR_PROTECT_CURRENT_MAX_AGE_US == 0u)
        faults |= PMSM_FAULT_CONFIG;
    if (!input->current_valid || input->current_age_us > MOTOR_PROTECT_CURRENT_MAX_AGE_US ||
        !finite_value(input->iu_ma) || !finite_value(input->iv_ma) || !finite_value(input->iw_ma))
        faults |= PMSM_FAULT_CURRENT_DATA;
    else if (over_limit(input->iu_ma, MOTOR_PROTECT_PHASE_LIMIT_MA) ||
             over_limit(input->iv_ma, MOTOR_PROTECT_PHASE_LIMIT_MA) ||
             over_limit(input->iw_ma, MOTOR_PROTECT_PHASE_LIMIT_MA))
        faults |= PMSM_FAULT_OVERCURRENT;
#endif
#if MOTOR_PROTECT_DQ_ENABLE
    if (!positive(MOTOR_PROTECT_DQ_LIMIT_MA) || MOTOR_PROTECT_CURRENT_MAX_AGE_US == 0u)
        faults |= PMSM_FAULT_CONFIG;
    if (!input->dq_valid || !input->current_valid ||
        input->current_age_us > MOTOR_PROTECT_CURRENT_MAX_AGE_US ||
        !finite_value(input->id_ma) || !finite_value(input->iq_ma)) faults |= PMSM_FAULT_DQ_DATA;
    else if (over_limit(input->id_ma, MOTOR_PROTECT_DQ_LIMIT_MA) ||
             over_limit(input->iq_ma, MOTOR_PROTECT_DQ_LIMIT_MA)) faults |= PMSM_FAULT_OVERCURRENT;
#endif
#if MOTOR_PROTECT_ENCODER_ENABLE
    if (MOTOR_PROTECT_ENCODER_MAX_AGE_MS == 0u || !positive(MOTOR_PROTECT_SPEED_LIMIT_COUNTS_S))
        faults |= PMSM_FAULT_CONFIG;
    if (!input->encoder_valid || input->encoder_age_ms > MOTOR_PROTECT_ENCODER_MAX_AGE_MS ||
        !finite_value(input->speed_counts_s)) faults |= PMSM_FAULT_ENCODER;
    else {
        if (over_limit(input->speed_counts_s, MOTOR_PROTECT_SPEED_LIMIT_COUNTS_S))
            faults |= PMSM_FAULT_OVERSPEED;
#if MOTOR_PROTECT_SPEED_ERROR_ENABLE
        if (!positive(MOTOR_NORMAL_SPEED_ERROR_LIMIT_COUNTS_S)) faults |= PMSM_FAULT_CONFIG;
        if (input->speed_loop_active) {
            float error = input->target_speed_counts_s - input->speed_counts_s;
            if (!finite_value(input->target_speed_counts_s) || !finite_value(error)) faults |= PMSM_FAULT_NUMERIC;
            else if (over_limit(error, MOTOR_NORMAL_SPEED_ERROR_LIMIT_COUNTS_S)) faults |= PMSM_FAULT_SPEED_ERROR;
        }
#endif
    }
#endif
#if MOTOR_PROTECT_BUS_ENABLE
    if (MOTOR_BUS_OVERVOLTAGE_ADC > 65535u || MOTOR_BUS_UNDERVOLTAGE_ADC >= MOTOR_BUS_OVERVOLTAGE_ADC ||
        MOTOR_PROTECT_AUX_MAX_AGE_MS == 0u) faults |= PMSM_FAULT_CONFIG;
    if (!input->bus_valid || input->bus_age_ms > MOTOR_PROTECT_AUX_MAX_AGE_MS) faults |= PMSM_FAULT_BUS_DATA;
    else {
        if (input->bus_raw > MOTOR_BUS_OVERVOLTAGE_ADC) faults |= PMSM_FAULT_BUS_HIGH;
#if MOTOR_BUS_UNDERVOLTAGE_ADC > 0
        if (input->bus_raw < MOTOR_BUS_UNDERVOLTAGE_ADC) faults |= PMSM_FAULT_BUS_LOW;
#endif
    }
#endif
#if MOTOR_PROTECT_TEMP_ENABLE
    if (MOTOR_TEMP_MAX_ADC > 65535u || MOTOR_TEMP_MIN_ADC >= MOTOR_TEMP_MAX_ADC ||
        MOTOR_PROTECT_AUX_MAX_AGE_MS == 0u) faults |= PMSM_FAULT_CONFIG;
    if (!input->temperature_valid || input->temperature_age_ms > MOTOR_PROTECT_AUX_MAX_AGE_MS)
        faults |= PMSM_FAULT_TEMP_DATA;
    else {
        if (input->temperature_raw > MOTOR_TEMP_MAX_ADC) faults |= PMSM_FAULT_TEMP_HIGH;
#if MOTOR_TEMP_MIN_ADC > 0
        if (input->temperature_raw < MOTOR_TEMP_MIN_ADC) faults |= PMSM_FAULT_TEMP_LOW;
#endif
    }
#endif
    return faults;
}

void pmsm_protect_init(pmsm_protect_t *protect)
{
    if (!protect) return;
    protect->active_faults = protect->latched_faults = protect->first_faults = 0;
    protect->ready = false;
}

uint32_t pmsm_protect_check(pmsm_protect_t *protect, const pmsm_protect_input_t *input)
{
    uint32_t faults, latch;
    const uint32_t not_ready = PMSM_FAULT_CURRENT_DATA | PMSM_FAULT_ENCODER |
        PMSM_FAULT_BUS_DATA | PMSM_FAULT_TEMP_DATA | PMSM_FAULT_DQ_DATA;
    if (!protect) return PMSM_FAULT_INPUT;
    faults = evaluate(input);
    protect->active_faults = faults;
    protect->ready = faults == 0;
    latch = input && input->running ? faults : faults & ~not_ready;
    if (latch && !protect->latched_faults) protect->first_faults = latch;
    protect->latched_faults |= latch;
    return protect->active_faults | protect->latched_faults;
}

bool pmsm_protect_can_enable(const pmsm_protect_t *protect)
{
    return protect && protect->ready && !protect->active_faults && !protect->latched_faults;
}

int pmsm_protect_clear(pmsm_protect_t *protect, const pmsm_protect_input_t *input)
{
    if (!protect || !input) return -1;
    if (input->running) return -2;
    /* 清除前重新检查，不允许凭旧ready清除仍存在的故障。 */
    pmsm_protect_check(protect, input);
    if (protect->active_faults) return -3;
    protect->latched_faults = protect->first_faults = 0;
    return 0;
}
