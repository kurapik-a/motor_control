/* SPDX-License-Identifier: BSD-3-Clause */
#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
#include "pmsm_current_ctrl.h"
#include "pid.h"
#include "motor_config.h"
#include "adc.h"
#include "encoder.h"
#include "pwm.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"
#include <float.h>
#include <string.h>
#if defined(__GNUC__) && defined(__riscv)
static inline __attribute__((always_inline)) uint64_t hpm_csr_get_core_cycle(void);
#endif
extern uint32_t hpm_core_clock;
volatile pmsm_current_status_t pmsm_current_monitor;
static pmsm_pi_t id_pi, iq_pi;
static pmsm_protect_t protection;
static pmsm_protect_input_t input;
static bool initialized;
static volatile float command_iq;
static float filtered_u, filtered_v, id_ref, iq_ref;
static float cycles_per_second, period_s, slew, offset;
static float us_per_cycle, ms_per_cycle;
static uint64_t last_capture, last_control, period_cycles, current_age_cycles, aux_age_cycles;
static uint64_t budget_cycles, watchdog_cycles;
static uint32_t last_count, speed_phase, reload, align_ticks, stable_ticks;
static uint32_t align_anchor_phase;
static uint64_t speed_capture;
static bool speed_reference, speed_valid;
static uint32_t ramp_ticks, sweep_ticks, required_stable_ticks, timeout_ticks, required_final_good_ticks;
static uint32_t align_good_ticks, align_bad_ticks, align_final_good_ticks;
static float align_id_error_sum, align_iq_abs_sum;

MOTOR_INLINE bool cc_finite(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }
MOTOR_INLINE float cc_abs(float x) { return x < 0 ? -x : x; }
MOTOR_INLINE bool active(void) { return pmsm_current_monitor.state == PMSM_CURRENT_ALIGNING || pmsm_current_monitor.state == PMSM_CURRENT_RUNNING; }
MOTOR_INLINE float approach(float value, float target, float step)
{
    if (value < target) return value + step < target ? value + step : target;
    return value - step > target ? value - step : target;
}
MOTOR_INLINE int32_t phase_delta(uint32_t phase, uint32_t anchor)
{
    int32_t delta = (int32_t)phase - (int32_t)anchor;
    if (delta > (int32_t)(MOTOR_ABZ_COUNTS_PER_REV/2u)) delta -= MOTOR_ABZ_COUNTS_PER_REV;
    if (delta < -(int32_t)(MOTOR_ABZ_COUNTS_PER_REV/2u)) delta += MOTOR_ABZ_COUNTS_PER_REV;
    return delta;
}
MOTOR_INLINE float phase_angle(uint32_t phase)
{
    uint32_t electrical = (phase * MOTOR_POLE_PAIRS) % MOTOR_ABZ_COUNTS_PER_REV;
    float angle = electrical * (360.0f / MOTOR_ABZ_COUNTS_PER_REV);
#if MOTOR_ENCODER_DIRECTION == -1
    if (electrical) angle = 360.0f - angle;
#endif
    return angle;
}
static void current_trip(uint32_t faults) MOTOR_RAMFUNC;
static void current_sample(const adc_data_t *sample, int status) MOTOR_RAMFUNC;
static void current_watchdog(void) MOTOR_RAMFUNC;
static void collect_input(const adc_data_t *sample, int status, int encoder_status, uint64_t now) MOTOR_RAMFUNC;
static void prepare_feedback(const adc_data_t *sample, float angle) MOTOR_RAMFUNC;
static uint32_t qualify_alignment(const adc_data_t *sample, uint32_t phase, const pmsm_dq_t *dq) MOTOR_RAMFUNC;
MOTOR_INLINE void reset_alignment_window(uint32_t phase)
{
    stable_ticks = align_good_ticks = align_bad_ticks = align_final_good_ticks = 0;
    align_id_error_sum = align_iq_abs_sum = 0;
    align_anchor_phase = phase;
}


static void current_trip(uint32_t faults)
{
    pwm_disable(); /* 先关桥，再保存状态。 */
    pwm_set_current_loop_callback(0); /* 释放PWM看门狗中断；ADC仍可观察恢复后的速度。 */
    if (pmsm_current_monitor.state != PMSM_CURRENT_FAULT) pmsm_current_monitor.first_faults = faults;
    pmsm_current_monitor.faults |= faults;
    pmsm_current_monitor.state = PMSM_CURRENT_FAULT;
    command_iq = id_ref = iq_ref = 0;
    pmsm_pi_reset(&id_pi); pmsm_pi_reset(&iq_pi);
}

static void collect_input(const adc_data_t *sample, int status, int encoder_status, uint64_t now)
{
    input.running = active();
    input.current_valid = status == 0 && sample->sample_valid && sample->current_valid && !sample->current_clipped;
    input.iu_ma = sample->current_u_ma; input.iv_ma = sample->current_v_ma; input.iw_ma = sample->current_w_ma;
    /* 超时年龄饱和到门限+1，正常年龄用32位差值换算，避免64位除法。 */
    input.current_age_us = now - sample->capture_cycles > current_age_cycles ? MOTOR_PROTECT_CURRENT_MAX_AGE_US + 1u :
        (uint32_t)((float)(uint32_t)(now-sample->capture_cycles)*us_per_cycle);
    input.encoder_valid = encoder_status == 0 && speed_valid;
    input.encoder_age_ms = 0; /* 该次回调刚读取硬件相位；不借用前台旧角度。 */
    input.speed_counts_s = pmsm_current_monitor.speed_counts_s;
    input.bus_raw = sample->bus_raw; input.temperature_raw = sample->temperature_raw;
    input.bus_valid = sample->bus_valid; input.temperature_valid = sample->temperature_valid;
    input.bus_age_ms = now - sample->bus_cycles > aux_age_cycles ? MOTOR_PROTECT_AUX_MAX_AGE_MS + 1u :
        (uint32_t)((float)(uint32_t)(now-sample->bus_cycles)*ms_per_cycle);
    input.temperature_age_ms = now - sample->temperature_cycles > aux_age_cycles ? MOTOR_PROTECT_AUX_MAX_AGE_MS + 1u :
        (uint32_t)((float)(uint32_t)(now-sample->temperature_cycles)*ms_per_cycle);
}

static void prepare_feedback(const adc_data_t *sample, float angle)
{
    pmsm_trig_t trig;
    pmsm_ab_t ab;
    pmsm_dq_t dq;
    input.dq_valid = false;
    if (input.current_valid && !pmsm_sincos(angle, &trig) &&
        !pmsm_clarke(sample->current_u_ma, sample->current_v_ma, &ab) && !pmsm_park(&ab, &trig, &dq)) {
        input.id_ma = dq.d; input.iq_ma = dq.q; input.dq_valid = true;
    }
}

static void current_sample(const adc_data_t *sample, int status)
{
    uint64_t began, elapsed;
    uint32_t phase = 0, faults;
    int encoder_status;
    int32_t delta;
    float angle, scale, previous_d, previous_q, magnitude;
    pmsm_trig_t trig;
    pmsm_ab_t ab, voltage;
    pmsm_dq_t dq, udq;
    pmsm_pwm_t wave;
    bool controlling = active();
    bool aligning = pmsm_current_monitor.state == PMSM_CURRENT_ALIGNING;
    began = hpm_csr_get_core_cycle();
    if (status != 0) { if (controlling) current_trip(PMSM_CURRENT_ADC_FAULT); return; }
    encoder_status = encoder_read_phase(&phase);
    /* 停机/故障后仍更新实际速度，避免旧超速值使故障永久无法清除。 */
    elapsed = sample->capture_cycles - speed_capture;
    if (!encoder_status && speed_reference && elapsed > 0 && elapsed <= period_cycles * 4u) {
        delta = (int32_t)phase - (int32_t)speed_phase;
        if (delta > (int32_t)(MOTOR_ABZ_COUNTS_PER_REV/2u)) delta -= MOTOR_ABZ_COUNTS_PER_REV;
        if (delta < -(int32_t)(MOTOR_ABZ_COUNTS_PER_REV/2u)) delta += MOTOR_ABZ_COUNTS_PER_REV;
        scale = (float)(delta * MOTOR_ENCODER_DIRECTION) * cycles_per_second / (float)(uint32_t)elapsed;
        pmsm_current_monitor.speed_counts_s += MOTOR_SPEED_FILTER_ALPHA * (scale - pmsm_current_monitor.speed_counts_s);
        speed_valid = true;
    } else {
        speed_valid = false;
        pmsm_current_monitor.speed_counts_s = 0;
    }
    speed_reference = encoder_status == 0;
    speed_capture = sample->capture_cycles; speed_phase = phase;
    if (!controlling) return;
    if (!pwm_is_enabled()) { current_trip(PMSM_CURRENT_DRIVER_FAULT); return; }
    elapsed = sample->capture_cycles - last_capture;
    if (sample->sample_count - last_count != 1u || elapsed < period_cycles / 2u || elapsed > period_cycles + period_cycles / 2u) {
        current_trip(PMSM_CURRENT_TIMING_FAULT); return;
    }
    last_capture = sample->capture_cycles; last_count = sample->sample_count;
    collect_input(sample, status, encoder_status, began);
    angle = phase_angle(phase) - offset;
    if (angle < 0) angle += 360.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    if (pmsm_current_monitor.state == PMSM_CURRENT_ALIGNING) {
        ++align_ticks;
        id_ref = MOTOR_CURRENT_ALIGN_CURRENT_MA * (align_ticks < ramp_ticks ? (float)align_ticks/ramp_ticks : 1.0f);
        angle = align_ticks <= ramp_ticks ? MOTOR_ALIGN_START_ELECTRICAL_DEG :
            (align_ticks < ramp_ticks+sweep_ticks ? MOTOR_ALIGN_START_ELECTRICAL_DEG * (1.0f-(float)(align_ticks-ramp_ticks)/sweep_ticks) : 0.0f);
        iq_ref = 0;
    } else {
        id_ref = approach(id_ref, 0.0f, slew);
        iq_ref = approach(iq_ref, command_iq, slew);
    }
    filtered_u += MOTOR_CURRENT_FILTER_ALPHA * (sample->current_u_ma-filtered_u);
    filtered_v += MOTOR_CURRENT_FILTER_ALPHA * (sample->current_v_ma-filtered_v);
    if (pmsm_sincos(angle, &trig) || pmsm_clarke(filtered_u, filtered_v, &ab) || pmsm_park(&ab, &trig, &dq)) {
        current_trip(PMSM_FAULT_NUMERIC); return;
    }
    input.dq_valid = true; input.id_ma = dq.d; input.iq_ma = dq.q;
    faults = pmsm_protect_check(&protection, &input);
    if (faults) { current_trip(faults); return; }
    if (pmsm_current_monitor.state == PMSM_CURRENT_ALIGNING) {
        faults = qualify_alignment(sample, phase, &dq);
        if (faults) { current_trip(faults); return; }
    }
    previous_d = id_pi.mem; previous_q = iq_pi.mem;
    if (pmsm_pi_step(&id_pi, id_ref, dq.d) || pmsm_pi_step(&iq_pi, iq_ref, dq.q)) {
        current_trip(PMSM_FAULT_NUMERIC); return;
    }
    udq.d = id_pi.out; udq.q = iq_pi.out;
    pmsm_current_monitor.id_voltage_limited = id_pi.limited;
    pmsm_current_monitor.iq_voltage_limited = iq_pi.limited;
    pmsm_current_monitor.alignment_voltage_limited = false;
    if (aligning) {
        magnitude = cc_abs(udq.d)+cc_abs(udq.q);
        if (magnitude > MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS) {
            pmsm_current_monitor.alignment_voltage_limited = true;
            scale = MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS / magnitude;
            udq.d *= scale; udq.q *= scale;
            id_pi.mem = previous_d; iq_pi.mem = previous_q; /* 仅对齐阶段总限幅时冻结两轴积分。 */
        }
    }
    pmsm_current_monitor.voltage_limited = id_pi.limited || iq_pi.limited || pmsm_current_monitor.alignment_voltage_limited;
    if (pmsm_inv_park(&udq, &trig, &voltage) || pmsm_svpwm(voltage.alpha, voltage.beta, reload, &wave)) {
        current_trip(PMSM_FAULT_NUMERIC); return;
    }
    elapsed = hpm_csr_get_core_cycle() - began;
    if (elapsed > budget_cycles) { current_trip(PMSM_CURRENT_TIMING_FAULT); return; }
    if (pwm_set_counts(wave.u, wave.v, wave.w) != 0) { current_trip(PMSM_CURRENT_DRIVER_FAULT); return; }
    last_control = hpm_csr_get_core_cycle();
    elapsed = last_control - began;
    if (elapsed > budget_cycles) { current_trip(PMSM_CURRENT_TIMING_FAULT); return; }
    if (elapsed > pmsm_current_monitor.max_exec_cycles) pmsm_current_monitor.max_exec_cycles = (uint32_t)elapsed;
    ++pmsm_current_monitor.updates;
    pmsm_current_monitor.alignment_ticks = align_ticks;
    pmsm_current_monitor.id_ref_ma = id_ref; pmsm_current_monitor.iq_ref_ma = iq_ref;
    pmsm_current_monitor.id_ma = dq.d; pmsm_current_monitor.iq_ma = dq.q;
    pmsm_current_monitor.ud = udq.d; pmsm_current_monitor.uq = udq.q;
    pmsm_current_monitor.angle_deg = angle;
    pmsm_current_monitor.pwm_u = wave.u; pmsm_current_monitor.pwm_v = wave.v; pmsm_current_monitor.pwm_w = wave.w;
}

/* 位置连续稳定；电流按窗口统计资格，避免偶发采样尖峰永久打断对齐。
 * 相电流/Id/Iq过流等硬门禁已在调用前逐拍执行，不通过该比例规则豁免。 */
static uint32_t qualify_alignment(const adc_data_t *sample, uint32_t phase, const pmsm_dq_t *dq)
{
    float id_error = cc_abs(dq->d-MOTOR_CURRENT_ALIGN_CURRENT_MA);
    float iq_abs = cc_abs(dq->q);
    uint32_t reason = 0;
    bool restart_window = false, qualified = false;
    bool currents_good, fraction_good, averages_good, speed_good;
    int32_t delta;
    if (align_ticks < ramp_ticks+sweep_ticks) {
        reset_alignment_window(phase);
        pmsm_current_monitor.alignment_reason = PMSM_ALIGN_SWEEP;
        pmsm_current_monitor.alignment_stable_ticks = 0;
        return 0;
    }
    if (id_error > MOTOR_CURRENT_ALIGN_ERROR_MA) reason |= PMSM_ALIGN_ID_ERROR;
    if (iq_abs > MOTOR_CURRENT_ALIGN_ERROR_MA) reason |= PMSM_ALIGN_IQ_ERROR;
    currents_good = reason == 0;
    if (!currents_good) {
        ++pmsm_current_monitor.alignment_current_resets; /* 兼容旧字段：现在统计越界拍数。 */
        pmsm_current_monitor.alignment_last_reset = reason;
        pmsm_current_monitor.alignment_last_bad_tick = align_ticks;
        pmsm_current_monitor.alignment_last_bad_reason = reason;
        pmsm_current_monitor.alignment_last_bad_id_ma = dq->d;
        pmsm_current_monitor.alignment_last_bad_iq_ma = dq->q;
        pmsm_current_monitor.alignment_last_bad_raw_u = sample->raw_u;
        pmsm_current_monitor.alignment_last_bad_raw_v = sample->raw_v;
    }
    if (!stable_ticks) align_anchor_phase = phase;
    delta = phase_delta(phase, align_anchor_phase);
    pmsm_current_monitor.alignment_phase_delta = delta;
    if (delta > (int32_t)MOTOR_CURRENT_ALIGN_POSITION_TOLERANCE_COUNTS ||
        delta < -(int32_t)MOTOR_CURRENT_ALIGN_POSITION_TOLERANCE_COUNTS) {
        reason |= PMSM_ALIGN_MOVING;
        ++pmsm_current_monitor.alignment_motion_resets;
        pmsm_current_monitor.alignment_last_reset = PMSM_ALIGN_MOVING;
        restart_window = true;
    } else {
        ++stable_ticks;
        if (currents_good) { ++align_good_ticks; ++align_final_good_ticks; }
        else { ++align_bad_ticks; align_final_good_ticks = 0; }
        align_id_error_sum += id_error;
        align_iq_abs_sum += iq_abs;
        pmsm_current_monitor.alignment_average_speed_counts_s =
            (float)(delta * MOTOR_ENCODER_DIRECTION) * MOTOR_PWM_FREQUENCY_HZ / stable_ticks;
        pmsm_current_monitor.alignment_window_good_ticks = align_good_ticks;
        pmsm_current_monitor.alignment_window_bad_ticks = align_bad_ticks;
        pmsm_current_monitor.alignment_final_good_ticks = align_final_good_ticks;
        pmsm_current_monitor.alignment_window_good_percent = 100.0f * align_good_ticks / stable_ticks;
        pmsm_current_monitor.alignment_mean_id_error_ma = align_id_error_sum / stable_ticks;
        pmsm_current_monitor.alignment_mean_abs_iq_ma = align_iq_abs_sum / stable_ticks;
        if (stable_ticks >= required_stable_ticks) {
            fraction_good = align_good_ticks * 100u >= stable_ticks * MOTOR_CURRENT_ALIGN_WINDOW_GOOD_PERCENT;
            averages_good = pmsm_current_monitor.alignment_mean_id_error_ma <= MOTOR_CURRENT_ALIGN_ERROR_MA &&
                pmsm_current_monitor.alignment_mean_abs_iq_ma <= MOTOR_CURRENT_ALIGN_ERROR_MA;
            speed_good = cc_abs(pmsm_current_monitor.alignment_average_speed_counts_s) <= MOTOR_CURRENT_ALIGN_MAX_SPEED_COUNTS_S;
            qualified = fraction_good && averages_good && speed_good && align_final_good_ticks >= required_final_good_ticks;
            if (!qualified && stable_ticks >= required_stable_ticks+required_final_good_ticks) {
                restart_window = true;
                if (!speed_good) {
                    ++pmsm_current_monitor.alignment_motion_resets;
                    pmsm_current_monitor.alignment_last_reset = PMSM_ALIGN_MOVING;
                    reason |= PMSM_ALIGN_MOVING;
                } else {
                    pmsm_current_monitor.alignment_last_reset = pmsm_current_monitor.alignment_last_bad_reason;
                    reason |= pmsm_current_monitor.alignment_last_bad_reason;
                }
            }
        }
    }
    if (stable_ticks > pmsm_current_monitor.alignment_best_ticks) pmsm_current_monitor.alignment_best_ticks = stable_ticks;
    pmsm_current_monitor.alignment_stable_ticks = stable_ticks;
    pmsm_current_monitor.alignment_reason = reason ? reason : (qualified ? 0u : PMSM_ALIGN_WAIT_STABLE);
    if (qualified) {
        offset = phase_angle(phase);
        pmsm_current_monitor.offset_deg = offset;
        pmsm_current_monitor.state = PMSM_CURRENT_RUNNING;
        return 0;
    }
    if (align_ticks >= timeout_ticks) {
        uint32_t detail = pmsm_current_monitor.alignment_last_reset;
        return PMSM_CURRENT_ALIGN_FAULT |
            ((detail & (PMSM_ALIGN_ID_ERROR|PMSM_ALIGN_IQ_ERROR)) ? PMSM_CURRENT_ALIGN_CURRENT_FAULT : PMSM_CURRENT_ALIGN_POSITION_FAULT);
    }
    if (restart_window) {
        reset_alignment_window(phase);
        pmsm_current_monitor.alignment_stable_ticks = 0;
    }
    return 0;
}

static void current_watchdog(void)
{
    if (active() && hpm_csr_get_core_cycle()-last_control > watchdog_cycles)
        current_trip(PMSM_CURRENT_TIMING_FAULT);
}

int pmsm_current_ctrl_init(void)
{
    if (pwm_is_enabled() || !hpm_core_clock || pwm_get_reload() < 4u) return -1;
    memset((void *)&pmsm_current_monitor, 0, sizeof(pmsm_current_monitor));
    memset(&input, 0, sizeof(input));
    pmsm_protect_init(&protection);
    reload = pwm_get_reload();
    id_pi.kp = MOTOR_CURRENT_D_KP; id_pi.ki = MOTOR_CURRENT_D_KI; id_pi.limit = MOTOR_CURRENT_D_VOLTAGE_LIMIT_TICKS;
    iq_pi.kp = MOTOR_CURRENT_Q_KP; iq_pi.ki = MOTOR_CURRENT_Q_KI; iq_pi.limit = MOTOR_CURRENT_Q_VOLTAGE_LIMIT_TICKS;
    pmsm_pi_reset(&id_pi); pmsm_pi_reset(&iq_pi);
    if (!(MOTOR_CURRENT_FILTER_ALPHA > 0 && MOTOR_CURRENT_FILTER_ALPHA <= 1) ||
        !(MOTOR_SPEED_FILTER_ALPHA > 0 && MOTOR_SPEED_FILTER_ALPHA <= 1) ||
        !(MOTOR_CURRENT_CTRL_MAX_COMMAND_MA > 0) ||
        !cc_finite(MOTOR_CURRENT_CTRL_MAX_COMMAND_MA) || MOTOR_CURRENT_CTRL_MAX_COMMAND_MA > MOTOR_PROTECT_PHASE_LIMIT_MA ||
        !cc_finite(MOTOR_CURRENT_CTRL_DEFAULT_IQ_MA) || cc_abs(MOTOR_CURRENT_CTRL_DEFAULT_IQ_MA) > MOTOR_CURRENT_CTRL_MAX_COMMAND_MA ||
        !(MOTOR_CURRENT_ALIGN_CURRENT_MA > 0 && MOTOR_CURRENT_ALIGN_CURRENT_MA < MOTOR_PROTECT_PHASE_LIMIT_MA) ||
        !(MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS > 0 && MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS < reload) ||
        !(MOTOR_CURRENT_CTRL_SLEW_MA_S > 0) ||
        !cc_finite(MOTOR_CURRENT_CTRL_SLEW_MA_S) ||
        !(MOTOR_CURRENT_ALIGN_ERROR_MA > 0 && MOTOR_CURRENT_ALIGN_ERROR_MA < MOTOR_CURRENT_ALIGN_CURRENT_MA * 0.5f) ||
        !(MOTOR_CURRENT_ALIGN_MAX_SPEED_COUNTS_S > 0) || !cc_finite(MOTOR_CURRENT_ALIGN_MAX_SPEED_COUNTS_S) ||
        pmsm_pi_step(&id_pi, 0, 0) || pmsm_pi_step(&iq_pi, 0, 0)) return -1;
    cycles_per_second = (float)hpm_core_clock;
    us_per_cycle = 1000000.0f / cycles_per_second;
    ms_per_cycle = 1000.0f / cycles_per_second;
    period_s = 1.0f / MOTOR_PWM_FREQUENCY_HZ;
    period_cycles = hpm_core_clock / MOTOR_PWM_FREQUENCY_HZ;
    slew = MOTOR_CURRENT_CTRL_SLEW_MA_S * period_s;
    current_age_cycles = (uint64_t)hpm_core_clock * MOTOR_PROTECT_CURRENT_MAX_AGE_US / 1000000u;
    aux_age_cycles = (uint64_t)hpm_core_clock * MOTOR_PROTECT_AUX_MAX_AGE_MS / 1000u;
    budget_cycles = (uint64_t)hpm_core_clock * MOTOR_CURRENT_CTRL_BUDGET_US / 1000000u;
    watchdog_cycles = (uint64_t)hpm_core_clock * MOTOR_CURRENT_CTRL_WATCHDOG_US / 1000000u;
    ramp_ticks = (uint64_t)MOTOR_PWM_FREQUENCY_HZ * MOTOR_CURRENT_ALIGN_RAMP_MS / 1000u;
    sweep_ticks = (uint64_t)MOTOR_PWM_FREQUENCY_HZ * MOTOR_CURRENT_ALIGN_SWEEP_MS / 1000u;
    required_stable_ticks = (uint64_t)MOTOR_PWM_FREQUENCY_HZ * MOTOR_CURRENT_ALIGN_STABLE_MS / 1000u;
    timeout_ticks = (uint64_t)MOTOR_PWM_FREQUENCY_HZ * MOTOR_CURRENT_ALIGN_TIMEOUT_MS / 1000u;
    required_final_good_ticks = (uint64_t)MOTOR_PWM_FREQUENCY_HZ * MOTOR_CURRENT_ALIGN_FINAL_GOOD_MS / 1000u;
    if (!period_cycles || !budget_cycles || budget_cycles >= period_cycles || !ramp_ticks || !sweep_ticks || !required_stable_ticks ||
        timeout_ticks <= ramp_ticks+sweep_ticks+required_stable_ticks+required_final_good_ticks ||
        !required_final_good_ticks || timeout_ticks > UINT32_MAX/100u) return -1;
    speed_reference = speed_valid = false;
    command_iq = MOTOR_CURRENT_CTRL_DEFAULT_IQ_MA;
    initialized = adc_set_control_callback(current_sample) == 0;
    return initialized ? 0 : -1;
}

int pmsm_current_ctrl_start(void)
{
    adc_data_t sample;
    uint32_t phase = 0;
    int status, enc;
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized) { restore_global_irq(irq); return -1; }
    if (pmsm_current_monitor.state == PMSM_CURRENT_FAULT) { restore_global_irq(irq); return -4; }
    if (active() || pwm_is_enabled()) { restore_global_irq(irq); return -2; }
    status = adc_read(&sample); enc = encoder_read_phase(&phase);
    collect_input(&sample, status, enc, hpm_csr_get_core_cycle());
    prepare_feedback(&sample, MOTOR_ALIGN_START_ELECTRICAL_DEG);
    pmsm_current_monitor.inhibit_faults = pmsm_protect_check(&protection, &input);
    if (!pmsm_protect_can_enable(&protection)) {
        if (protection.latched_faults) {
            current_trip(protection.latched_faults);
            restore_global_irq(irq); return -4;
        }
        restore_global_irq(irq); return -3;
    }
    pmsm_pi_reset(&id_pi); pmsm_pi_reset(&iq_pi);
    filtered_u = filtered_v = id_ref = iq_ref = offset = 0;
    align_ticks = 0;
    reset_alignment_window(phase);
    pmsm_current_monitor.alignment_stable_ticks = pmsm_current_monitor.alignment_best_ticks = 0;
    pmsm_current_monitor.alignment_reason = PMSM_ALIGN_SWEEP;
    pmsm_current_monitor.alignment_last_reset = 0;
    pmsm_current_monitor.alignment_current_resets = pmsm_current_monitor.alignment_motion_resets = 0;
    pmsm_current_monitor.alignment_phase_delta = 0;
    pmsm_current_monitor.alignment_average_speed_counts_s = 0;
    pmsm_current_monitor.alignment_window_good_ticks = pmsm_current_monitor.alignment_window_bad_ticks = 0;
    pmsm_current_monitor.alignment_final_good_ticks = 0;
    pmsm_current_monitor.alignment_window_good_percent = 0;
    pmsm_current_monitor.alignment_mean_id_error_ma = pmsm_current_monitor.alignment_mean_abs_iq_ma = 0;
    pmsm_current_monitor.alignment_last_bad_tick = pmsm_current_monitor.alignment_last_bad_reason = 0;
    pmsm_current_monitor.alignment_last_bad_id_ma = pmsm_current_monitor.alignment_last_bad_iq_ma = 0;
    pmsm_current_monitor.alignment_last_bad_raw_u = pmsm_current_monitor.alignment_last_bad_raw_v = 0;
    last_count = sample.sample_count; last_capture = sample.capture_cycles;
    last_control = hpm_csr_get_core_cycle();
    pmsm_current_monitor.updates = pmsm_current_monitor.alignment_ticks = pmsm_current_monitor.max_exec_cycles = 0;
    if (pwm_set_counts(reload/2u,reload/2u,reload/2u) || pwm_set_current_loop_callback(current_watchdog)) {
        current_trip(PMSM_CURRENT_DRIVER_FAULT); restore_global_irq(irq); return -4;
    }
    pmsm_current_monitor.state = PMSM_CURRENT_ALIGNING;
    if (pwm_enable()) { current_trip(PMSM_CURRENT_DRIVER_FAULT); restore_global_irq(irq); return -4; }
    restore_global_irq(irq);
    return 0;
}

void pmsm_current_ctrl_stop(void)
{
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    pwm_disable(); pwm_set_current_loop_callback(0);
    if (pmsm_current_monitor.state != PMSM_CURRENT_FAULT) pmsm_current_monitor.state = PMSM_CURRENT_STOPPED;
    command_iq = id_ref = iq_ref = 0;
    pmsm_current_monitor.id_ref_ma = pmsm_current_monitor.iq_ref_ma = 0;
    pmsm_pi_reset(&id_pi); pmsm_pi_reset(&iq_pi);
    restore_global_irq(irq);
}

int pmsm_current_ctrl_set_target(float iq_ma)
{
    if (!initialized || !cc_finite(iq_ma) || cc_abs(iq_ma) > MOTOR_CURRENT_CTRL_MAX_COMMAND_MA) return -1;
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (pmsm_current_monitor.state == PMSM_CURRENT_FAULT) { restore_global_irq(irq); return -4; }
    command_iq = iq_ma;
    restore_global_irq(irq);
    return 0;
}

int pmsm_current_ctrl_clear_fault(void)
{
    adc_data_t sample;
    uint32_t phase;
    int status, enc, result;
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized || active() || pwm_is_enabled()) { restore_global_irq(irq); return -2; }
    status = adc_read(&sample); enc = encoder_read_phase(&phase);
    collect_input(&sample, status, enc, hpm_csr_get_core_cycle());
    prepare_feedback(&sample, phase_angle(phase)-offset);
    result = pmsm_protect_clear(&protection, &input);
    if (result == 0) {
        pmsm_current_monitor.faults = pmsm_current_monitor.first_faults = 0;
        pmsm_current_monitor.inhibit_faults = 0;
        pmsm_current_monitor.state = PMSM_CURRENT_STOPPED;
        pmsm_current_ctrl_stop();
    }
    restore_global_irq(irq);
    return result;
}

void pmsm_current_ctrl_poll(void)
{
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (active()) {
        if (!pwm_is_enabled()) current_trip(PMSM_CURRENT_DRIVER_FAULT);
        else current_watchdog();
    }
    restore_global_irq(irq);
}

void pmsm_current_ctrl_get_status(pmsm_current_status_t *status)
{
    if (!status) return;
    uint32_t irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    *status = pmsm_current_monitor;
    restore_global_irq(irq);
}
