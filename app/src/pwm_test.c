/* SPDX-License-Identifier: BSD-3-Clause */
#if defined(__GNUC__) && !defined(__clang__)
#pragma GCC optimize ("O2")
#endif
#include "pwm_test.h"
#include "pwm.h"
#include "adc.h"
#include "motor_config.h"
#include "pmsm_foc.h"
#include "hpm_soc.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"
#include <math.h>
#include <float.h>

extern uint32_t hpm_core_clock;
volatile uint32_t pwm_test_request;
volatile pwm_test_monitor_t pwm_test_monitor;
static volatile bool running;
static bool callback_bound;
static volatile uint32_t ticks, adc_tick;
static uint32_t duration_ticks, last_wave_tick, last_sample_count;
static float phase, frequency, modulation;
#define TWO_PI 6.28318530717958647692f

/* 关中断提交PWM及停止路径在ILM执行，避免挤占ADC的10us完成窗口。 */
static void test_tick(void) ATTR_RAMFUNC;
void pwm_test_stop(void) ATTR_RAMFUNC;

/* 使用同步ADC最近快照作软件检查；检查和波形更新仍在前台，不是硬件过流保护。 */
static int check_adc(const adc_data_t *sample, int result)
{
    if (result != 0 || !sample->sample_valid || !sample->current_valid ||
        sample->offset_state != ADC_OFFSET_READY || sample->current_clipped) return PWM_TEST_ADC_NOT_READY;
    uint64_t age = hpm_csr_get_core_cycle() - sample->capture_cycles;
    if (age > (uint64_t)hpm_core_clock * MOTOR_PWM_TEST_ADC_TIMEOUT_MS / 1000u) return PWM_TEST_ADC_STALE;
    float u = fabsf(sample->current_u_ma), v = fabsf(sample->current_v_ma), w = fabsf(sample->current_w_ma);
    /* 峰值包含触发保护的那组样本，避免“已过流，但峰值仍低于门限”的误导。 */
    uint32_t sources = 0;
    float peak = 0;
    float values[3];
    values[0] = u; values[1] = v; values[2] = w;
    for (uint32_t axis = 0; axis < 3; ++axis) {
        if (!(values[axis] <= FLT_MAX)) sources |= (1u << axis) | 8u;
        else {
            if (values[axis] > peak) peak = values[axis];
            if (values[axis] > MOTOR_PWM_TEST_CURRENT_LIMIT_MA) sources |= 1u << axis;
        }
    }
    if (peak > pwm_test_monitor.peak_current_ma) pwm_test_monitor.peak_current_ma = peak;
    if (sources != 0) {
        /* 先关桥再保存诊断，不为记录现场延迟关断；原始码和有符号电流来自同一组采样。 */
        pwm_disable();
        if (!pwm_test_monitor.trip.valid) {
            pwm_test_monitor.trip.during_run = running;
            pwm_test_monitor.trip.source_mask = sources;
            pwm_test_monitor.trip.limit_ma = MOTOR_PWM_TEST_CURRENT_LIMIT_MA;
            pwm_test_monitor.trip.current_u_ma = sample->current_u_ma;
            pwm_test_monitor.trip.current_v_ma = sample->current_v_ma;
            pwm_test_monitor.trip.current_w_ma = sample->current_w_ma;
            pwm_test_monitor.trip.raw_u = sample->raw_u;
            pwm_test_monitor.trip.raw_v = sample->raw_v;
            pwm_test_monitor.trip.offset_u = sample->offset_u;
            pwm_test_monitor.trip.offset_v = sample->offset_v;
            pwm_test_monitor.trip.sample_count = sample->sample_count;
            pwm_test_monitor.trip.sample_age_us = (uint64_t)age * 1000000u / hpm_core_clock;
            pwm_test_monitor.trip.irq_ticks = running ? ticks : 0;
            pwm_test_monitor.trip.elapsed_ms = (uint64_t)pwm_test_monitor.trip.irq_ticks * 1000u / MOTOR_PWM_FREQUENCY_HZ;
            pwm_test_monitor.trip.valid = true;
        }
        return PWM_TEST_OVERCURRENT;
    }
    return 0;
}

/* PWM 周期中断仅检查超时和限时，不在这里调用 ADC 前台接口或正弦函数。
 * 即使主循环卡住，PWM 中断仍可在 ADC 心跳超时后关断；调试器暂停 CPU 时中断也会暂停。 */
static void test_tick(void)
{
    if (!running) return;
    uint32_t now = ++ticks;
    pwm_test_monitor.irq_ticks = now;
    int reason = 0;
    if (now - adc_tick > (uint64_t)MOTOR_PWM_FREQUENCY_HZ * MOTOR_PWM_TEST_ADC_TIMEOUT_MS / 1000u)
        reason = PWM_TEST_ADC_STALE;
    else if (now >= duration_ticks) reason = PWM_TEST_FINISHED;
    if (reason) {
        pwm_disable();
        running = false;
        pwm_test_monitor.output_enabled = false;
        /* 启动时限制计数范围，避免ISR调用Flash中的64位除法库。 */
        pwm_test_monitor.elapsed_ms = now * 1000u / MOTOR_PWM_FREQUENCY_HZ;
        pwm_test_monitor.result = reason;
    }
}

void pwm_test_stop(void)
{
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    running = false;
    pwm_disable();
    pwm_set_current_loop_callback(0);
    callback_bound = false;
    pwm_test_monitor.output_enabled = false;
    if (pwm_test_monitor.result == PWM_TEST_RUNNING) pwm_test_monitor.result = PWM_TEST_STOPPED;
    restore_global_irq(state);
}

int pwm_test_start(float electrical_hz, float amplitude, uint32_t duration_ms)
{
    if (running) return PWM_TEST_BUSY;
    /* 此模块独占测试时的 PWM 控制回调；不能与后续 FOC 同时使用。 */
    pwm_test_stop();
    if (!(fabsf(electrical_hz) >= 0.1f && fabsf(electrical_hz) <= MOTOR_PWM_TEST_MAX_ELECTRICAL_HZ) ||
        !(amplitude > 0.0f && amplitude <= MOTOR_PWM_TEST_MAX_AMPLITUDE) ||
        duration_ms == 0 || duration_ms > MOTOR_PWM_TEST_MAX_DURATION_MS || hpm_core_clock == 0 ||
        ((uint64_t)MOTOR_PWM_FREQUENCY_HZ * duration_ms + 999u) / 1000u > UINT32_MAX / 1000u) {
        pwm_test_monitor.result = PWM_TEST_BAD_ARGUMENT;
        return PWM_TEST_BAD_ARGUMENT;
    }
    adc_data_t sample;
    pwm_test_monitor.peak_current_ma = 0;
    pwm_test_trip_t empty_trip = {0};
    pwm_test_monitor.trip = empty_trip;
    int adc_result = adc_read(&sample);
    int result = check_adc(&sample, adc_result);
    if (result != 0) { pwm_test_monitor.result = result; return result; }
    if (pwm_get_reload() == 0 || pwm_set_duty(0.5f, 0.5f, 0.5f) != 0) {
        pwm_test_monitor.result = PWM_TEST_DRIVER_ERROR;
        return PWM_TEST_DRIVER_ERROR;
    }
    frequency = electrical_hz;
    modulation = amplitude;
    phase = 0;
    ticks = adc_tick = last_wave_tick = 0;
    last_sample_count = sample.sample_count;
    duration_ticks = ((uint64_t)MOTOR_PWM_FREQUENCY_HZ * duration_ms + 999u) / 1000u;
    pwm_test_monitor.reload = pwm_get_reload();
    pwm_test_monitor.irq_ticks = pwm_test_monitor.elapsed_ms = pwm_test_monitor.waveform_updates = 0;
    pwm_test_monitor.electrical_hz = electrical_hz;
    pwm_test_monitor.amplitude = amplitude;
    pwm_test_monitor.phase_deg = 0;
    pwm_test_monitor.duty_u = pwm_test_monitor.duty_v = pwm_test_monitor.duty_w = 0.5f;
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    /* 前台准备期间ADC中断可能锁存故障；使能前在同一临界区重新确认。 */
    adc_result = adc_read(&sample);
    if (adc_result != 0 || !sample.sample_valid || !sample.current_valid || sample.current_clipped) {
        pwm_test_monitor.result = PWM_TEST_ADC_NOT_READY;
        restore_global_irq(state);
        return PWM_TEST_ADC_NOT_READY;
    }
    result = pwm_set_current_loop_callback(test_tick);
    callback_bound = result == 0;
    if (result == 0) result = pwm_enable();
    if (result == 0) {
        running = true;
        pwm_test_monitor.result = PWM_TEST_RUNNING;
        pwm_test_monitor.output_enabled = true;
    } else {
        pwm_disable();
        pwm_set_current_loop_callback(0);
        callback_bound = false;
        pwm_test_monitor.result = PWM_TEST_DRIVER_ERROR;
    }
    restore_global_irq(state);
    return result == 0 ? 0 : PWM_TEST_DRIVER_ERROR;
}

void pwm_test_poll(void)
{
    uint32_t request = pwm_test_request;
    if (request != 0) {
        pwm_test_request = 0;
        if (request == 3) pwm_test_stop();
        else if (request == 1 || request == 2)
            pwm_test_start(request == 1 ? MOTOR_PWM_TEST_ELECTRICAL_HZ : -MOTOR_PWM_TEST_ELECTRICAL_HZ,
                           MOTOR_PWM_TEST_AMPLITUDE, MOTOR_PWM_TEST_DURATION_MS);
    }
    pwm_test_monitor.reload = pwm_get_reload();
    pwm_test_monitor.output_enabled = pwm_is_enabled();
    if (!running) {
        /* 自动结束时中断已关桥，此处释放中断回调。 */
        if (callback_bound) {
            pwm_set_current_loop_callback(0);
            callback_bound = false;
        }
        return;
    }
    adc_data_t sample;
    int adc_result = adc_read(&sample);
    int result = check_adc(&sample, adc_result);
    if (result != 0 || !pwm_is_enabled()) {
        uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
        if (running) {
            pwm_test_stop();
            pwm_test_monitor.result = result != 0 ? result : PWM_TEST_DRIVER_ERROR;
        }
        restore_global_irq(state);
        return;
    }
    if (sample.sample_count != last_sample_count) {
        adc_tick = ticks;
        last_sample_count = sample.sample_count;
    }
    uint32_t now = ticks;
    uint32_t delta = now - last_wave_tick;
    uint32_t update_ticks = (MOTOR_PWM_FREQUENCY_HZ + 999u) / 1000u;
    if (delta < update_ticks) return;
    last_wave_tick = now;
    uint32_t elapsed = (uint64_t)now * 1000u / MOTOR_PWM_FREQUENCY_HZ;
    float ramp = fminf(1.0f, (float)elapsed / MOTOR_PWM_TEST_RAMP_MS);
    /* 先在固定电角度缓升幅值，再缓升旋转频率，减少上电直接跳变。 */
    float speed_ramp = elapsed > MOTOR_PWM_TEST_RAMP_MS
        ? fminf(1.0f, (float)(elapsed - MOTOR_PWM_TEST_RAMP_MS) / MOTOR_PWM_TEST_RAMP_MS) : 0;
    phase += TWO_PI * frequency * speed_ramp * ((float)delta / MOTOR_PWM_FREQUENCY_HZ);
    if (phase >= TWO_PI) phase -= TWO_PI;
    if (phase < 0) phase += TWO_PI;
    float a = modulation * ramp;
    pmsm_pwm_t wave;
    /* 原SVPWM的alpha/beta计数尺度=相占空比峰值*reload*sqrt(3)。
     * 给定旋转电角度和Uq，电流仅用于保护，PI不参与此次开环。 */
    result = pmsm_voltage_to_pwm(0.0f, 1.73205080757f * a * pwm_get_reload(),
                                phase * 360.0f / TWO_PI, pwm_get_reload(), &wave);
    if (result != 0) {
        pwm_test_stop();
        pwm_test_monitor.result = PWM_TEST_DRIVER_ERROR;
        return;
    }
    /* 防止超时 ISR 在前台计算期间结束测试后，前台继续提交新的占空比。 */
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (running) {
        result = pwm_set_counts(wave.u, wave.v, wave.w);
        if (result != 0) {
            pwm_test_stop();
            pwm_test_monitor.result = PWM_TEST_DRIVER_ERROR;
        } else {
            pwm_test_monitor.elapsed_ms = elapsed;
            pwm_test_monitor.phase_deg = phase * 360.0f / TWO_PI;
            pwm_test_monitor.duty_u = (float)wave.u / pwm_test_monitor.reload;
            pwm_test_monitor.duty_v = (float)wave.v / pwm_test_monitor.reload;
            pwm_test_monitor.duty_w = (float)wave.w / pwm_test_monitor.reload;
            pwm_test_monitor.sector = wave.sector;
            ++pwm_test_monitor.waveform_updates;
        }
    }
    restore_global_irq(state);
}
