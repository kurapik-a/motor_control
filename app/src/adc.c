/* SPDX-License-Identifier: BSD-3-Clause */
#include "adc.h"
#include "pwm.h"
#include "hpm_soc.h"
#include "hpm_interrupt.h"
#include "board.h"
#include "hpm_adc16_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include <float.h>
#include <string.h>

#define ADC_U BOARD_PMSM0_ADC_U_BASE
#define ADC_V BOARD_PMSM0_ADC_V_BASE
#define ADC_TRIGGER BOARD_PMSM0_ADC_TRG
#define ADC_DMA_WORDS ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES
#define ADC_DMA_INDEX (ADC_TRIGGER * ADC_SOC_MAX_TRIG_CH_LEN)
#define ADC_ERRORS (adc16_event_trig_sw_conflict | adc16_event_trig_hw_conflict | \
                    adc16_event_read_conflict | adc16_event_seq_sw_conflict |     \
                    adc16_event_seq_hw_conflict | adc16_event_dma_fifo_full)

#if MOTOR_ADC_CONVERSION_TIMEOUT_US == 0
#error "ADC sample frequency and timeout must be positive"
#endif
#if ADC_DMA_INDEX >= ADC_DMA_WORDS
#error "ADC DMA buffer cannot hold the selected trigger queue"
#endif
#if MOTOR_ADC_AUX_PERIOD_PRESCALE > 31u || MOTOR_ADC_AUX_PERIOD_COUNT == 0 || MOTOR_ADC_AUX_PERIOD_COUNT > 255u
#error "Invalid auxiliary ADC period configuration"
#endif

/* DMA 必须能访问系统地址，且不能读到 CPU 缓存中的旧数据。
 * 每个触发队列占 4 个字；ADC0仅使用[U]，ADC1仅使用[V]，各取第一个字。
 * 母线和温度沿用原工程的ADC0低优先级周期采样，不进入PWM触发队列。 */
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
static volatile uint32_t dma_u[ADC_DMA_WORDS];
ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(ADC_SOC_DMA_ADDR_ALIGNMENT)
static volatile uint32_t dma_v[ADC_DMA_WORDS];

typedef union
{
    uint32_t word;
    adc16_pmt_dma_data_t fields;
} adc_dma_word_t;

extern uint32_t hpm_core_clock;
static adc_data_t latest;
static bool initialized, pending, hardware_ready;
/* 首次重臂若延期，从前台首次接管时开始监视；不把其它外设初始化算成采样停滞。 */
static bool rearm_watch_started;
static uint8_t completion_mask;
static int fault;
static float ma_per_count;
static uint64_t timeout_cycles, settle_cycles, offset_interval_cycles;
static uint64_t request_cycles, frame_deadline, calibration_started, last_offset_cycles;
static uint64_t offset_sum_u, offset_sum_v;
static uint64_t last_progress_cycles, rearm_timeout_cycles;

static bool positive_finite(float value) { return value > 0.0f && value <= FLT_MAX; }
static int current_status(void)
{
    if (fault) return fault;
    if (!initialized) return -1;
    if (latest.offset_state == ADC_OFFSET_FAILED) return -5;
    return latest.sample_valid ? 0 : 1;
}
static int latch_fault(int code)
{
    pwm_set_adc_trigger_enabled(false);
    pwm_disable();
    if (hardware_ready) {
        adc16_disable_interrupts(ADC_U, UINT32_MAX);
        adc16_disable_interrupts(ADC_V, UINT32_MAX);
    }
    intc_m_disable_irq(IRQn_ADC0);
    intc_m_disable_irq(IRQn_ADC1);
    pending = false;
    fault = code;
    latest.trigger_armed = latest.sample_valid = latest.current_valid = false;
    latest.fault_code = code;
    return code;
}

static int init_channel(ADC16_Type *ptr, uint8_t channel, uint32_t cycles)
{
    adc16_channel_config_t cfg;
    adc16_get_channel_default_config(&cfg);
    cfg.ch = channel;
    cfg.sample_cycle = cycles;
    return adc16_init_channel(ptr, &cfg) == status_success ? 0 : -4;
}

static int init_converter(ADC16_Type *ptr, uint8_t current_channel)
{
    adc16_config_t cfg;
    adc16_pmt_config_t queue = {0};
    adc16_get_default_config(&cfg);
    cfg.res = adc16_res_16_bits;
    cfg.conv_mode = adc16_conv_mode_preemption;
    cfg.adc_clk_div = MOTOR_ADC_CLOCK_DIVIDER;
    cfg.sel_sync_ahb = false;
    cfg.adc_ahb_en = true;
    if (adc16_init(ptr, &cfg) != status_success)
    {
        return -4;
    }
    /* 每路队列最后一项产生完成中断，两路都完成后才发布。 */
    adc16_disable_interrupts(ptr, UINT32_MAX);
    queue.trig_ch = ADC_TRIGGER;
    queue.trig_len = 1u;
    queue.adc_ch[0] = current_channel;
    if (init_channel(ptr, queue.adc_ch[0], MOTOR_ADC_CURRENT_SAMPLE_CYCLES) != 0)
    {
        return -4;
    }
    queue.inten[queue.trig_len - 1u] = true;
    if (adc16_set_pmt_config(ptr, &queue) != status_success ||
        adc16_set_pmt_queue_enable(ptr, ADC_TRIGGER, true) != status_success)
    {
        return -4;
    }
    adc16_clear_status_flags(ptr, UINT32_MAX);
    return 0;
}

/* 与原工程相同：ADC0 CH15母线、CH1温度，硬件周期计数；0可在重初始化前停采。
 * 电流抢占队列优先；两类采样共用ADC0，不重复初始化转换器或切换全局时钟。 */
static int set_aux_periods(uint8_t period_count)
{
    adc16_prd_config_t cfg = {0};
    cfg.prescale = MOTOR_ADC_AUX_PERIOD_PRESCALE;
    cfg.period_count = period_count;
    cfg.ch = BOARD_APP_ADC16_UDC_CH;
    if (adc16_set_prd_config(BOARD_APP_ADC16_BASE, &cfg) != status_success) return -4;
    cfg.ch = BOARD_APP_ADC16_TEMP_CH;
    return adc16_set_prd_config(BOARD_APP_ADC16_BASE, &cfg) == status_success ? 0 : -4;
}

static int init_aux_channels(void)
{
    if (init_channel(BOARD_APP_ADC16_BASE, BOARD_APP_ADC16_UDC_CH, MOTOR_ADC_AUX_SAMPLE_CYCLES) != 0 ||
        init_channel(BOARD_APP_ADC16_BASE, BOARD_APP_ADC16_TEMP_CH, MOTOR_ADC_AUX_SAMPLE_CYCLES) != 0)
        return -4;
    return set_aux_periods(MOTOR_ADC_AUX_PERIOD_COUNT);
}

/* 前台只读周期结果寄存器；不发起转换，也不等待两项与电流同一时刻更新。 */
static void read_aux_results(void)
{
    uint16_t value;
    if (adc16_get_prd_result(BOARD_APP_ADC16_BASE, BOARD_APP_ADC16_UDC_CH, &value) == status_success)
        latest.bus_raw = value;
    if (adc16_get_prd_result(BOARD_APP_ADC16_BASE, BOARD_APP_ADC16_TEMP_CH, &value) == status_success)
        latest.temperature_raw = value;
}

static int read_dma_word(volatile uint32_t *buffer, uint8_t index, uint8_t channel, uint16_t *result)
{
    adc_dma_word_t item;
    item.word = buffer[ADC_DMA_INDEX + index];
    if (!item.fields.cycle_bit)
    {
        return 1;
    }
    if (item.fields.trig_ch != ADC_TRIGGER || item.fields.seq_num != index || item.fields.adc_ch != channel)
    {
        return -6;
    }
    *result = (uint16_t)item.fields.result;
    return 0;
}

static void update_sample(uint16_t u, uint16_t v)
{
    latest.raw_u = u;
    latest.raw_v = v;
    latest.capture_cycles = request_cycles;
    ++latest.sample_count;
    latest.sample_valid = true;
    latest.current_clipped = latest.raw_u == 0u || latest.raw_u == 65535u ||
                             latest.raw_v == 0u || latest.raw_v == 65535u;
    if (latest.offset_state == ADC_OFFSET_SETTLING && request_cycles >= calibration_started && request_cycles - calibration_started >= settle_cycles)
    {
        latest.offset_state = ADC_OFFSET_COLLECTING;
    }
    if (latest.offset_state == ADC_OFFSET_COLLECTING &&
        request_cycles >= last_offset_cycles && request_cycles - last_offset_cycles >= offset_interval_cycles)
    {
        last_offset_cycles = request_cycles;
        if (latest.current_clipped)
        {
            latest.offset_state = ADC_OFFSET_FAILED;
        }
        else
        {
            offset_sum_u += latest.raw_u;
            offset_sum_v += latest.raw_v;
            if (++latest.offset_samples == MOTOR_CURRENT_OFFSET_SAMPLES)
            {
                latest.offset_u = (uint16_t)((offset_sum_u + MOTOR_CURRENT_OFFSET_SAMPLES / 2u) / MOTOR_CURRENT_OFFSET_SAMPLES);
                latest.offset_v = (uint16_t)((offset_sum_v + MOTOR_CURRENT_OFFSET_SAMPLES / 2u) / MOTOR_CURRENT_OFFSET_SAMPLES);
                latest.offset_state = ADC_OFFSET_READY;
            }
        }
    }
    latest.current_valid = latest.offset_state == ADC_OFFSET_READY && !latest.current_clipped;
    if (latest.current_valid)
    {
        latest.current_u_ma = ((int32_t)latest.offset_u - latest.raw_u) * ma_per_count;
        latest.current_v_ma = ((int32_t)latest.offset_v - latest.raw_v) * ma_per_count;
        latest.current_w_ma = -latest.current_u_ma - latest.current_v_ma;
    }
    else
    {
        latest.current_u_ma = latest.current_v_ma = latest.current_w_ma = 0.0f;
    }
}
/* 返回1表示接近PWM边沿；稍后重试，不忙等。 */
static int arm_frame(void)
{
    uint64_t expected, period;
    int result = pwm_get_adc_trigger_timing(&expected, &period);
    if (result == 1) { ++latest.rearm_deferred_count; return 1; }
    if (result != 0 || timeout_cycles >= period) return latch_fault(-3);
    /* 路由仍关闭且上一组DMA已完成，清标识后才允许下一个硬件边沿。 */
    dma_u[ADC_DMA_INDEX] = 0;
    dma_v[ADC_DMA_INDEX] = 0;
    adc16_clear_status_flags(ADC_U, UINT32_MAX);
    adc16_clear_status_flags(ADC_V, UINT32_MAX);
    completion_mask = 0;
    request_cycles = expected;
    frame_deadline = expected + timeout_cycles;
    pending = true;
    rearm_watch_started = true;
    latest.trigger_armed = true;
    fenceiorw();
    if (pwm_set_adc_trigger_enabled(true) != 0) return latch_fault(-3);
    return 0;
}

/* ADC中断与前台临界区共用；两路完成且本轮DMA标识齐全后，才发布快照。 */
static int service_frame(void)
{
    if (!initialized || fault) return current_status();
    if (!pending) {
        if (!rearm_watch_started) {
            last_progress_cycles = hpm_csr_get_core_cycle();
            rearm_watch_started = true;
        }
        /* PWM停止在保护窗口或前台迟迟不能重臂，也不能永远返回旧的有效采样。 */
        if (hpm_csr_get_core_cycle() - last_progress_cycles > rearm_timeout_cycles) {
            ++latest.timeout_count;
            latest.timeout_stage = ADC_TIMEOUT_REARM;
            return latch_fault(-7);
        }
        arm_frame();
        return current_status();
    }
    uint32_t flags_u = adc16_get_status_flags(ADC_U);
    uint32_t flags_v = adc16_get_status_flags(ADC_V);
    if ((flags_u | flags_v) & ADC_ERRORS) return latch_fault(-8);
    if (flags_u & adc16_event_trig_complete) {
        completion_mask |= 1u;
        adc16_clear_status_flags(ADC_U, adc16_event_trig_complete);
    }
    if (flags_v & adc16_event_trig_complete) {
        completion_mask |= 2u;
        adc16_clear_status_flags(ADC_V, adc16_event_trig_complete);
    }
    if (completion_mask == 3u) {
        pwm_set_adc_trigger_enabled(false);
        latest.trigger_armed = false;
    }
    /* 截止时间小于PWM周期；迟到的数据不读取、不作为同一轮采样发布。 */
    if (hpm_csr_get_core_cycle() >= frame_deadline) {
        ++latest.timeout_count;
        latest.timeout_stage = ADC_TIMEOUT_FRAME;
        latest.timeout_completion_mask = completion_mask;
        return latch_fault(-7);
    }
    if (completion_mask != 3u) return current_status();
    uint16_t values[2];
    int states[2];
    fenceiorw();
    states[0] = read_dma_word(dma_u, 0, BOARD_PMSM0_ADC_CH_U, &values[0]);
    states[1] = read_dma_word(dma_v, 0, BOARD_PMSM0_ADC_CH_V, &values[1]);
    for (uint32_t i = 0; i < 2; ++i) if (states[i] < 0) return latch_fault(states[i]);
    if ((states[0] | states[1]) != 0) {
        ++latest.dma_wait_count;
        return current_status();
    }
    pending = false;
    update_sample(values[0], values[1]);
    latest.publish_cycles = hpm_csr_get_core_cycle();
    last_progress_cycles = latest.publish_cycles;
    if (latest.offset_state == ADC_OFFSET_FAILED) return latch_fault(-5);
    arm_frame();
    return current_status();
}

int adc_start_offset_calibration(void)
{
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized || fault) { int result = current_status(); restore_global_irq(state); return result; }
    if (pwm_is_enabled()) { restore_global_irq(state); return -9; }
    calibration_started = hpm_csr_get_core_cycle();
    last_offset_cycles = calibration_started;
    offset_sum_u = offset_sum_v = 0;
    latest.offset_samples = 0;
    latest.offset_u = latest.offset_v = 0;
    latest.current_u_ma = latest.current_v_ma = latest.current_w_ma = 0;
    latest.current_valid = false;
    latest.offset_state = ADC_OFFSET_SETTLING;
    restore_global_irq(state);
    return 0;
}

int adc_init(void)
{
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (pwm_is_enabled()) { restore_global_irq(state); return -9; }
    pwm_set_adc_trigger_enabled(false);
    intc_m_disable_irq(IRQn_ADC0);
    intc_m_disable_irq(IRQn_ADC1);
    initialized = pending = hardware_ready = false;
    rearm_watch_started = false;
    fault = 0;
    memset(&latest, 0, sizeof latest);
    uint32_t hz = hpm_core_clock;
    if (!hz || pwm_get_reload() == 0 || MOTOR_ADC_CLOCK_DIVIDER == 0 ||
        MOTOR_CURRENT_OFFSET_INTERVAL_MS == 0 ||
        !positive_finite(MOTOR_ADC_REFERENCE_V) || MOTOR_ADC_EFFECTIVE_FULL_SCALE != 65535.0f ||
        !positive_finite(MOTOR_CURRENT_AMPLIFIER_GAIN) || !positive_finite(MOTOR_SHUNT_RESISTANCE_OHM)) {
        latch_fault(-3); restore_global_irq(state); return -3;
    }
    ma_per_count = MOTOR_ADC_REFERENCE_V * 1000.0f /
        (MOTOR_ADC_EFFECTIVE_FULL_SCALE * MOTOR_CURRENT_AMPLIFIER_GAIN * MOTOR_SHUNT_RESISTANCE_OHM);
    if (!positive_finite(ma_per_count) || ma_per_count > FLT_MAX / 131070.0f) {
        latch_fault(-3); restore_global_irq(state); return -3;
    }
    timeout_cycles = ((uint64_t)hz * MOTOR_ADC_CONVERSION_TIMEOUT_US + 999999u) / 1000000u;
    if (timeout_cycles >= (uint64_t)hz / MOTOR_PWM_FREQUENCY_HZ) {
        latch_fault(-3); restore_global_irq(state); return -3;
    }
    settle_cycles = (uint64_t)hz * MOTOR_CURRENT_OFFSET_SETTLE_MS / 1000u;
    rearm_timeout_cycles = (uint64_t)hz * 2u / MOTOR_PWM_FREQUENCY_HZ;
    offset_interval_cycles = ((uint64_t)hz * MOTOR_CURRENT_OFFSET_INTERVAL_MS + 999u) / 1000u;
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    if (!board_init_adc_clock(ADC_U, true) || !board_init_adc_clock(ADC_V, true)) {
        latch_fault(-3); restore_global_irq(state); return -3;
    }
    HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PF08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    board_init_adc16_pins();
    /* 先关闭可能由上一次初始化留下的周期采样，再校准/配置转换器。 */
    if (set_aux_periods(0) != 0 ||
        init_converter(ADC_U, BOARD_PMSM0_ADC_CH_U) != 0 ||
        init_converter(ADC_V, BOARD_PMSM0_ADC_CH_V) != 0 || init_aux_channels() != 0) {
        latch_fault(-4); restore_global_irq(state); return -4;
    }
    hardware_ready = true;
    for (uint32_t i = 0; i < ADC_DMA_WORDS; ++i) dma_u[i] = dma_v[i] = 0;
    adc16_init_pmt_dma(ADC_U, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)(uintptr_t)dma_u));
    adc16_init_pmt_dma(ADC_V, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)(uintptr_t)dma_v));
    initialized = true;
    last_progress_cycles = hpm_csr_get_core_cycle();
    adc_start_offset_calibration();
    adc16_enable_interrupts(ADC_U, adc16_event_trig_complete | ADC_ERRORS);
    adc16_enable_interrupts(ADC_V, adc16_event_trig_complete | ADC_ERRORS);
    intc_m_enable_irq_with_priority(IRQn_ADC0, 5);
    intc_m_enable_irq_with_priority(IRQn_ADC1, 5);
    arm_frame();
    int result = fault;
    restore_global_irq(state);
    return result;
}

int adc_poll(void)
{
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    int result = service_frame();
    if (initialized && !fault) read_aux_results();
    restore_global_irq(state);
    return result;
}

int adc_read(adc_data_t *data)
{
    if (!data) return -2;
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    *data = latest;
    int result = current_status();
    restore_global_irq(state);
    return result;
}

static void adc_u_isr(void) { ++latest.irq_count; service_frame(); }
static void adc_v_isr(void) { ++latest.irq_count; service_frame(); }
SDK_DECLARE_EXT_ISR_M(IRQn_ADC0, adc_u_isr)
SDK_DECLARE_EXT_ISR_M(IRQn_ADC1, adc_v_isr)
