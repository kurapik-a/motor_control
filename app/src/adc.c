/* SPDX-License-Identifier: BSD-3-Clause */
#include "adc.h"
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

#if MOTOR_ADC_SAMPLE_HZ == 0 || MOTOR_ADC_CONVERSION_TIMEOUT_US == 0
#error "ADC sample frequency and timeout must be positive"
#endif
#if ADC_DMA_INDEX + 3 > ADC_DMA_WORDS
#error "ADC DMA buffer cannot hold the selected trigger queue"
#endif

/* DMA 必须能访问系统地址，且不能读到 CPU 缓存中的旧数据。
 * 每个触发队列占 4 个字；ADC0 使用 [U,母线,温度]，ADC1 仅使用 [V]。 */
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
static bool initialized, pending;
static int fault;
static float ma_per_count;
static uint64_t period_cycles, timeout_cycles, settle_cycles, offset_interval_cycles;
static uint64_t request_cycles, last_request_cycles, calibration_started, last_offset_cycles;
static uint64_t offset_sum_u, offset_sum_v;

static bool positive_finite(float value)
{
    return value > 0.0f && value <= FLT_MAX;
}

static int current_status(void)
{
    if (fault != 0)
    {
        return fault;
    }
    if (!initialized)
    {
        return -1;
    }
    if (latest.offset_state == ADC_OFFSET_FAILED)
    {
        return -5;
    }
    return latest.sample_valid ? 0 : 1;
}

static int latch_fault(int code)
{
    fault = code;
    latest.sample_valid = false;
    latest.current_valid = false;
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

static int init_converter(ADC16_Type *ptr, bool with_aux)
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
    /* 只轮询状态，不向 PLIC 开放 ADC 中断；inten 用来产生队列完成状态。 */
    adc16_disable_interrupts(ptr, UINT32_MAX);
    queue.trig_ch = ADC_TRIGGER;
    queue.trig_len = with_aux ? 3u : 1u;
    queue.adc_ch[0] = with_aux ? BOARD_PMSM0_ADC_CH_U : BOARD_PMSM0_ADC_CH_V;
    if (init_channel(ptr, queue.adc_ch[0], MOTOR_ADC_CURRENT_SAMPLE_CYCLES) != 0)
    {
        return -4;
    }
    if (with_aux)
    {
        queue.adc_ch[1] = BOARD_APP_ADC16_UDC_CH;
        queue.adc_ch[2] = BOARD_APP_ADC16_TEMP_CH;
        if (init_channel(ptr, queue.adc_ch[1], MOTOR_ADC_AUX_SAMPLE_CYCLES) != 0 ||
            init_channel(ptr, queue.adc_ch[2], MOTOR_ADC_AUX_SAMPLE_CYCLES) != 0)
        {
            return -4;
        }
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

int adc_start_offset_calibration(void)
{
    if (!initialized || fault != 0)
    {
        return fault != 0 ? fault : -1;
    }
    if (pending)
    {
        return 1;
    }
    calibration_started = hpm_csr_get_core_cycle();
    last_offset_cycles = calibration_started;
    offset_sum_u = offset_sum_v = 0;
    latest.offset_samples = 0;
    latest.offset_u = latest.offset_v = 0;
    latest.current_u_ma = latest.current_v_ma = latest.current_w_ma = 0.0f;
    latest.current_valid = false;
    latest.offset_state = ADC_OFFSET_SETTLING;
    return 0;
}

int adc_init(void)
{
    uint32_t hz = hpm_core_clock;
    initialized = pending = false;
    fault = 0;
    memset(&latest, 0, sizeof(latest));
    if (hz < MOTOR_ADC_SAMPLE_HZ || MOTOR_ADC_CLOCK_DIVIDER == 0u ||
        MOTOR_CURRENT_OFFSET_INTERVAL_MS == 0u ||
        !positive_finite(MOTOR_ADC_REFERENCE_V) ||
        MOTOR_ADC_EFFECTIVE_FULL_SCALE != 65535.0f ||
        !positive_finite(MOTOR_CURRENT_AMPLIFIER_GAIN) ||
        !positive_finite(MOTOR_SHUNT_RESISTANCE_OHM))
    {
        return latch_fault(-3);
    }
    ma_per_count = MOTOR_ADC_REFERENCE_V * 1000.0f /
                   (MOTOR_ADC_EFFECTIVE_FULL_SCALE * MOTOR_CURRENT_AMPLIFIER_GAIN * MOTOR_SHUNT_RESISTANCE_OHM);
    if (!positive_finite(ma_per_count) || ma_per_count > FLT_MAX / 131070.0f)
    {
        return latch_fault(-3);
    }
    period_cycles = ((uint64_t)hz + MOTOR_ADC_SAMPLE_HZ - 1u) / MOTOR_ADC_SAMPLE_HZ;
    timeout_cycles = ((uint64_t)hz * MOTOR_ADC_CONVERSION_TIMEOUT_US + 999999u) / 1000000u;
    settle_cycles = (uint64_t)hz * MOTOR_CURRENT_OFFSET_SETTLE_MS / 1000u;
    offset_interval_cycles = ((uint64_t)hz * MOTOR_CURRENT_OFFSET_INTERVAL_MS + 999u) / 1000u;
    clock_add_to_group(clock_adc0, 0);
    clock_add_to_group(clock_adc1, 0);
    if (board_init_adc_clock(ADC_U, true) == 0u || board_init_adc_clock(ADC_V, true) == 0u)
    {
        return latch_fault(-3);
    }
    HPM_IOC->PAD[IOC_PAD_PF06].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    HPM_IOC->PAD[IOC_PAD_PF08].FUNC_CTL = IOC_PAD_FUNC_CTL_ANALOG_MASK;
    board_init_adc16_pins(); /* 本板 PF31 母线、PF26 温度。 */
    if (init_converter(ADC_U, true) != 0 || init_converter(ADC_V, false) != 0)
    {
        return latch_fault(-4);
    }
    for (uint32_t i = 0; i < ADC_DMA_WORDS; ++i)
    {
        dma_u[i] = dma_v[i] = 0;
    }
    adc16_init_pmt_dma(ADC_U, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)(uintptr_t)dma_u));
    adc16_init_pmt_dma(ADC_V, core_local_mem_to_sys_address(BOARD_RUNNING_CORE, (uint32_t)(uintptr_t)dma_v));
    initialized = true;
    last_request_cycles = hpm_csr_get_core_cycle() - period_cycles;
    return adc_start_offset_calibration();
}

/* cycle_bit=1 表示该位置已写入新结果。软件每次触发前清零，不把它当作交替翻转位。
 * 先检查完成状态，再检查每个 DMA 字；防止只完成一路或 DMA 尚未落地就使用旧数据。 */
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

static void update_sample(uint16_t u, uint16_t v, uint16_t bus, uint16_t temperature)
{
    latest.raw_u = u;
    latest.raw_v = v;
    latest.bus_raw = bus;
    latest.temperature_raw = temperature;
    latest.capture_cycles = request_cycles;
    ++latest.sample_count;
    latest.sample_valid = true;
    latest.current_clipped = latest.raw_u == 0u || latest.raw_u == 65535u ||
                             latest.raw_v == 0u || latest.raw_v == 65535u;
    if (latest.offset_state == ADC_OFFSET_SETTLING && request_cycles - calibration_started >= settle_cycles)
    {
        latest.offset_state = ADC_OFFSET_COLLECTING;
    }
    if (latest.offset_state == ADC_OFFSET_COLLECTING &&
        request_cycles - last_offset_cycles >= offset_interval_cycles)
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

int adc_poll(void)
{
    uint64_t now;
    if (!initialized || fault != 0)
    {
        return current_status();
    }
    now = hpm_csr_get_core_cycle();
    if (pending)
    {
        uint32_t flags_u = adc16_get_status_flags(ADC_U);
        uint32_t flags_v = adc16_get_status_flags(ADC_V);
        if ((flags_u | flags_v) & ADC_ERRORS)
        {
            return latch_fault(-8);
        }
        if ((flags_u & flags_v & adc16_event_trig_complete) != 0u)
        {
            uint16_t results[4];
            int states[4];
            fenceiorw();
            states[0] = read_dma_word(dma_u, 0, BOARD_PMSM0_ADC_CH_U, &results[0]);
            states[1] = read_dma_word(dma_v, 0, BOARD_PMSM0_ADC_CH_V, &results[1]);
            states[2] = read_dma_word(dma_u, 1, BOARD_APP_ADC16_UDC_CH, &results[2]);
            states[3] = read_dma_word(dma_u, 2, BOARD_APP_ADC16_TEMP_CH, &results[3]);
            for (uint32_t i = 0; i < 4u; ++i)
            {
                if (states[i] < 0)
                {
                    return latch_fault(states[i]);
                }
            }
            if ((states[0] | states[1] | states[2] | states[3]) == 0)
            {
                pending = false;
                update_sample(results[0], results[1], results[2], results[3]);
            }
        }
        if (pending)
        {
            if (now - request_cycles >= timeout_cycles)
            {
                return latch_fault(-7);
            }
            return current_status();
        }
    }
    if (now - last_request_cycles >= period_cycles)
    {
        dma_u[ADC_DMA_INDEX] = dma_u[ADC_DMA_INDEX + 1u] = dma_u[ADC_DMA_INDEX + 2u] = 0;
        dma_v[ADC_DMA_INDEX] = 0;
        adc16_clear_status_flags(ADC_U, UINT32_MAX);
        adc16_clear_status_flags(ADC_V, UINT32_MAX);
        fenceiorw();
        request_cycles = last_request_cycles = now;
        pending = true;
        if (adc16_trigger_pmt_by_sw(ADC_U, ADC_TRIGGER) != status_success ||
            adc16_trigger_pmt_by_sw(ADC_V, ADC_TRIGGER) != status_success)
        {
            return latch_fault(-4);
        }
    }
    return current_status();
}

int adc_read(adc_data_t *data)
{
    if (data == NULL)
    {
        return -2;
    }
    *data = latest;
    return current_status();
}
