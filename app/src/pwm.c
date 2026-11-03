/* SPDX-License-Identifier: BSD-3-Clause */
#include "pwm.h"
#include "motor_config.h"
#include "board.h"
#include "hpm_pwmv2_drv.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"
#include "hpm_trgm_drv.h"
extern uint32_t hpm_core_clock;

static uint32_t reload;
static bool initialized;
static volatile bool output_enabled;
static pwm_current_loop_callback_t volatile current_loop;

/* 保留原工程的配对波形、比较器和影子寄存器分配。 */
static void init_waveforms(PWMV2_Type *ptr, uint32_t PWM_PRD, uint8_t CMP_SHADOW_REGISTER_UPDATE_TYPE, uint8_t CMP_PWM_REGISTER_UPDATE_TYPE, uint8_t CMP_SOURCE)
{
    pwmv2_cmp_config_t cmp_cfg[2] = {0};
    pwmv2_pair_config_t pwm_cfg = {0};

    cmp_cfg[0].cmp = PWM_PRD;
    cmp_cfg[0].enable_half_cmp = false;
    cmp_cfg[0].enable_hrcmp = false;
    cmp_cfg[0].cmp_source = CMP_SOURCE;
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(1);
    cmp_cfg[0].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;

    cmp_cfg[1].cmp = PWM_PRD;
    cmp_cfg[1].enable_half_cmp = false;
    cmp_cfg[1].enable_hrcmp = false;
    cmp_cfg[1].cmp_source = CMP_SOURCE;
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(2);
    cmp_cfg[1].update_trigger = CMP_SHADOW_REGISTER_UPDATE_TYPE;

    pwm_cfg.pwm[0].enable_output = true;
    pwm_cfg.pwm[0].enable_async_fault = false;
    pwm_cfg.pwm[0].enable_sync_fault = false;
    pwm_cfg.pwm[0].invert_output = false;
    pwm_cfg.pwm[0].enable_four_cmp = false;
    pwm_cfg.pwm[0].update_trigger = CMP_PWM_REGISTER_UPDATE_TYPE;
    pwm_cfg.pwm[0].dead_zone_in_half_cycle = MOTOR_PWM_DEADTIME_HALF_TICKS;

    pwm_cfg.pwm[1].enable_output = true;
    pwm_cfg.pwm[1].enable_async_fault = false;
    pwm_cfg.pwm[1].enable_sync_fault = false;
    pwm_cfg.pwm[1].invert_output = false;
    pwm_cfg.pwm[1].enable_four_cmp = false;
    pwm_cfg.pwm[1].update_trigger = CMP_PWM_REGISTER_UPDATE_TYPE;
    pwm_cfg.pwm[1].dead_zone_in_half_cycle = MOTOR_PWM_DEADTIME_HALF_TICKS;

    pwmv2_disable_counter(ptr, pwm_counter_0);
    pwmv2_reset_counter(ptr, pwm_counter_0);

    pwmv2_shadow_register_unlock(ptr);
    pwmv2_set_shadow_val(ptr, PWMV2_SHADOW_INDEX(0), PWM_PRD, 0, false);
    pwmv2_set_shadow_val(ptr, PWMV2_SHADOW_INDEX(10), 1, 0, false);
    pwmv2_shadow_register_lock(ptr);

    pwmv2_setup_waveform_in_pair(ptr, pwm_channel_0, &pwm_cfg, PWMV2_CMP_INDEX(0), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(3);
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(4);
    pwmv2_setup_waveform_in_pair(ptr, pwm_channel_2, &pwm_cfg, PWMV2_CMP_INDEX(4), &cmp_cfg[0], 2);
    cmp_cfg[0].cmp_source_index = PWMV2_SHADOW_INDEX(5);
    cmp_cfg[1].cmp_source_index = PWMV2_SHADOW_INDEX(6);
    pwmv2_setup_waveform_in_pair(ptr, pwm_channel_4, &pwm_cfg, PWMV2_CMP_INDEX(8), &cmp_cfg[0], 2);

    pwmv2_counter_select_data_offset_from_shadow_value(BOARD_PMSM0PWM, pwm_counter_0, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(ptr, pwm_counter_0);
    pwmv2_set_reload_update_time(ptr, pwm_counter_0, CMP_PWM_REGISTER_UPDATE_TYPE);

    pwmv2_counter_select_data_offset_from_shadow_value(BOARD_PMSM0PWM, pwm_counter_1, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(ptr, pwm_counter_1);
    pwmv2_set_reload_update_time(ptr, pwm_counter_1, CMP_PWM_REGISTER_UPDATE_TYPE);

    pwmv2_counter_select_data_offset_from_shadow_value(BOARD_PMSM0PWM, pwm_counter_2, PWMV2_SHADOW_INDEX(0));
    pwmv2_counter_burst_disable(ptr, pwm_counter_2);
    pwmv2_set_reload_update_time(ptr, pwm_counter_2, CMP_PWM_REGISTER_UPDATE_TYPE);
}

/* 原工程 disable_all_pwm_output：强制输出值为低，不把“零占空比”当成关闭桥臂。 */
void pwm_disable(void)
{
    if (!initialized) return;
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    PWMV2_Type *ptr = BOARD_PMSM0PWM;
    pwmv2_shadow_register_unlock(ptr);
    for (uint8_t ch = 0; ch < 6; ++ch) {
        pwmv2_set_force_update_time(ptr, ch, pwm_force_immediately);
        pwmv2_force_update_time_by_shadow(ptr, ch, pwm_force_update_shadow_immediately);
        pwmv2_force_output(ptr, ch, pwm_force_output_1, false);
        pwmv2_enable_force_by_software(ptr, ch);
        pwmv2_enable_software_force(ptr, ch);
    }
    pwmv2_shadow_register_lock(ptr);
    output_enabled = false;
    restore_global_irq(state);
}

int pwm_enable(void)
{
    if (!initialized) return -1;
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    for (uint8_t ch = 0; ch < 6; ++ch) {
        pwmv2_disable_force_by_software(BOARD_PMSM0PWM, ch);
    }
    output_enabled = true;
    restore_global_irq(state);
    return 0;
}

/* 对应原 bldc_foc_pwmset；三相宽度转为中心对齐的左右两个比较点。 */
int pwm_set_counts(uint32_t u, uint32_t v, uint32_t w)
{
    if (!initialized) return -1;
    if (u > reload || v > reload || w > reload) return -3;
    uint32_t half = reload >> 1;
    uint32_t width[3] = {u >> 1, v >> 1, w >> 1};
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    pwmv2_shadow_register_unlock(BOARD_PMSM0PWM);
    for (uint8_t phase = 0; phase < 3; ++phase) {
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, (PWMV2_SHADOW_INDEX(1) + phase * 2), half - width[phase], 0, false);
        pwmv2_set_shadow_val(BOARD_PMSM0PWM, (PWMV2_SHADOW_INDEX(2) + phase * 2), half + width[phase], 0, false);
    }
    pwmv2_shadow_register_lock(BOARD_PMSM0PWM);
    restore_global_irq(state);
    return 0;
}

int pwm_set_duty(float u, float v, float w)
{
    if (!initialized) return -1;
    if (!(u >= 0.0f && u <= 1.0f) || !(v >= 0.0f && v <= 1.0f) || !(w >= 0.0f && w <= 1.0f)) return -3;
    return pwm_set_counts((uint32_t)(u * reload + 0.5f), (uint32_t)(v * reload + 0.5f), (uint32_t)(w * reload + 0.5f));
}

static void init_trigger(uint8_t cmp, uint8_t shadow, uint8_t trgo, uint32_t point)
{
    pwmv2_cmp_config_t config = {0};
    pwmv2_config_t output = {0};
    config.cmp = point;
    config.cmp_source = cmp_value_from_shadow_val;
    config.cmp_source_index = shadow;
    config.update_trigger = pwm_shadow_register_update_on_reload;
    output.enable_output = true;
    output.update_trigger = pwm_reload_update_on_reload;
    output.dead_zone_in_half_cycle = MOTOR_PWM_DEADTIME_HALF_TICKS;
    pwmv2_shadow_register_unlock(BOARD_PMSM0PWM);
    pwmv2_config_cmp(BOARD_PMSM0PWM, cmp, &config);
    pwmv2_config_pwm(BOARD_PMSM0PWM, cmp, &output, false);
    pwmv2_select_cmp_source(BOARD_PMSM0PWM, cmp, cmp_value_from_shadow_val, shadow);
    pwmv2_set_trigout_cmp_index(BOARD_PMSM0PWM, trgo, cmp);
    pwmv2_cmp_select_counter(BOARD_PMSM0PWM, cmp, pwm_counter_0);
    pwmv2_shadow_register_lock(BOARD_PMSM0PWM);
}

int pwm_init(void)
{
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    /* 重复调用保持现有配置，避免运行中的通道重配时短暂解除强制关断。 */
    if (initialized) {
        restore_global_irq(state);
        return 0;
    }
    initialized = false;
    output_enabled = false;
    reload = 0;
    current_loop = 0;
    clock_add_to_group(clock_mot0, 0);
    clock_add_to_group(clock_pwm0, 0);
    pwmv2_disable_cmp_irq(BOARD_PMSM0PWM, BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP);
    intc_m_disable_irq(BOARD_PMSM0APP_PWM_IRQ);
    uint32_t hz = clock_get_frequency(BOARD_STEPPER0_MOTOR_CLOCK_SOURCE);
    uint32_t period = hz / MOTOR_PWM_FREQUENCY_HZ;
    if (hz % MOTOR_PWM_FREQUENCY_HZ != 0 || period < 4u || (period & 1u) ||
        MOTOR_PWM_DEADTIME_HALF_TICKS >= period || MOTOR_PWM_CURRENT_ISR_DELAY_TICKS >= period) {
        restore_global_irq(state);
        return -2;
    }
    reload = period;
    init_waveforms(BOARD_PMSM0PWM, reload, pwm_shadow_register_update_on_reload,
                   pwm_reload_update_on_reload, cmp_value_from_shadow_val);
    initialized = true;
    pwm_disable();
    pwm_set_counts(reload / 2u, reload / 2u, reload / 2u);
    init_trigger(BOARD_PMSM0_PWM_TRIG_CMP_INDEX, 9, BOARD_PMSM0_PWM_TRIGOUT_CH_ADC, reload - 1u);
    init_trigger(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP, 10, BOARD_PMSM0_PWM_TRIGOUT_CH_CURRENTLOOP,
                 MOTOR_PWM_CURRENT_ISR_DELAY_TICKS);
    pwm_set_adc_trigger_enabled(false); /* 采样路由先关闭，由adc_init配置完成后重臂。 */
    /* 六路功率输出仍保持强制低电平。 */
    HPM_IOC->PAD[IOC_PAD_PE00].FUNC_CTL = IOC_PE00_FUNC_CTL_PWM0_P_0;
    HPM_IOC->PAD[IOC_PAD_PE01].FUNC_CTL = IOC_PE01_FUNC_CTL_PWM0_P_1;
    HPM_IOC->PAD[IOC_PAD_PE02].FUNC_CTL = IOC_PE02_FUNC_CTL_PWM0_P_2;
    HPM_IOC->PAD[IOC_PAD_PE03].FUNC_CTL = IOC_PE03_FUNC_CTL_PWM0_P_3;
    HPM_IOC->PAD[IOC_PAD_PE04].FUNC_CTL = IOC_PE04_FUNC_CTL_PWM0_P_4;
    HPM_IOC->PAD[IOC_PAD_PE05].FUNC_CTL = IOC_PE05_FUNC_CTL_PWM0_P_5;
    pwmv2_enable_multi_counter_sync(BOARD_PMSM0PWM, 0x07);
    pwmv2_start_pwm_output_sync(BOARD_PMSM0PWM, 0x07);
    pwmv2_clear_cmp_irq_status(BOARD_PMSM0PWM, PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP));
    restore_global_irq(state);
    return 0;
}

uint32_t pwm_get_reload(void) { return initialized ? reload : 0; }
bool pwm_is_enabled(void) { return initialized && output_enabled; }

int pwm_set_current_loop_callback(pwm_current_loop_callback_t callback)
{
    if (!initialized) return -1;
    uint32_t state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    pwmv2_disable_cmp_irq(BOARD_PMSM0PWM, BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP);
    pwmv2_clear_cmp_irq_status(BOARD_PMSM0PWM, PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP));
    current_loop = callback;
    if (callback) {
        intc_m_enable_irq_with_priority(BOARD_PMSM0APP_PWM_IRQ, 6);
        pwmv2_enable_cmp_irq(BOARD_PMSM0PWM, BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP);
    } else {
        intc_m_disable_irq(BOARD_PMSM0APP_PWM_IRQ);
        pwm_disable();
    }
    restore_global_irq(state);
    return 0;
}

static void pwm_current_loop_isr(void)
{
    uint32_t mask = PWM_IRQ_CMP(BOARD_PMSM0_PWM_TRIG_CMP_INDEX_CURRENTLOOP);
    if (pwmv2_get_cmp_irq_status(BOARD_PMSM0PWM) & mask) {
        pwmv2_clear_cmp_irq_status(BOARD_PMSM0PWM, mask);
        if (current_loop) current_loop();
    }
}
SDK_DECLARE_EXT_ISR_M(BOARD_PMSM0APP_PWM_IRQ, pwm_current_loop_isr)

/* TRGO0 同时送到ADC0/ADC1的TRIG0A，关闭时将路由接恒低输入；由adc.c独占管理。 */
int pwm_set_adc_trigger_enabled(bool enable)
{
    if (!initialized) return -1;
    trgm_output_t route = {0};
    route.type = trgm_output_same_as_input;
    route.input = enable ? BOARD_PMSM0_TRIGMUX_IN_NUM : HPM_TRGM0_INPUT_SRC_VSS;
    trgm_output_config(BOARD_PMSM0PWM_TRGM, BOARD_PMSM0_TRG_NUM, &route);
    fenceiorw();
    return 0;
}

/* 在调用方短临界区内取得下一个采样边沿的保守时刻；不在边沿附近切换路由。
 * 用读计数器之前的CPU时刻计算，截止时间略早，不会把下一周期数据误判为本周期。 */
int pwm_get_adc_trigger_timing(uint64_t *trigger_cycles, uint64_t *period_cycles)
{
    if (!initialized) return -1;
    if (!trigger_cycles || !period_cycles) return -3;
    uint32_t hz = clock_get_frequency(BOARD_STEPPER0_MOTOR_CLOCK_SOURCE);
    if (!hz || !hpm_core_clock || reload <= MOTOR_ADC_TRIGGER_GUARD_TICKS + 2u) return -2;
    uint64_t before = hpm_csr_get_core_cycle();
    /* CNT_VAL为24位整数+8位小数；SDK读取函数直接返回寄存器，不会右移。
     * reload/point/guard均为普通PWM时钟计数，边沿判断前必须统一单位。
     * 对应HPM6E00用户手册42.4.3及SDK pwmv2_set_shadow_val的value<<8。 */
    uint32_t count_raw = pwmv2_get_counter_value(BOARD_PMSM0PWM, pwm_counter_0);
    uint32_t count = count_raw >> 8;
    uint32_t point = reload - 1u;
    if (count < 2u || count >= point - MOTOR_ADC_TRIGGER_GUARD_TICKS) return 1;
    /* 用原始精度算剩余时间，避免丢掉低8位后把边沿时刻估晚。 */
    *trigger_cycles = before + (uint64_t)((point << 8) - count_raw) * hpm_core_clock / ((uint64_t)hz << 8);
    *period_cycles = (uint64_t)reload * hpm_core_clock / hz;
    return 0;
}
