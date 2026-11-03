static inline uint32_t pwmv2_get_counter_value(PWMV2_Type *pwm_x, pwm_counter_t counter_index)
{
    return PWMV2_CNT_VAL_VALUE_GET(pwm_x->CNT_VAL[counter_index]);
}
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