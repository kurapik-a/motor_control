/* 正式保护纯模块：只依赖motor头文件，不借用adc/pwm_test实现。 */
#include "pmsm_protect.h"
#include "motor_config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
static pmsm_protect_input_t healthy(void)
{
    pmsm_protect_input_t in;
    memset(&in,0,sizeof(in));
    in.current_valid=in.encoder_valid=in.dq_valid=in.bus_valid=in.temperature_valid=true;
    in.bus_raw=(MOTOR_BUS_UNDERVOLTAGE_ADC+MOTOR_BUS_OVERVOLTAGE_ADC)/2;
    in.temperature_raw=(MOTOR_TEMP_MIN_ADC+MOTOR_TEMP_MAX_ADC)/2;
    return in;
}
int main(void)
{
    pmsm_protect_t state;
    pmsm_protect_input_t in;
    uint32_t result, first;
    union {uint32_t bits;float value;} nonfinite;
    pmsm_protect_init(&state);
    CHECK(!pmsm_protect_can_enable(&state));
    memset(&in,0,sizeof(in)); /* 上电标定/辅助采样未完成不能误锁存。 */
    result=pmsm_protect_check(&state,&in);
    CHECK(result && !state.latched_faults && !pmsm_protect_can_enable(&state));
    in=healthy();
    CHECK(pmsm_protect_check(&state,&in)==0 && pmsm_protect_can_enable(&state));
    in.iu_ma=MOTOR_PROTECT_PHASE_LIMIT_MA;
    in.iv_ma=-MOTOR_PROTECT_PHASE_LIMIT_MA;
    CHECK(pmsm_protect_check(&state,&in)==0); /* 等于门限允许，正负对称。 */
    in.iv_ma=-MOTOR_PROTECT_PHASE_LIMIT_MA-1;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_OVERCURRENT);
    CHECK(state.first_faults==PMSM_FAULT_OVERCURRENT && !pmsm_protect_can_enable(&state));
    CHECK(pmsm_protect_clear(&state,&in)==-3); /* 故障仍在不允许清除。 */
    in=healthy(); in.running=true;
    CHECK(pmsm_protect_check(&state,&in)==PMSM_FAULT_OVERCURRENT);
    CHECK(pmsm_protect_clear(&state,&in)==-2); /* 正在运行也不允许清除。 */
    in.running=false;
    CHECK(pmsm_protect_clear(&state,&in)==0 && !state.first_faults && pmsm_protect_can_enable(&state));
    in.iw_ma=MOTOR_PROTECT_PHASE_LIMIT_MA+1;
    in.bus_raw=MOTOR_BUS_OVERVOLTAGE_ADC+1;
    in.temperature_raw=MOTOR_TEMP_MAX_ADC+1;
    result=pmsm_protect_check(&state,&in);
    first=PMSM_FAULT_OVERCURRENT|PMSM_FAULT_BUS_HIGH|PMSM_FAULT_TEMP_HIGH;
    CHECK(result==first && state.first_faults==first); /* 保留同次多故障。 */
    in=healthy(); in.speed_counts_s=-MOTOR_PROTECT_SPEED_LIMIT_COUNTS_S-1;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_OVERSPEED);
    CHECK(state.first_faults==first); /* 后续故障不覆盖首次现场原因。 */
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
    /* 零电流、零速度合法；有效的零辅助码不等于“尚未采样”。 */
    in.bus_raw=MOTOR_BUS_UNDERVOLTAGE_ADC; in.temperature_raw=MOTOR_TEMP_MIN_ADC;
    CHECK(pmsm_protect_check(&state,&in)==0);
#if MOTOR_BUS_UNDERVOLTAGE_ADC > 0 && MOTOR_TEMP_MIN_ADC > 0
    in.bus_raw=MOTOR_BUS_UNDERVOLTAGE_ADC-1;
    in.temperature_raw=MOTOR_TEMP_MIN_ADC-1;
    CHECK(pmsm_protect_check(&state,&in)==(PMSM_FAULT_BUS_LOW|PMSM_FAULT_TEMP_LOW));
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
#endif
    in=healthy();
    in.current_age_us=MOTOR_PROTECT_CURRENT_MAX_AGE_US;
    in.encoder_age_ms=MOTOR_PROTECT_ENCODER_MAX_AGE_MS;
    in.bus_age_ms=in.temperature_age_ms=MOTOR_PROTECT_AUX_MAX_AGE_MS;
    CHECK(pmsm_protect_check(&state,&in)==0);
    ++in.current_age_us; ++in.encoder_age_ms; ++in.bus_age_ms; ++in.temperature_age_ms;
    result=pmsm_protect_check(&state,&in);
    CHECK((result & (PMSM_FAULT_CURRENT_DATA|PMSM_FAULT_ENCODER|PMSM_FAULT_BUS_DATA|PMSM_FAULT_TEMP_DATA))==
                    (PMSM_FAULT_CURRENT_DATA|PMSM_FAULT_ENCODER|PMSM_FAULT_BUS_DATA|PMSM_FAULT_TEMP_DATA));
    CHECK(!state.latched_faults && !pmsm_protect_can_enable(&state));
    in.running=true;
    CHECK(pmsm_protect_check(&state,&in)==state.latched_faults);
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
    nonfinite.bits=0x7fc00000u; in.iu_ma=nonfinite.value; in.running=true;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_CURRENT_DATA);
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
    nonfinite.bits=0x7f800000u; in.speed_counts_s=nonfinite.value; in.running=true;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_ENCODER);
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
#if MOTOR_PROTECT_DQ_ENABLE
    in.iq_ma=-MOTOR_PROTECT_DQ_LIMIT_MA-1;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_OVERCURRENT);
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
    in.dq_valid=false;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_DQ_DATA);
    CHECK(!state.latched_faults);
#else
    in.dq_valid=false; in.id_ma=nonfinite.value;
    CHECK(pmsm_protect_check(&state,&in)==0); /* 尚未接入dq不影响基础保护。 */
#endif
    in=healthy(); CHECK(pmsm_protect_clear(&state,&in)==0);
#if MOTOR_PROTECT_SPEED_ERROR_ENABLE
    in.target_speed_counts_s=MOTOR_NORMAL_SPEED_ERROR_LIMIT_COUNTS_S+1;
    CHECK(pmsm_protect_check(&state,&in)==0); /* 非速度模式不检查跟踪误差。 */
    in.speed_loop_active=true;
    CHECK(pmsm_protect_check(&state,&in)&PMSM_FAULT_SPEED_ERROR);
#else
    in.speed_loop_active=true; in.target_speed_counts_s=nonfinite.value;
    CHECK(pmsm_protect_check(&state,&in)==0);
#endif
    CHECK(pmsm_protect_check(0,&in)==PMSM_FAULT_INPUT);
    CHECK(!pmsm_protect_can_enable(0));
    CHECK(pmsm_protect_clear(0,&in)==-1 && pmsm_protect_clear(&state,0)==-1);
    CHECK(pmsm_protect_check(&state,0)&PMSM_FAULT_INPUT);
    CHECK(!pmsm_protect_can_enable(&state));
    puts("PASS formal protection: signed limits, startup gating, data age/validity, NaN/Inf, multi-fault latch, explicit clear, optional dq/speed error");
    return 0;
}
