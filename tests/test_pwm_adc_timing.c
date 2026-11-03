/* 运行时自动提取实际pwm.c和SDK读取函数，验证PWM原始寄存器格式到CPU时间的换算。 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include "motor_config.h"
#define __R volatile
#define __RW volatile
#define __W volatile
#include "hpm_pwmv2_regs.h"
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);} } while (0)
typedef enum {pwm_counter_0=0} pwm_counter_t;
static PWMV2_Type mock_pwm;
#define BOARD_PMSM0PWM (&mock_pwm)
#define BOARD_STEPPER0_MOTOR_CLOCK_SOURCE 0
static bool initialized=true;
static uint32_t reload=10000;
static uint32_t pwm_hz=200000000;
uint32_t hpm_core_clock=600000000;
static uint64_t now=1000000;
static uint32_t clock_get_frequency(int source) {(void)source;return pwm_hz;}
static uint64_t hpm_csr_get_core_cycle(void) {return now;}
#include "pwm_adc_timing_under_test.inc"
int main(void)
{
    uint64_t trigger=0,period=0;
    /* 5000个时钟在CNT_VAL中为1280000，不能直接与reload=10000比较。 */
    mock_pwm.CNT_VAL[0]=5000u<<8;
    CHECK(pwmv2_get_counter_value(&mock_pwm,pwm_counter_0)==1280000u);
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==0);
    CHECK(trigger==now+14997 && period==30000);
    /* 保留低8位精度，避免向下取整计数后把触发时刻估晚。 */
    mock_pwm.CNT_VAL[0]=(5000u<<8)|128u;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==0 && trigger==now+14995);
    mock_pwm.CNT_VAL[0]=(8998u<<8)|255u;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==0);
    mock_pwm.CNT_VAL[0]=8999u<<8;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==1);
    mock_pwm.CNT_VAL[0]=9999u<<8;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==1);
    mock_pwm.CNT_VAL[0]=0;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==1);
    mock_pwm.CNT_VAL[0]=1u<<8;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==1);
    mock_pwm.CNT_VAL[0]=2u<<8;
    CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==0 && trigger==now+29991);
    CHECK(pwm_get_adc_trigger_timing(0,&period)==-3);
    initialized=false;CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==-1);initialized=true;
    pwm_hz=0;CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==-2);pwm_hz=200000000;
    hpm_core_clock=0;CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==-2);hpm_core_clock=600000000;
    reload=MOTOR_ADC_TRIGGER_GUARD_TICKS+2u;CHECK(pwm_get_adc_trigger_timing(&trigger,&period)==-2);
    puts("PASS PWM ADC timing: actual SDK raw register getter, fixed-point scaling, fractional time, edge guards, invalid config");
    return 0;
}
