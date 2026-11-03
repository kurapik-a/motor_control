/* 用真实开环测试状态机验证启动门禁、旋转波形、超时与停机；仅替换 ADC/PWM 硬件访问。 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include "adc.h"
#include "pwm.h"
#include "pwm_test.h"
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"FAIL line %d: %s\n", __LINE__, #x); exit(1);} } while(0)
uint32_t hpm_core_clock = 1000000;
static uint64_t cycles;
static unsigned irq_depth, commits;
static bool enabled;
static pwm_current_loop_callback_t callback;
static adc_data_t input;
uint64_t hpm_csr_get_core_cycle(void) {return cycles;}
uint32_t disable_global_irq(uint32_t mask) {(void)mask; ++irq_depth; return 0;}
void restore_global_irq(uint32_t state) {(void)state; CHECK(irq_depth); --irq_depth;}
int adc_read(adc_data_t *data) {*data=input; return 0;}
uint32_t pwm_get_reload(void) {return 10000;}
bool pwm_is_enabled(void) {return enabled;}
int pwm_set_duty(float u,float v,float w) {
 CHECK(u>=0.48f && u<=0.52f && v>=0.48f && v<=0.52f && w>=0.48f && w<=0.52f);
 CHECK(fabs(u+v+w-1.5f)<0.00001f); ++commits; return 0;
}
int pwm_set_counts(uint32_t u,uint32_t v,uint32_t w) {
 CHECK(u<=10000 && v<=10000 && w<=10000);
 /* SVPWM允许共模偏移；验证线电压与最大调制幅值，而非三相和。 */
 CHECK(abs((int)u-(int)v)<=350 && abs((int)v-(int)w)<=350 && abs((int)w-(int)u)<=350);
 ++commits; return 0;
}
int pwm_enable(void) {enabled=true; return 0;}
void pwm_disable(void) {enabled=false;}
int pwm_set_current_loop_callback(pwm_current_loop_callback_t cb) {callback=cb; if(!cb) enabled=false; return 0;}
/* 旧主机运行库可能没有 C99 min/max，替换为等价的有限值实现。 */
#define fminf(a,b) ((a)<(b)?(a):(b))
#define fmaxf(a,b) ((a)>(b)?(a):(b))
#define fmodf(x,y) ((float)fmod((double)(x),(double)(y)))
#include "../motor/src/pmsm_control.c"
#include "pwm_test.c"
static void fresh(void) {++input.sample_count; input.capture_cycles=cycles;}
static void tick(unsigned count, bool service) {
 unsigned i;
 for(i=0;i<count;++i) {
  cycles+=50;
  if(callback) callback();
  if(service && i%20==19) {fresh(); pwm_test_poll();}
 }
 CHECK(!irq_depth);
}
int main(void) {
 union {uint32_t bits;float value;} nan_value;
 nan_value.bits=0x7fc00000u;
 memset(&input,0,sizeof input);
 CHECK(pwm_test_start(1,.01f,1500)==PWM_TEST_ADC_NOT_READY && !enabled);
 input.sample_valid=input.current_valid=true; input.offset_state=ADC_OFFSET_READY;
 fresh();
 CHECK(pwm_test_start(0,.01f,1500)==PWM_TEST_BAD_ARGUMENT && !enabled);
 CHECK(pwm_test_start(1,.03f,1500)==PWM_TEST_BAD_ARGUMENT && !enabled);
 CHECK(pwm_test_start(nan_value.value,.01f,1500)==PWM_TEST_BAD_ARGUMENT && !enabled);
 CHECK(pwm_test_start(1,.01f,1500)==0 && enabled);
 CHECK(pwm_test_start(1,.01f,1500)==PWM_TEST_BUSY && enabled);
 tick(12000,true);
 CHECK(pwm_test_monitor.phase_deg>0 && pwm_test_monitor.phase_deg<180);
 tick(18000,true);
 CHECK(!enabled && pwm_test_monitor.result==PWM_TEST_FINISHED && commits>0);
 fresh(); CHECK(pwm_test_start(-1,.01f,1500)==0);
 tick(12000,true);
 CHECK(pwm_test_monitor.phase_deg>180 && pwm_test_monitor.phase_deg<360);
 pwm_test_request=3; pwm_test_poll();
 CHECK(!enabled && pwm_test_monitor.result==PWM_TEST_STOPPED);
 fresh(); CHECK(pwm_test_start(1,.01f,1500)==0);
 tick(201,false);
 CHECK(!enabled && pwm_test_monitor.result==PWM_TEST_ADC_STALE);
 pwm_test_poll(); CHECK(!callback);
 fresh(); CHECK(pwm_test_start(1,.01f,1500)==0);
 input.current_u_ma=MOTOR_PWM_TEST_CURRENT_LIMIT_MA+1; input.current_v_ma=-200; input.current_w_ma=199-MOTOR_PWM_TEST_CURRENT_LIMIT_MA;
 input.raw_u=31000; input.raw_v=33000; input.offset_u=32700; input.offset_v=32710;
 fresh(); pwm_test_poll(); CHECK(!enabled && pwm_test_monitor.result==PWM_TEST_OVERCURRENT);
 CHECK(pwm_test_monitor.peak_current_ma==MOTOR_PWM_TEST_CURRENT_LIMIT_MA+1);
 CHECK(pwm_test_monitor.trip.valid && pwm_test_monitor.trip.during_run);
 CHECK(pwm_test_monitor.trip.source_mask==1 && pwm_test_monitor.trip.limit_ma==MOTOR_PWM_TEST_CURRENT_LIMIT_MA);
 CHECK(pwm_test_monitor.trip.current_u_ma==MOTOR_PWM_TEST_CURRENT_LIMIT_MA+1 && pwm_test_monitor.trip.current_w_ma==199-MOTOR_PWM_TEST_CURRENT_LIMIT_MA);
 CHECK(pwm_test_monitor.trip.raw_u==31000 && pwm_test_monitor.trip.offset_u==32700);
 CHECK(pwm_test_monitor.trip.sample_count==input.sample_count);
 pwm_test_stop(); CHECK(pwm_test_monitor.trip.valid);
 input.current_u_ma=0; input.current_v_ma=input.current_w_ma=0;
 fresh(); CHECK(pwm_test_start(1,.01f,1500)==0);
 CHECK(!pwm_test_monitor.trip.valid);
 enabled=false; pwm_test_poll(); CHECK(pwm_test_monitor.result==PWM_TEST_DRIVER_ERROR);
 fresh(); input.current_u_ma=nan_value.value;
 CHECK(pwm_test_start(1,.01f,1500)==PWM_TEST_OVERCURRENT && !enabled);
 CHECK(pwm_test_monitor.trip.valid && !pwm_test_monitor.trip.during_run);
 CHECK(pwm_test_monitor.trip.source_mask==9);
 input.current_u_ma=MOTOR_PWM_TEST_CURRENT_LIMIT_MA+1; input.current_w_ma=-(MOTOR_PWM_TEST_CURRENT_LIMIT_MA+1); fresh();
 CHECK(pwm_test_start(1,.01f,1500)==PWM_TEST_OVERCURRENT);
 CHECK(pwm_test_monitor.trip.source_mask==5);
 input.current_u_ma=0; input.current_w_ma=0; input.current_clipped=true;
 CHECK(pwm_test_start(1,.01f,1500)==PWM_TEST_ADC_NOT_READY && !enabled);
 input.current_clipped=false;
 cycles+=11000;
 CHECK(pwm_test_start(1,.01f,1500)==PWM_TEST_ADC_STALE && !enabled);
 CHECK(!irq_depth);
 puts("PASS PWM open-loop: startup guards, 3-phase waveform, direction, duration, ADC watchdog, overcurrent, stop");
 return 0;
}
