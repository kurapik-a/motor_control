/* 实际控制器/FOC/PI/保护/VOFA协议集成；仅替换ADC、QEI、PWM、UART及时间。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pmsm_current_ctrl.h"
#include "motor_config.h"
#include "motor_vofa.h"
#include "vofa_hpm_uart.h"
#include "adc.h"
#include "encoder.h"
#include "pwm.h"
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"FAIL %d: %s (state=%d fault=%u)\n",__LINE__,#x,pmsm_current_monitor.state,pmsm_current_monitor.faults);exit(1);} } while(0)
/* 离线电气模型参数，不参与固件在线控制，也不占用motor_config配置项。 */
#define PLANT_RESISTANCE_OHM 0.225f
#define PLANT_INDUCTANCE_H 0.00026f
#define PLANT_BUS_V 24.0f
uint32_t hpm_core_clock=1000000;
static uint64_t cycles;
static uint32_t cycle_step, phase=3999, pwm_commits;
static int adc_status, encoder_status;
static bool enabled;
static unsigned irq_depth;
static unsigned noise_period;
static float plant_u, plant_v, opposing_beta_v;
static uint32_t plant_pwm_u=5000, plant_pwm_v=5000, plant_pwm_w=5000;
static adc_data_t sample;
static adc_control_callback_t adc_callback;
static pwm_current_loop_callback_t watchdog;
static const char *rx;
static uint8_t tx[16384];
static size_t tx_size;
extern vofa_debug_t motor_vofa_context;
uint64_t hpm_csr_get_core_cycle(void) {cycles+=cycle_step;return cycles;}
uint32_t disable_global_irq(uint32_t mask) {(void)mask;++irq_depth;return 0;}
void restore_global_irq(uint32_t value) {(void)value;CHECK(irq_depth);--irq_depth;}
bool pwm_is_enabled(void) {return enabled;}
uint32_t pwm_get_reload(void) {return 10000;}
int pwm_set_counts(uint32_t u,uint32_t v,uint32_t w) {
    CHECK(u<=10000 && v<=10000 && w<=10000);
    plant_pwm_u=u;plant_pwm_v=v;plant_pwm_w=w;++pwm_commits;return 0;
}
int pwm_enable(void) {enabled=true;return 0;}
void pwm_disable(void) {enabled=false;}
int pwm_set_current_loop_callback(pwm_current_loop_callback_t callback) {watchdog=callback;if(!callback)enabled=false;return 0;}
int adc_read(adc_data_t *data) {*data=sample;return adc_status;}
int adc_set_control_callback(adc_control_callback_t callback) {CHECK(!enabled);adc_callback=callback;return 0;}
int encoder_read_phase(uint32_t *value) {*value=phase;return encoder_status;}
void board_init_uart(UART_Type *ptr) {(void)ptr;}
uint32_t clock_get_frequency(int name) {(void)name;return 24000000;}
int vofa_hpm_uart_init(vofa_hpm_uart_t *port) {return port ? 0 : -1;}
int vofa_hpm_uart_read(void *user,uint8_t *data,size_t size) {
    size_t count=0;(void)user;
    while(rx && *rx && count<size)data[count++]=(uint8_t)*rx++;
    return (int)count;
}
int vofa_hpm_uart_write(void *user,const uint8_t *data,size_t size) {
    (void)user;CHECK(tx_size+size<=sizeof(tx));memcpy(tx+tx_size,data,size);tx_size+=size;return (int)size;
}
void vofa_hpm_uart_bind(vofa_debug_config_t *config,vofa_hpm_uart_t *port) {
    config->read=vofa_hpm_uart_read;config->write=vofa_hpm_uart_write;config->io_user=port;
}
int encoder_to_output_axis(const encoder_data_t *motor,encoder_output_data_t *output) {
    output->angle_deg=motor->mechanical_angle_deg/MOTOR_SPEED_REDUCTION_RATIO;
    output->speed_rpm=motor->speed_rpm/MOTOR_SPEED_REDUCTION_RATIO;
    output->speed_filtered_rpm=motor->speed_filtered_rpm/MOTOR_SPEED_REDUCTION_RATIO;return 0;
}
static int command(const char *text) {rx=text;motor_vofa_poll();return motor_vofa_context.stats.last_command_result;}
static void fresh(void) {
    sample.sample_valid=sample.current_valid=sample.bus_valid=sample.temperature_valid=true;
    sample.offset_state=ADC_OFFSET_READY;sample.current_clipped=false;
    sample.capture_cycles=sample.bus_cycles=sample.temperature_cycles=cycles;
    sample.bus_raw=20000;sample.temperature_raw=25000;
}
static void frame(bool follow) {
    if(follow) {
        /* 固定转子的三相RL电气模型，由实际PWM输出驱动，不把目标直接当反馈。
         * 不模拟机械对齐运动；这里只验证闭环电气反馈及状态机。 */
        float common=((float)plant_pwm_u+plant_pwm_v+plant_pwm_w)/3.0f;
        float vu=((float)plant_pwm_u-common)*PLANT_BUS_V/10000.0f;
        float vv=((float)plant_pwm_v-common)*PLANT_BUS_V/10000.0f;
        /* 固定dq坐标下叠加beta轴反向电压，验证反电动势占用电压裕量。
         * 这是电气扰动模型，不代表机械转速/真实转子对齐仿真。 */
        vv-=0.86602540378f*opposing_beta_v;
        plant_u+=(vu-PLANT_RESISTANCE_OHM*plant_u)*0.00005f/PLANT_INDUCTANCE_H;
        plant_v+=(vv-PLANT_RESISTANCE_OHM*plant_v)*0.00005f/PLANT_INDUCTANCE_H;
        sample.current_u_ma=plant_u*1000.0f;sample.current_v_ma=plant_v*1000.0f;
        sample.current_w_ma=-sample.current_u_ma-sample.current_v_ma;
    }
    if(noise_period && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING &&
        pmsm_current_monitor.alignment_ticks>=20000 && sample.sample_count%noise_period==17) {
        /* 单拍2A q轴测量尖峰：原始三相仍低于3A，但滤波Iq约4拍超过200mA。
         * 20kHz连续零越界资格会被这种低占比噪声永久打断。 */
        pmsm_trig_t trig;pmsm_dq_t disturbance;pmsm_ab_t ab;
        disturbance.d=0;disturbance.q=2000;
        CHECK(pmsm_sincos(pmsm_current_monitor.angle_deg,&trig)==0);
        CHECK(pmsm_inv_park(&disturbance,&trig,&ab)==0);
        sample.current_u_ma+=ab.alpha;
        sample.current_v_ma+=(-ab.alpha+1.73205080757f*ab.beta)*0.5f;
        sample.current_w_ma=-sample.current_u_ma-sample.current_v_ma;
    }
    cycles+=50;fresh();++sample.sample_count;
    CHECK(adc_callback);adc_callback(&sample,adc_status);
    if(enabled && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING) {
        CHECK(fabsf(pmsm_current_monitor.ud)+fabsf(pmsm_current_monitor.uq)<=MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS+0.01f);
    }
    CHECK(!irq_depth);
}
static void zero_and_clear(void) {
    cycle_step=0;encoder_status=adc_status=0;
    plant_u=plant_v=opposing_beta_v=0;
    sample.current_u_ma=sample.current_v_ma=sample.current_w_ma=0;fresh();
    frame(false);frame(false); /* ADC/编码器恢复后提供新的速度观测，不能复用故障前缓存。 */
    CHECK(command("CLEAR_FAULT\r")==0 && !enabled);
}
int main(void) {
    unsigned i, count;
    CHECK(pmsm_current_ctrl_init()==0 && adc_callback && !enabled);
    CHECK(motor_vofa_init()==0 && MOTOR_VOFA_CHANNEL_COUNT==8);
    CHECK(command("ENABLE,1\r")==-3 && !enabled);
    CHECK(pmsm_current_monitor.inhibit_faults!=0);
    fresh();frame(false);frame(false);pmsm_current_ctrl_poll();CHECK(!enabled); /* 未就绪请求不排队。 */
    CHECK(command("SET,CURRENT_MA,1500\r")==-1);
    CHECK(command("SET,CURRENT_MA,500\r")==0);
    CHECK(command("ENABLE,1\r")==0 && enabled);
    CHECK(command("ENABLE,1\r")==-2);
    noise_period=1000; /* 50ms一次，约0.4%的滤波采样越界。 */
    /* 量化边界每20ms跳1个编码器计数：转子基本静止，不能反复清空200ms稳定计时。 */
    for(i=0;i<40000 && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING;++i) {
        if(i%400==0)phase=phase==3999 ? 0 : 3999;
        frame(true);
    }
    CHECK(pmsm_current_monitor.state==PMSM_CURRENT_RUNNING && enabled);
    CHECK(pmsm_current_monitor.alignment_window_bad_ticks>0);
    CHECK(pmsm_current_monitor.alignment_window_good_percent>=MOTOR_CURRENT_ALIGN_WINDOW_GOOD_PERCENT);
    CHECK(pmsm_current_monitor.alignment_final_good_ticks>=200);
    CHECK(pmsm_current_monitor.alignment_last_bad_reason&PMSM_ALIGN_IQ_ERROR);
    noise_period=0;
    CHECK(pmsm_current_monitor.alignment_best_ticks>=4000 && pmsm_current_monitor.alignment_motion_resets==0);
    CHECK(pmsm_current_monitor.offset_deg>=0 && pmsm_current_monitor.offset_deg<360);
    count=pmsm_current_monitor.updates;
    for(i=0;i<12000;++i)frame(true);
    CHECK(pmsm_current_monitor.updates==count+12000);
    CHECK(fabsf(pmsm_current_monitor.iq_ref_ma-500)<1 && fabsf(pmsm_current_monitor.iq_ma-500)<30);
    CHECK(pmsm_current_monitor.id_ref_ma==0 && fabsf(pmsm_current_monitor.angle_deg)<0.001f);
    /* 500mA在0.625V反向电压下需要约0.7375V；旧500计数总限幅
     * 仅能输出约0.693V，使反馈停在约300mA。缓升扰动避免阶跃过流。 */
    for(i=0;i<10000;++i) {opposing_beta_v=0.625f*(i+1)/10000.0f;frame(true);}
    for(i=0;i<15000;++i)frame(true);
    printf("Opposing-voltage test: Iq_ref=%.1f mA Iq=%.1f mA Ud=%.1f Uq=%.1f ticks\n",
        pmsm_current_monitor.iq_ref_ma,pmsm_current_monitor.iq_ma,pmsm_current_monitor.ud,pmsm_current_monitor.uq);
    CHECK(pmsm_current_monitor.state==PMSM_CURRENT_RUNNING && !pmsm_current_monitor.faults);
    CHECK(fabsf(pmsm_current_monitor.iq_ma-500)<30 && fabsf(pmsm_current_monitor.id_ma)<30);
    CHECK(pmsm_current_monitor.uq>500);
    CHECK(!pmsm_current_monitor.alignment_voltage_limited && !pmsm_current_monitor.voltage_limited);
    /* 先缓慢去除扰动，再检查负电流与停机。 */
    for(i=0;i<10000;++i) {opposing_beta_v=0.625f*(9999-i)/10000.0f;frame(true);}
    CHECK(command("SET,CURRENT_MA,-500\r")==0);
    for(i=0;i<15000;++i)frame(true);
    CHECK(fabsf(pmsm_current_monitor.iq_ref_ma+500)<1);
    CHECK(fabsf(pmsm_current_monitor.iq_ma+500)<30);
    CHECK(command("STOP\r")==0 && !enabled && pmsm_current_monitor.state==PMSM_CURRENT_STOPPED);
    CHECK(pmsm_current_monitor.id_ref_ma==0 && pmsm_current_monitor.iq_ref_ma==0);
    sample.current_u_ma=sample.current_v_ma=sample.current_w_ma=0;fresh();
    CHECK(command("ENABLE,1\r")==0);frame(true);
    sample.current_u_ma=-MOTOR_PROTECT_PHASE_LIMIT_MA-1;frame(false);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_FAULT_OVERCURRENT));
    CHECK(command("ENABLE,1\r")==-4);
    CHECK(command("CLEAR_FAULT\r")==-3);
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    cycles+=50;fresh();++sample.sample_count;sample.bus_valid=false;adc_callback(&sample,0);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_FAULT_BUS_DATA));
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    cycles+=50;fresh();++sample.sample_count;sample.current_valid=false;adc_callback(&sample,0);
    CHECK(!enabled && pmsm_current_monitor.faults==(PMSM_FAULT_CURRENT_DATA|PMSM_FAULT_DQ_DATA));
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    adc_callback(&sample,0); /* 重复帧不能再算PI。 */
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_TIMING_FAULT));
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    cycles+=101;CHECK(watchdog);watchdog();
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_TIMING_FAULT));
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    encoder_status=-1;frame(false);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_FAULT_ENCODER));
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    for(i=0;i<30 && enabled;++i) {phase=(phase+10)%MOTOR_ABZ_COUNTS_PER_REV;frame(false);}
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_FAULT_OVERSPEED));
    CHECK(command("CLEAR_FAULT\r")==-3);
    for(i=0;i<80;++i)frame(false); /* 停机后速度仍更新，真实恢复后才能清除。 */
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    adc_callback(&sample,-8);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ADC_FAULT));
    frame(false);frame(false);zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    cycle_step=30;frame(false);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_TIMING_FAULT));
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    /* 电流正常但位置持续漂移时，仍须超时关断，不能靠放宽速度阈值绕过对齐。 */
    for(i=0;i<65000 && enabled;++i) {
        if(i%20==0)phase=(phase+1)%MOTOR_ABZ_COUNTS_PER_REV;
        frame(true);
    }
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_POSITION_FAULT));
    CHECK(pmsm_current_monitor.alignment_motion_resets>0);
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    noise_period=100; /* 约4%越界，超过99%合格比例，不能完成对齐。 */
    for(i=0;i<65000 && enabled;++i)frame(true);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_CURRENT_FAULT));
    noise_period=0;
    zero_and_clear();CHECK(command("ENABLE,1\r")==0);
    sample.current_u_ma=sample.current_v_ma=sample.current_w_ma=0;
    for(i=0;i<65000 && enabled;++i)frame(false);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_FAULT));
    CHECK(pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_CURRENT_FAULT);
    zero_and_clear();cycles+=10000;motor_vofa_poll();motor_vofa_poll();
    CHECK(tx_size && tx_size%36==0);
    for(i=0;i<tx_size;i+=36)CHECK(tx[i+32]==0 && tx[i+33]==0 && tx[i+34]==0x80 && tx[i+35]==0x7f);
    CHECK(pwm_commits>30000);
    puts("PASS current loop: real FOC/PI/protection/protocol, alignment, signed Iq tracking, 20kHz frame accounting, STOP, faults/clear, watchdog, budget, alignment timeout, 8-channel JustFloat");
    return 0;
}
