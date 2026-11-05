/* 实际控制器/FOC/PI/保护/VOFA协议集成；仅替换ADC、QEI、PWM、UART及时间。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "pmsm_current_ctrl.h"
#include "pmsm_init.h"
#include "pmsm_speed_ctrl.h"
#include "pmsm_position_ctrl.h"
#include "pmsm_trajectory.h"
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
#define TEST_COUNTS_S_PER_OUTPUT_RPM (MOTOR_ABZ_COUNTS_PER_REV*MOTOR_SPEED_REDUCTION_RATIO/60.0f)
#define TEST_SPEED_OUTPUT_RPM (pmsm_speed_monitor.feedback_counts_s/TEST_COUNTS_S_PER_OUTPUT_RPM)
uint32_t hpm_core_clock=1000000;
static uint64_t cycles;
static uint32_t cycle_step, phase=3999, pwm_commits;
static int adc_status, encoder_status;
static bool enabled;
static unsigned irq_depth;
static unsigned noise_period;
static uint32_t generation, encoder_resets;
static unsigned init_order;
static int reset_result;
static bool inhibit_finish;
static bool mechanical;
static float plant_omega, plant_position, load_torque;
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
int pwm_init(void) {CHECK(init_order==0);++init_order;enabled=false;return 0;}
int encoder_init(void) {CHECK(init_order==1);++init_order;++generation;return 0;}
int adc_init(void) {CHECK(init_order==2);++init_order;return 0;}
uint32_t encoder_get_generation(void) {return generation;}
int encoder_reset(void) {CHECK(!enabled);if(encoder_status || reset_result)return -1;phase=0;++generation;++encoder_resets;return 0;}
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
int encoder_read_position(int32_t *value) {*value=mechanical ? (int32_t)floor(plant_position) : (int32_t)phase*MOTOR_ENCODER_DIRECTION;return encoder_status;}
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
        float theta=plant_position*0.00157079632679f*MOTOR_POLE_PAIRS;
        if(mechanical) {
            float ealpha=-0.0333333333f*plant_omega*sinf(theta);
            float ebeta=0.0333333333f*plant_omega*cosf(theta);
            if(!enabled)vu=vv=0;
            vu-=ealpha;vv-=(-ealpha+1.73205080757f*ebeta)*0.5f;
        }
        /* 固定dq坐标下叠加beta轴反向电压，验证反电动势占用电压裕量。
         * 这是电气扰动模型，不代表机械转速/真实转子对齐仿真。 */
        vv-=0.86602540378f*opposing_beta_v;
        plant_u+=(vu-PLANT_RESISTANCE_OHM*plant_u)*0.00005f/PLANT_INDUCTANCE_H;
        plant_v+=(vv-PLANT_RESISTANCE_OHM*plant_v)*0.00005f/PLANT_INDUCTANCE_H;
        if(mechanical) {
            float beta=(plant_u+2*plant_v)*0.57735026919f;
            float iq=-sinf(theta)*plant_u+cosf(theta)*beta;
            int32_t raw_phase;
            /* PMSM电气RL+反电动势+转矩/惯量/摩擦模型；编码器来自积分位置。 */
            plant_omega+=(0.05f*iq-0.0005f*plant_omega-load_torque)*0.00005f/0.003f;
            plant_position+=plant_omega*636.61977237f*0.00005f;
            raw_phase=(int32_t)floor(plant_position*MOTOR_ENCODER_DIRECTION);
            phase=(raw_phase%(int32_t)MOTOR_ABZ_COUNTS_PER_REV+MOTOR_ABZ_COUNTS_PER_REV)%MOTOR_ABZ_COUNTS_PER_REV;
        }
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
    if(inhibit_finish && pmsm_current_monitor.state==PMSM_CURRENT_ALIGN_COMPLETE)sample.current_valid=false;
    CHECK(adc_callback);adc_callback(&sample,adc_status);
    pmsm_init_poll();
    if(enabled && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING) {
        CHECK(fabsf(pmsm_current_monitor.ud)+fabsf(pmsm_current_monitor.uq)<=MOTOR_CURRENT_ALIGN_VOLTAGE_LIMIT_TICKS+0.01f);
    }
    if(enabled && pmsm_current_monitor.state==PMSM_CURRENT_RUNNING && pmsm_speed_monitor.speed_mode) {
        CHECK(fabsf(pmsm_current_monitor.iq_ref_ma)<=MOTOR_SPEED_CTRL_IQ_LIMIT_MA);
    }
    if(enabled && pmsm_position_monitor.position_mode) {
        CHECK(fabsf(pmsm_position_monitor.speed_ref_counts_s)<=MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK*MOTOR_SPEED_LOOP_HZ);
    }
    CHECK(!irq_depth);
}
static void prepare_to_ready(void) {
    unsigned i;
    CHECK(pmsm_prepare()==0);
    for(i=0;i<110000 && !pmsm_is_ready() && pmsm_current_monitor.state!=PMSM_CURRENT_FAULT;++i)frame(true);
    CHECK(pmsm_is_ready() && !enabled && pmsm_current_monitor.aligned);
}
static void restart_alignment(void) {
    CHECK(!enabled && encoder_reset()==0);frame(false);frame(false);
    CHECK(!pmsm_current_monitor.aligned && pmsm_prepare()==0);
    frame(true);CHECK(enabled && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING);
}
static void zero_and_clear(void) {
    cycle_step=0;encoder_status=adc_status=0;
    plant_u=plant_v=opposing_beta_v=0;
    sample.current_u_ma=sample.current_v_ma=sample.current_w_ma=0;fresh();
    frame(false);frame(false); /* ADC/编码器恢复后提供新的速度观测，不能复用故障前缓存。 */
    CHECK(command("CLEAR_FAULT\r")==0 && !enabled);
    pmsm_init_poll();
    if(!pmsm_current_monitor.aligned)prepare_to_ready();
}
int main(void) {
    unsigned i, count;
    CHECK(motor_vofa_init()==0 && MOTOR_VOFA_CHANNEL_COUNT==12);
    CHECK(pmsm_init()==0 && init_order==3 && adc_callback && !enabled);
    CHECK(pmsm_init()==-2); /* 不允许重复初始化清除故障或重新对齐。 */
    pmsm_init_poll();CHECK(pmsm_init_monitor.state==PMSM_INIT_WAIT_SAMPLES && !enabled);
    CHECK(command("ENABLE,1\r")==-3 && !enabled);
    CHECK(command("SET,SPEED_RPM,10\r")==-3);
    CHECK(command("SET,POSITION_DEG,90\r")==-3);
    CHECK(command("SET,POSITION_DEG,360001\r")==-1);
    CHECK(command("SET,SPEED_RPM,101\r")==-1);
    CHECK(command("STOP\r")==0);fresh();frame(false);frame(false);
    CHECK(pmsm_init_monitor.state==PMSM_INIT_CANCELLED && !enabled);
    for(i=0;i<10;++i)frame(false);CHECK(!enabled); /* STOP不能延后自动重启准备。 */
    CHECK(pmsm_prepare()==0);frame(false);CHECK(enabled);
    CHECK(command("SET,CURRENT_MA,1500\r")==-1);
    CHECK(command("SET,CURRENT_MA,500\r")==-3);
    CHECK(command("ENABLE,1\r")==-2);
    CHECK(command("STOP\r")==0 && !enabled); /* 升流中取消后不会自动重新对齐。 */
    for(i=0;i<20;++i)frame(true);CHECK(!enabled);
    CHECK(pmsm_prepare()==0);frame(true);CHECK(enabled);
    noise_period=1000; /* 50ms一次，约0.4%的滤波采样越界。 */
    /* 量化边界每20ms跳1个编码器计数：转子基本静止，不能反复清空200ms稳定计时。 */
    for(i=0;i<40000 && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING;++i) {
        if(i%400==0)phase=phase==3999 ? 0 : 3999;
        frame(true);
    }
    CHECK(pmsm_current_monitor.state==PMSM_CURRENT_ALIGN_RELEASING && enabled);
    CHECK(pmsm_current_monitor.id_ref_ma==MOTOR_CURRENT_ALIGN_CURRENT_MA);
    /* STOP在降流阶段也必须取消准备，不留下可运行零点。 */
    CHECK(command("STOP\r")==0 && !enabled && !pmsm_current_monitor.aligned);
    CHECK(command("ENABLE,1\r")==-3 && !enabled);
    prepare_to_ready();
    CHECK(encoder_resets==1 && phase==0 && pmsm_current_monitor.speed_counts_s==0);
    CHECK(pmsm_current_monitor.offset_deg==0 && pmsm_current_monitor.iq_ref_ma==0);
    CHECK(pmsm_current_monitor.alignment_window_bad_ticks>0);
    CHECK(pmsm_current_monitor.alignment_window_good_percent>=MOTOR_CURRENT_ALIGN_WINDOW_GOOD_PERCENT);
    CHECK(pmsm_current_monitor.alignment_final_good_ticks>=200);
    CHECK(pmsm_current_monitor.alignment_last_bad_reason&PMSM_ALIGN_IQ_ERROR);
    noise_period=0;
    {
        pmsm_trajectory_config_t config, readback;
        CHECK(pmsm_trajectory_get_config(&config)==0);
        config.max_accel_counts_s2*=0.5f;
        CHECK(pmsm_trajectory_set_config(&config)==0 && pmsm_is_ready());
        CHECK(command("ENABLE,1\r")==0);
        CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_BUSY); /* 电流模式也拒绝修改。 */
        CHECK(command("STOP\r")==0);
        CHECK(pmsm_trajectory_get_config(&readback)==0 && readback.max_accel_counts_s2==config.max_accel_counts_s2);
        config.max_accel_counts_s2=MOTOR_TRAJECTORY_ACCEL_COUNTS_S2;
        CHECK(pmsm_trajectory_set_config(&config)==0);
    }
    CHECK(pmsm_current_monitor.alignment_best_ticks>=4000 && pmsm_current_monitor.alignment_motion_resets==0);
    CHECK(pmsm_current_monitor.offset_deg>=0 && pmsm_current_monitor.offset_deg<360);
    CHECK(command("SET,CURRENT_MA,500\r")==0);
    CHECK(command("ENABLE,1\r")==0 && enabled && pmsm_current_monitor.state==PMSM_CURRENT_RUNNING);
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
    CHECK(encoder_resets==1 && pmsm_current_monitor.state==PMSM_CURRENT_RUNNING);
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
    zero_and_clear();restart_alignment();
    /* 电流正常但位置持续漂移时，仍须超时关断，不能靠放宽速度阈值绕过对齐。 */
    for(i=0;i<65000 && enabled;++i) {
        if(i%20==0)phase=(phase+1)%MOTOR_ABZ_COUNTS_PER_REV;
        frame(true);
    }
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_POSITION_FAULT));
    CHECK(pmsm_current_monitor.alignment_motion_resets>0);
    zero_and_clear();restart_alignment();
    noise_period=100; /* 约4%越界，超过99%合格比例，不能完成对齐。 */
    for(i=0;i<65000 && enabled;++i)frame(true);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_CURRENT_FAULT));
    noise_period=0;
    zero_and_clear();restart_alignment();
    sample.current_u_ma=sample.current_v_ma=sample.current_w_ma=0;
    for(i=0;i<65000 && enabled;++i)frame(false);
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_FAULT));
    CHECK(pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_CURRENT_FAULT);
    zero_and_clear();
    /* 普通停机后手动转轴，只要QEI持续计数，零点仍有效，不清零也不重对齐。 */
    count=encoder_resets;phase=123;for(i=0;i<80;++i)frame(false);
    CHECK(pmsm_is_ready() && command("ENABLE,1\r")==0);
    CHECK(pmsm_current_monitor.state==PMSM_CURRENT_RUNNING && encoder_resets==count);
    CHECK(command("STOP\r")==0);
    CHECK(encoder_reset()==0);frame(false);frame(false);
    CHECK(!pmsm_is_ready() && !pmsm_current_monitor.aligned);
    CHECK(command("ENABLE,1\r")==-3 && !enabled);
    for(i=0;i<100;++i)frame(false);CHECK(!enabled); /* 零点失效不自动给电机通电。 */
    prepare_to_ready();
    CHECK(command("ENABLE,1\r")==0);
    ++generation;phase=200;frame(false); /* 运行中发生外部QEI重置。 */
    CHECK(!enabled && !pmsm_current_monitor.aligned && (pmsm_current_monitor.faults&PMSM_FAULT_ENCODER));
    zero_and_clear();restart_alignment();
    for(i=0;i<65000 && pmsm_current_monitor.state==PMSM_CURRENT_ALIGNING;++i)frame(true);
    CHECK(pmsm_current_monitor.state==PMSM_CURRENT_ALIGN_RELEASING);
    sample.current_u_ma=1000;sample.current_v_ma=-500;sample.current_w_ma=-500;
    for(i=0;i<45000 && enabled;++i)frame(false); /* 实际电流不回零，降流超时。 */
    CHECK(!enabled && (pmsm_current_monitor.faults&PMSM_CURRENT_ALIGN_CURRENT_FAULT));
    zero_and_clear();restart_alignment();reset_result=-1;
    for(i=0;i<110000 && pmsm_current_monitor.state!=PMSM_CURRENT_FAULT;++i)frame(true);
    CHECK(!enabled && !pmsm_current_monitor.aligned && (pmsm_current_monitor.faults&PMSM_FAULT_ENCODER));
    reset_result=0;zero_and_clear();restart_alignment();inhibit_finish=true;
    for(i=0;i<110000 && pmsm_current_monitor.state!=PMSM_CURRENT_ALIGN_COMPLETE;++i)frame(true);
    CHECK(!enabled && pmsm_current_monitor.state==PMSM_CURRENT_ALIGN_COMPLETE);
    for(i=0;i<20;++i)frame(true);
    CHECK(!pmsm_is_ready() && !pmsm_current_monitor.aligned && !pmsm_current_monitor.faults);
    inhibit_finish=false;frame(true);
    CHECK(pmsm_is_ready() && !enabled);
    CHECK(command("SET,SPEED_RPM,100\r")==0 && command("ENABLE,1\r")==0);
    for(i=0;i<600000 && pmsm_trajectory_monitor.active;++i)frame(true);
    for(i=0;i<4000;++i)frame(true); /* 固定转子模型：完整加速后持续速度误差，验证PI抗饱和。 */
    CHECK(pmsm_speed_monitor.current_limited && pmsm_speed_monitor.iq_ref_ma==MOTOR_SPEED_CTRL_IQ_LIMIT_MA);
    CHECK(command("SET,SPEED_RPM,-100\r")==0);
    for(i=0;i<600000 && pmsm_trajectory_monitor.active;++i)frame(true);
    for(i=0;i<4000;++i)frame(true);
    CHECK(pmsm_speed_monitor.current_limited && pmsm_speed_monitor.iq_ref_ma==-MOTOR_SPEED_CTRL_IQ_LIMIT_MA);
    CHECK(command("SET,SPEED_RPM,0\r")==0);
    for(i=0;i<600000 && pmsm_trajectory_monitor.active;++i)frame(true);
    for(i=0;i<4000;++i)frame(true);
    CHECK(pmsm_speed_monitor.reference_counts_s==0 && !pmsm_trajectory_monitor.active);
    CHECK(fabsf(pmsm_speed_monitor.iq_ref_ma)<=MOTOR_SPEED_CTRL_IQ_LIMIT_MA);
    CHECK(command("STOP\r")==0);
    mechanical=true;plant_u=plant_v=plant_omega=plant_position=load_torque=0;phase=0;
    CHECK(command("SET,SPEED_RPM,10\r")==0 && pmsm_speed_monitor.speed_mode);
    CHECK(fabsf(pmsm_speed_monitor.target_counts_s-5166.666667f)<0.01f);
    CHECK(pmsm_current_ctrl_set_speed_counts_s(MOTOR_SPEED_CTRL_MAX_COUNTS_S+1)==-1);
    CHECK(command("ENABLE,1\r")==0);
    count=pmsm_speed_monitor.updates;
    for(i=0;i<160000;++i)frame(true);
    printf("Speed +10rpm: feedback=%.3f actual=%.3f Iq=%.1f mA updates=%u\n",TEST_SPEED_OUTPUT_RPM,
        plant_omega*9.54929658551f/MOTOR_SPEED_REDUCTION_RATIO,pmsm_current_monitor.iq_ma,pmsm_speed_monitor.updates-count);
    CHECK(pmsm_current_monitor.state==PMSM_CURRENT_RUNNING && !pmsm_current_monitor.faults);
    CHECK(pmsm_speed_monitor.updates-count>=7999 && pmsm_speed_monitor.updates-count<=8001);
    CHECK(fabsf(TEST_SPEED_OUTPUT_RPM-10)<0.5f);
    CHECK(fabsf(plant_omega*9.54929658551f/MOTOR_SPEED_REDUCTION_RATIO-10)<0.5f);
    load_torque=0.01f;for(i=0;i<120000;++i)frame(true);
    CHECK(fabsf(TEST_SPEED_OUTPUT_RPM-10)<0.5f);
    CHECK(fabsf(pmsm_speed_monitor.iq_ref_ma)<=MOTOR_SPEED_CTRL_IQ_LIMIT_MA);
    CHECK(command("SET,SPEED_RPM,-10\r")==0);
    for(i=0;i<200000;++i)frame(true);
    printf("Speed -10rpm with load: feedback=%.3f Iq=%.1f mA\n",TEST_SPEED_OUTPUT_RPM,pmsm_current_monitor.iq_ma);
    CHECK(fabsf(TEST_SPEED_OUTPUT_RPM+10)<0.5f && !pmsm_current_monitor.faults);
    CHECK(command("SET,CURRENT_MA,100\r")==0 && !pmsm_speed_monitor.speed_mode);
    frame(true);
    CHECK(command("SET,SPEED_RPM,0\r")==0 && pmsm_speed_monitor.speed_mode && pmsm_speed_monitor.running);
    for(i=0;i<160000;++i)frame(true);
    CHECK(fabsf(TEST_SPEED_OUTPUT_RPM)<0.5f);
    CHECK(command("STOP\r")==0 && !enabled && !pmsm_speed_monitor.running && pmsm_speed_monitor.iq_ref_ma==0);
    CHECK(pmsm_speed_monitor.target_counts_s==0);
    {
        char text[80];
        int32_t base, goal, hold;
        uint32_t position_updates, speed_updates;
        float late_peak;
        encoder_read_position(&base);goal=base+8000; /* 正向跨两圈；目标相对于对齐零点。 */
        sprintf(text,"SET,POSITION_DEG,%.3f\r",goal*360.0f/(MOTOR_ABZ_COUNTS_PER_REV*MOTOR_SPEED_REDUCTION_RATIO));
        CHECK(command(text)==0 && pmsm_position_monitor.position_mode && pmsm_speed_monitor.speed_mode);
        CHECK(fabsf(pmsm_position_monitor.target_count-goal)<0.1f);
        CHECK(command("ENABLE,1\r")==0);
        position_updates=pmsm_position_monitor.updates;speed_updates=pmsm_speed_monitor.updates;
        late_peak=0;
        for(i=0;i<400000;++i) {
            frame(true);
            if(i==19) {
                CHECK(pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.mode==PMSM_TRAJECTORY_POSITION);
                CHECK(fabsf(pmsm_position_monitor.reference_count-goal)>1000);
            }
            if(i==20000 || i==40000) {
                float before_p=pmsm_trajectory_monitor.position_count;
                float before_v=pmsm_trajectory_monitor.speed_counts_s;
                float before_a=pmsm_trajectory_monitor.acceleration_counts_s2;
                CHECK(pmsm_current_ctrl_set_position_counts(i==20000 ? goal-4000 : goal)==0);
                CHECK(pmsm_trajectory_monitor.position_count==before_p && pmsm_trajectory_monitor.speed_counts_s==before_v && pmsm_trajectory_monitor.acceleration_counts_s2==before_a);
            }
            if(i>=380000 && fabsf(pmsm_position_monitor.error_count)>late_peak)late_peak=fabsf(pmsm_position_monitor.error_count);
        }
        printf("Position +2 turns: target=%d feedback=%d error=%.2f counts\n",goal,pmsm_position_monitor.position_count,pmsm_position_monitor.error_count);
        CHECK(pmsm_position_monitor.updates-position_updates==20000 && pmsm_speed_monitor.updates-speed_updates==20000);
        CHECK(fabsf(pmsm_position_monitor.error_count)<5 && !pmsm_current_monitor.faults);
        CHECK(late_peak<5); /* 不仅检查最后一拍，还检查末段持续稳定。 */
        CHECK(!pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.position_count==goal);
        goal=-4000; /* 穿过机械零点，到负向绝对位置。 */
        sprintf(text,"SET,POSITION_DEG,%.3f\r",goal*360.0f/(MOTOR_ABZ_COUNTS_PER_REV*MOTOR_SPEED_REDUCTION_RATIO));
        CHECK(command(text)==0);
        late_peak=0;
        for(i=0;i<400000;++i) {
            frame(true);
            if(i>=380000 && fabsf(pmsm_position_monitor.error_count)>late_peak)late_peak=fabsf(pmsm_position_monitor.error_count);
        }
        printf("Position reverse: target=%d feedback=%d error=%.2f counts\n",goal,pmsm_position_monitor.position_count,pmsm_position_monitor.error_count);
        CHECK(fabsf(pmsm_position_monitor.error_count)<5 && !pmsm_current_monitor.faults);
        CHECK(late_peak<5);
        CHECK(command("STOP\r")==0 && !enabled && !pmsm_position_monitor.running && !pmsm_position_monitor.target_pending);
        plant_position+=8000;for(i=0;i<100;++i)frame(true);encoder_read_position(&hold);
        CHECK(command("ENABLE,1\r")==0);
        CHECK(fabsf(pmsm_position_monitor.target_count-hold)<1);
        for(i=0;i<40000;++i)frame(true);
        CHECK(fabsf(pmsm_position_monitor.error_count)<5);
        CHECK(command("SET,SPEED_RPM,0\r")==0 && !pmsm_position_monitor.position_mode);
        CHECK(command("SET,POSITION_DEG,0\r")==0 && pmsm_position_monitor.position_mode);
        CHECK(command("SET,CURRENT_MA,0\r")==0 && !pmsm_position_monitor.position_mode && !pmsm_speed_monitor.speed_mode);
        CHECK(command("SET,POSITION_DEG,0\r")==0 && pmsm_position_monitor.position_mode);
        CHECK(command("SET,POSITION_DEG,90\r")==0 && pmsm_position_monitor.target_count==7750);
        CHECK(command("SET,POSITION_DEG,360\r")==0 && pmsm_position_monitor.target_count==31000);
        CHECK(command("SET,POSITION_DEG,-360\r")==0 && pmsm_position_monitor.target_count==-31000);
        sprintf(text,"SET,POSITION_DEG,%.6f\r",MOTOR_POSITION_MAX_COMMAND_COUNTS*360.0f/(MOTOR_ABZ_COUNTS_PER_REV*MOTOR_SPEED_REDUCTION_RATIO));
        CHECK(command(text)==0);
        CHECK(pmsm_position_monitor.target_count==MOTOR_POSITION_MAX_COMMAND_COUNTS);
        sprintf(text,"SET,POSITION_DEG,%.6f\r",-MOTOR_POSITION_MAX_COMMAND_COUNTS*360.0f/(MOTOR_ABZ_COUNTS_PER_REV*MOTOR_SPEED_REDUCTION_RATIO));
        CHECK(command(text)==0);
        CHECK(pmsm_position_monitor.target_count==-MOTOR_POSITION_MAX_COMMAND_COUNTS);
        CHECK(pmsm_current_ctrl_set_position_counts(MOTOR_POSITION_MAX_COMMAND_COUNTS+1)==-1);
        CHECK(command("STOP\r")==0);
    }
    {
        pmsm_trajectory_cycle_config_t cycle;
        int32_t origin;
        cycle.mode=PMSM_TRAJECTORY_ROUND_TRIP;cycle.cycle_count=1;cycle.dwell_ms=100;
        CHECK(pmsm_trajectory_set_cycle_config(&cycle)==0);
        CHECK(encoder_read_position(&origin)==0);
        CHECK(pmsm_current_ctrl_set_position_counts(origin+500)==0 && command("ENABLE,1\r")==0);
        CHECK(pmsm_trajectory_set_cycle_config(&cycle)==PMSM_TRAJECTORY_BUSY);
        for(i=0;i<600000 && pmsm_trajectory_monitor.active;++i)frame(true);
        CHECK(!pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.cycles_completed==1);
        CHECK(pmsm_trajectory_monitor.position_count==origin && pmsm_position_monitor.target_count==origin);
        for(i=0;i<200000;++i)frame(true);
        CHECK(fabsf(pmsm_position_monitor.position_count-origin)<5 && !pmsm_current_monitor.faults);
        CHECK(command("STOP\r")==0);
        cycle.cycle_count=0; CHECK(pmsm_trajectory_set_cycle_config(&cycle)==0);
        CHECK(pmsm_current_ctrl_set_position_counts(origin+500)==0 && command("ENABLE,1\r")==0);
        for(i=0;i<20000;++i)frame(true);
        CHECK(pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.cycling);
        CHECK(command("STOP\r")==0 && !pmsm_trajectory_monitor.cycling && !pmsm_trajectory_monitor.active);
        CHECK(command("ENABLE,1\r")==0);
        for(i=0;i<4000;++i)frame(true);
        CHECK(!pmsm_trajectory_monitor.cycling); /* 未下新目标不得恢复旧往返。 */
        CHECK(pmsm_current_ctrl_set_position_counts(origin+500)==0);
        frame(true);CHECK(pmsm_trajectory_monitor.cycling);
    }
    cycles+=101;CHECK(watchdog);watchdog();
    CHECK(!enabled && !pmsm_speed_monitor.running && pmsm_speed_monitor.iq_ref_ma==0 && pmsm_speed_monitor.target_counts_s==0);
    CHECK(pmsm_current_monitor.faults&PMSM_CURRENT_TIMING_FAULT);
    CHECK(command("SET,SPEED_RPM,10\r")==-4);
    CHECK(!pmsm_position_monitor.running && pmsm_position_monitor.speed_ref_counts_s==0 && !pmsm_position_monitor.target_pending);
    CHECK(pmsm_trajectory_monitor.mode==PMSM_TRAJECTORY_OFF && !pmsm_trajectory_monitor.active);
    CHECK(command("SET,POSITION_DEG,0\r")==-4);
    {
        encoder_data_t report;
        float uploaded[12];
        memset(&report,0,sizeof(report));
        report.speed_counts_per_s=5166.666667f;
        pmsm_position_ctrl_observe(31000); /* 用已知累计计数验证上传输出轴角度。 */
        CHECK(motor_vofa_set_encoder(&report)==0);
        cycles+=10000;motor_vofa_poll();motor_vofa_poll();
        CHECK(tx_size>=52);
        memcpy(uploaded,tx+tx_size-52,48);
        CHECK(fabsf(uploaded[6]-10)<0.001f && uploaded[8]==0);
        CHECK(fabsf(uploaded[9]-TEST_SPEED_OUTPUT_RPM)<0.001f);
        CHECK(fabsf(uploaded[10]-pmsm_position_monitor.target_count*360.0f/(MOTOR_ABZ_COUNTS_PER_REV*MOTOR_SPEED_REDUCTION_RATIO))<0.01f);
        CHECK(fabsf(uploaded[11]-360)<0.001f);
    }
    CHECK(tx_size && tx_size%52==0);
    for(i=0;i<tx_size;i+=52)CHECK(tx[i+48]==0 && tx[i+49]==0 && tx[i+50]==0x80 && tx[i+51]==0x7f);
    CHECK(pwm_commits>30000);
    puts("PASS preparation/current/speed/position: startup, signed Iq, 1kHz cascade, position multi-turn/reverse/hold, mode switches, STOP/faults, 12-channel JustFloat");
    return 0;
}
