/* 用实际adc.c验证PWM触发握手、双路DMA发布、零偏与故障门禁；只替换硬件访问。 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "adc_test_platform.h"
#include "adc.h"
#include "pwm.h"
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"FAIL %d: %s\n",__LINE__,#x);exit(1);} } while(0)
ADC16_Type mock_adc[2];
mock_ioc_t mock_ioc;
uint32_t hpm_core_clock=1000000;
static uint64_t now;
static uint32_t cycle_step,timing_pause;
static bool gate,enabled,near_edge;
static unsigned irq_depth,reload_value=10000;
static int init_fail,aux_fail;
static unsigned reject_enable_count;
static unsigned frame_callbacks, fault_callbacks;
static void control_received(const adc_data_t *sample, int status)
{
 CHECK(sample);
 if(status) {++fault_callbacks;CHECK(!enabled && !sample->sample_valid);}
 else {++frame_callbacks;CHECK(sample->sample_valid && sample->publish_cycles>=sample->capture_cycles);}
}
static void check_arm_prepared(void);
uint32_t disable_global_irq(uint32_t x){(void)x;++irq_depth;return 0;}
void restore_global_irq(uint32_t x){(void)x;CHECK(irq_depth);--irq_depth;}
uint64_t hpm_csr_get_core_cycle(void){now+=cycle_step;return now;}
void intc_m_disable_irq(int x){(void)x;}
void intc_m_enable_irq_with_priority(int x,int p){(void)x;CHECK(p==5);}
void clock_add_to_group(int x,int g){(void)x;(void)g;}
uint32_t board_init_adc_clock(ADC16_Type *p,bool x){(void)p;(void)x;return 200000000;}
void board_init_adc16_pins(void){}
void adc16_get_channel_default_config(adc16_channel_config_t *p){memset(p,0,sizeof *p);}
int adc16_init_channel(ADC16_Type *p,adc16_channel_config_t *c){p->sample_cycles[c->ch]=c->sample_cycle;return 0;}
void adc16_get_default_config(adc16_config_t *p){memset(p,0,sizeof *p);}
int adc16_init(ADC16_Type *p,adc16_config_t *c){CHECK(c->res==21 && c->conv_mode==3 && !c->sel_sync_ahb && c->adc_ahb_en);CHECK(p->period_count[15]==0 && p->period_count[1]==0);return init_fail;}
void adc16_disable_interrupts(ADC16_Type *p,uint32_t m){p->interrupts &= ~m;}
void adc16_enable_interrupts(ADC16_Type *p,uint32_t m){p->interrupts |= m;}
int adc16_set_pmt_config(ADC16_Type *p,adc16_pmt_config_t *c){CHECK(c->trig_len==1 && c->adc_ch[0]==(p->id?10:14));CHECK(c->inten[0] && !c->inten[1] && !c->inten[2]);return 0;}
int adc16_set_pmt_queue_enable(ADC16_Type *p,unsigned c,bool e){(void)p;CHECK(!c&&e);return 0;}
void adc16_clear_status_flags(ADC16_Type *p,uint32_t m){p->flags &= ~m;}
uint32_t adc16_get_status_flags(ADC16_Type *p){return p->flags;}
void adc16_init_pmt_dma(ADC16_Type *p,uint32_t a){(void)p;(void)a;}
int adc16_set_prd_config(ADC16_Type *p,adc16_prd_config_t *c){CHECK(p==&mock_adc[0]);CHECK(c->ch==15 || c->ch==1);CHECK(c->prescale==22);if(aux_fail && c->period_count)return -1;p->period_count[c->ch]=c->period_count;p->prescale[c->ch]=c->prescale;return 0;}
int adc16_get_prd_result(ADC16_Type *p,uint8_t c,uint16_t *v){CHECK(!irq_depth);CHECK(p==&mock_adc[0] && (c==15 || c==1));*v=p->period_results[c];return 0;}
bool pwm_is_enabled(void){return enabled;}
void pwm_disable(void){enabled=false;}
uint32_t pwm_get_reload(void){return reload_value;}
int pwm_set_adc_trigger_enabled(bool e){if(e && reject_enable_count){--reject_enable_count;return 1;}gate=e;return 0;}
int pwm_get_adc_trigger_timing(uint64_t *t,uint64_t *p){check_arm_prepared();if(near_edge)return 1;*t=now+48;*p=50;now+=timing_pause;return 0;}
#include "adc.c"
/* 清理先完成再读取PWM时刻，保护有效采样窗口。 */
static void check_arm_prepared(void)
{
 CHECK(!gate && !dma_u[ADC_DMA_INDEX] && !dma_v[ADC_DMA_INDEX]);
}
static void word(volatile uint32_t *buffer,unsigned index,unsigned channel,uint16_t value)
{
 adc_dma_word_t x;
 x.word=0;x.fields.result=value;x.fields.adc_ch=channel;x.fields.seq_num=index;x.fields.cycle_bit=1;
 buffer[ADC_DMA_INDEX+index]=x.word;
}
static void data(uint16_t u,uint16_t v)
{
 word(dma_u,0,14,u);word(dma_v,0,10,v);
}
static void frame(uint16_t u,uint16_t v)
{
 CHECK(gate && pending);now=request_cycles+2;data(u,v);
 mock_adc[1].flags=1;adc_v_isr();CHECK(pending);
 mock_adc[0].flags=1;adc_u_isr();CHECK(!irq_depth);
}
static void reset(void)
{
 enabled=false;near_edge=false;init_fail=aux_fail=0;mock_adc[0].id=0;mock_adc[1].id=1;
 CHECK(adc_init()==0);CHECK(gate);CHECK(!enabled);
 CHECK(mock_adc[0].period_count[15]==5 && mock_adc[0].period_count[1]==5);
 CHECK(mock_adc[0].sample_cycles[14]==15 && mock_adc[1].sample_cycles[10]==15);
 CHECK(mock_adc[0].sample_cycles[15]==20 && mock_adc[0].sample_cycles[1]==20);
}
int main(void)
{
 adc_data_t sample;unsigned i;
 reset();CHECK(adc_read(&sample)==1);CHECK(adc_read(0)==-2);
 /* 一路先完成不发布；仅U/V两个DMA字齐全即可发布，不等待辅助通道。 */
 now=request_cycles+1;word(dma_v,0,10,32768);mock_adc[1].flags=1;adc_v_isr();
 CHECK(latest.sample_count==0 && gate);
 now=request_cycles+2;data(32768,32768);mock_adc[0].flags=1;adc_u_isr();
 CHECK(adc_read(&sample)==0 && sample.sample_count==1);
 CHECK(sample.raw_u==32768 && sample.raw_v==32768 && sample.bus_raw==0 && sample.temperature_raw==0);
 CHECK(sample.publish_cycles>sample.capture_cycles && !sample.current_valid);
 CHECK(adc_read(&sample)==0 && sample.sample_count==1);
 /* 母线/温度来自独立周期结果，前台更新；不增加电流组数或ADC完成中断数。 */
 mock_adc[0].period_results[15]=44000;mock_adc[0].period_results[1]=25000;
 adc_poll();adc_read(&sample);
 CHECK(sample.bus_raw==44000 && sample.temperature_raw==25000 && sample.sample_count==1 && sample.irq_count==2);
 /* 中断虽到，DMA尚未落地时，路由关闭，后续poll接着检查，不发布半组。 */
 now=request_cycles+2;data(32768,32768);dma_v[0]=0;
 mock_adc[0].flags=mock_adc[1].flags=1;adc_u_isr();
 CHECK(latest.sample_count==1 && !gate && latest.dma_wait_count==1);
 word(dma_v,0,10,32768);adc_poll();CHECK(latest.sample_count==2 && gate);
 /* 按实际PWM时间跨过1秒稳定期，再按至少1ms间隔收256组，不把20k样本全计入。 */
 for(i=0;i<26000;++i)frame(32768,32768);
 CHECK(adc_read(&sample)==0 && sample.current_valid && sample.offset_samples==256);
 CHECK(sample.offset_u==32768 && sample.offset_v==32768);
 frame(32760,32776);adc_read(&sample);
 CHECK(fabs(sample.current_u_ma-40.2838f)<0.01f && fabs(sample.current_w_ma)<0.01f);
 enabled=true;CHECK(adc_start_offset_calibration()==-9);CHECK(adc_init()==-9);enabled=false;
 /* 重标定请求时已有一组在飞：旧触发时刻不得因无符号减法而跳过稳定等待。 */
 now=request_cycles+1;CHECK(adc_start_offset_calibration()==0);
 frame(32768,32768);CHECK(latest.offset_state==ADC_OFFSET_SETTLING && !latest.offset_samples);
 /* 非本轮DMA标识拒绝并锁存故障；不会重臂。 */
 now=request_cycles+2;data(32768,32768);word(dma_v,0,11,32768);
 mock_adc[0].flags=mock_adc[1].flags=1;adc_u_isr();CHECK(adc_read(&sample)==-6 && !gate && !enabled);
 CHECK(adc_poll()==-6);reset();
 /* 处理超过转换截止时间，完整但可能已被下一轮覆盖的数据也不接受。 */
 now=frame_deadline+1;data(32768,32768);mock_adc[0].flags=mock_adc[1].flags=1;adc_u_isr();
 CHECK(adc_read(&sample)==-7 && sample.timeout_count==1 && !sample.sample_valid && !gate);
 CHECK(sample.timeout_stage==ADC_TIMEOUT_FRAME && sample.timeout_completion_mask==3);
 reset();mock_adc[0].flags=adc16_event_trig_hw_conflict;adc_u_isr();CHECK(adc_read(&sample)==-8 && !gate);
 reset();latest.offset_state=ADC_OFFSET_COLLECTING;last_offset_cycles=now-1000;
 frame(0,32768);CHECK(adc_read(&sample)==-5 && !sample.current_valid && !gate);
 near_edge=true;CHECK(adc_init()==0 && !gate);CHECK(latest.rearm_deferred_count==1);
 /* 首次初始化延期：其它外设/调试器耗时不应在首次前台接管前误报重臂超时。 */
 now+=5000;near_edge=false;CHECK(adc_poll()==1 && gate);
 frame(32768,32768);CHECK(latest.sample_count==1 && !fault);
 /* 从前台首次接管开始计时，PWM一直停在边沿保护窗口仍会超时。 */
 near_edge=true;CHECK(adc_init()==0 && !gate);now+=5000;CHECK(adc_poll()==1 && !gate);
 now+=101;CHECK(adc_poll()==-7 && !gate);
 CHECK(latest.timeout_stage==ADC_TIMEOUT_REARM && latest.timeout_completion_mask==0);
 /* 已运行后的延期不享受首次等待豁免，避免一直沿用旧数据。 */
 near_edge=false;reset();near_edge=true;frame(32768,32768);
 CHECK(!gate && !pending);now+=101;CHECK(adc_poll()==-7);
 CHECK(latest.timeout_stage==ADC_TIMEOUT_REARM);
 near_edge=false;
 /* U先完成、V后完成同样可发布；周期值直到前台读取才变化，不进入电流DMA。 */
 reset();now=request_cycles+2;data(32768,32768);mock_adc[0].period_results[15]=15824;mock_adc[0].period_results[1]=7680;
 mock_adc[0].flags=1;adc_u_isr();CHECK(latest.sample_count==0);
 mock_adc[1].flags=1;adc_v_isr();CHECK(latest.sample_count==1 && latest.bus_raw==0);
 adc_poll();adc_read(&sample);CHECK(sample.bus_raw==15824 && sample.temperature_raw==7680 && !fault);
 /* DMA读取结束才到截止点，也不能发布。此处时间步进只用于注入超时。 */
 reset();cycle_step=1;now=request_cycles+8;data(32768,32768);
 mock_adc[0].flags=mock_adc[1].flags=1;adc_u_isr();
 CHECK(adc_read(&sample)==-7 && !sample.sample_count);cycle_step=0;
 /* 接通前窗口变化或CPU预测已经过期时，保持关路由并重试。 */
 reject_enable_count=1;CHECK(adc_init()==0 && !gate && !pending && latest.rearm_deferred_count==1);
 CHECK(adc_poll()==1 && gate && pending);frame(32768,32768);CHECK(!fault);
 timing_pause=51;CHECK(adc_init()==0 && !gate && !pending && latest.rearm_deferred_count==1);
 timing_pause=0;CHECK(adc_poll()==1 && gate && pending);frame(32768,32768);CHECK(!fault);
 reload_value=0;CHECK(adc_init()==-3);reload_value=10000;
 init_fail=-1;CHECK(adc_init()==-4);init_fail=0;
 aux_fail=1;CHECK(adc_init()==-4 && !gate && !enabled);aux_fail=0;
 CHECK(!irq_depth);
 /* 新帧只回调一次；两路中断、前台重复read/poll均不能重复算电流环。 */
 reset();CHECK(adc_set_control_callback(control_received)==0);
 enabled=true;CHECK(adc_set_control_callback(0)==-9);enabled=false;
 frame(32768,32768);CHECK(frame_callbacks==1 && !fault_callbacks);
 adc_poll();adc_read(&sample);CHECK(frame_callbacks==1);
 /* 辅助SDK读取成功不等于新转换；只有WDOG事件推进有效标志和时间。 */
 CHECK(!sample.bus_valid && !sample.temperature_valid);
 mock_adc[0].flags|=ADC_AUX_BUS_FLAG|ADC_AUX_TEMP_FLAG;
 adc_poll();adc_read(&sample);
 CHECK(sample.bus_valid && sample.temperature_valid);
 {uint64_t stamp=sample.bus_cycles;adc_poll();adc_read(&sample);CHECK(sample.bus_cycles==stamp);}
 now=frame_deadline+1;adc_poll();CHECK(fault_callbacks==1);
 adc_poll();CHECK(fault_callbacks==1);
 CHECK(adc_set_control_callback(0)==0);
 puts("PASS ADC PWM sync: U/V-only DMA, independent auxiliary periods, both IRQ orders, deferred DMA, offset timing, currents, deadline, conflict, tags, calibration guard");
 return 0;
}
