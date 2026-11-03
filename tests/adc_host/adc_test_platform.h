#ifndef ADC_TEST_PLATFORM_H
#define ADC_TEST_PLATFORM_H
#include <stdint.h>
#include <stdbool.h>
#define ADC_SOC_PMT_MAX_DMA_BUFF_LEN_IN_4BYTES 40
#define ADC_SOC_MAX_TRIG_CH_LEN 4
#define ADC_SOC_DMA_ADDR_ALIGNMENT 4
#define ATTR_PLACE_AT_NONCACHEABLE_WITH_ALIGNMENT(x)
#define BOARD_PMSM0_ADC_U_BASE (&mock_adc[0])
#define BOARD_PMSM0_ADC_V_BASE (&mock_adc[1])
#define BOARD_PMSM0_ADC_TRG 0
#define BOARD_PMSM0_ADC_CH_U 14
#define BOARD_PMSM0_ADC_CH_V 10
#define BOARD_APP_ADC16_UDC_CH 15
#define BOARD_APP_ADC16_TEMP_CH 1
#define BOARD_RUNNING_CORE 0
#define IOC_PAD_PF06 0
#define IOC_PAD_PF08 1
#define IOC_PAD_FUNC_CTL_ANALOG_MASK 1
#define HPM_IOC (&mock_ioc)
#define CSR_MSTATUS_MIE_MASK 1
#define IRQn_ADC0 0
#define IRQn_ADC1 1
#define clock_adc0 0
#define clock_adc1 1
#define status_success 0
#define adc16_res_16_bits 21
#define adc16_conv_mode_preemption 3
#define adc16_event_trig_complete 1u
#define adc16_event_trig_sw_conflict 2u
#define adc16_event_trig_hw_conflict 4u
#define adc16_event_read_conflict 8u
#define adc16_event_seq_sw_conflict 16u
#define adc16_event_seq_hw_conflict 32u
#define adc16_event_dma_fifo_full 64u
#define SDK_DECLARE_EXT_ISR_M(irq,handler)
#define fenceiorw() ((void)0)
#define core_local_mem_to_sys_address(core,addr) (addr)
typedef struct {unsigned id,flags,interrupts;} ADC16_Type;
typedef struct {struct {unsigned FUNC_CTL;} PAD[2];} mock_ioc_t;
typedef struct {unsigned ch,sample_cycle;} adc16_channel_config_t;
typedef struct {unsigned res,conv_mode,adc_clk_div;bool sel_sync_ahb,adc_ahb_en;} adc16_config_t;
typedef struct {unsigned trig_ch,trig_len,adc_ch[4];bool inten[4];} adc16_pmt_config_t;
typedef struct {uint32_t result:16;uint32_t reserved:4;uint32_t adc_ch:5;uint32_t trig_ch:4;uint32_t seq_num:2;uint32_t cycle_bit:1;} adc16_pmt_dma_data_t;
extern ADC16_Type mock_adc[2];
extern mock_ioc_t mock_ioc;
uint32_t disable_global_irq(uint32_t);
void restore_global_irq(uint32_t);
uint64_t hpm_csr_get_core_cycle(void);
void intc_m_disable_irq(int);
void intc_m_enable_irq_with_priority(int,int);
void clock_add_to_group(int,int);
uint32_t board_init_adc_clock(ADC16_Type *,bool);
void board_init_adc16_pins(void);
void adc16_get_channel_default_config(adc16_channel_config_t *);
int adc16_init_channel(ADC16_Type *,adc16_channel_config_t *);
void adc16_get_default_config(adc16_config_t *);
int adc16_init(ADC16_Type *,adc16_config_t *);
void adc16_disable_interrupts(ADC16_Type *,uint32_t);
void adc16_enable_interrupts(ADC16_Type *,uint32_t);
int adc16_set_pmt_config(ADC16_Type *,adc16_pmt_config_t *);
int adc16_set_pmt_queue_enable(ADC16_Type *,unsigned,bool);
void adc16_clear_status_flags(ADC16_Type *,uint32_t);
uint32_t adc16_get_status_flags(ADC16_Type *);
void adc16_init_pmt_dma(ADC16_Type *,uint32_t);
#endif
