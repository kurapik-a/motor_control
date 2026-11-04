/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"

typedef enum
{
    ADC_OFFSET_SETTLING = 0, /* 等待放大器稳定，尚未开始求平均。 */
    ADC_OFFSET_COLLECTING,   /* 正在收集新样本。 */
    ADC_OFFSET_READY,        /* 零偏有效，可使用电流换算结果。 */
    ADC_OFFSET_FAILED        /* 标定时电流 ADC 到达量程端点。 */
} adc_offset_state_t;

typedef enum
{
    ADC_TIMEOUT_NONE = 0, /* 未出现超时。 */
    ADC_TIMEOUT_REARM,    /* 触发路由关闭，未能及时重新接通；还未等待本轮转换。 */
    ADC_TIMEOUT_FRAME     /* 已接通触发，预计边沿后仍未取得完整双路DMA。 */
} adc_timeout_stage_t;

typedef struct
{
    uint16_t raw_u;           /* U 相 ADC 完整16位，范围0~65535。 */
    uint16_t raw_v;           /* V 相 ADC 完整16位，范围0~65535。 */
    uint16_t bus_raw;         /* 最近母线周期采样16位码，非同帧电流数据；尚未换算成V。 */
    uint16_t temperature_raw; /* 最近温度周期采样16位码，非同帧电流数据；尚未换算成℃。 */
    bool bus_valid, temperature_valid; /* 观察到对应通道真实转换事件后才置位。 */
    uint64_t bus_cycles, temperature_cycles; /* 最近辅助转换事件的软件观察时刻。 */
    uint16_t offset_u;        /* U 相零电流基准，完整16位码。 */
    uint16_t offset_v;        /* V 相零电流基准，完整16位码。 */
    float current_u_ma;       /* 未滤波相电流，mA；方向沿用原工程：零偏减采样码。 */
    float current_v_ma;       /* 未滤波相电流，mA。 */
    float current_w_ma;       /* -Iu-Iv，推算值，没有独立采样 W 相。 */
    uint32_t sample_count;    /* 完整新采样组计数；重复 adc_read 不增加。 */
    uint32_t offset_samples;  /* 本轮标定已接受的独立采样组数。 */
    uint64_t capture_cycles;  /* 本组PWM边沿的估计时刻，CPU周期计数；不是硬件时间戳。 */
    uint64_t publish_cycles;  /* 完整采样发布时刻，CPU周期计数。 */
    uint32_t irq_count, timeout_count, dma_wait_count, rearm_deferred_count;
    adc_timeout_stage_t timeout_stage; /* -7发生阶段，重初始化清零。 */
    uint8_t timeout_completion_mask;   /* 转换超时时：bit0=U队列完成，bit1=V队列完成；3=均完成。 */
    int fault_code;                    /* 锁存的故障码；正常为0。 */
    bool trigger_armed;                /* 当前PWM采样触发路由是否接通。 */
    adc_offset_state_t offset_state;
    bool sample_valid;    /* 至少取得一组完整U/V电流采样，且当前没有采样硬件故障；不代表辅助采样新鲜度。 */
    bool current_clipped; /* U/V 原始码到达 0 或 65535，电流换算不可用。 */
    bool current_valid;   /* 零偏完成、采样有效且未达到量程端点。 */
} adc_data_t;

/* 0=有完整采样；1=尚无完整采样；-1=未初始化，-2=空指针，-3=配置/时序无效，
 * -4=驱动初始化失败，-5=标定饱和，-6=DMA标识错误，-7=转换/DMA超时，
 * -8=ADC冲突/溢出，-9=桥臂已使能，拒绝初始化或重标定。
 * 故障锁存后关闭触发路由和桥臂，需重新adc_init恢复。 */

/* board_init、pwm_init后调用；PWM抢占队列：ADC0=[U:PF06/CH14]，ADC1=[V:PF08/CH10]。
 * 同一PWM0 TRGO0触发两路，ADC中断检查完成及DMA完整性。
 * 母线PF31/CH15、温度PF26/CH1沿用原工程ADC0周期采样，参数见MOTOR_ADC_AUX_PERIOD_*。
 * 自动进行零偏标定。桥臂必须关闭；初始化不会使能功率输出。 */
int adc_init(void);

/* 主循环持续调用：电流采样健康检查、必要重试，读取母线/温度周期结果；不软件触发ADC。 */
int adc_poll(void);

/* 主循环读取最近一致快照；短临界区复制，不直接读取正在刷新的DMA。
 * 返回0仅表示采样有效，使用电流还须检查current_valid；标定期间电流字段置0。 */
int adc_read(adc_data_t *data);

/* 仅在桥臂关闭时接受请求，立即使旧零偏无效；不要求等待采样空隙。 */
int adc_start_offset_calibration(void);
/* 新完整帧在发布后、重臂前调用一次；故障时status为负。
 * 可在ADC中断或adc_poll临界区执行，禁止阻塞/递归访问ADC；回调须在ILM。
 * 仅桥臂关闭时更换；不移动或替换ADC内部保护。 */
typedef void (*adc_control_callback_t)(const adc_data_t *sample, int status);
int adc_set_control_callback(adc_control_callback_t callback);
#endif /* MOTOR_ADC_H */
