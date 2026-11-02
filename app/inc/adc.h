/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_ADC_H
#define MOTOR_ADC_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_config.h"

typedef enum {
    ADC_OFFSET_SETTLING = 0, /* 等待放大器稳定，尚未开始求平均。 */
    ADC_OFFSET_COLLECTING,   /* 正在收集新样本。 */
    ADC_OFFSET_READY,        /* 零偏有效，可使用电流换算结果。 */
    ADC_OFFSET_FAILED       /* 标定时电流 ADC 到达量程端点。 */
} adc_offset_state_t;

typedef struct {
    uint16_t raw_u;           /* U 相 ADC 高 12 位，范围 0~4095。 */
    uint16_t raw_v;           /* V 相 ADC 高 12 位，范围 0~4095。 */
    uint16_t bus_raw;         /* 母线原始 16 位码，尚未换算成 V。 */
    uint16_t temperature_raw; /* 温度原始 16 位码，尚未换算成摄氏度。 */
    uint16_t offset_u;        /* U 相零电流基准，高 12 位码。 */
    uint16_t offset_v;        /* V 相零电流基准，高 12 位码。 */
    float current_u_ma;       /* 未滤波相电流，mA；方向沿用原工程：零偏减采样码。 */
    float current_v_ma;       /* 未滤波相电流，mA。 */
    float current_w_ma;       /* -Iu-Iv，推算值，没有独立采样 W 相。 */
    uint32_t sample_count;    /* 完整新采样组计数；重复 adc_read 不增加。 */
    uint32_t offset_samples;  /* 本轮标定已接受的独立采样组数。 */
    uint64_t capture_cycles;  /* 本组软件触发时间，CPU 周期计数；不是毫秒。 */
    adc_offset_state_t offset_state;
    bool sample_valid;       /* 至少取得一组完整采样，且当前没有采样硬件故障。 */
    bool current_clipped;     /* U/V 原始码到达 0 或 4095，电流换算不可用。 */
    bool current_valid;       /* 零偏完成、采样有效且未达到量程端点。 */
} adc_data_t;

/* 返回值：0=成功；1=尚无完整采样，或重标定请求时采样仍在进行。
 * -1=未初始化，-2=空指针，-3=配置/时钟无效，-4=ADC 驱动初始化/触发失败，
 * -5=零偏标定遇到量程端点，-6=DMA 数据标识错误，-7=转换超时，-8=ADC 冲突/溢出。
 * 所有接口仅在主循环调用，不要与中断并发使用。 */
/* board_init 后调用一次。ADC0: U=CH14/PF06、母线=CH15/PF31、温度=CH1/PF26；
 * ADC1: V=CH10/PF08。初始化后自动进行零偏标定，期间必须保证无相电流。
 * 驱动初始化包含芯片内部 ADC 校准；放大器零偏则由后续 adc_poll 异步完成。
 * 不启动 PWM、不使能功率桥；当前软件先后触发两个 ADC，并非严格同步采样。 */
int adc_init(void);
/* 主循环持续调用；按 MOTOR_ADC_SAMPLE_HZ 发起采样，等待 DMA 完整后再更新。
 * 不忙等转换完成；主循环延迟时不补发多组。采样故障锁存，需要重新初始化。
 * 当前仅用于独立采样验证；FOC 必须另行接入 PWM 同步触发。 */
int adc_poll(void);
/* 复制最近数据，返回值含义同上。除空指针外，失败时也复制状态以便诊断。
 * 返回 0 只说明原始采样可用；使用电流前还必须检查 current_valid。
 * 标定期间 current_valid=false，电流字段为 0；此时不要把 0 当成实测电流。 */
int adc_read(adc_data_t *data);
/* 停止电机且相电流为零后可手动重标定。若返回 1，先继续 poll 再重试。
 * 接受请求后立即使旧零偏失效，重新等待稳定并收集样本；不会自动停止功率桥。 */
int adc_start_offset_calibration(void);
#endif /* MOTOR_ADC_H */
