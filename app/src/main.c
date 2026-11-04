/* 上电自动完成标定/一次对齐后关桥；ENABLE只启动正式电流闭环。 */
#include "board.h"
#include "encoder.h"
#include "adc.h"
#include "motor_vofa.h"
#include "pmsm_init.h"

/* 调试器中查看编码器初始化/读取结果；含义见 encoder.h。 */
volatile int encoder_result;
/* 调试器查看 ADC 返回值及采样/零偏状态；电流有效性见 current_valid。 */
volatile int adc_result;
volatile adc_data_t adc_sample;

int main(void)
{
    encoder_data_t sample;
    adc_data_t adc_data;
    board_init();
    /* 串口等可能耗时的初始化先完成，再启动带微秒级截止时间的同步采样。 */
    if (motor_vofa_init() != 0)
    {
        while (1)
        { /* 使用调试器查看 motor_vofa_io_result。 */
        }
    }
    if (pmsm_init() != 0)
    {
        while (1)
        { /* 检查pmsm_init_monitor.stage/result。 */
        }
    }
    while (1)
    {
        adc_poll(); /* 先处理首次/延期重臂；采样仍由PWM硬件触发。 */
        adc_result = adc_read(&adc_data);
        adc_sample = adc_data;
        pmsm_init_poll();
        encoder_result = encoder_read(&sample);
        if (encoder_result == 0)
        {
            motor_vofa_set_encoder(&sample);
        }
        motor_vofa_poll();
    }
}
