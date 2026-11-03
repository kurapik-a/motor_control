/* PWM同步ADC采样阶段：桥臂保持关闭，主循环读取快照，VOFA仍上传输出轴数据。 */
#include "board.h"
#include "encoder.h"
#include "adc.h"
#include "pwm.h"
#include "motor_vofa.h"

/* 调试器中查看编码器初始化/读取结果；含义见 encoder.h。 */
volatile int encoder_result;
/* PWM 初始化返回值；初始化成功后仍需显式 pwm_enable 才能输出。 */
volatile int pwm_result;
/* 调试器查看 ADC 返回值及采样/零偏状态；电流有效性见 current_valid。 */
volatile int adc_result;
volatile adc_data_t adc_sample;

int main(void)
{
    encoder_data_t sample;
    adc_data_t adc_data;
    board_init();
    pwm_result = pwm_init();
    if (pwm_result != 0)
    {
        while (1) { /* 检查 PWM 时钟及 motor_config.h 的 PWM 配置。 */ }
    }
    encoder_result = encoder_init();
    if (encoder_result != 0)
    {
        while (1)
        { /* 检查 QEI 时钟和 encoder_result。 */
        }
    }
    adc_result = adc_init();
    if (adc_result < 0)
    {
        while (1)
        { /* 检查 adc_result；PWM 桥臂保持关闭。 */
        }
    }
    if (motor_vofa_init() != 0)
    {
        while (1)
        { /* 使用调试器查看 motor_vofa_io_result。 */
        }
    }

    while (1)
    {
        encoder_result = encoder_read(&sample);
        if (encoder_result == 0)
        {
            motor_vofa_set_encoder(&sample);
        }
        adc_poll(); /* ADC由PWM硬件触发；这里只检查健康状态和处理必要重试。 */
        adc_result = adc_read(&adc_data);
        adc_sample = adc_data;
        motor_vofa_poll();
    }
}
