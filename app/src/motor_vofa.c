/* SPDX-License-Identifier: BSD-3-Clause */
#include "motor_vofa.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"
#include "vofa_debug.h"
#include "vofa_hpm_uart.h"
#include <stdbool.h>
#include <float.h>

#if MOTOR_VOFA_SAMPLE_HZ == 0
#error "VOFA upload frequency must be positive"
#endif
#if (4u * (MOTOR_VOFA_CHANNEL_COUNT + 1u)) * 10u * MOTOR_VOFA_SAMPLE_HZ > BOARD_CONSOLE_UART_BAUDRATE
#error "VOFA upload exceeds UART line capacity"
#endif

extern uint32_t hpm_core_clock;

/* 保留全局上下文，便于在调试器中观察帧队列和串口错误统计。 */
vofa_debug_t motor_vofa_context;
volatile int motor_vofa_io_result;
volatile int motor_vofa_publish_result;
static vofa_hpm_uart_t uart_port;
static uint64_t sample_period_cycles;
static uint64_t last_sample_cycles;
static bool initialized;
/* 短临界区保证上传数据来自同一次编码器采样。 */
static volatile encoder_data_t encoder_snapshot;

static int finite_value(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

int motor_vofa_set_encoder(const encoder_data_t *data)
{
    uint32_t irq_state;
    if (data == 0) {
        return VOFA_EINVAL;
    }
    if (!finite_value(data->mechanical_angle_deg) || !finite_value(data->electrical_angle_deg)
        || !finite_value(data->speed_counts_per_s) || !finite_value(data->speed_filtered_counts_per_s)
        || !finite_value(data->speed_rpm) || !finite_value(data->speed_filtered_rpm)) {
        return VOFA_ENONFINITE;
    }
    irq_state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    encoder_snapshot = *data;
    restore_global_irq(irq_state);
    return VOFA_OK;
}
int motor_vofa_init(void)
{
    vofa_debug_config_t config;
    initialized = false;
    motor_vofa_publish_result = VOFA_OK;
    sample_period_cycles = (uint64_t)hpm_core_clock / MOTOR_VOFA_SAMPLE_HZ;
    if (sample_period_cycles == 0u) {
        motor_vofa_io_result = VOFA_EINVAL;
        return motor_vofa_io_result;
    }

    /* 使用板级控制台 UART，先开启时钟和引脚复用，再配置串口。 */
    board_init_uart(BOARD_CONSOLE_UART_BASE);
    uart_port.base = BOARD_CONSOLE_UART_BASE;
    uart_port.source_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    uart_port.baudrate = BOARD_CONSOLE_UART_BAUDRATE;
    motor_vofa_io_result = vofa_hpm_uart_init(&uart_port);
    if (motor_vofa_io_result != VOFA_OK) {
        return motor_vofa_io_result;
    }

    vofa_debug_config_default(&config);
    vofa_hpm_uart_bind(&config, &uart_port);
    config.channels = MOTOR_VOFA_CHANNEL_COUNT;
    /* 当前只上报输出轴数据，不注册电机使能、目标设置或 PID 调参回调。 */
    config.on_command = 0;
    motor_vofa_io_result = vofa_debug_init(&motor_vofa_context, &config);
    if (motor_vofa_io_result != VOFA_OK) {
        return motor_vofa_io_result;
    }
    last_sample_cycles = hpm_csr_get_core_cycle();
    initialized = true;
    return VOFA_OK;
}

void motor_vofa_poll(void)
{
    uint64_t now;
    float values[MOTOR_VOFA_CHANNEL_COUNT];
    encoder_data_t sample;
    encoder_output_data_t output;
    uint32_t irq_state;
    int conversion_result;
    if (!initialized) {
        return;
    }
    motor_vofa_io_result = vofa_debug_poll(&motor_vofa_context);
    now = hpm_csr_get_core_cycle();
    if (now - last_sample_cycles < sample_period_cycles) {
        return;
    }
    /* 前台繁忙时跳过错过的周期，不集中补发旧帧。 */
    last_sample_cycles = now;
    irq_state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    sample = encoder_snapshot;
    restore_global_irq(irq_state);

    /* 在临界区外换算，每个上传周期仅计算一次；不改变电机轴原始数据。 */
    conversion_result = encoder_to_output_axis(&sample, &output);
    if (conversion_result != 0) {
        motor_vofa_publish_result = conversion_result == -3 ? VOFA_ENONFINITE : VOFA_EINVAL;
        return;
    }
    values[0] = output.angle_deg;
    values[1] = output.speed_rpm;
    values[2] = output.speed_filtered_rpm;
    motor_vofa_publish_result = vofa_debug_publish(
        &motor_vofa_context, values, MOTOR_VOFA_CHANNEL_COUNT);
}
