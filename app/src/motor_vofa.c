/* SPDX-License-Identifier: BSD-3-Clause */
#include "motor_vofa.h"
#include "pmsm_current_ctrl.h"
#include "pmsm_speed_ctrl.h"
#include "pmsm_position_ctrl.h"
#include "pmsm_init.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_interrupt.h"
#include "vofa_debug.h"
#include "vofa_hpm_uart.h"
#include <stdbool.h>
#include <float.h>

/* 编码器快照的短临界区从ILM执行，避免Flash取指导致ADC完成中断长时间等待。 */
int motor_vofa_set_encoder(const encoder_data_t *data) ATTR_RAMFUNC;
void motor_vofa_poll(void) ATTR_RAMFUNC;

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
volatile int motor_vofa_command_result;
static vofa_hpm_uart_t uart_port;
static uint64_t sample_period_cycles;
static uint64_t last_sample_cycles;
static bool initialized;
/* 收发接口统一转换：控制器不持有rpm或减速比。初始化时只计算一次系数。 */
static float counts_s_per_output_rpm, output_rpm_per_counts_s;
static float counts_per_output_deg, output_deg_per_count;
/* 短临界区保证上传数据来自同一次编码器采样。 */
static volatile encoder_data_t encoder_snapshot;

static int finite_value(float value)
{
    return value >= -FLT_MAX && value <= FLT_MAX;
}

/* 协议库在前台poll中调用；经过电流闭环启动门禁，返回实际接受/拒绝结果。
 * 不打印文本回执，避免混入JustFloat上传帧。 */
static int motor_vofa_command(void *user, const vofa_command_t *command)
{
    int result = VOFA_EINVAL;
    (void)user;
    if (!command)
        return VOFA_EINVAL;
    switch (command->kind)
    {
    case VOFA_CMD_ENABLE:
        if (command->value == 1.0f)
        {
            result = pmsm_current_ctrl_start();
            break;
        }
        if (command->value != 0.0f)
            break;
        /* ENABLE,0与STOP都立即停机并清零目标。 */
        pmsm_init_cancel();
        result = VOFA_OK;
        break;
    case VOFA_CMD_STOP:
        pmsm_init_cancel();
        result = VOFA_OK;
        break;
    case VOFA_CMD_PING:
        result = VOFA_OK;
        break;
    case VOFA_CMD_CURRENT_MA:
        result = pmsm_current_ctrl_set_target(command->value);
        break;
    case VOFA_CMD_SPEED_RPM:
        result = pmsm_current_ctrl_set_speed_counts_s(command->value * counts_s_per_output_rpm);
        break;
    case VOFA_CMD_POSITION_DEG:
    {
        float counts = command->value * counts_per_output_deg;
        /* 目标落在实际编码器整数计数上，也消除大角度端点的浮点比例舍入。 */
        counts = (float)(int32_t)(counts + (counts >= 0 ? 0.5f : -0.5f));
        result = pmsm_current_ctrl_set_position_counts(counts);
        break;
    }
    case VOFA_CMD_CLEAR_FAULT:
        result = pmsm_current_ctrl_clear_fault();
        break;
    default:
        break; /* 在线PID命令尚未接入。 */
    }
    motor_vofa_command_result = result;
    return result;
}

int motor_vofa_set_encoder(const encoder_data_t *data)
{
    uint32_t irq_state;
    if (data == 0)
    {
        return VOFA_EINVAL;
    }
    if (!finite_value(data->speed_counts_per_s) || !finite_value(data->speed_filtered_counts_per_s))
    {
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
    counts_s_per_output_rpm = MOTOR_ABZ_COUNTS_PER_REV * MOTOR_SPEED_REDUCTION_RATIO / 60.0f;
    if (!finite_value(counts_s_per_output_rpm) || counts_s_per_output_rpm <= 0)
    {
        motor_vofa_io_result = VOFA_EINVAL;
        return VOFA_EINVAL;
    }
    output_rpm_per_counts_s = 1.0f / counts_s_per_output_rpm;
    counts_per_output_deg = MOTOR_ABZ_COUNTS_PER_REV * MOTOR_SPEED_REDUCTION_RATIO / 360.0f;
    output_deg_per_count = 360.0f / (MOTOR_ABZ_COUNTS_PER_REV * MOTOR_SPEED_REDUCTION_RATIO);
    motor_vofa_publish_result = VOFA_OK;
    motor_vofa_command_result = VOFA_OK;
    sample_period_cycles = (uint64_t)hpm_core_clock / MOTOR_VOFA_SAMPLE_HZ;
    if (sample_period_cycles == 0u)
    {
        motor_vofa_io_result = VOFA_EINVAL;
        return motor_vofa_io_result;
    }

    /* 使用板级控制台 UART，先开启时钟和引脚复用，再配置串口。 */
    board_init_uart(BOARD_CONSOLE_UART_BASE);
    uart_port.base = BOARD_CONSOLE_UART_BASE;
    uart_port.source_hz = clock_get_frequency(BOARD_CONSOLE_UART_CLK_NAME);
    uart_port.baudrate = BOARD_CONSOLE_UART_BAUDRATE;
    motor_vofa_io_result = vofa_hpm_uart_init(&uart_port);
    if (motor_vofa_io_result != VOFA_OK)
    {
        return motor_vofa_io_result;
    }

    vofa_debug_config_default(&config);
    vofa_hpm_uart_bind(&config, &uart_port);
    config.channels = MOTOR_VOFA_CHANNEL_COUNT;
    config.current_ma.min = -MOTOR_CURRENT_CTRL_MAX_COMMAND_MA;
    config.current_ma.max = MOTOR_CURRENT_CTRL_MAX_COMMAND_MA;
    config.speed_rpm.max = MOTOR_SPEED_CTRL_MAX_COUNTS_S * output_rpm_per_counts_s;
    config.speed_rpm.min = -config.speed_rpm.max;
    config.position_deg.max = MOTOR_POSITION_MAX_COMMAND_COUNTS * output_deg_per_count;
    config.position_deg.min = -config.position_deg.max;
    /* ENABLE仅在上电准备完成后启动闭环。 */
    config.on_command = motor_vofa_command;
    motor_vofa_io_result = vofa_debug_init(&motor_vofa_context, &config);
    if (motor_vofa_io_result != VOFA_OK)
    {
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
    uint32_t irq_state;
    if (!initialized)
    {
        return;
    }
    motor_vofa_io_result = vofa_debug_poll(&motor_vofa_context);
    now = hpm_csr_get_core_cycle();
    if (now - last_sample_cycles < sample_period_cycles)
    {
        return;
    }
    /* 前台繁忙时跳过错过的周期，不集中补发旧帧。 */
    last_sample_cycles = now;
    irq_state = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    sample = encoder_snapshot;
    restore_global_irq(irq_state);

    /* 所有显示单位换算仅在100Hz上传时进行，不进入1kHz/20kHz控制链。 */
    pmsm_current_status_t current;
    pmsm_speed_status_t speed;
    pmsm_position_status_t position;
    pmsm_current_ctrl_get_status(&current);
    pmsm_speed_ctrl_get_status(&speed);
    pmsm_position_ctrl_get_status(&position);
    values[0] = current.id_ref_ma;
    values[1] = current.id_ma;
    values[2] = current.iq_ref_ma;
    values[3] = current.iq_ma;
    values[4] = current.ud;
    values[5] = current.uq;
    values[6] = sample.speed_counts_per_s * output_rpm_per_counts_s;
    values[7] = (float)(current.faults | current.inhibit_faults);
    values[8] = speed.target_counts_s * output_rpm_per_counts_s;
    values[9] = speed.feedback_counts_s * output_rpm_per_counts_s;
    values[10] = position.target_count * output_deg_per_count;
    values[11] = (float)position.position_count * output_deg_per_count;
    motor_vofa_publish_result = vofa_debug_publish(
        &motor_vofa_context, values, MOTOR_VOFA_CHANNEL_COUNT);
}
