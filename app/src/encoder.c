/* SPDX-License-Identifier: BSD-3-Clause */
#include "encoder.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_qeiv2_drv.h"
#include <stdbool.h>
#include <limits.h>

#define ENCODER_QEI_BASE BOARD_PMSM0_QEI_BASE
/* QEI0 有独立的资源门控；clock_mot0 仅表示电机总线时钟，不能代替资源开启。 */
#define ENCODER_QEI_CLOCK BOARD_BLDC_QEI_CLOCK_SOURCE

static bool initialized;

int encoder_init(void)
{
    initialized = false;
    clock_add_to_group(ENCODER_QEI_CLOCK, 0);
    if (clock_get_frequency(ENCODER_QEI_CLOCK) == 0u) {
        return -1;
    }
    /* 复用已有板级引脚配置：PB07=A，PB06=B。 */
    init_qeiv2_ab_pins(ENCODER_QEI_BASE);
    ENCODER_QEI_BASE->PHIDX = 0u;
    qeiv2_reset_counter(ENCODER_QEI_BASE);
    qeiv2_set_work_mode(ENCODER_QEI_BASE, qeiv2_work_mode_abz);
    /* A/B 均启用上升/下降沿；关闭物理 Z 输入，圈数由相位溢出/下溢维护。 */
    qeiv2_config_abz_uvw_signal_edge(ENCODER_QEI_BASE, true, true, false, true, true);
    qeiv2_config_z_phase_counter_mode(ENCODER_QEI_BASE, qeiv2_z_count_inc_on_phase_count_max);
    qeiv2_config_phmax_phparam(ENCODER_QEI_BASE, ENCODER_COUNTS_PER_REV);
    /* 当前只进行前台读取，禁止其他事件覆盖软件读取快照，也不产生中断/DMA。 */
    qeiv2_disable_irq(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_disable_dma_request(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_disable_load_read_trigger_event(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_clear_status(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_release_counter(ENCODER_QEI_BASE);
    initialized = true;
    return 0;
}

int encoder_reset(void)
{
    if (!initialized) {
        return -1;
    }
    ENCODER_QEI_BASE->PHIDX = 0u;
    qeiv2_reset_counter(ENCODER_QEI_BASE);
    qeiv2_clear_status(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_release_counter(ENCODER_QEI_BASE);
    return 0;
}

int encoder_read(encoder_data_t *data)
{
    uint32_t raw_turns, phase;
    int64_t turns, position;
    if (!initialized) {
        return -1;
    }
    if (data == 0) {
        return -2;
    }
    /* 将圈数、相位同时锁存到 READ 寄存器，避免跨圈时分开读取造成一圈跳变。 */
    qeiv2_load_counter_to_read_registers(ENCODER_QEI_BASE);
    raw_turns = qeiv2_get_count_on_read_event(ENCODER_QEI_BASE, qeiv2_counter_type_z);
    phase = QEIV2_COUNT_PH_PHCNT_GET(qeiv2_get_count_on_read_event(
        ENCODER_QEI_BASE, qeiv2_counter_type_phase));
    if (phase >= ENCODER_COUNTS_PER_REV) {
        return -2;
    }
    /* 圈计数是完整 32 位，反向越过零点时为 0xffffffff，即有符号的 -1 圈。
     * 先用 64 位完成符号换算及乘法，避免依赖强制转换或有符号整数溢出。 */
    turns = raw_turns;
    if (raw_turns > INT32_MAX) {
        turns -= 0x100000000LL;
    }
    position = (turns * ENCODER_COUNTS_PER_REV + phase) * ENCODER_DIRECTION;
    if (position < INT32_MIN || position > INT32_MAX) {
        return -3;
    }
    data->position_count = (int32_t)position;
    data->mechanical_angle_deg = (float)position * (360.0f / (float)ENCODER_COUNTS_PER_REV);
    return 0;
}
