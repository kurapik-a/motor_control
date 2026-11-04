/* SPDX-License-Identifier: BSD-3-Clause */
#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
#include "encoder.h"
#include "board.h"
#include "hpm_clock_drv.h"
#include "hpm_csr_drv.h"
#include "hpm_qeiv2_drv.h"
#include <stdbool.h>
#include <limits.h>
#include "motor_rt.h"
int encoder_read_phase(uint32_t *phase) MOTOR_RAMFUNC;
#if defined(__GNUC__)
static inline __attribute__((always_inline)) uint32_t qeiv2_get_current_count(QEIV2_Type *qeiv2_x, qeiv2_counter_type_t type);
static inline __attribute__((always_inline)) void qeiv2_reset_counter(QEIV2_Type *qeiv2_x);
static inline __attribute__((always_inline)) void qeiv2_release_counter(QEIV2_Type *qeiv2_x);
static inline __attribute__((always_inline)) void qeiv2_clear_status(QEIV2_Type *qeiv2_x, uint32_t mask);
#endif

#define ENCODER_QEI_BASE BOARD_PMSM0_QEI_BASE
#define ENCODER_QEI_CLOCK BOARD_BLDC_QEI_CLOCK_SOURCE
extern uint32_t hpm_core_clock;
static bool initialized;
static uint32_t generation;
static bool speed_has_reference;
static uint32_t cycle_clock_hz;
static uint64_t speed_period_cycles, last_speed_cycles;
static int32_t last_speed_position;
static float speed_counts_per_s, speed_filtered_counts_per_s;
static float speed_filter_tau_s;

MOTOR_INLINE void reset_speed_history(void)
{
    speed_has_reference = false;
    last_speed_cycles = 0;
    last_speed_position = 0;
    speed_counts_per_s = 0.0f;
    speed_filtered_counts_per_s = 0.0f;
}

int encoder_init(void)
{
    float alpha = ENCODER_SPEED_FILTER_ALPHA;
    initialized = false;
    reset_speed_history();
    if (!(alpha > 0.0f && alpha <= 1.0f)) {
        return -4;
    }
    cycle_clock_hz = hpm_core_clock;
    if (cycle_clock_hz < ENCODER_SPEED_SAMPLE_HZ) {
        return -1;
    }
    speed_period_cycles = ((uint64_t)cycle_clock_hz + ENCODER_SPEED_SAMPLE_HZ - 1u)
        / ENCODER_SPEED_SAMPLE_HZ;
    /* 标称周期时有效系数恰好为 alpha，延迟读取时仍保持相近的滤波时间尺度。 */
    speed_filter_tau_s = (1.0f - alpha) / (alpha * ENCODER_SPEED_SAMPLE_HZ);
    clock_add_to_group(ENCODER_QEI_CLOCK, 0);
    if (clock_get_frequency(ENCODER_QEI_CLOCK) == 0u) {
        return -1;
    }
    init_qeiv2_ab_pins(ENCODER_QEI_BASE);
    ENCODER_QEI_BASE->PHIDX = 0u;
    qeiv2_reset_counter(ENCODER_QEI_BASE);
    qeiv2_set_work_mode(ENCODER_QEI_BASE, qeiv2_work_mode_abz);
    /* A/B 上升、下降沿均计数；关闭物理 Z，圈数由相位溢出/下溢维护。 */
    qeiv2_config_abz_uvw_signal_edge(ENCODER_QEI_BASE, true, true, false, true, true);
    qeiv2_config_z_phase_counter_mode(ENCODER_QEI_BASE, qeiv2_z_count_inc_on_phase_count_max);
    qeiv2_config_phmax_phparam(ENCODER_QEI_BASE, ENCODER_COUNTS_PER_REV);
    qeiv2_disable_irq(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_disable_dma_request(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_disable_load_read_trigger_event(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_clear_status(ENCODER_QEI_BASE, UINT32_MAX);
    qeiv2_release_counter(ENCODER_QEI_BASE);
    initialized = true;
    ++generation;
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
    reset_speed_history();
    ++generation;
    return 0;
}

uint32_t encoder_get_generation(void) { return generation; }

int encoder_read_phase(uint32_t *phase)
{
    uint32_t value;
    if (!initialized || !phase) return -1;
    value = QEIV2_COUNT_PH_PHCNT_GET(qeiv2_get_current_count(ENCODER_QEI_BASE, qeiv2_counter_type_phase));
    if (value >= ENCODER_COUNTS_PER_REV) return -2;
    *phase = value;
    return 0;
}

int encoder_read_position(int32_t *position_count)
{
    uint32_t before, after, phase, attempt;
    int64_t turns, position;
    if (!initialized || !position_count) return -1;
    for (attempt = 0; attempt < 2; ++attempt) {
        before = qeiv2_get_current_count(ENCODER_QEI_BASE, qeiv2_counter_type_z);
        phase = QEIV2_COUNT_PH_PHCNT_GET(qeiv2_get_current_count(ENCODER_QEI_BASE, qeiv2_counter_type_phase));
        after = qeiv2_get_current_count(ENCODER_QEI_BASE, qeiv2_counter_type_z);
        if (before != after) continue;
        if (phase >= ENCODER_COUNTS_PER_REV) return -2;
        turns = before;
        if (before > INT32_MAX) turns -= 0x100000000LL;
        position = (turns * ENCODER_COUNTS_PER_REV + phase) * ENCODER_DIRECTION;
        if (position < INT32_MIN || position > INT32_MAX) return -3;
        *position_count = (int32_t)position;
        return 0;
    }
    return -2;
}

static void update_speed(int32_t position, uint64_t now)
{
    uint64_t elapsed;
    float elapsed_s, effective_alpha;
    int64_t delta;
    if (!speed_has_reference) {
        last_speed_position = position;
        last_speed_cycles = now;
        speed_has_reference = true;
        return;
    }
    elapsed = now - last_speed_cycles;
    if (elapsed < speed_period_cycles) {
        return;
    }
    delta = (int64_t)position - last_speed_position;
    elapsed_s = (float)elapsed / (float)cycle_clock_hz;
    speed_counts_per_s = (float)delta / elapsed_s;
    effective_alpha = elapsed_s / (speed_filter_tau_s + elapsed_s);
    speed_filtered_counts_per_s += effective_alpha
        * (speed_counts_per_s - speed_filtered_counts_per_s);
    last_speed_position = position;
    last_speed_cycles = now;
}

int encoder_read(encoder_data_t *data)
{
    uint32_t raw_turns, phase;
    int64_t turns, position;
    uint64_t now;
    if (!initialized) {
        return -1;
    }
    if (data == 0) {
        return -2;
    }
    /* 同时锁存圈数、相位，避免跨圈时分开读取造成位置/速度跳变。 */
    qeiv2_load_counter_to_read_registers(ENCODER_QEI_BASE);
    now = hpm_csr_get_core_cycle();
    raw_turns = qeiv2_get_count_on_read_event(ENCODER_QEI_BASE, qeiv2_counter_type_z);
    phase = QEIV2_COUNT_PH_PHCNT_GET(qeiv2_get_count_on_read_event(
        ENCODER_QEI_BASE, qeiv2_counter_type_phase));
    if (phase >= ENCODER_COUNTS_PER_REV) {
        return -2;
    }
    turns = raw_turns;
    if (raw_turns > INT32_MAX) {
        turns -= 0x100000000LL;
    }
    position = (turns * ENCODER_COUNTS_PER_REV + phase) * ENCODER_DIRECTION;
    if (position < INT32_MIN || position > INT32_MAX) {
        return -3;
    }
    update_speed((int32_t)position, now);
    data->position_count = (int32_t)position;
    data->phase_count = phase;
    data->speed_counts_per_s = speed_counts_per_s;
    data->speed_filtered_counts_per_s = speed_filtered_counts_per_s;
    return 0;
}
