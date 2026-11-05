/* SPDX-License-Identifier: BSD-3-Clause */
#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
#include "pmsm_trajectory.h"
#include "motor_config.h"
#include "pwm.h"
#include "hpm_interrupt.h"
#include "hpm_csr_drv.h"
#include <float.h>
#include <string.h>

volatile pmsm_trajectory_status_t pmsm_trajectory_monitor;
enum { TRAJ_IDLE, TRAJ_NEUTRAL, TRAJ_BRAKE, TRAJ_SPEED, TRAJ_POSITION, TRAJ_DWELL };
static float duration[7], jerk[7];
static float anchor_p, anchor_v, anchor_a, carry, dt;
static uint32_t ticks;
static unsigned segment, segments, stage;
static bool dirty;
static bool initialized;
static pmsm_trajectory_config_t trajectory_config;
static pmsm_trajectory_cycle_config_t cycle_config;
MOTOR_INLINE float trajectory_abs(float x) { return x < 0 ? -x : x; }
MOTOR_INLINE bool trajectory_finite(float x) { return x >= -FLT_MAX && x <= FLT_MAX; }
MOTOR_INLINE bool trajectory_config_valid(const pmsm_trajectory_config_t *config)
{
    return config->position_max_speed_counts_s > 0 && trajectory_finite(config->position_max_speed_counts_s) &&
        config->position_max_speed_counts_s <= MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK * MOTOR_SPEED_LOOP_HZ &&
        config->position_max_speed_counts_s <= MOTOR_SPEED_CTRL_MAX_COUNTS_S &&
        config->max_accel_counts_s2 > 0 && trajectory_finite(config->max_accel_counts_s2) &&
        config->max_jerk_counts_s3 > 0 && trajectory_finite(config->max_jerk_counts_s3);
}

int pmsm_trajectory_set_config(const pmsm_trajectory_config_t *config)
{
    pmsm_trajectory_config_t next;
    uint32_t irq;
    if (!config) return PMSM_TRAJECTORY_INVALID_CONFIG;
    next = *config;
    if (!trajectory_config_valid(&next)) return PMSM_TRAJECTORY_INVALID_CONFIG;
    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized) { restore_global_irq(irq); return PMSM_TRAJECTORY_NOT_INITIALIZED; }
    if (pwm_is_enabled()) { restore_global_irq(irq); return PMSM_TRAJECTORY_BUSY; }
    trajectory_config = next;
    restore_global_irq(irq);
    return PMSM_TRAJECTORY_OK;
}

int pmsm_trajectory_get_config(pmsm_trajectory_config_t *config)
{
    uint32_t irq;
    if (!config) return PMSM_TRAJECTORY_INVALID_CONFIG;
    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized) { restore_global_irq(irq); return PMSM_TRAJECTORY_NOT_INITIALIZED; }
    *config = trajectory_config;
    restore_global_irq(irq);
    return PMSM_TRAJECTORY_OK;
}

MOTOR_INLINE bool trajectory_cycle_valid(const pmsm_trajectory_cycle_config_t *config)
{
    return (config->mode == PMSM_TRAJECTORY_SINGLE || config->mode == PMSM_TRAJECTORY_ROUND_TRIP) &&
        config->dwell_ms <= (UINT32_MAX - 999u) / MOTOR_SPEED_LOOP_HZ;
}

int pmsm_trajectory_set_cycle_config(const pmsm_trajectory_cycle_config_t *config)
{
    pmsm_trajectory_cycle_config_t next;
    uint32_t irq;
    if (!config) return PMSM_TRAJECTORY_INVALID_CONFIG;
    next = *config;
    if (!trajectory_cycle_valid(&next)) return PMSM_TRAJECTORY_INVALID_CONFIG;
    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized) { restore_global_irq(irq); return PMSM_TRAJECTORY_NOT_INITIALIZED; }
    if (pwm_is_enabled()) { restore_global_irq(irq); return PMSM_TRAJECTORY_BUSY; }
    cycle_config = next;
    restore_global_irq(irq);
    return PMSM_TRAJECTORY_OK;
}

int pmsm_trajectory_get_cycle_config(pmsm_trajectory_cycle_config_t *config)
{
    uint32_t irq;
    if (!config) return PMSM_TRAJECTORY_INVALID_CONFIG;
    irq = disable_global_irq(CSR_MSTATUS_MIE_MASK);
    if (!initialized) { restore_global_irq(irq); return PMSM_TRAJECTORY_NOT_INITIALIZED; }
    *config = cycle_config;
    restore_global_irq(irq);
    return PMSM_TRAJECTORY_OK;
}

/* 有界迭代，避免IRQ调用库中的double数学函数或Flash代码。 */
static float trajectory_root(float x, bool cube) MOTOR_RAMFUNC;
static float trajectory_root(float x, bool cube)
{
    float lo = 0, hi = x > 1 ? x : 1;
    unsigned i;
    for (i = 0; i < 30; ++i) {
        float mid = (lo + hi) * 0.5f;
        float value = mid * mid;
        if (cube) value *= mid;
        if (value > x) hi = mid; else lo = mid;
    }
    return (lo + hi) * 0.5f;
}

static void trajectory_anchor(void) MOTOR_RAMFUNC;
static void trajectory_anchor(void)
{
    anchor_p = pmsm_trajectory_monitor.position_count;
    anchor_v = pmsm_trajectory_monitor.speed_counts_s;
    anchor_a = pmsm_trajectory_monitor.acceleration_counts_s2;
    ticks = 0;
}

/* 从零加速度连接任意两个速度：+J、恒加速度、-J。 */
static void trajectory_velocity(float target, unsigned next_stage) MOTOR_RAMFUNC;
static void trajectory_velocity(float target, unsigned next_stage)
{
    float delta = target - pmsm_trajectory_monitor.speed_counts_s;
    float amount = trajectory_abs(delta);
    float tj = trajectory_config.max_accel_counts_s2 / trajectory_config.max_jerk_counts_s3;
    float ta = amount / trajectory_config.max_accel_counts_s2 - tj;
    float j = delta < 0 ? -trajectory_config.max_jerk_counts_s3 : trajectory_config.max_jerk_counts_s3;
    if (ta < 0) { tj = trajectory_root(amount / trajectory_config.max_jerk_counts_s3, false); ta = 0; }
    duration[0] = duration[2] = tj; duration[1] = ta;
    jerk[0] = j; jerk[1] = 0; jerk[2] = -j;
    segment = 0; segments = 3; stage = next_stage;
    trajectory_anchor();
}

/* 静止到静止的七段S曲线；短行程自动取消匀速/恒加速度段。 */
static void trajectory_position(void) MOTOR_RAMFUNC;
static void trajectory_position(void)
{
    float delta = pmsm_trajectory_monitor.target_count - pmsm_trajectory_monitor.position_count;
    float distance = trajectory_abs(delta);
    float vmax = trajectory_config.position_max_speed_counts_s;
    float a = trajectory_config.max_accel_counts_s2, jmax = trajectory_config.max_jerk_counts_s3;
    float tj = a / jmax, ta = vmax / a - tj, tv;
    float j = delta < 0 ? -jmax : jmax;
    if (ta < 0) { tj = trajectory_root(vmax / jmax, false); ta = 0; }
    tv = distance / vmax - (2 * tj + ta);
    if (tv < 0) {
        tv = 0; tj = a / jmax;
        if (distance >= 2 * a * tj * tj) {
            ta = (trajectory_root(tj * tj + 4 * distance / a, false) - 3 * tj) * 0.5f;
            if (ta < 0) ta = 0;
        } else { tj = trajectory_root(distance / (2 * jmax), true); ta = 0; }
    }
    duration[0] = duration[2] = duration[4] = duration[6] = tj;
    duration[1] = duration[5] = ta; duration[3] = tv;
    jerk[0] = j; jerk[1] = 0; jerk[2] = -j; jerk[3] = 0;
    jerk[4] = -j; jerk[5] = 0; jerk[6] = j;
    segment = 0; segments = 7; stage = TRAJ_POSITION;
    trajectory_anchor();
}

static void trajectory_plan(void) MOTOR_RAMFUNC;
static void trajectory_plan(void)
{
    if (pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_SPEED)
        trajectory_velocity(pmsm_trajectory_monitor.target_counts_s, TRAJ_SPEED);
    else if (trajectory_abs(pmsm_trajectory_monitor.speed_counts_s) > 0.0001f)
        trajectory_velocity(0, TRAJ_BRAKE);
    else {
        pmsm_trajectory_monitor.speed_counts_s = 0;
        trajectory_position();
    }
}

int pmsm_trajectory_init(void)
{
    const pmsm_trajectory_config_t defaults = { MOTOR_TRAJECTORY_POSITION_MAX_COUNTS_S,
        MOTOR_TRAJECTORY_ACCEL_COUNTS_S2, MOTOR_TRAJECTORY_JERK_COUNTS_S3 };
    const pmsm_trajectory_cycle_config_t cycles = {
        MOTOR_TRAJECTORY_CYCLE_MODE, MOTOR_TRAJECTORY_CYCLE_COUNT, MOTOR_TRAJECTORY_DWELL_MS };
    if (pwm_is_enabled()) return PMSM_TRAJECTORY_BUSY;
    if (!trajectory_config_valid(&defaults) || !trajectory_cycle_valid(&cycles)) return PMSM_TRAJECTORY_INVALID_CONFIG;
    trajectory_config = defaults;
    cycle_config = cycles;
    dt = 1.0f / MOTOR_SPEED_LOOP_HZ;
    memset((void *)&pmsm_trajectory_monitor, 0, sizeof(pmsm_trajectory_monitor));
    pmsm_trajectory_reset(0, 0);
    initialized = true;
    return 0;
}

void pmsm_trajectory_reset(float position_count, float speed_counts_s)
{
    pmsm_trajectory_monitor.mode = PMSM_TRAJECTORY_OFF;
    pmsm_trajectory_monitor.active = false;
    pmsm_trajectory_monitor.target_count = pmsm_trajectory_monitor.position_count = position_count;
    pmsm_trajectory_monitor.target_counts_s = pmsm_trajectory_monitor.speed_counts_s = speed_counts_s;
    pmsm_trajectory_monitor.acceleration_counts_s2 = 0;
    pmsm_trajectory_monitor.cycling = pmsm_trajectory_monitor.returning = pmsm_trajectory_monitor.dwelling = false;
    pmsm_trajectory_monitor.cycles_completed = pmsm_trajectory_monitor.dwell_ticks_remaining = 0;
    pmsm_trajectory_monitor.cycle_origin_count = pmsm_trajectory_monitor.cycle_destination_count = position_count;
    dirty = false; stage = TRAJ_IDLE; segment = segments = 0; carry = 0;
    trajectory_anchor();
}

int pmsm_trajectory_set_speed(float target_counts_s)
{
    if (!trajectory_finite(target_counts_s) || trajectory_abs(target_counts_s) > MOTOR_SPEED_CTRL_MAX_COUNTS_S) return -1;
    if (pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_SPEED && pmsm_trajectory_monitor.target_counts_s == target_counts_s) return 0;
    pmsm_trajectory_monitor.mode = PMSM_TRAJECTORY_SPEED;
    pmsm_trajectory_monitor.cycling = pmsm_trajectory_monitor.dwelling = pmsm_trajectory_monitor.returning = false;
    pmsm_trajectory_monitor.dwell_ticks_remaining = 0;
    pmsm_trajectory_monitor.target_counts_s = target_counts_s;
    pmsm_trajectory_monitor.active = dirty = true;
    return 0;
}

int pmsm_trajectory_set_position(float target_count)
{
    if (!trajectory_finite(target_count) || trajectory_abs(target_count) > MOTOR_POSITION_MAX_COMMAND_COUNTS) return -1;
    if (pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_POSITION && pmsm_trajectory_monitor.cycle_destination_count == target_count &&
        (pmsm_trajectory_monitor.active || cycle_config.mode == PMSM_TRAJECTORY_SINGLE)) return 0;
    pmsm_trajectory_monitor.mode = PMSM_TRAJECTORY_POSITION;
    pmsm_trajectory_monitor.cycle_origin_count = pmsm_trajectory_monitor.position_count;
    pmsm_trajectory_monitor.cycle_destination_count = target_count;
    pmsm_trajectory_monitor.cycling = cycle_config.mode == PMSM_TRAJECTORY_ROUND_TRIP &&
        target_count != pmsm_trajectory_monitor.cycle_origin_count;
    pmsm_trajectory_monitor.returning = pmsm_trajectory_monitor.dwelling = false;
    pmsm_trajectory_monitor.cycles_completed = pmsm_trajectory_monitor.dwell_ticks_remaining = 0;
    pmsm_trajectory_monitor.target_count = target_count;
    pmsm_trajectory_monitor.active = dirty = true;
    return 0;
}

static void trajectory_position_done(void) MOTOR_RAMFUNC;
static void trajectory_position_done(void)
{
    if (pmsm_trajectory_monitor.cycling && pmsm_trajectory_monitor.returning) {
        if (pmsm_trajectory_monitor.cycles_completed != UINT32_MAX) ++pmsm_trajectory_monitor.cycles_completed;
        if (cycle_config.cycle_count && pmsm_trajectory_monitor.cycles_completed >= cycle_config.cycle_count) {
            pmsm_trajectory_monitor.cycling = false;
            pmsm_trajectory_monitor.active = false;
            stage = TRAJ_IDLE;
            return;
        }
    }
    pmsm_trajectory_monitor.dwell_ticks_remaining = (cycle_config.dwell_ms * MOTOR_SPEED_LOOP_HZ + 999u) / 1000u;
    pmsm_trajectory_monitor.dwelling = true;
    stage = TRAJ_DWELL;
}

static void trajectory_dwell_done(void) MOTOR_RAMFUNC;
static void trajectory_dwell_done(void)
{
    pmsm_trajectory_monitor.dwelling = false;
    if (!pmsm_trajectory_monitor.cycling) {
        stage = TRAJ_IDLE; pmsm_trajectory_monitor.active = false; return;
    }
    pmsm_trajectory_monitor.returning = !pmsm_trajectory_monitor.returning;
    pmsm_trajectory_monitor.target_count = pmsm_trajectory_monitor.returning ?
        pmsm_trajectory_monitor.cycle_origin_count : pmsm_trajectory_monitor.cycle_destination_count;
    carry = 0;
    trajectory_position();
}

int pmsm_trajectory_step(void)
{
    unsigned transitions = 0;
    if (pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_OFF) return 0;
    if (dirty) {
        float a = pmsm_trajectory_monitor.acceleration_counts_s2;
        dirty = false; carry = 0;
        /* 改目标先连续地卸掉已有加速度，再减速/重规划，避免参考位置、速度跳变。 */
        if (trajectory_abs(a) > 0.0001f) {
            duration[0] = trajectory_abs(a) / trajectory_config.max_jerk_counts_s3;
            jerk[0] = a < 0 ? trajectory_config.max_jerk_counts_s3 : -trajectory_config.max_jerk_counts_s3;
            segment = 0; segments = 1; stage = TRAJ_NEUTRAL; trajectory_anchor();
        } else { pmsm_trajectory_monitor.acceleration_counts_s2 = 0; trajectory_plan(); }
    }
    ++pmsm_trajectory_monitor.updates;
    if (stage == TRAJ_DWELL) {
        if (pmsm_trajectory_monitor.dwell_ticks_remaining) {
            --pmsm_trajectory_monitor.dwell_ticks_remaining;
            if (pmsm_trajectory_monitor.dwell_ticks_remaining) return 0;
        }
        trajectory_dwell_done();
        return 0; /* 完整驻留拍不推进新段；下一拍开始运动。 */
    }
    ++ticks;
    if (stage == TRAJ_IDLE) {
        if (pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_SPEED)
            pmsm_trajectory_monitor.position_count = anchor_p + anchor_v * (ticks * dt + carry);
        return trajectory_finite(pmsm_trajectory_monitor.position_count) ? 0 : -1;
    }
    while (stage != TRAJ_IDLE && transitions++ < 16) {
        float elapsed = ticks * dt + carry;
        float t = elapsed < duration[segment] ? elapsed : duration[segment];
        float j = jerk[segment];
        float position = anchor_p + t * (anchor_v + t * (0.5f * anchor_a + t * j / 6));
        if (stage == TRAJ_POSITION) {
            /* 静止到静止本应单调；消除float分段求值在末端的亚计数反向舍入。 */
            if (jerk[0] > 0) {
                if (position < pmsm_trajectory_monitor.position_count) position = pmsm_trajectory_monitor.position_count;
                if (position > pmsm_trajectory_monitor.target_count) position = pmsm_trajectory_monitor.target_count;
            } else {
                if (position > pmsm_trajectory_monitor.position_count) position = pmsm_trajectory_monitor.position_count;
                if (position < pmsm_trajectory_monitor.target_count) position = pmsm_trajectory_monitor.target_count;
            }
        }
        pmsm_trajectory_monitor.position_count = position;
        pmsm_trajectory_monitor.speed_counts_s = anchor_v + t * (anchor_a + 0.5f * t * j);
        pmsm_trajectory_monitor.acceleration_counts_s2 = anchor_a + t * j;
        if (elapsed < duration[segment]) break;
        carry = elapsed - duration[segment];
        if (++segment == segments) {
            pmsm_trajectory_monitor.acceleration_counts_s2 = 0;
            if (stage == TRAJ_NEUTRAL) trajectory_plan();
            else if (stage == TRAJ_BRAKE) { pmsm_trajectory_monitor.speed_counts_s = 0; trajectory_position(); }
            else {
                if (stage == TRAJ_POSITION) {
                    pmsm_trajectory_monitor.position_count = pmsm_trajectory_monitor.target_count;
                    pmsm_trajectory_monitor.speed_counts_s = 0;
                } else pmsm_trajectory_monitor.speed_counts_s = pmsm_trajectory_monitor.target_counts_s;
                if (stage == TRAJ_POSITION) trajectory_position_done();
                else { stage = TRAJ_IDLE; pmsm_trajectory_monitor.active = false; }
            }
        }
        trajectory_anchor();
        if (stage == TRAJ_DWELL) {
            carry = 0;
            if (pmsm_trajectory_monitor.dwell_ticks_remaining) break;
            trajectory_dwell_done();
        }
    }
    if (stage == TRAJ_IDLE && pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_SPEED)
        pmsm_trajectory_monitor.position_count = anchor_p + anchor_v * carry;
    if (transitions > 16 || !trajectory_finite(pmsm_trajectory_monitor.position_count) ||
        !trajectory_finite(pmsm_trajectory_monitor.speed_counts_s) || !trajectory_finite(pmsm_trajectory_monitor.acceleration_counts_s2)) return -1;
    return 0;
}
