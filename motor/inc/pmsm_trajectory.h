/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_TRAJECTORY_H
#define PMSM_TRAJECTORY_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_rt.h"

typedef enum {
    PMSM_TRAJECTORY_OFF, PMSM_TRAJECTORY_SPEED, PMSM_TRAJECTORY_POSITION
} pmsm_trajectory_mode_t;
typedef struct {
    pmsm_trajectory_mode_t mode;
    bool active; /* 参考轨迹尚未结束；不代表实际电机已经到位。 */
    uint32_t updates;
    float target_count, target_counts_s;
    float position_count, speed_counts_s, acceleration_counts_s2;
} pmsm_trajectory_status_t;
extern volatile pmsm_trajectory_status_t pmsm_trajectory_monitor;
int pmsm_trajectory_init(void);
/* 与外环相同的1kHz串行上下文；全部使用电机轴计数，无角度/rpm换算。 */
void pmsm_trajectory_reset(float position_count, float speed_counts_s) MOTOR_RAMFUNC;
int pmsm_trajectory_set_speed(float target_counts_s) MOTOR_RAMFUNC;
int pmsm_trajectory_set_position(float target_count) MOTOR_RAMFUNC;
int pmsm_trajectory_step(void) MOTOR_RAMFUNC;
#endif
