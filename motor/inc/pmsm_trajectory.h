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
    float position_max_speed_counts_s;
    float max_accel_counts_s2;
    float max_jerk_counts_s3;
} pmsm_trajectory_config_t;

enum {
    PMSM_TRAJECTORY_OK = 0,
    PMSM_TRAJECTORY_INVALID_CONFIG = -1,
    PMSM_TRAJECTORY_BUSY = -2,
    PMSM_TRAJECTORY_NOT_INITIALIZED = -3
};
typedef struct {
    pmsm_trajectory_mode_t mode;
    bool active; /* 参考轨迹尚未结束；不代表实际电机已经到位。 */
    uint32_t updates;
    float target_count, target_counts_s;
    float position_count, speed_counts_s, acceleration_counts_s2;
} pmsm_trajectory_status_t;
extern volatile pmsm_trajectory_status_t pmsm_trajectory_monitor;
int pmsm_trajectory_init(void);
/* 前台调用：0成功，-1无效参数，-2桥已使能，-3尚未初始化。
 * 一次更新整组参数；失败保留旧配置。STOP/reset保留配置，上电init重新载入宏默认值。
 * 桥使能期间（包括匀速、零速保持、对齐、电流模式）均拒绝修改。 */
int pmsm_trajectory_set_config(const pmsm_trajectory_config_t *config) MOTOR_RAMFUNC;
int pmsm_trajectory_get_config(pmsm_trajectory_config_t *config) MOTOR_RAMFUNC;
/* 与外环相同的1kHz串行上下文；全部使用电机轴计数，无角度/rpm换算。 */
void pmsm_trajectory_reset(float position_count, float speed_counts_s) MOTOR_RAMFUNC;
int pmsm_trajectory_set_speed(float target_counts_s) MOTOR_RAMFUNC;
int pmsm_trajectory_set_position(float target_count) MOTOR_RAMFUNC;
int pmsm_trajectory_step(void) MOTOR_RAMFUNC;
#endif
