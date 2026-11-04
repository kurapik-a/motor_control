/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_PROTECT_H
#define PMSM_PROTECT_H
#include <stdbool.h>
#include <stdint.h>
#include "motor_rt.h"

/* 前7位沿用原工程含义；过流同时检查正负方向及三相，而非只检查正Iq。 */
typedef enum {
    PMSM_FAULT_BUS_HIGH        = 1u << 0,
    PMSM_FAULT_BUS_LOW         = 1u << 1,
    PMSM_FAULT_OVERCURRENT     = 1u << 2,
    PMSM_FAULT_OVERSPEED       = 1u << 3,
    PMSM_FAULT_ENCODER         = 1u << 4,
    PMSM_FAULT_TEMP_HIGH       = 1u << 5,
    PMSM_FAULT_TEMP_LOW        = 1u << 6,
    PMSM_FAULT_CURRENT_DATA    = 1u << 7,
    PMSM_FAULT_BUS_DATA        = 1u << 8,
    PMSM_FAULT_TEMP_DATA       = 1u << 9,
    PMSM_FAULT_DQ_DATA         = 1u << 10,
    PMSM_FAULT_NUMERIC         = 1u << 11,
    /* bit12保留，原未接入的速度误差保护已移除；其余故障位保持不变。 */
    PMSM_FAULT_CONFIG          = 1u << 13,
    PMSM_FAULT_INPUT           = 1u << 14
} pmsm_fault_t;

/* 由正式控制层填写一致快照，不依赖ADC或编码器驱动实现。
 * age为实际数据年龄，不是本次复制快照后的时间；valid不能凭raw非零推断。
 * 电流mA，速度为电机轴编码器计数/秒，辅助采样为16位原始码。 */
typedef struct {
    bool running; /* 实际功率输出/控制运行状态，不能用速度是否为零代替。 */
    bool current_valid;
    float iu_ma, iv_ma, iw_ma;
    uint32_t current_age_us;
    bool encoder_valid;
    float speed_counts_s;
    uint32_t encoder_age_ms;
    bool dq_valid;
    float id_ma, iq_ma;
    bool bus_valid, temperature_valid;
    uint16_t bus_raw, temperature_raw;
    uint32_t bus_age_ms, temperature_age_ms;
} pmsm_protect_input_t;

typedef struct {
    uint32_t active_faults;  /* 本次检查的问题，含未就绪/过旧。 */
    uint32_t latched_faults; /* 历次锁存故障；数据恢复不会自动清除。 */
    uint32_t first_faults;   /* 首次锁存时的故障组合，清除前保留。 */
    bool ready;             /* 本次启用的全部检查均通过。 */
} pmsm_protect_t;

/* 上电初始化；不得用重复init绕过运行故障，应走clear。 */
void pmsm_protect_init(pmsm_protect_t *protect);
/* 每次检查输入，返回active|latched；参数错误返回PMSM_FAULT_INPUT。
 * 关桥时缺少/过旧数据只阻止使能；有效数据超限仍锁存。
 * 运行时任何启用的检查失败都锁存。无硬件副作用，不自动关桥或复位PI。
 * 同一状态仅由一个上下文访问；上层按每个完整ADC新帧调用。 */
uint32_t pmsm_protect_check(pmsm_protect_t *protect, const pmsm_protect_input_t *input) MOTOR_RAMFUNC;
/* 只查询最近检查结果；使能前必须以新鲜输入再次check。 */
bool pmsm_protect_can_enable(const pmsm_protect_t *protect) MOTOR_RAMFUNC;
/* 0=清除成功；-1=空指针；-2=仍在运行；-3=当前检查未通过。
 * 仅关桥且所有启用检查通过才清除；不恢复PWM、不重置ADC硬件故障。 */
int pmsm_protect_clear(pmsm_protect_t *protect, const pmsm_protect_input_t *input) MOTOR_RAMFUNC;
#endif
