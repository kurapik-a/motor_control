/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef PMSM_INIT_H
#define PMSM_INIT_H
#include <stdbool.h>
#include <stdint.h>

typedef enum {
    PMSM_INIT_OFF, PMSM_INIT_WAIT_SAMPLES, PMSM_INIT_ALIGNING,
    PMSM_INIT_READY, PMSM_INIT_CANCELLED, PMSM_INIT_FAULT
} pmsm_init_state_t;
typedef enum { PMSM_INIT_STAGE_NONE, PMSM_INIT_STAGE_PWM, PMSM_INIT_STAGE_ENCODER,
    PMSM_INIT_STAGE_ADC, PMSM_INIT_STAGE_CURRENT, PMSM_INIT_STAGE_PREPARE } pmsm_init_stage_t;
typedef struct {
    pmsm_init_state_t state;
    pmsm_init_stage_t stage;
    int result;
    uint32_t faults;
} pmsm_init_status_t;
extern volatile pmsm_init_status_t pmsm_init_monitor;

/* board/VOFA初始化后调用一次：配置外设、绑定控制回调，启动非阻塞准备。
 * 上电采样就绪后会自动给电机通电对齐；完成后关桥等待ENABLE。 */
int pmsm_init(void);
/* 主循环在adc_poll后持续调用；不得在中断中调用。 */
void pmsm_init_poll(void);
bool pmsm_is_ready(void);
/* STOP取消尚未完成的准备；普通停机保留已对齐零点。 */
void pmsm_init_cancel(void);
/* 取消/零点失效/准备故障清除后，显式重新准备；不会自动重试通电。
 * 0接受（或已经就绪），-1未初始化，-2忙，-4故障未清除。 */
int pmsm_prepare(void);
#endif
