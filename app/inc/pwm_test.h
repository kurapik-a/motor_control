/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef MOTOR_PWM_TEST_H
#define MOTOR_PWM_TEST_H
#include <stdint.h>
#include <stdbool.h>
/* 开环三相正弦验证；不使用位置/速度/电流 PI，不保证转子跟随给定频率。 */
typedef enum {
    PWM_TEST_IDLE = 0, PWM_TEST_RUNNING = 1, PWM_TEST_FINISHED = 2, PWM_TEST_STOPPED = 3,
    PWM_TEST_BAD_ARGUMENT = -1, PWM_TEST_ADC_NOT_READY = -2,
    PWM_TEST_OVERCURRENT = -3, PWM_TEST_DRIVER_ERROR = -4,
    PWM_TEST_ADC_STALE = -5, PWM_TEST_BUSY = -6
} pwm_test_result_t;
/* 首次过流现场：STOP 后保留，下次有效参数的启动尝试开始时清除。 */
typedef struct {
    bool valid, during_run;
    uint32_t source_mask; /* 位1=U、位2=V、位4=重建W、位8=非有限电流；可叠加。 */
    float limit_ma, current_u_ma, current_v_ma, current_w_ma;
    uint16_t raw_u, raw_v, offset_u, offset_v;
    uint32_t sample_count, sample_age_us, irq_ticks, elapsed_ms;
} pwm_test_trip_t;
typedef struct {
    int result;
    bool output_enabled;
    uint32_t reload, irq_ticks, elapsed_ms, waveform_updates;
    float electrical_hz, amplitude, phase_deg, duty_u, duty_v, duty_w, peak_current_ma;
    pwm_test_trip_t trip;
} pwm_test_monitor_t;
/* 调试器修改：0=无请求，1=默认正转测试，2=默认反转测试，3=立即停止。
 * 请求由主循环消费并归零；标定完成后才接受启动，禁止改程序让它上电自动使能。 */
extern volatile uint32_t pwm_test_request;
extern volatile pwm_test_monitor_t pwm_test_monitor;
/* 仅主循环调用。电角频率可正负，amplitude 为占空比偏离 0.5 的峰值，例如 0.01。
 * 返回 0=启动成功；负值见枚举。运行中重复启动返回 BUSY，原测试继续运行。 */
int pwm_test_start(float electrical_hz, float amplitude, uint32_t duration_ms);
void pwm_test_poll(void); /* main 中 adc_poll 后持续调用；本函数不触发 ADC。 */
void pwm_test_stop(void); /* 主循环调用；关闭输出、解绑测试回调。 */
#endif
