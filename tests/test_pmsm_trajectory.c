#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "pmsm_trajectory.h"
#include "motor_config.h"
#define CHECK(x) do { if (!(x)) { printf("FAIL trajectory line %d: %s\n", __LINE__, #x); exit(1); } } while (0)
static unsigned samples;
static pmsm_trajectory_config_t limits;
static bool bridge_enabled;
static uint32_t irq_depth;
bool pwm_is_enabled(void) { return bridge_enabled; }
uint32_t disable_global_irq(uint32_t mask) { (void)mask; return irq_depth++; }
void restore_global_irq(uint32_t previous) { CHECK(irq_depth==previous+1); irq_depth=previous; }
static void tick(float speed_limit)
{
    float a = pmsm_trajectory_monitor.acceleration_counts_s2;
    float v = pmsm_trajectory_monitor.speed_counts_s;
    float p = pmsm_trajectory_monitor.position_count;
    float dt = 1.0f / MOTOR_SPEED_LOOP_HZ;
    CHECK(pmsm_trajectory_step() == 0);
    CHECK(fabsf(pmsm_trajectory_monitor.acceleration_counts_s2) <= limits.max_accel_counts_s2 + 0.1f);
    CHECK(fabsf(pmsm_trajectory_monitor.acceleration_counts_s2-a) <= limits.max_jerk_counts_s3*dt + 0.1f);
    CHECK(fabsf(pmsm_trajectory_monitor.speed_counts_s-v) <= limits.max_accel_counts_s2*dt + 0.1f);
    CHECK(fabsf(pmsm_trajectory_monitor.speed_counts_s) <= speed_limit + 0.1f);
    /* 极端±400万计数跨越时，float位置求值包含大数相加的舍入误差。 */
    CHECK(fabsf(pmsm_trajectory_monitor.position_count-p) <= speed_limit*dt + 0.01f + 4*MOTOR_POSITION_MAX_COMMAND_COUNTS*FLT_EPSILON);
    ++samples;
}
static void finish(float speed_limit)
{
    unsigned i;
    for (i=0; pmsm_trajectory_monitor.active && i<2000000; ++i) tick(speed_limit);
    CHECK(!pmsm_trajectory_monitor.active);
    CHECK(pmsm_trajectory_monitor.acceleration_counts_s2 == 0);
}
static void position(float start, float goal)
{
    float last = start;
    pmsm_trajectory_reset(start, 0);
    CHECK(pmsm_trajectory_set_position(goal) == 0);
    while (pmsm_trajectory_monitor.active) {
        tick(limits.position_max_speed_counts_s);
        if (goal >= start ? pmsm_trajectory_monitor.position_count < last-0.001f : pmsm_trajectory_monitor.position_count > last+0.001f)
            printf("non-monotone start=%f goal=%f previous=%f next=%f velocity=%f acceleration=%f active=%d\n",start,goal,last,pmsm_trajectory_monitor.position_count,pmsm_trajectory_monitor.speed_counts_s,pmsm_trajectory_monitor.acceleration_counts_s2,pmsm_trajectory_monitor.active);
        CHECK(goal >= start ? pmsm_trajectory_monitor.position_count >= last-0.001f : pmsm_trajectory_monitor.position_count <= last+0.001f);
        last = pmsm_trajectory_monitor.position_count;
    }
    CHECK(last == goal && pmsm_trajectory_monitor.speed_counts_s == 0);
}
static void cycle_tests(void)
{
    pmsm_trajectory_cycle_config_t config, saved;
    unsigned i, legs=0, held=0;
    bool was_dwell=false;
    config.mode=PMSM_TRAJECTORY_ROUND_TRIP; config.cycle_count=2; config.dwell_ms=7;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==0);
    CHECK(pmsm_trajectory_get_cycle_config(&saved)==0 && saved.cycle_count==2);
    bridge_enabled=true;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==PMSM_TRAJECTORY_BUSY);
    bridge_enabled=false;
    config.mode=(pmsm_trajectory_cycle_mode_t)9;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    config=saved; config.dwell_ms=UINT32_MAX;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    CHECK(pmsm_trajectory_set_cycle_config(NULL)==PMSM_TRAJECTORY_INVALID_CONFIG);
    CHECK(pmsm_trajectory_get_cycle_config(NULL)==PMSM_TRAJECTORY_INVALID_CONFIG);
    pmsm_trajectory_reset(-100,0);
    CHECK(pmsm_trajectory_set_position(100)==0);
    for(i=0; i<200000 && pmsm_trajectory_monitor.active; ++i) {
        if(pmsm_trajectory_monitor.dwelling) {
            CHECK(pmsm_trajectory_monitor.speed_counts_s==0 && pmsm_trajectory_monitor.acceleration_counts_s2==0);
            CHECK(pmsm_trajectory_monitor.position_count==pmsm_trajectory_monitor.target_count);
            if(!was_dwell) { ++legs; held=0; }
            ++held;
        }
        was_dwell=pmsm_trajectory_monitor.dwelling;
        tick(limits.position_max_speed_counts_s);
        if(was_dwell && !pmsm_trajectory_monitor.dwelling) CHECK(held==7);
    }
    CHECK(!pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.position_count==-100);
    CHECK(pmsm_trajectory_monitor.cycles_completed==2 && legs==3);
    CHECK(pmsm_trajectory_set_position(100)==0 && pmsm_trajectory_monitor.active); /* 同目标可重启完整往返。 */
    finish(limits.position_max_speed_counts_s);
    config=saved; config.cycle_count=0; config.dwell_ms=0;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==0);
    pmsm_trajectory_reset(50,0); CHECK(pmsm_trajectory_set_position(-50)==0);
    for(i=0;i<200000 && pmsm_trajectory_monitor.cycles_completed<3;++i)tick(limits.position_max_speed_counts_s);
    CHECK(pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.cycles_completed==3);
    pmsm_trajectory_reset(12,0); /* STOP/fault调用此接口，取消无限循环。 */
    for(i=0;i<100;++i)tick(limits.position_max_speed_counts_s);
    CHECK(!pmsm_trajectory_monitor.active && !pmsm_trajectory_monitor.cycling && pmsm_trajectory_monitor.position_count==12);
    CHECK(pmsm_trajectory_set_position(12)==0);finish(limits.position_max_speed_counts_s); /* 零行程不循环。 */
    config=saved; config.dwell_ms=100;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==0);
    CHECK(pmsm_trajectory_set_position(100)==0);
    for(i=0;i<200000 && !pmsm_trajectory_monitor.dwelling;++i)tick(limits.position_max_speed_counts_s);
    CHECK(pmsm_trajectory_monitor.dwelling);
    CHECK(pmsm_trajectory_set_position(200)==0 && !pmsm_trajectory_monitor.dwelling && pmsm_trajectory_monitor.cycles_completed==0);
    CHECK(pmsm_trajectory_monitor.cycle_origin_count==100);
    CHECK(pmsm_trajectory_set_speed(0)==0 && !pmsm_trajectory_monitor.cycling);
    finish(limits.position_max_speed_counts_s);
    config.mode=PMSM_TRAJECTORY_SINGLE; config.cycle_count=1; config.dwell_ms=5;
    CHECK(pmsm_trajectory_set_cycle_config(&config)==0);
    pmsm_trajectory_reset(0,0); CHECK(pmsm_trajectory_set_position(100)==0);
    for(i=0;i<200000 && !pmsm_trajectory_monitor.dwelling;++i)tick(limits.position_max_speed_counts_s);
    CHECK(pmsm_trajectory_monitor.dwelling && pmsm_trajectory_monitor.dwell_ticks_remaining==5);
    for(i=0;i<5;++i)tick(limits.position_max_speed_counts_s);
    CHECK(!pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.position_count==100);
    puts("PASS cycles: finite/infinite round trips, exact dwell ticks, retrigger, retarget, zero distance, cancellation, config gates");
}
int main(void)
{
    unsigned i, k;
    pmsm_trajectory_config_t config, readback;
    union { uint32_t bits; float value; } special;
    const unsigned retarget_ticks[] = {10, 90, 270, 810, 2430, 4100, 5000, 5900, 8000, 9500};
    config.position_max_speed_counts_s=1000; config.max_accel_counts_s2=1000; config.max_jerk_counts_s3=1000;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_NOT_INITIALIZED);
    CHECK(pmsm_trajectory_get_config(&readback)==PMSM_TRAJECTORY_NOT_INITIALIZED);
    CHECK(pmsm_trajectory_init() == 0);
    CHECK(pmsm_trajectory_get_config(&limits)==0);
    CHECK(limits.position_max_speed_counts_s==MOTOR_TRAJECTORY_POSITION_MAX_COUNTS_S);
    position(0, 0); position(0, 1); position(0, 100); position(0, 1000);
    position(0, 31000); position(31000, -31000);
    position(-MOTOR_POSITION_MAX_COMMAND_COUNTS, MOTOR_POSITION_MAX_COMMAND_COUNTS);
    position(MOTOR_POSITION_MAX_COMMAND_COUNTS, MOTOR_POSITION_MAX_COMMAND_COUNTS-1);
    for (k=0; k<sizeof(retarget_ticks)/sizeof(retarget_ticks[0]); ++k) {
        pmsm_trajectory_reset(0, 0);
        CHECK(pmsm_trajectory_set_position(31000) == 0);
        for (i=0; i<retarget_ticks[k]; ++i) tick(limits.position_max_speed_counts_s);
        CHECK(pmsm_trajectory_set_position(-12000) == 0);
        finish(limits.position_max_speed_counts_s);
        CHECK(pmsm_trajectory_monitor.position_count == -12000);
    }
    pmsm_trajectory_reset(0, 0);
    CHECK(pmsm_trajectory_set_speed(MOTOR_SPEED_CTRL_MAX_COUNTS_S) == 0);
    finish(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    for (i=0; i<100; ++i) tick(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    CHECK(pmsm_trajectory_set_speed(-MOTOR_SPEED_CTRL_MAX_COUNTS_S) == 0);
    for (i=0; i<20; ++i) tick(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    CHECK(pmsm_trajectory_set_speed(0) == 0);
    finish(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    CHECK(pmsm_trajectory_monitor.speed_counts_s == 0);
    pmsm_trajectory_reset(0, MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    CHECK(pmsm_trajectory_set_position(-1000) == 0);
    finish(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    CHECK(pmsm_trajectory_monitor.position_count == -1000 && pmsm_trajectory_monitor.speed_counts_s == 0);
    CHECK(pmsm_trajectory_set_speed(MOTOR_SPEED_CTRL_MAX_COUNTS_S+1) == -1);
    CHECK(pmsm_trajectory_set_position(MOTOR_POSITION_MAX_COMMAND_COUNTS+1) == -1);
    pmsm_trajectory_reset(123, 0);
    tick(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    CHECK(pmsm_trajectory_monitor.mode == PMSM_TRAJECTORY_OFF && !pmsm_trajectory_monitor.active && pmsm_trajectory_monitor.position_count == 123);
    CHECK(pmsm_trajectory_set_config(NULL)==PMSM_TRAJECTORY_INVALID_CONFIG);
    CHECK(pmsm_trajectory_get_config(NULL)==PMSM_TRAJECTORY_INVALID_CONFIG);
    config=limits; config.position_max_speed_counts_s=MOTOR_POSITION_OUTPUT_LIMIT_COUNTS_PER_TICK*MOTOR_SPEED_LOOP_HZ+1;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    config=limits; config.max_accel_counts_s2=0;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    config=limits; config.max_jerk_counts_s3=-1;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    special.bits=0x7f800000u;
    config=limits; config.max_accel_counts_s2=special.value;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    special.bits=0x7fc00000u;
    config=limits; config.max_jerk_counts_s3=special.value;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_INVALID_CONFIG);
    CHECK(pmsm_trajectory_get_config(&readback)==0);
    CHECK(readback.position_max_speed_counts_s==limits.position_max_speed_counts_s && readback.max_accel_counts_s2==limits.max_accel_counts_s2 && readback.max_jerk_counts_s3==limits.max_jerk_counts_s3);
    config=limits; config.position_max_speed_counts_s*=0.5f; config.max_accel_counts_s2*=0.5f; config.max_jerk_counts_s3*=0.5f;
    bridge_enabled=true;
    CHECK(pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_BUSY);
    CHECK(pmsm_trajectory_init()==PMSM_TRAJECTORY_BUSY);
    bridge_enabled=false;
    CHECK(pmsm_trajectory_set_config(&config)==0);
    CHECK(pmsm_trajectory_get_config(&limits)==0 && limits.max_accel_counts_s2==config.max_accel_counts_s2);
    position(0,31000); position(31000,-31000);
    CHECK(pmsm_trajectory_set_speed(1000)==0);
    finish(MOTOR_SPEED_CTRL_MAX_COUNTS_S);
    bridge_enabled=true; /* 匀速时active=false，仍必须拒绝修改。 */
    CHECK(!pmsm_trajectory_monitor.active && pmsm_trajectory_set_config(&config)==PMSM_TRAJECTORY_BUSY);
    bridge_enabled=false;
    pmsm_trajectory_reset(0,0);
    CHECK(pmsm_trajectory_get_config(&readback)==0 && readback.max_accel_counts_s2==config.max_accel_counts_s2);
    CHECK(irq_depth==0);
    cycle_tests();
    printf("PASS trajectory: %u samples, S-curve limits, short/long/signed moves, live retarget, speed reversal, cancellation\n",samples);
    return 0;
}
