/* 测试真实encoder_read_position函数体；模拟Z/PH读数及跨圈瞬间。 */
#include <stdint.h>
#include <stdbool.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#define CHECK(x) do {if(!(x)){fprintf(stderr,"FAIL encoder position line %d: %s\n",__LINE__,#x);exit(1);}}while(0)
#define ENCODER_COUNTS_PER_REV 4000u
#define ENCODER_DIRECTION (-1)
#define ENCODER_QEI_BASE 0
#define QEIV2_COUNT_PH_PHCNT_GET(x) ((x)&0x1fffffu)
enum {qeiv2_counter_type_z,qeiv2_counter_type_phase};
static bool initialized=true;
static uint32_t z_values[4],phase_values[2],z_index,phase_index;
static uint32_t qeiv2_get_current_count(int base,int type) {
    (void)base;
    if(type==qeiv2_counter_type_z){CHECK(z_index<4);return z_values[z_index++];}
    CHECK(phase_index<2);return phase_values[phase_index++];
}
#include "encoder_position_under_test.inc"
static void setup(uint32_t z0,uint32_t z1,uint32_t z2,uint32_t z3,uint32_t ph0,uint32_t ph1) {
    z_values[0]=z0;z_values[1]=z1;z_values[2]=z2;z_values[3]=z3;
    phase_values[0]=ph0;phase_values[1]=ph1;z_index=phase_index=0;
}
int main(void) {
    int32_t value=123;
    CHECK(encoder_read_position(0)==-1);
    initialized=false;CHECK(encoder_read_position(&value)==-1 && value==123);initialized=true;
    setup(0,0,0,0,3999,0);CHECK(encoder_read_position(&value)==0 && value==-3999);
    setup(UINT32_MAX,UINT32_MAX,0,0,3999,0);CHECK(encoder_read_position(&value)==0 && value==1);
    setup(0,1,1,1,3999,0);CHECK(encoder_read_position(&value)==0 && value==-4000 && phase_index==2);
    setup(0,UINT32_MAX,UINT32_MAX,UINT32_MAX,0,3999);CHECK(encoder_read_position(&value)==0 && value==1);
    setup(0,1,1,2,3999,0);value=123;CHECK(encoder_read_position(&value)==-2 && value==123);
    setup(0,0,0,0,4000,0);CHECK(encoder_read_position(&value)==-2 && value==123);
    setup(INT32_MAX,INT32_MAX,0,0,0,0);CHECK(encoder_read_position(&value)==-3 && value==123);
    setup(536870,536870,0,0,3648,0);CHECK(encoder_read_position(&value)==0 && value==INT32_MIN);
    setup((uint32_t)-536871,(uint32_t)-536871,0,0,353,0);CHECK(encoder_read_position(&value)==0 && value==INT32_MAX);
    puts("PASS real encoder position: signed Z, phase rollover/retry, inconsistent reads, int32 bounds, invalid input");
    return 0;
}
