/* 独立验证控制单位、积分饱和行为及SVPWM线电压，不依赖芯片。 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <float.h>
#include "pid.h"
#define fmodf(x,y) ((float)fmod((double)(x),(double)(y))) /* LCC运行库兼容 */
#include "../motor/src/pmsm_control.c"
#define CHECK(x) do { if (!(x)) {fprintf(stderr,"FAIL line %d: %s\n",__LINE__,#x);exit(1);} } while(0)
int main(void)
{
    pmsm_pi_t pi = {2.0f, 0.5f, 10.0f, 0, 0};
    pmsm_pwm_t pwm, saved;
    unsigned sectors = 0;
    int angle;
    union {uint32_t bits; float value;} nan_value;
    nan_value.bits = 0x7fc00000u;
    CHECK(pmsm_pi_step(&pi, 2, 0)==0 && pi.out==5 && pi.mem==1);
    CHECK(pmsm_pi_step(&pi, 100, 0)==0 && pi.out==10 && pi.mem==1);
    CHECK(pmsm_pi_step(&pi, -100, 0)==0 && pi.out==-10 && pi.mem==1);
    CHECK(pmsm_pi_step(&pi, 0, 2)==0 && pi.out==-4 && pi.mem==0);
    CHECK(pmsm_pi_step(&pi, nan_value.value, 0)==-1 && pi.out==-4);
    pmsm_pi_reset(&pi); CHECK(pi.mem==0 && pi.out==0);
    CHECK(pmsm_svpwm(0,0,10000,&pwm)==0 && pwm.u==5000 && pwm.v==5000 && pwm.w==5000);
    /* 以相等电压得到零线电压；反Park+SVPWM的线电压应满足解析关系。
     * 原算法先截断计数，14189/16384近似sqrt(3)/2，允许3计数误差。 */
    for(angle=-360;angle<720;++angle) {
        float r=angle*0.01745329251994329577f;
        float alpha=1000.0f*cosf(r), beta=1000.0f*sinf(r);
        CHECK(pmsm_voltage_to_pwm(1000,0,(float)angle,10000,&pwm)==0);
        CHECK(pwm.u<=10000 && pwm.v<=10000 && pwm.w<=10000);
        CHECK(fabsf(((float)pwm.u-pwm.v)-(0.86602540378f*alpha-0.5f*beta))<3.0f);
        CHECK(fabsf(((float)pwm.v-pwm.w)-beta)<3.0f);
        sectors |= 1u<<pwm.sector;
        CHECK(pmsm_voltage_to_pwm(0,19000,(float)angle,10000,&pwm)==0);
        CHECK(pwm.u<=10000 && pwm.v<=10000 && pwm.w<=10000);
        CHECK(pwm.u==0 || pwm.v==0 || pwm.w==0);
    }
    CHECK((sectors & 0x7e)==0x7e);
    saved=pwm;
    CHECK(pmsm_svpwm(nan_value.value,0,10000,&pwm)==-1);
    CHECK(pwm.u==saved.u && pwm.v==saved.v && pwm.w==saved.w);
    CHECK(pmsm_svpwm(FLT_MAX,0,10000,&pwm)==-1);
    CHECK(pmsm_svpwm(0,0,0,&pwm)==-1);
    CHECK(pmsm_voltage_to_pwm(1,1,nan_value.value,10000,&pwm)==-1);
    puts("PASS PI units/anti-windup/reset/invalid data; SVPWM six sectors/line voltage/angle wrap/overmodulation");
    return 0;
}
