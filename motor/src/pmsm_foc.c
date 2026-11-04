#if defined(__GNUC__)
#pragma GCC optimize ("O2")
#endif
/* Copyright (c) 2025 hpmicro. SPDX-License-Identifier: BSD-3-Clause */
/* 从原工程 pmsm_currentctrl_svpwm.c 拆出纯算法；不依赖旧 MOTOR_PARA。 */
#include "pmsm_foc.h"
#include <float.h>

#if HPM_MOTOR_MATH_MOD != HPM_MOTOR_MATH_MOD_FP
#error "This port currently supports floating-point motor math only"
#endif
MOTOR_INLINE int foc_finite(float value) { return value >= -FLT_MAX && value <= FLT_MAX; }

int pmsm_svpwm(float alpha, float beta, uint32_t reload, pmsm_pwm_t *output)
{
    pmsm_pwm_t result;
    if (!output || reload < 4u || reload > 0xffffffu ||
        !foc_finite(alpha) || !foc_finite(beta) ||
        (alpha > 2.0f * reload || alpha < -2.0f * reload) || (beta > 2.0f * reload || beta < -2.0f * reload)) return -1;
    int32_t ualpha_60, ubeta_30;
    uint32_t pwm_reload;
    int32_t uref1, uref2, uref3;
    int32_t tx, ty, t0;
    int32_t tuon, tvon, twon = 0;
    int8_t sector = 0;

    uref1 = HPM_MOTOR_MATH_MDF_FL(beta);
    ualpha_60 = HPM_MOTOR_MATH_MDF_FL(alpha);
    ubeta_30 = HPM_MOTOR_MATH_MDF_FL(beta);
    ualpha_60 = (int32_t)(((int64_t)ualpha_60 * 14189) >> 14);
    ubeta_30 = ubeta_30 >> 1;
    uref2 = HPM_MOTOR_MATH_MDF_FL(ualpha_60 - ubeta_30);
    uref3 = HPM_MOTOR_MATH_MDF_FL(-ualpha_60 - ubeta_30);
    pwm_reload = reload;

    if (uref1 >= 0)
        sector = 1;
    if (uref2 >= 0)
        sector = sector + 2;
    if (uref3 >= 0)
        sector = sector + 4;

    switch (sector) {
    case 1: /* sector2  000 010 110 111 110 010 000 */
        tx = -uref2;
        ty = -uref3;
        t0 = pwm_reload - tx - ty;
        /* tx + ty > T */
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        twon = t0 >> 1;
        tuon = ty + twon;
        tvon = tx + tuon;
        break;
    case 2: /* sector6  000   100 101 111 101 100 000 */
        tx = -uref3;
        ty = -uref1;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tvon = t0 >> 1;
        twon = ty + tvon;
        tuon = tx + twon;
        break;
    case 3: /* sector1  000 100 110 111 110 100 000 */
        tx = uref2;
        ty = uref1;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        twon = t0 >> 1;
        tvon = ty + twon;
        tuon = tx + tvon;
        break;
    case 4: /* sector4  000 001 011 111 011 001 000 */
        tx = -uref1;
        ty = -uref2;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tuon = t0 >> 1;
        tvon = ty + tuon;
        twon = tx + tvon;
        break;
    case 5: /* sector5 000 010 011 111 011 010 000 */
        tx = uref1;
        ty = uref3;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tuon = t0 >> 1;
        twon = ty + tuon;
        tvon = tx + twon;
        break;
    case 6: /* sector6 000 001 101 111 101 001 000 */
        tx = uref3;
        ty = uref2;
        t0 = pwm_reload - tx - ty;
        if (t0 < 0) {
            tx = ((float)tx / (tx + ty)) * pwm_reload;
            ty = pwm_reload - tx;
            t0 = 0;
        }
        tvon = t0 >> 1;
        tuon = ty + tvon;
        twon = tx + tuon;
        break;

    default:
        tuon = (int)(pwm_reload/2);
        tvon = (int)(pwm_reload/2);
        twon = (int)(pwm_reload/2);
    }
    result.sector = sector;

    if (tuon < 0)
        tuon = 0;
    if (tvon < 0)
        tvon = 0;
    if (twon < 0)
        twon = 0;
    if (tuon > (int32_t)pwm_reload) tuon = pwm_reload;
    if (tvon > (int32_t)pwm_reload) tvon = pwm_reload;
    if (twon > (int32_t)pwm_reload) twon = pwm_reload;

    result.u = tuon;
    result.v = tvon;
    result.w = twon;
    *output = result;
    return 0;
}


/* 非const表放DLM，由启动代码初始化；实时路径不从XIP取查表数据。 */
static float foc_sine_quarter[257] = {
    0.0000000000f, 0.0061358846f, 0.0122715383f, 0.0184067299f, 0.0245412285f, 0.0306748032f, 0.0368072229f, 0.0429382569f,
    0.0490676743f, 0.0551952443f, 0.0613207363f, 0.0674439196f, 0.0735645636f, 0.0796824380f, 0.0857973123f, 0.0919089565f,
    0.0980171403f, 0.1041216339f, 0.1102222073f, 0.1163186309f, 0.1224106752f, 0.1284981108f, 0.1345807085f, 0.1406582393f,
    0.1467304745f, 0.1527971853f, 0.1588581433f, 0.1649131205f, 0.1709618888f, 0.1770042204f, 0.1830398880f, 0.1890686641f,
    0.1950903220f, 0.2011046348f, 0.2071113762f, 0.2131103199f, 0.2191012402f, 0.2250839114f, 0.2310581083f, 0.2370236060f,
    0.2429801799f, 0.2489276057f, 0.2548656596f, 0.2607941179f, 0.2667127575f, 0.2726213554f, 0.2785196894f, 0.2844075372f,
    0.2902846773f, 0.2961508882f, 0.3020059493f, 0.3078496400f, 0.3136817404f, 0.3195020308f, 0.3253102922f, 0.3311063058f,
    0.3368898534f, 0.3426607173f, 0.3484186802f, 0.3541635254f, 0.3598950365f, 0.3656129978f, 0.3713171940f, 0.3770074102f,
    0.3826834324f, 0.3883450467f, 0.3939920401f, 0.3996241998f, 0.4052413140f, 0.4108431711f, 0.4164295601f, 0.4220002708f,
    0.4275550934f, 0.4330938189f, 0.4386162385f, 0.4441221446f, 0.4496113297f, 0.4550835871f, 0.4605387110f, 0.4659764958f,
    0.4713967368f, 0.4767992301f, 0.4821837721f, 0.4875501601f, 0.4928981922f, 0.4982276670f, 0.5035383837f, 0.5088301425f,
    0.5141027442f, 0.5193559902f, 0.5245896827f, 0.5298036247f, 0.5349976199f, 0.5401714727f, 0.5453249884f, 0.5504579729f,
    0.5555702330f, 0.5606615762f, 0.5657318108f, 0.5707807459f, 0.5758081914f, 0.5808139581f, 0.5857978575f, 0.5907597019f,
    0.5956993045f, 0.6006164794f, 0.6055110414f, 0.6103828063f, 0.6152315906f, 0.6200572118f, 0.6248594881f, 0.6296382389f,
    0.6343932842f, 0.6391244449f, 0.6438315429f, 0.6485144010f, 0.6531728430f, 0.6578066933f, 0.6624157776f, 0.6669999223f,
    0.6715589548f, 0.6760927036f, 0.6806009978f, 0.6850836678f, 0.6895405447f, 0.6939714609f, 0.6983762494f, 0.7027547445f,
    0.7071067812f, 0.7114321957f, 0.7157308253f, 0.7200025080f, 0.7242470830f, 0.7284643904f, 0.7326542717f, 0.7368165689f,
    0.7409511254f, 0.7450577854f, 0.7491363945f, 0.7531867990f, 0.7572088465f, 0.7612023855f, 0.7651672656f, 0.7691033376f,
    0.7730104534f, 0.7768884657f, 0.7807372286f, 0.7845565972f, 0.7883464276f, 0.7921065773f, 0.7958369046f, 0.7995372691f,
    0.8032075315f, 0.8068475535f, 0.8104571983f, 0.8140363297f, 0.8175848132f, 0.8211025150f, 0.8245893028f, 0.8280450453f,
    0.8314696123f, 0.8348628750f, 0.8382247056f, 0.8415549774f, 0.8448535652f, 0.8481203448f, 0.8513551931f, 0.8545579884f,
    0.8577286100f, 0.8608669386f, 0.8639728561f, 0.8670462455f, 0.8700869911f, 0.8730949784f, 0.8760700942f, 0.8790122264f,
    0.8819212643f, 0.8847970984f, 0.8876396204f, 0.8904487232f, 0.8932243012f, 0.8959662498f, 0.8986744657f, 0.9013488470f,
    0.9039892931f, 0.9065957045f, 0.9091679831f, 0.9117060320f, 0.9142097557f, 0.9166790599f, 0.9191138517f, 0.9215140393f,
    0.9238795325f, 0.9262102421f, 0.9285060805f, 0.9307669611f, 0.9329927988f, 0.9351835099f, 0.9373390119f, 0.9394592236f,
    0.9415440652f, 0.9435934582f, 0.9456073254f, 0.9475855910f, 0.9495281806f, 0.9514350210f, 0.9533060404f, 0.9551411683f,
    0.9569403357f, 0.9587034749f, 0.9604305194f, 0.9621214043f, 0.9637760658f, 0.9653944417f, 0.9669764710f, 0.9685220943f,
    0.9700312532f, 0.9715038910f, 0.9729399522f, 0.9743393828f, 0.9757021300f, 0.9770281427f, 0.9783173707f, 0.9795697657f,
    0.9807852804f, 0.9819638691f, 0.9831054874f, 0.9842100924f, 0.9852776424f, 0.9863080972f, 0.9873014182f, 0.9882575677f,
    0.9891765100f, 0.9900582103f, 0.9909026354f, 0.9917097537f, 0.9924795346f, 0.9932119492f, 0.9939069700f, 0.9945645707f,
    0.9951847267f, 0.9957674145f, 0.9963126122f, 0.9968202993f, 0.9972904567f, 0.9977230666f, 0.9981181129f, 0.9984755806f,
    0.9987954562f, 0.9990777278f, 0.9993223846f, 0.9995294175f, 0.9996988187f, 0.9998305818f, 0.9999247018f, 0.9999811753f,
    1.0000000000f
};
MOTOR_INLINE float foc_sine(float degree)
{
    unsigned quadrant = (unsigned)(degree / 90.0f);
    float x = (degree - quadrant * 90.0f) * (256.0f / 90.0f);
    unsigned index;
    float value;
    if (quadrant & 1u) x = 256.0f - x;
    index = (unsigned)x;
    value = index >= 256u ? foc_sine_quarter[256] :
        foc_sine_quarter[index] + (x-index) * (foc_sine_quarter[index+1]-foc_sine_quarter[index]);
    return quadrant >= 2u ? -value : value;
}
int pmsm_sincos(float angle_deg, pmsm_trig_t *trig)
{
    float angle;
    if (!trig || !foc_finite(angle_deg) || angle_deg < -360000.0f || angle_deg > 360000.0f) return -1;
    angle = angle_deg - (int32_t)(angle_deg / 360.0f) * 360.0f;
    if (angle < 0) angle += 360.0f;
    trig->sine = foc_sine(angle);
    angle += 90.0f;
    if (angle >= 360.0f) angle -= 360.0f;
    trig->cosine = foc_sine(angle);
    return 0;
}
int pmsm_clarke(float iu, float iv, pmsm_ab_t *ab)
{
    float beta = (iu + iv + iv) * 0.57735026919f;
    if (!ab || !foc_finite(iu) || !foc_finite(iv) || !foc_finite(beta)) return -1;
    ab->alpha = iu; ab->beta = beta;
    return 0;
}
int pmsm_park(const pmsm_ab_t *ab, const pmsm_trig_t *trig, pmsm_dq_t *dq)
{
    float d, q;
    if (!ab || !trig || !dq) return -1;
    d = trig->cosine * ab->alpha + trig->sine * ab->beta;
    q = -trig->sine * ab->alpha + trig->cosine * ab->beta;
    if (!foc_finite(d) || !foc_finite(q)) return -1;
    dq->d = d; dq->q = q;
    return 0;
}
int pmsm_inv_park(const pmsm_dq_t *dq, const pmsm_trig_t *trig, pmsm_ab_t *ab)
{
    float alpha, beta;
    if (!ab || !trig || !dq) return -1;
    alpha = trig->cosine * dq->d - trig->sine * dq->q;
    beta = trig->sine * dq->d + trig->cosine * dq->q;
    if (!foc_finite(alpha) || !foc_finite(beta)) return -1;
    ab->alpha = alpha; ab->beta = beta;
    return 0;
}
int pmsm_voltage_to_pwm(float ud, float uq, float angle_deg, uint32_t reload, pmsm_pwm_t *output)
{
    pmsm_trig_t trig;
    pmsm_ab_t ab;
    pmsm_dq_t dq;
    dq.d = ud; dq.q = uq;
    if (pmsm_sincos(angle_deg, &trig) || pmsm_inv_park(&dq, &trig, &ab)) return -1;
    return pmsm_svpwm(ab.alpha, ab.beta, reload, output);
}
