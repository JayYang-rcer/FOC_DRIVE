#include "foc_math.h"
#include "adc.h"
#include "main.h"
#include "vofa.h"

_RAM_FUNC void SinCosVal(foc_param_t *foc)
{
    foc->sin_val = sin_f32(foc->theta);
    foc->cos_val = cos_f32(foc->theta);
}

_RAM_FUNC void Clarke(foc_param_t *foc)
{
    foc->i_alpha = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

_RAM_FUNC void Park(foc_param_t *foc)
{
    foc->i_d =  foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = -foc->i_alpha * foc->sin_val + foc->i_beta * foc->cos_val;
}

_RAM_FUNC void InvPark(foc_param_t *foc)
{
    foc->v_alpha = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    foc->v_beta  = foc->v_q * foc->cos_val + foc->v_d * foc->sin_val;
}

_RAM_FUNC void InvClarke(foc_param_t *foc)
{
    foc->v_a = foc->v_alpha;
    foc->v_b = -0.5f * foc->v_alpha + SQRT3_BY_2 * foc->v_beta;
    foc->v_c = -0.5f * foc->v_alpha - SQRT3_BY_2 * foc->v_beta;
}

/**
* @brief:      svpwm_sector(foc_para_t *foc_param)
* @param[in]:  foc  FOC参数结构体指针
* @retval:     void
* @details:    SVPWM扇区判断
*/
_RAM_FUNC int SvpwmSector(foc_param_t *foc)
{
    uint8_t N,A,B,C;
    float TS = 1.f;
    float ta = 0.0f, tb = 0.0f, tc = 0.0f;
    float k = (TS*SQRT3)/(foc->vbus);

    float tx,ty;
    float temp=0;

    float U1 = foc->v_beta;
    float U2 = (SQRT3 *foc->v_alpha - foc->v_beta) * 0.5f;
    float U3 = -(SQRT3 *foc->v_alpha + foc->v_beta) * 0.5f;
    // InvClarke(foc_param);

    if(U1>0) A=1; else A=0;
    if(U2>0) B=1; else B=0;
    if(U3>0) C=1; else C=0;

    N = A + B*2 + C*4;
    int sector=0;

    switch (N)
    {
        case 1: sector=2; break;
        case 2: sector=6; break;
        case 3: sector=1; break;
        case 4: sector=4; break;
        case 5: sector=3; break;
        case 6: sector=5; break;
        default: break;
    }

    switch (sector)
    {
        case 1: tx=U2*k; ty=U1*k; break;
        case 2: tx=-U2*k; ty=-U3*k; break;
        case 3: tx=U1*k; ty=U3*k; break;
        case 4: tx=-U1*k; ty=-U2*k; break;
        case 5: tx=U3*k; ty=U2*k; break;
        case 6: tx=-U3*k; ty=-U1*k; break;
    }

    //过调制处理
    if(tx + ty > TS)
    {
        temp = tx+ty;
        tx=tx/temp*TS;
        ty=ty/temp*TS;
    }

    ta = (TS + tx + ty) * 0.25f;
    tb = ta - tx * 0.5f;
    tc = tb - ty * 0.5f;

    switch (sector)
    {
        case 1: foc->dtc_a = ta; foc->dtc_b = tb; foc->dtc_c = tc; break;
        case 2: foc->dtc_a = tb; foc->dtc_b = ta; foc->dtc_c = tc; break;
        case 3: foc->dtc_a = tc; foc->dtc_b = ta; foc->dtc_c = tb; break;
        case 4: foc->dtc_a = tc; foc->dtc_b = tb; foc->dtc_c = ta; break;
        case 5: foc->dtc_a = tb; foc->dtc_b = tc; foc->dtc_c = ta; break;
        case 6: foc->dtc_a = ta; foc->dtc_b = tc; foc->dtc_c = tb; break;
        default: break;
    }

	// if any of the results becomes NaN, result_valid will evaluate to false
    int result_valid = foc->dtc_a >= 0.0f && foc->dtc_a <= 1.0f &&
                       foc->dtc_b >= 0.0f && foc->dtc_b <= 1.0f &&
                       foc->dtc_c >= 0.0f && foc->dtc_c <= 1.0f;

    return result_valid ? 0 : -1;
}


smo_param_t smo;
pll_t pll_smo;

#define RC 1.f/(M_2PI * 50)
#define DT 1.f/10000
float alpha;

void SmoParamInit(smo_param_t *smo)
{
    smo->A = expf(-(motor_cfg.rs/1000)/(motor_cfg.ls/1000000)/10000);
    smo->B = (1 - smo->A)/(motor_cfg.rs/1000);
    smo->ksw = 0.13f;
    pll_smo.loop_hz = 10000;
    pll_smo.kp = 7.f*5000.f/60.f*M_2PI * 0.707f * 2.f;
    pll_smo.ki = (7.f*5000.f/60.f*M_2PI) * (7.f*5000.f/60.f*M_2PI);
    alpha = DT / (RC + DT);  // 计算滤波系数
}

_RAM_FUNC float AnglePll(smo_param_t* param, pll_t* pll)
{
    static float omega_out;
    pll->ref = -param->Ealpha * cos_f32(pll->angle_out);
    pll->fbk = sin_f32(pll->angle_out) * param->Ebeta;

    pll->p_term = (pll->ref - pll->fbk)*pll->kp;
    pll->i_term += (pll->ref - pll->fbk)*pll->ki/pll->loop_hz;
    pll->out_value = pll->p_term + pll->i_term;

    omega_out = pll->out_value*0.1f + omega_out*(1.f-0.1f);
    omega_out = pll->out_value;

    pll->angle_out += pll->out_value/pll->loop_hz;
    WRAP_0_2PI(pll->angle_out)

    float out = pll->angle_out + 0.7f;
//    float out = pll->angle_out + sqrtf(param->Ealpha*param->Ealpha + param->Ebeta*param->Ebeta)/(motor_cfg.flux/1000);
    return WRAP_0_2PI(out);
}


_RAM_FUNC float SmoViewer(foc_param_t *foc, smo_param_t *smo)
{
    static float valpha_last, vbeta_last;
    float ualpha,ubeta;

    ualpha = (2*foc->v_a-foc->v_b-foc->v_c)/3.f;
    ubeta  = (foc->v_b - foc->v_c) * ONE_BY_SQRT3;
    Clarke(foc);

    //计算预测电流
    if(motor_cfg.rotor_rev <1500)
    {
        smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (foc->v_alpha - smo->valpah);
        smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (foc->v_beta - smo->vbeta);
    }
    else
    {
        if(motor_cfg.rotor_vel <500)
        {
            smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (foc->v_alpha - smo->valpah);
            smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (foc->v_beta - smo->vbeta);
        }
        else
        {
            //貌似使用端电压采样在高速情况下比给定电压采样更好
            smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (ualpha - smo->valpah);
            smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (ubeta - smo->vbeta);
        }
    }

    //计算电动势观测值
    smo->valpah = smo->ksw*SIGN(smo->ialpha_view - foc->i_alpha);
    smo->vbeta = smo->ksw*SIGN(smo->ibeta_view - foc->i_beta);
//    smo->valpah = smo->ksw*sat1_datf(smo->ialpha_view - foc_param->i_alpha, 0.5);
//    smo->vbeta = smo->ksw*sat1_datf(smo->ibeta_view - foc_param->i_beta,0.5);

    //计算滤波后的拓展反电动势
    smo->valpah = alpha*smo->valpah + (1-alpha)*valpha_last;
    smo->vbeta =  alpha*smo->vbeta +  (1-alpha)*vbeta_last;

    //更新数据
    smo->ialpha_view_last = smo->ialpha_view;
    smo->ibeta_view_last = smo->ibeta_view;
    vbeta_last = smo->vbeta;
    valpha_last = smo->valpah;

    smo->Ealpha = smo->valpah;
    smo->Ebeta = smo->vbeta;

    return AnglePll(smo,&pll_smo);
}

