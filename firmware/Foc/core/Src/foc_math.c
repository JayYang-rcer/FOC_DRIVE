#include "foc_math.h"
#include "main.h"
#include "util.h"
#include "foc_ctrl.h"

_RAM_FUNC void SinCosVal(foc_param_t *foc) {
    foc->sin_val = sin_f32(foc->theta);
    foc->cos_val = cos_f32(foc->theta);
}

_RAM_FUNC void Clarke(foc_param_t *foc) {
    foc->i_alpha = foc->i_a;
    foc->i_beta = (foc->i_b - foc->i_c) * ONE_BY_SQRT3;
}

_RAM_FUNC void Park(foc_param_t *foc) {
    foc->i_d = foc->i_alpha * foc->cos_val + foc->i_beta * foc->sin_val;
    foc->i_q = -foc->i_alpha * foc->sin_val + foc->i_beta * foc->cos_val;
}

_RAM_FUNC void InvPark(foc_param_t *foc) {
    foc->v_alpha = foc->v_d * foc->cos_val - foc->v_q * foc->sin_val;
    foc->v_beta = foc->v_q * foc->cos_val + foc->v_d * foc->sin_val;
}

_RAM_FUNC void InvClarke(foc_param_t *foc) {
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
    float Ua = foc->v_alpha;
    float Ub = foc->v_beta;

    /* Clarke */
    float U1 = Ub;
    float U2 = (SQRT3 * Ua - Ub) * 0.5f;
    float U3 = -(SQRT3 * Ua + Ub) * 0.5f;

    uint8_t A, B, C;
    uint8_t N;
    int sector = 0;

    /* sector */
    // clang-format off
    A = (U1 > 0.0f);
    B = (U2 > 0.0f);
    C = (U3 > 0.0f);

    N = (A) | (B << 1) | (C << 2);

    switch (N) {
        case 1: sector = 2; break;
        case 2: sector = 6; break;
        case 3: sector = 1; break;
        case 4: sector = 4; break;
        case 5: sector = 3; break;
        case 6: sector = 5; break;
        default: sector = 0; break;
    }
    // clang-format on

    foc->sector = sector;

    float Tn = 0.95f;
    float k  = (Tn * SQRT3) / foc->vbus;
    float T1, T2;
    switch (sector)
    {
    case 1: T1 = U2 * k;  T2 = U1 * k;  break;
    case 2: T1 = -U2 * k; T2 = -U3 * k; break;
    case 3: T1 = U1 * k;  T2 = U3 * k;  break;
    case 4: T1 = -U1 * k; T2 = -U2 * k; break;
    case 5: T1 = U3 * k;  T2 = U2 * k;  break;
    case 6: T1 = -U3 * k; T2 = -U1 * k; break;
    default:
        foc->dtc_a = foc->dtc_b = foc->dtc_c = 0.5f;
        return -1;
    }

    /* Overmodulation clamp */
    float S = T1 + T2;
    if (S > Tn)
    {
        T1 = T1 / S * Tn;
        T2 = T2 / S * Tn;
    }

    float T0 = (Tn - T1 - T2) * 0.5f;
    float Ta = T0 + T1 + T2;
    float Tb = T0 + T2;
    float Tc = T0;

    switch (sector)
    {
    case 1: foc->dtc_a = Ta; foc->dtc_b = Tb; foc->dtc_c = Tc; break;
    case 2: foc->dtc_a = Tb; foc->dtc_b = Ta; foc->dtc_c = Tc; break;
    case 3: foc->dtc_a = Tc; foc->dtc_b = Ta; foc->dtc_c = Tb; break;
    case 4: foc->dtc_a = Tc; foc->dtc_b = Tb; foc->dtc_c = Ta; break;
    case 5: foc->dtc_a = Tb; foc->dtc_b = Tc; foc->dtc_c = Ta; break;
    case 6: foc->dtc_a = Ta; foc->dtc_b = Tc; foc->dtc_c = Tb; break;
    }

    if (foc->dtc_a < 0 || foc->dtc_a > 1) return -1;
    if (foc->dtc_b < 0 || foc->dtc_b > 1) return -1;
    if (foc->dtc_c < 0 || foc->dtc_c > 1) return -1;

    return 0;
}


#define RC 1.f/(M_2PI * 50)
#define DT 1.f/10000
float alpha;

void SmoParamInit(smo_param_t *smo) {
    smo->A = expf(-(motor_cfg.rs / 1000) / (motor_cfg.ls / 1000000) / 10000);
    smo->B = (1 - smo->A) / (motor_cfg.rs / 1000);
    smo->ksw = 0.13f;
    pll_smo.loop_hz = 10000;
    pll_smo.kp = 7.f * 5000.f / 60.f * M_2PI * 0.707f * 2.f;
    pll_smo.ki = (7.f * 5000.f / 60.f * M_2PI) * (7.f * 5000.f / 60.f * M_2PI);
    alpha = DT / (RC + DT);  // 计算滤波系数
}


/**
 * @brief 滑膜观测器锁相环
 * @param param
 * @param pll
 * @return
 */
_RAM_FUNC float SmoPllAngle(smo_param_t *param, pll_t *pll) {
    static float omega_out;
    pll->ref = -param->Ealpha * cos_f32(pll->angle_out);
    pll->fbk = sin_f32(pll->angle_out) * param->Ebeta;

    pll->p_term = (pll->ref - pll->fbk) * pll->kp;
    pll->i_term += (pll->ref - pll->fbk) * pll->ki / pll->loop_hz;
    pll->out_value = pll->p_term + pll->i_term;

    omega_out = pll->out_value * 0.1f + omega_out * (1.f - 0.1f);
    omega_out = pll->out_value;

    pll->angle_out += pll->out_value / pll->loop_hz;
    WRAP_0_2PI(pll->angle_out)

    float out = pll->angle_out + 0.7f;
//    float out = pll->angle_out + sqrtf(param->Ealpha*param->Ealpha + param->Ebeta*param->Ebeta)/(motor_cfg.flux/1000);
    return WRAP_0_2PI(out);
}


/**
 * @brief 滑膜观测器
 * @param foc
 * @param smo
 * @return
 */
_RAM_FUNC float SmoViewer(foc_param_t *foc, smo_param_t *smo) {
    static float valpha_last, vbeta_last;
    float ualpha, ubeta;

    ualpha = (2 * foc->v_a - foc->v_b - foc->v_c) / 3.f;
    ubeta = (foc->v_b - foc->v_c) * ONE_BY_SQRT3;
    Clarke(foc);

    //计算预测电流
    if (ABS(motor_ctrl.speed_set < 800)) {
        smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (foc->v_alpha - smo->valpah);
        smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (foc->v_beta - smo->vbeta);
    } else {
        if (ABS(motor_cfg.rotor_vel < 500)) {
            smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (foc->v_alpha - smo->valpah);
            smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (foc->v_beta - smo->vbeta);
        } else {
            //貌似使用端电压采样在高速情况下比给定电压采样更好
            smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (ualpha - smo->valpah);
            smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (ubeta - smo->vbeta);
        }
    }

    //计算电动势观测值
    smo->valpah = smo->ksw * SIGN(smo->ialpha_view - foc->i_alpha);
    smo->vbeta = smo->ksw * SIGN(smo->ibeta_view - foc->i_beta);
//    smo_param->valpah = smo_param->ksw*sat1_datf(smo_param->ialpha_view - foc_param->i_alpha, 0.5);
//    smo_param->vbeta = smo_param->ksw*sat1_datf(smo_param->ibeta_view - foc_param->i_beta,0.5);

    //计算滤波后的拓展反电动势
    smo->valpah = alpha * smo->valpah + (1 - alpha) * valpha_last;
    smo->vbeta = alpha * smo->vbeta + (1 - alpha) * vbeta_last;

    //更新数据
    smo->ialpha_view_last = smo->ialpha_view;
    smo->ibeta_view_last = smo->ibeta_view;
    vbeta_last = smo->vbeta;
    valpha_last = smo->valpah;

    smo->Ealpha = smo->valpah;
    smo->Ebeta = smo->vbeta;

    return SmoPllAngle(smo, &pll_smo);
}


/**
 *
 * @param inject_U 注入电压
 * @return 定轴的高频注入值
 */
float HfiInjectSign(float inject_U) {
//    static float u = -1.f;
//    u *= -1.f;
    static int data[4] = {0,1,0,-1}, i=0;
    i++;
    if(i==4)
        i=0;
    return inject_U * data[i];
}


// 二阶巴特沃斯低通滤波器系数（采样频率1 kHz，截止频率100 Hz）
float b0 = 0.0201f, b1 = 0.0402f, b2 = 0.0201f; // 分子系数
float a1 = -1.5610f, a2 = 0.6414f;              // 分母系数
float x1 = 0.0f, x2 = 0.0f;                     // 输入延迟状态
float y3 = 0.0f, y2 = 0.0f;                     // 输出延迟状态

// 滤波器函数
float butterworth_lpf(float input) {
    float output = b0 * input + b1 * x1 + b2 * x2 - a1 * y3 - a2 * y2;
    // 更新状态变量
    x2 = x1;
    x1 = input;
    y2 = y3;
    y3 = output;
    return output;
}


lpf_t lpf_hfi = {.in_last = 0.0f, .trust = 0.1f}; // id低通滤波器
/**
 * @brief 高频注入锁相环
 * @param pll
 * @param hfi
 * @return
 */
float HfiPllAngle(pll_t *pll, hfi_param_t *hfi) {
    //目前问题：
    // 1.锁相环的积分比较鸡肋，考虑升级一下。
    // 2.相位上具有一定的延时
    pll->ref = cos_f32(pll->angle_out) * hfi->envelope.beta;
    pll->fbk = hfi->envelope.alpha * sin_f32(pll->angle_out);

    pll->error = pll->ref - pll->fbk;

    pll->p_term = pll->error * pll->kp;
    pll->i_term += pll->error * pll->ki / pll->loop_hz;
    pll->i_term = AbsLimit(pll->i_term, pll->i_term_limit); //积分限幅

    pll->out_value = pll->p_term + pll->i_term;
    hfi->omega_e = butterworth_lpf(pll->out_value * 1.3648f);
    LowPassFilter(&pll->out_value, &lpf_hfi);

    pll->angle_out += pll->out_value / pll->loop_hz;
    WRAP_0_2PI(pll->angle_out)

    return WRAP_0_2PI(pll->angle_out);
}


/**
 * @brief 高频注入角度计算
 * @param foc
 * @param hfi
 */
void HfiAngleCalc(foc_param_t *foc, hfi_param_t *hfi) {
    //更新数据
    Clarke(foc);
    hfi->ab_laster.alpha = hfi->ab_last.alpha;
    hfi->ab_laster.beta = hfi->ab_last.beta;
    hfi->ab_last.alpha = hfi->ab.alpha;
    hfi->ab_last.beta = hfi->ab.beta;
    hfi->ab.alpha = foc->i_alpha;
    hfi->ab.beta = foc->i_beta;
    hfi->ab_h_last.alpha = hfi->ab_h.alpha;
    hfi->ab_h_last.beta = hfi->ab_h.beta;

    //提取高频电流
//    hfi->ab_h.alpha = (hfi->ab.alpha - 2.f * hfi->ab_last.alpha + hfi->ab_laster.alpha) * 0.25f;
//    hfi->ab_h.beta = (hfi->ab.beta - 2.f * hfi->ab_last.beta + hfi->ab_laster.beta) * 0.25f;
    hfi->ab_h.alpha = (hfi->ab.alpha - hfi->ab_last.alpha)*0.5f;
    hfi->ab_h.beta = (hfi->ab.beta - hfi->ab_last.beta)*0.5f;

    hfi->envelope.alpha = (hfi->ab_h.alpha - hfi->ab_h_last.alpha) * hfi->sign;
    hfi->envelope.beta = (hfi->ab_h.beta - hfi->ab_h_last.beta) * hfi->sign;

    hfi->theta_e = HfiPllAngle(&pll_hfi, hfi);
}


/**
 * @brief 提取定轴和转轴的基频电流
 * @param foc
 * @param hfi
 */
void IdqToIdqF(foc_param_t *foc, hfi_param_t *hfi) {
//    hfi->idq_f.id = (foc->i_d + 2 * hfi->idq_f_last.id + hfi->idq_f_laster.id) * 0.25f;
//    hfi->idq_f.iq = (foc->i_q + 2 * hfi->idq_f_last.iq + hfi->idq_f_laster.iq) * 0.25f;
    hfi->idq_f.id = (foc->i_d + hfi->idq_f_last.id)*0.5f;
    hfi->idq_f.iq = (foc->i_q + hfi->idq_f_last.iq)*0.5f;

    //update
    hfi->idq_f_laster.id = hfi->idq_f_last.id;
    hfi->idq_f_laster.iq = hfi->idq_f_last.iq;
    hfi->idq_f_last.id = foc->i_d;
    hfi->idq_f_last.iq = foc->i_q;
}


/**
 * @brief 提取定轴和转轴的高频电流
 * @param foc
 * @param hfi
 */
void IdqToIdqH(foc_param_t *foc, hfi_param_t *hfi) {
//    hfi->idq_h.id = (foc->i_d - 2 * hfi->idq_h_last.id + hfi->idq_h_laster.id) * 0.25f;
//    hfi->idq_h.iq = (foc->i_q - 2 * hfi->idq_h_last.iq + hfi->idq_h_laster.iq) * 0.25f;
    hfi->idq_h.id = (foc->i_d - hfi->idq_h_last.id)*0.5f;
    hfi->idq_h.iq = (foc->i_q - hfi->idq_h_last.iq)*0.5f;

    //update
    hfi->idq_h_laster.id = hfi->idq_h_last.id;
    hfi->idq_h_laster.iq = hfi->idq_h_last.iq;
    hfi->idq_h_last.id = foc->i_d;
    hfi->idq_h_last.iq = foc->i_q;
}


/**
 * @brief north-south identify
 * @param hfi
 * @param foc
 * @return
 */
bool HfiNsIdentify(hfi_param_t *hfi, foc_param_t *foc) {
    float gain = 10.f; //放大增益
    IdqToIdqH(foc, hfi); //提取d轴的高频电流

    hfi->nsd_count++;
    if (hfi->nsd_count < 400)   //0
    {
        motor_ctrl.id_set = 0;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 400 && hfi->nsd_count < 600) {
        motor_ctrl.id_set = 2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 600 && hfi->nsd_count < 620) {
        motor_ctrl.id_set = 2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
        hfi->isum_positive += fabsf(hfi->idq_h.id);
    } else if (hfi->nsd_count >= 620 && hfi->nsd_count < 820) {
        motor_ctrl.id_set = 0.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 820 && hfi->nsd_count < 1020) {
        motor_ctrl.id_set = -2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 1020 && hfi->nsd_count < 1040) {
        motor_ctrl.id_set = -2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
        hfi->isum_negetive += fabsf(hfi->idq_h.id);
    } else {
        motor_ctrl.id_set = 0;
        if (hfi->isum_positive < hfi->isum_negetive)
            hfi->theta_e += M_PI;
        if (hfi->theta_e > M_2PI)
            hfi->theta_e -= M_2PI;
        HfiVolt(0, 0, hfi->theta_e);
        return true;
    }
    return false;
}

float FluxPllAngle(pll_t *pll, float error) {

    pll->error = error;

    pll->p_term = pll->error * pll->kp;
    pll->i_term += pll->error * pll->ki / pll->loop_hz;
    pll->i_term = AbsLimit(pll->i_term, pll->i_term_limit); //积分限幅

    pll->out_value = pll->p_term + pll->i_term;
    LowPassFilter(&pll->out_value, &lpf_hfi);

    pll->angle_out += pll->out_value / pll->loop_hz;
    WRAP_0_2PI(pll->angle_out)

    return WRAP_0_2PI(pll->angle_out);
}

#define PI      3.14159265358979f
#define TWO_PI  (2.0f * PI)
#define EPS     1e-8f
static inline float Angle_Atan2_0To2Pi(float y, float x)
{
    if (fabsf(y) < EPS && fabsf(x) < EPS)
        return 0.0f;

    float angle = atan2f(y, x); // [-π, +π]

    if (angle < 0.0f)
        angle += TWO_PI;

    return angle; // [0, 2π)
}

float DEBUG_eta,DEBUG_etb;
float DEBUG_theta_e;
int16_t non_flux_observer(non_flux_t* flux, foc_param_t* foc, motor_cfg_t *motor)
{
    flux->Vs.fab.alpha = foc->v_alpha;
    flux->Vs.fab.beta = foc->v_beta;
    flux->Is.fab.alpha = foc->i_alpha;
    flux->Is.fab.beta = foc->i_beta;
    float Vy = flux->Vs.fab.alpha - motor->rs/1000.f * flux->Is.fab.alpha;
    float Vb = flux->Vs.fab.beta - motor->rs/1000.f * flux->Is.fab.beta;

    /* ---- Step 2: Non-linear Flux Observer ---- */
    float L = motor->ls/1000000.f;
    float Phi = motor->flux/1000.f;
    float Ts = flux->Ts;

    float LI_a = L *  flux->Is.fab.alpha;
    float LI_b = L * flux->Is.fab.beta;

    float eta_a = flux->state.fab.alpha - LI_a;
    float eta_b = flux->state.fab.beta - LI_b;

    float eta_sq = Phi * Phi - eta_a * eta_a - eta_b * eta_b;
//    float eta_sq = Phi*Phi - eta_a*eta_a - eta_b*eta_b;
//    if (eta_sq < 0.01f * Phi*Phi) eta_sq = 0.01f * Phi*Phi;  // 防止反向

    float gamma2 = flux->Gamma * 0.5f;

    flux->state.fab.alpha += Ts * (Vy + gamma2 * eta_a * eta_sq);
    flux->state.fab.beta  += Ts * (Vb + gamma2 * eta_b * eta_sq);

    /* Recompute eta for PLL */
    eta_a = flux->state.fab.alpha - LI_a;
    eta_b = flux->state.fab.beta - LI_b;

    /* ---- Step 3: PLL ---- */
    float theta = flux->theta_e;
    float x_theta = eta_b * cos_f32(theta) - eta_a * sin_f32(theta);

    flux->theta_e = FluxPllAngle(&pll_flux,x_theta/Phi);
    flux->omega_e = pll_flux.out_value/7.f/M_2PI*60.f;
//    flux->theta_e = Angle_Atan2_0To2Pi(eta_b, eta_a);
    DEBUG_theta_e = Angle_Atan2_0To2Pi(eta_b, eta_a);
    DEBUG_eta = eta_a*1000.f;
    DEBUG_etb = eta_b*1000.f;
    return 0;
}

