#include "foc_math.h"
#include "foc_ctrl.h"
#include "main.h"
#include "util.h"

_RAM_FUNC void SinCosVal(FocParam_t *foc)
{
    foc->sin_val = sin_f32(foc->theta);
    foc->cos_val = cos_f32(foc->theta);
}

_RAM_FUNC void Clarke(FocParam_t *foc)
{
    foc->iab.s.alpha = foc->current.fU;
    foc->iab.s.beta  = (foc->current.fV - foc->current.fW) * ONE_BY_SQRT3;
}

_RAM_FUNC void Park(FocParam_t *foc)
{
    foc->idq.r.d = foc->iab.s.alpha * foc->cos_val + foc->iab.s.beta * foc->sin_val;
    foc->idq.r.q = -foc->iab.s.alpha * foc->sin_val + foc->iab.s.beta * foc->cos_val;
}

_RAM_FUNC void InvPark(FocParam_t *foc)
{
    foc->vab.s.alpha = foc->vdq.r.d * foc->cos_val - foc->vdq.r.q * foc->sin_val;
    foc->vab.s.beta  = foc->vdq.r.q * foc->cos_val + foc->vdq.r.d * foc->sin_val;
}

_RAM_FUNC void InvClarke(FocParam_t *foc)
{
    foc->vphase.fU = foc->vab.s.alpha;
    foc->vphase.fV = -0.5f * foc->vab.s.alpha + SQRT3_BY_2 * foc->vab.s.beta;
    foc->vphase.fW = -0.5f * foc->vab.s.alpha - SQRT3_BY_2 * foc->vab.s.beta;
}

/**
 * @brief:      svpwm_sector(foc_para_t *foc_param)
 * @param[in]:  foc  FOC参数结构体指针
 * @retval:     void
 * @details:    SVPWM扇区判断
 */
_RAM_FUNC int SvpwmSector(FocParam_t *foc)
{
    /* Clarke */
    float U1 = foc->vab.s.beta;
    float U2 = (SQRT3 * foc->vab.s.alpha - foc->vab.s.beta) * 0.5f;
    float U3 = -(SQRT3 * foc->vab.s.alpha + foc->vab.s.beta) * 0.5f;

    uint8_t A, B, C;
    uint8_t N;
    int     sector = 0;

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

    float Tn = 1.0f;
    float k  = (Tn * SQRT3) / foc->vbus;
    float T1, T2;
    switch (sector) {
    case 1:
        T1 = U2 * k;
        T2 = U1 * k;
        break;
    case 2:
        T1 = -U2 * k;
        T2 = -U3 * k;
        break;
    case 3:
        T1 = U1 * k;
        T2 = U3 * k;
        break;
    case 4:
        T1 = -U1 * k;
        T2 = -U2 * k;
        break;
    case 5:
        T1 = U3 * k;
        T2 = U2 * k;
        break;
    case 6:
        T1 = -U3 * k;
        T2 = -U1 * k;
        break;
    default:
        foc->dtc_a = foc->dtc_b = foc->dtc_c = 0.5f;
        return -1;
    }

    /* Overmodulation clamp */
    float S = T1 + T2;
    if (S > Tn) {
        T1 = T1 / S * Tn;
        T2 = T2 / S * Tn;
    }

    float T0 = (Tn - T1 - T2) * 0.5f;
    float Ta = T0 + T1 + T2;
    float Tb = T0 + T2;
    float Tc = T0;

    switch (sector) {
    case 1:
        foc->dtc_a = Ta;
        foc->dtc_b = Tb;
        foc->dtc_c = Tc;
        break;
    case 2:
        foc->dtc_a = Tb;
        foc->dtc_b = Ta;
        foc->dtc_c = Tc;
        break;
    case 3:
        foc->dtc_a = Tc;
        foc->dtc_b = Ta;
        foc->dtc_c = Tb;
        break;
    case 4:
        foc->dtc_a = Tc;
        foc->dtc_b = Tb;
        foc->dtc_c = Ta;
        break;
    case 5:
        foc->dtc_a = Tb;
        foc->dtc_b = Tc;
        foc->dtc_c = Ta;
        break;
    case 6:
        foc->dtc_a = Ta;
        foc->dtc_b = Tc;
        foc->dtc_c = Tb;
        break;
    }

    if (foc->dtc_a < 0 || foc->dtc_a > 1)
        return -1;
    if (foc->dtc_b < 0 || foc->dtc_b > 1)
        return -1;
    if (foc->dtc_c < 0 || foc->dtc_c > 1)
        return -1;

    return 0;
}

#define RC 1.f / (M_2PI * 50)
#define DT 1.f / 10000
float alpha;

void SmoParamInit(SmoParam_t *smo)
{
    smo->A          = expf(-(motor_cfg.rs / 1000) / (motor_cfg.ls / 1000000) / 10000);
    smo->B          = (1 - smo->A) / (motor_cfg.rs / 1000);
    smo->ksw        = 0.13f;
    pll_smo.loop_hz = 10000;
    pll_smo.kp      = 7.f * 5000.f / 60.f * M_2PI * 0.707f * 2.f;
    pll_smo.ki      = (7.f * 5000.f / 60.f * M_2PI) * (7.f * 5000.f / 60.f * M_2PI);
    alpha           = DT / (RC + DT); // 计算滤波系数
}

/**
 * @brief 滑膜观测器锁相环
 * @param param
 * @param pll
 * @return
 */
_RAM_FUNC float SmoPllAngle(SmoParam_t *param, pll_t *pll)
{
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
_RAM_FUNC float SmoViewer(FocParam_t *foc, SmoParam_t *smo)
{
    static float valpha_last, vbeta_last;
    Clarke(foc);

    // 计算预测电流
    smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (foc->vab.s.alpha - smo->valpah);
    smo->ibeta_view  = smo->A * smo->ibeta_view_last + smo->B * (foc->vab.s.beta - smo->vbeta);

    // 计算电动势观测值
    smo->valpah = smo->ksw * SIGN(smo->ialpha_view - foc->iab.s.alpha);
    smo->vbeta  = smo->ksw * SIGN(smo->ibeta_view - foc->iab.s.beta);
    //    smo_param->valpah = smo_param->ksw*sat1_datf(smo_param->ialpha_view - foc_param->i_alpha, 0.5);
    //    smo_param->vbeta = smo_param->ksw*sat1_datf(smo_param->ibeta_view - foc_param->i_beta,0.5);

    // 计算滤波后的拓展反电动势
    smo->valpah = alpha * smo->valpah + (1 - alpha) * valpha_last;
    smo->vbeta  = alpha * smo->vbeta + (1 - alpha) * vbeta_last;

    // 更新数据
    smo->ialpha_view_last = smo->ialpha_view;
    smo->ibeta_view_last  = smo->ibeta_view;
    vbeta_last            = smo->vbeta;
    valpha_last           = smo->valpah;

    smo->Ealpha = smo->valpah;
    smo->Ebeta  = smo->vbeta;

    return SmoPllAngle(smo, &pll_smo);
}

/**
 *
 * @param inject_U 注入电压
 * @return 定轴的高频注入值
 */
float HfiInjectSign(float inject_U)
{
    static float u = -1.f;
    u *= -1.f;
    hfi_param.sign = u;
    return inject_U * u;
}

// 二阶巴特沃斯低通滤波器系数（采样频率1 kHz，截止频率100 Hz）
float b0 = 0.0201f, b1 = 0.0402f, b2 = 0.0201f; // 分子系数
float a1 = -1.5610f, a2 = 0.6414f;              // 分母系数
float x1 = 0.0f, x2 = 0.0f;                     // 输入延迟状态
float y3 = 0.0f, y2 = 0.0f;                     // 输出延迟状态

// 滤波器函数
float butterworth_lpf(float input)
{
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
float HfiPllAngle(pll_t *pll, HfiParam_t *hfi)
{
    // 目前问题：
    //  1.锁相环的积分比较鸡肋，考虑升级一下。
    //  2.相位上具有一定的延时
    pll->ref   = cos_f32(pll->angle_out) * hfi->ab_h.s.beta;
    pll->fbk   = hfi->ab_h.s.alpha * sin_f32(pll->angle_out);
    pll->error = pll->ref - pll->fbk;

    pll->p_term = pll->error * pll->kp;
    pll->i_term += pll->error * pll->ki / pll->loop_hz;
    pll->i_term = AbsLimit(pll->i_term, pll->i_term_limit); // 积分限幅

    pll->out_value = pll->p_term + pll->i_term;
    //    hfi->omega = butterworth_lpf(pll->out_value * 1.3648f);

    pll->angle_out += pll->out_value / pll->loop_hz;
    LowPassFilter(&pll->out_value, &lpf_hfi);
    hfi->omega_e = pll->out_value;
    WRAP_0_2PI(pll->angle_out)

    return WRAP_0_2PI(pll->angle_out);
}

/**
 * @brief 高频注入角度计算
 * @param foc
 * @param hfi
 */
void HfiAngleCalc(FocParam_t *foc, HfiParam_t *hfi)
{
    // 更新数据
    Clarke(foc);
    if (hfi->sign != 0) {
        hfi->ab_h.s.alpha = -(foc->iab.s.alpha - hfi->ab_last.s.alpha) * 0.5f * hfi->sign;
        hfi->ab_h.s.beta  = -(foc->iab.s.beta - hfi->ab_last.s.beta) * 0.5f * hfi->sign;

        hfi->ab_last.s.alpha   = foc->iab.s.alpha;
        hfi->ab_last.s.beta    = foc->iab.s.beta;
        hfi->ab_laster.s.alpha = hfi->ab_last.s.alpha;
        hfi->ab_laster.s.beta  = hfi->ab_last.s.beta;

        hfi->theta_e = HfiPllAngle(&pll_hfi, hfi);
    }
}

/**
 * @brief 提取定轴和转轴的基频电流
 * @param foc
 * @param hfi
 */
void IdqToIdqF(FocParam_t *foc, HfiParam_t *hfi)
{
    hfi->idq_f.r.d = (foc->idq.r.d + 2.0f * hfi->idq_f_laster.r.d + hfi->idq_f_laster.r.d) * 0.25f;
    hfi->idq_f.r.q = (foc->idq.r.q + 2.0f * hfi->idq_f_laster.r.q + hfi->idq_f_laster.r.q) * 0.25f;

    // update
    hfi->idq_f_laster.r.d = hfi->idq_f_last.r.d;
    hfi->idq_f_laster.r.q = hfi->idq_f_last.r.q;
    hfi->idq_f_last.r.d   = foc->idq.r.d;
    hfi->idq_f_last.r.q   = foc->idq.r.q;
}

/**
 * @brief 提取定轴和转轴的高频电流
 * @param foc
 * @param hfi
 */
void IdqToIdqH(FocParam_t *foc, HfiParam_t *hfi)
{
    hfi->idq_h.r.d = (foc->idq.r.d - hfi->idq_h_last.r.d) * 0.5f;
    hfi->idq_h.r.q = (foc->idq.r.q - hfi->idq_h_last.r.q) * 0.5f;

    // update
    hfi->idq_h_last.r.d = foc->idq.r.d;
    hfi->idq_h_last.r.q = foc->idq.r.q;
}

/**
 * @brief north-south identify
 * @param hfi
 * @param foc
 * @return
 */
bool HfiNsIdentify(HfiParam_t *hfi, FocParam_t *foc)
{
    float gain = 10.f;   // 放大增益
    IdqToIdqH(foc, hfi); // 提取d轴的高频电流

    hfi->nsd_count++;
    if (hfi->nsd_count < 400) // 0
    {
        motor_ctrl.id_set = 0;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 400 && hfi->nsd_count < 600) {
        motor_ctrl.id_set = 2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 600 && hfi->nsd_count < 620) {
        motor_ctrl.id_set = 2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
        hfi->isum_positive += fabsf(hfi->idq_h.r.d);
    } else if (hfi->nsd_count >= 620 && hfi->nsd_count < 820) {
        motor_ctrl.id_set = 0.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 820 && hfi->nsd_count < 1020) {
        motor_ctrl.id_set = -2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
    } else if (hfi->nsd_count >= 1020 && hfi->nsd_count < 1040) {
        motor_ctrl.id_set = -2.f;
        HfiCurrent(motor_ctrl.id_set, motor_ctrl.iq_set, hfi->theta_e);
        hfi->isum_negetive += fabsf(hfi->idq_h.r.d);
    } else {
        motor_ctrl.id_set = 0;
        if (hfi->isum_positive < hfi->isum_negetive)
            pll_hfi.angle_out += M_PI;
        if (pll_hfi.angle_out > M_2PI)
            pll_hfi.angle_out += M_2PI;
        //        HfiVolt(0, 0, hfi->theta_e);
        return true;
    }
    return false;
}

float FluxPllAngle(pll_t *pll, float error)
{

    pll->error = error;

    pll->p_term = pll->error * pll->kp;
    pll->i_term += pll->error * pll->ki / pll->loop_hz;
    pll->i_term = AbsLimit(pll->i_term, pll->i_term_limit); // 积分限幅

    pll->out_value = pll->p_term + pll->i_term;
    LowPassFilter(&pll->out_value, &lpf_hfi);

    pll->angle_out += pll->out_value / pll->loop_hz;
    WRAP_0_2PI(pll->angle_out)

    return WRAP_0_2PI(pll->angle_out);
}

#define PI     3.14159265358979f
#define TWO_PI (2.0f * PI)
#define EPS    1e-8f
static inline float Angle_Atan2_0To2Pi(float y, float x)
{
    if (fabsf(y) < EPS && fabsf(x) < EPS)
        return 0.0f;

    float angle = atan2f(y, x); // [-π, +π]

    if (angle < 0.0f)
        angle += TWO_PI;

    return angle; // [0, 2π)
}

int16_t non_flux_observer(void)
{
    MotorCfg_t *motor = &motor_cfg;
    FocParam_t *foc = &foc_param;
    NonFlux_t *flux = &nonFlux;

    flux->Vs.s.alpha = foc->vab.s.alpha;
    flux->Vs.s.beta  = foc->vab.s.beta;
    flux->Is.s.alpha = foc->iab.s.alpha;
    flux->Is.s.beta  = foc->iab.s.beta;
    float Vy         = flux->Vs.s.alpha - motor->rs / 1000.f * flux->Is.s.alpha;
    float Vb         = flux->Vs.s.beta - motor->rs / 1000.f * flux->Is.s.beta;

    /* ---- Step 2: Non-linear Flux Observer ---- */
    float L   = motor->ls / 1000000.f;
    float Phi = motor->flux / 1000.f;
    float Ts  = flux->Ts;

    float LI_a = L * flux->Is.s.alpha;
    float LI_b = L * flux->Is.s.beta;

    float eta_a = flux->state.s.alpha - LI_a;
    float eta_b = flux->state.s.beta - LI_b;

    float eta_sq = Phi * Phi - eta_a * eta_a - eta_b * eta_b;
    float gamma2 = flux->Gamma * 0.5f;

    flux->state.s.alpha += Ts * (Vy + gamma2 * eta_a * eta_sq);
    flux->state.s.beta += Ts * (Vb + gamma2 * eta_b * eta_sq);

    /* Recompute eta for PLL */
    eta_a = flux->state.s.alpha - LI_a;
    eta_b = flux->state.s.beta - LI_b;

    /* ---- Step 3: PLL ---- */
    float theta   = flux->theta_e;
    float x_theta = eta_b * cos_f32(theta) - eta_a * sin_f32(theta);

    flux->theta_e = FluxPllAngle(&pll_flux, x_theta / Phi);
    flux->omega   = pll_flux.out_value / motor->pn * RADS_TO_RPM;

    return 0;
}
