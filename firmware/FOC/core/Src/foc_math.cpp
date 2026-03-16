#include "foc_math.h"
#include "foc_ctrl.h"
#include "main.h"
#include "util.h"

_RAM_FUNC void SinCosVal(Vector2Df_t *out, float theta)
{
    out->f.sin = sin_f32(theta);
    out->f.cos = cos_f32(theta);
}

_RAM_FUNC void Clarke(Vector2Df_t *out, Vector3S_t *in)
{
    out->s.alpha = in->fU;
    out->s.beta  = (in->fV - in->fW) * ONE_BY_SQRT3;
}

_RAM_FUNC void Park(Vector2Df_t *out, Vector2Df_t *in, Vector2Df_t *sin_cos)
{
    out->r.d = in->s.alpha * sin_cos->f.cos + in->s.beta * sin_cos->f.sin;
    out->r.q = -in->s.alpha * sin_cos->f.sin + in->s.beta * sin_cos->f.cos;
}

_RAM_FUNC void InvPark(Vector2Df_t *out, Vector2Df_t *in, Vector2Df_t *sin_cos)
{
    out->s.alpha = in->r.d * sin_cos->f.cos - in->r.q * sin_cos->f.sin;
    out->s.beta  = in->r.q * sin_cos->f.cos + in->r.d * sin_cos->f.sin;
}

_RAM_FUNC void InvClarke(FocParam_t *foc)
{
    foc->vphase.fU = foc->vab.s.alpha;
    foc->vphase.fV = -0.5f * foc->vab.s.alpha + SQRT3_BY_2 * foc->vab.s.beta;
    foc->vphase.fW = -0.5f * foc->vab.s.alpha - SQRT3_BY_2 * foc->vab.s.beta;
}

// function: hfi north-south identify
#if 0
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
#endif
