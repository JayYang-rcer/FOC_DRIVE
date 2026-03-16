#ifndef __FOC_CAL_H
#define __FOC_CAL_H

#include "analog_sense.h"
#include "foc_cfg.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif
void SinCosVal(Vector2Df_t *out, float theta);
void Clarke(Vector2Df_t *out, Vector3S_t *in);
void Park(Vector2Df_t *out, Vector2Df_t *in, Vector2Df_t *sin_cos);
void InvPark(Vector2Df_t *out, Vector2Df_t *in, Vector2Df_t *sin_cos);
void InvClarke(FocParam_t *foc);

/*********************************** hfi *******************************/
//bool    HfiNsIdentify(HfiParam_t *hfi, FocParam_t *foc);
/*********************************** hfi *******************************/
#ifdef __cplusplus
}
#endif
class Svpwm
{
public:
    [[maybe_unused]] explicit Svpwm(float _volt_vbus, float _pwm_cnt)
    {
        volt_bus_  = _volt_vbus;
        pwm_count_ = _pwm_cnt;
    }

    [[nodiscard]] inline const Sector &GetSvpwmSector() const { return sector_; }

    /**
     * @brief:      Get the pwm out duty of Svpwm
     * @param[in]:  alpha_beta  the result of InvPark transform witch is the volt of alpha-beta asix
     */
    [[nodiscard]] inline const Vector3D_t &GetSvpwmDuty(Vector2Df_t alpha_beta)
    {
        /* Clarke */
        float   U1 = alpha_beta.s.beta;
        float   U2 = (SQRT3 * alpha_beta.s.alpha - alpha_beta.s.beta) * 0.5f;
        float   U3 = -(SQRT3 * alpha_beta.s.alpha + alpha_beta.s.beta) * 0.5f;
        float   k  = (pwm_count_ * SQRT3) / volt_bus_;
        uint8_t A, B, C;
        uint8_t N;

        /* sector */
        // clang-format off
        A = (U1 > 0.0f);
        B = (U2 > 0.0f);
        C = (U3 > 0.0f);

        N = (A) | (B << 1) | (C << 2);

        switch (N) {
            case 1: sector_ = Sector::S2; break;
            case 2: sector_ = Sector::S6; break;
            case 3: sector_ = Sector::S1; break;
            case 4: sector_ = Sector::S4; break;
            case 5: sector_ = Sector::S3; break;
            case 6: sector_ = Sector::S5; break;
            default: sector_ = Sector::Invalid; break;
        }
        // clang-format on

        float T1 = 0.0f, T2 = 0.0f;
        switch (sector_) {
        case Sector::S1:
            T1 = U2 * k;
            T2 = U1 * k;
            break;
        case Sector::S2:
            T1 = -U2 * k;
            T2 = -U3 * k;
            break;
        case Sector::S3:
            T1 = U1 * k;
            T2 = U3 * k;
            break;
        case Sector::S4:
            T1 = -U1 * k;
            T2 = -U2 * k;
            break;
        case Sector::S5:
            T1 = U3 * k;
            T2 = U2 * k;
            break;
        case Sector::S6:
            T1 = -U3 * k;
            T2 = -U1 * k;
            break;
        default:
            duty_.uhU = duty_.uhV = duty_.uhW = 0.5f * pwm_count_;
            return duty_;
            break;
        }

        /* Overmodulation clamp */
        float S = T1 + T2;
        if (S > pwm_count_) {
            T1 = T1 / S * pwm_count_;
            T2 = T2 / S * pwm_count_;
        }

        float T0 = (pwm_count_ - T1 - T2) * 0.5f;
        float Ta = T0 + T1 + T2;
        float Tb = T0 + T2;
        float Tc = T0;

        switch (sector_) {
        case Sector::S1:
            duty_.uhU = Ta;
            duty_.uhV = Tb;
            duty_.uhW = Tc;
            break;
        case Sector::S2:
            duty_.uhU = Tb;
            duty_.uhV = Ta;
            duty_.uhW = Tc;
            break;
        case Sector::S3:
            duty_.uhU = Tc;
            duty_.uhV = Ta;
            duty_.uhW = Tb;
            break;
        case Sector::S4:
            duty_.uhU = Tc;
            duty_.uhV = Tb;
            duty_.uhW = Ta;
            break;
        case Sector::S5:
            duty_.uhU = Tb;
            duty_.uhV = Tc;
            duty_.uhW = Ta;
            break;
        case Sector::S6:
            duty_.uhU = Ta;
            duty_.uhV = Tc;
            duty_.uhW = Tb;
            break;
        case Sector::Invalid:
            break;
        }

        if (duty_.uhU > pwm_count_ || duty_.uhV > pwm_count_ || duty_.uhW > pwm_count_) {
            duty_.uhU = duty_.uhV = duty_.uhW = 0.5f * pwm_count_;
            return duty_;
        }
        return duty_;
    }

    inline void SetVbus(float volt_bus) { volt_bus_ = volt_bus; } // 设置母线电压（可运行时更新）

private:
    Sector     sector_;
    Vector3D_t duty_{};
    float      volt_bus_;
    float      pwm_count_ = 0;
};

#endif
