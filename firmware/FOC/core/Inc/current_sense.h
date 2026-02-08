//
// Created by 28076 on 26-2-2.
//
#include "foc_cfg.h"
#include "util.h"

#ifndef DRIVE_CMAKE_CURRENT_SENSE_H
#define DRIVE_CMAKE_CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

enum class Sector : uint8_t {
    S1 = 1,
    S2,
    S3,
    S4,
    S5,
    S6,
    Invalid
};

typedef struct {
    volatile uint32_t const *addr_current_u_;
    volatile uint32_t const *addr_current_v_;
    volatile uint32_t const *addr_current_w_;
    uint8_t                  adc_bits;
    float                    adc_ref_volt;
    float                    resistance;
    float                    gain;
} InitTypedef_t;

class PhaseSenseBase
{
public:
    [[nodiscard]] bool               OffsetCalibrate();
    bool                             Init();
    [[nodiscard]] const Vector2Df_t &GetAlphaBeta() const { return alpha_beta_; }
    [[nodiscard]] const Vector3S_t  &GetCurrents() const { return currents_; }

    InitTypedef_t InitTypedef;

protected:
    Vector2Df_t alpha_beta_{0};
    Vector3S_t  currents_{0};
    Vector3S_t  offset_{0};

    volatile uint32_t const *addr_current_u_{nullptr};
    volatile uint32_t const *addr_current_v_{nullptr};
    volatile uint32_t const *addr_current_w_{nullptr};
    float                    fcc_   = 0.0f;
    uint16_t                 count_ = 0;
};

class InlineCurrentSense : public PhaseSenseBase
{
public:
    void Update();
};

class LowsideCurrentSense : public PhaseSenseBase
{
public:
    void Update(Sector sector_);
};

//class PhaseVoltSense : public PhaseSenseBase
//{
//
//};

class TempSense
{

};

class VoltBusSense
{

};

class Svpwm
{
public:
    [[maybe_unused]] explicit Svpwm(float _volt_vbus, float _pwm_cnt)
    {
        volt_bus_ = _volt_vbus;
        tn_       = _pwm_cnt;
    }

    [[nodiscard]] inline const Sector     &GetSvpwmSector() const { return sector_; }
    [[nodiscard]] inline const Vector3D_t &GetSvpwmDuty(Vector2Df_t alpha_beta)
    {
        /* Clarke */
        float U1 = alpha_beta.s.beta;
        float U2 = (SQRT3 * alpha_beta.s.alpha - alpha_beta.s.beta) * 0.5f;
        float U3 = -(SQRT3 * alpha_beta.s.alpha + alpha_beta.s.beta) * 0.5f;

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

        float k = (tn_ * SQRT3) / volt_bus_;
        float T1, T2;
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
            duty_.uhU = duty_.uhV = duty_.uhW = 0.5f * tn_;
            return duty_;
            break;
        }

        /* Overmodulation clamp */
        float S = T1 + T2;
        if (S > tn_) {
            T1 = T1 / S * tn_;
            T2 = T2 / S * tn_;
        }

        float T0 = (tn_ - T1 - T2) * 0.5f;
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

        if (duty_.uhU > tn_ || duty_.uhV > tn_ || duty_.uhW > tn_) {
            duty_.uhU = duty_.uhV = duty_.uhW = 0.5f * tn_;
            return duty_;
        }
        return duty_;
    }

    inline void SetVbus(float volt_bus) { volt_bus_ = volt_bus; } // 设置母线电压（可运行时更新）

private:
    Sector     sector_;
    Vector3D_t duty_{};
    float      volt_bus_;
    float      tn_ = 0;
};

#endif // DRIVE_CMAKE_CURRENT_SENSE_H
