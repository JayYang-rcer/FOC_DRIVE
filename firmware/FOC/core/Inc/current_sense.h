//
// Created by 28076 on 26-2-2.
//
#include "foc_cfg.h"

#ifndef DRIVE_CMAKE_CURRENT_SENSE_H
#define DRIVE_CMAKE_CURRENT_SENSE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

enum class Sector : uint8_t {
    S1,
    S2,
    S3,
    S4,
    S5,
    S6,
    Invalid
};

class CurrentSense
{
public:
    struct {
        uint16_t const *addr_current_u;
        uint16_t const *addr_current_v;
        uint16_t const *addr_current_w;
        uint8_t const   adc_bits;
        float const     adc_ref_volt;
        float const     resistance;
        float const     gain;
    } InitTypedef;

    bool OffsetCalibrate();
    bool Init();

    [[nodiscard]] const Vector2Df_t &GetAlphaBeta() const { return alpha_beta_; }
    [[nodiscard]] const Vector3S_t  &GetCurrents() const { return currents_; }

protected:
    Vector2Df_t alpha_beta_{0};
    Vector3S_t  currents_{0};
    Vector3S_t  offset_{0};

    uint16_t const *addr_current_u_;
    uint16_t const *addr_current_v_;
    uint16_t const *addr_current_w_;
    float           fcc_   = 0.0f;
    uint16_t        count_ = 0;
};

class InlineCurrentSense : public CurrentSense
{
public:
    inline void Update();
};

class LowsideCurrentSense : public CurrentSense
{
public:
    inline void Update(Sector sector_);
};

#endif // DRIVE_CMAKE_CURRENT_SENSE_H
