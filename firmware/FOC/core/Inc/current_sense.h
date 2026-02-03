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

class CurrentSense
{
public:
    virtual bool OffsetCalibrate() { return false; }

    [[nodiscard]] const Vector2Df_t &GetAlphaBeta() const { return alpha_beta_; }
    [[nodiscard]] const Vector3S_t  &GetCurrents() const { return currents_; }

protected:
    Vector2Df_t alpha_beta_{};
    Vector3S_t  currents_{};
    Vector3S_t  offset_{};
};

class InlineCurrentSense : public CurrentSense
{
public:

private:
};

class LowsideCurrentSense : public CurrentSense
{
public:

private:
};

#endif // DRIVE_CMAKE_CURRENT_SENSE_H
