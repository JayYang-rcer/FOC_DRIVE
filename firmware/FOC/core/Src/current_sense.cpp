//
// Created by 28076 on 26-2-2.
//

#include "current_sense.h"
#include "util.h"

bool PhaseSenseBase::OffsetCalibrate()
{
    if (count_ < 4096) {
        offset_.fU += (float)(*addr_current_u_) * fcc_;
        offset_.fV += (float)(*addr_current_v_) * fcc_;
        offset_.fW += (float)(*addr_current_w_) * fcc_;
        count_++;
    } else if (count_ == 4096) {
        offset_.fU /= 4096.0f;
        offset_.fV /= 4096.0f;
        offset_.fW /= 4096.0f;
        count_++;
    } else {
        return true;
    }
    return false;
}

bool PhaseSenseBase::Init()
{
    if (InitTypedef.addr_current_u_ != nullptr &&
        InitTypedef.addr_current_v_ != nullptr &&
        InitTypedef.addr_current_w_ != nullptr) {
        float adc_rate  = InitTypedef.adc_ref_volt / (float)(1 << InitTypedef.adc_bits);
        fcc_            = adc_rate / InitTypedef.resistance / InitTypedef.gain;
        addr_current_u_ = InitTypedef.addr_current_u_;
        addr_current_v_ = InitTypedef.addr_current_v_;
        addr_current_w_ = InitTypedef.addr_current_w_;
        return true;
    }

    return false;
}

void InlineCurrentSense::Update()
{
    currents_.fU = (float)(*addr_current_u_) * fcc_ - offset_.fU;
    currents_.fV = (float)(*addr_current_v_) * fcc_ - offset_.fV;
    currents_.fW = (float)(*addr_current_w_) * fcc_ - offset_.fW;

    alpha_beta_.s.alpha = currents_.fU;
    alpha_beta_.s.beta  = (currents_.fV - currents_.fW) * ONE_BY_SQRT3;
}

void LowsideCurrentSense::Update(Sector sector_)
{
    switch (sector_) {
    case Sector::S1:
    case Sector::S6:
        currents_.fV = (float)(*addr_current_v_) * fcc_ - offset_.fV;
        currents_.fW = (float)(*addr_current_w_) * fcc_ - offset_.fW;
        break;

    case Sector::S2:
    case Sector::S3:
        currents_.fU = (float)(*addr_current_u_) * fcc_ - offset_.fU;
        currents_.fW = (float)(*addr_current_w_) * fcc_ - offset_.fW;
        break;

    case Sector::S4:
    case Sector::S5:
        currents_.fU = (float)(*addr_current_u_) * fcc_ - offset_.fU;
        currents_.fV = (float)(*addr_current_v_) * fcc_ - offset_.fV;
        break;

    default:
        currents_.fU = 0.0f;
        currents_.fV = 0.0f;
        currents_.fW = 0.0f;
        break;
    }

    alpha_beta_.s.alpha = currents_.fU;
    alpha_beta_.s.beta  = (currents_.fV - currents_.fW) * ONE_BY_SQRT3;
}
