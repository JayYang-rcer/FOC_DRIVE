//
// Created by 28076 on 26-2-10.
//

#include "encoder.h"

void AngleGetBase::EncoderDataProc(uint16_t raw_data)
{
    if (dir_ == 1)
        theta_ = (float)raw_data * factor_;
    else
        theta_ = (float)(cpr_ - raw_data) * factor_;
    WRAP_0_2PI(theta_);

    // calculate the electrical position
    theta_elect_ = theta_ * (float)pole_pairs_ - (uint16_t)(theta_ * pole_pairs_ / M_2PI) * M_2PI - offset_elect_;
    WRAP_0_2PI(theta_elect_);

    // count the revolution
    if (theta_diff_ > 0.8f * M_2PI)
        rev_--;
    else if (theta_diff_ < -0.8f * M_2PI)
        rev_++;

    theta_last_  = theta_;
    theta_multi_ = theta_ + (float)rev_ * M_2PI;
}
