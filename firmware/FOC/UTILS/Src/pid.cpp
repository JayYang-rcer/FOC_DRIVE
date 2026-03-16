#include "pid.h"
#include "main.h"
#include "util.h"

_RAM_FUNC float PIController::Calculate(float error)
{
    // 1. P term
    float p_term = cfg_.kp * error;

    // 2. I term
    integral_ += cfg_.ki * error * cfg_.dt;

    // clang-format off
        if (integral_ > cfg_.integral_max) integral_ = cfg_.integral_max;
        if (integral_ < -cfg_.integral_max) integral_ = -cfg_.integral_max;

        float out_final = p_term + integral_;
        if (out_final > cfg_.output_max) out_final = cfg_.output_max;
        if (out_final < -cfg_.output_max) out_final = -cfg_.output_max;
    // clang-format on

    return out_final;
}

_RAM_FUNC float PositionalPid::Calculate(float error)
{
    // 1. P term
    float p_term = cfg_.kp * error;

    // 2. I term
    // clang-format off
        integral_ += cfg_.ki * cfg_.kp * (error - anti_term_) * cfg_.dt;
        if (integral_ > cfg_.integral_max) integral_ = cfg_.integral_max;
        if (integral_ < -cfg_.integral_max) integral_ = -cfg_.integral_max;
    // clang-format on

    // 3. D 项 (带简单滤波，防止位置环抖动)
    float d_error = (error - last_error_) / cfg_.dt;
    float d_term  = cfg_.kd * (d_error * cfg_.d_term_filter_coeff +
                              last_d_error_ * (1.0f - cfg_.d_term_filter_coeff));

    last_error_   = error;
    last_d_error_ = d_error;

    // 4. final output and output limit
    float out_raw   = p_term + integral_ + d_term;
    float out_final = out_raw;

    // clang-format off
        if (out_final > cfg_.output_max) out_final = cfg_.output_max;
        if (out_final < cfg_.output_min) out_final = cfg_.output_min;

        anti_term_ = cfg_.anti_coeff * ABS(out_raw - out_final);
    // clang-format on

    return out_final;
}

_RAM_FUNC float IncrementalPid::Calculate(float error)
{
    float delta = cfg_.kp * (error - err_last_) +
                  cfg_.ki * (error - err_last_) * cfg_.dt +
                  cfg_.kd * (error - 2.0f * err_last_ + err_laster_) / cfg_.dt;

    output_ += delta;

    // 输出限幅
    // clang-format off
        if (output_ > cfg_.out_limit) output_ = cfg_.out_limit;
        else if (output_ < -cfg_.out_limit) output_ = -cfg_.out_limit;
    // clang-format on

    err_laster_ = err_last_;
    err_last_   = error;
    return output_;
}
