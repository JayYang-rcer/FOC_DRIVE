//
// Created by 28076 on 26-2-21.
//

#ifndef DRIVE_CMAKE_PID_CLASS_H
#define DRIVE_CMAKE_PID_CLASS_H
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

class PIController
{
public:
    struct Config {
        float kp;
        float ki;           // gain 0f integral
        float output_max;   // output limit (rad/s)
        float integral_max; // integral limit
        float dt;           // sample period (s)
    };

    explicit PIController(const Config &cfg) : cfg_(cfg) {}

protected:
    float Calculate(float setpoint, float measure)
    {
        float error = setpoint - measure;

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

    /**
     * @brief 复位
     */
    void Reset()
    {
        integral_ = 0.0f;
    }

private:
    Config cfg_;
    float  integral_ = 0.0f; // 积分项
};

class PositionalPid
{
public:
    struct Config {
        float kp, ki, kd;
        float output_min, output_max;
        float d_term_filter_coeff; // D项滤波系数 (0~1, 1表示无滤波)
        float integral_max;
        float anti_coeff;
        float dt = 0.0f;
    };

    explicit PositionalPid(const Config &cfg) : cfg_(cfg) {}

    /**
     * @brief Position PID calculate
     * @param setpoint
     * @param measure
     * @param dt    // the period of Pid loops
     */
    float Calculate(float setpoint, float measure)
    {
        float error = setpoint - measure;

        // 1. P term
        float p_term = cfg_.kp * error;

        // 2. I term
        // clang-format off
        integral_ += cfg_.ki * cfg_.kp * (error - anti_term_);
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
        float out_pre_limit = p_term + integral_ + d_term;
        float out_final     = out_pre_limit;

        // clang-format off
        if (out_final > cfg_.output_max) out_final = cfg_.output_max;
        if (out_final < cfg_.output_min) out_final = cfg_.output_min;

        anti_term_ = cfg_.anti_coeff * ABS(out_pre_limit - out_final);
        // clang-format on

        return out_final;
    }

    void Reset()
    {
        integral_     = 0.0f;
        last_error_   = 0.0f;
        last_d_error_ = 0.0f;
    }

private:
    Config cfg_;
    float  integral_     = 0.0f;
    float  last_error_   = 0.0f;
    float  last_d_error_ = 0.0f;
    float  anti_term_    = 0.0f;
};

class IncrementalPid
{
public:
    struct Config {
        float kp, ki, kd;
        float out_limit;
        float dt;
    };
    explicit IncrementalPid(const Config &cfg) : cfg_(cfg) {}

    float Calculate(float target, float measure)
    {
        float error = target - measure;

        float delta = cfg_.kp * (error - err_last_) +
                      cfg_.ki * error * cfg_.dt +
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

    void Reset()
    {
        output_     = 0.0f;
        err_last_   = 0.0f;
        err_laster_ = 0.0f;
    }

private:
    Config cfg_;
    float  output_   = 0.0f;
    float  err_last_ = 0.0f, err_laster_ = 0.0f;
};

#endif // DRIVE_CMAKE_PID_CLASS_H
