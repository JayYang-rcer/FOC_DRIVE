//
// Created by 28076 on 26-2-21.
//

#ifndef DRIVE_CMAKE_PID_H
#define DRIVE_CMAKE_PID_H
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

protected:
    void PiInit(const Config &cfg) { cfg_ = cfg; }
    float Calculate(float error);

    /**
     * @brief 复位
     */
    void PiReset()
    {
        integral_ = 0.0f;
    }

private:
    Config cfg_{};
    float  integral_ = 0.0f; // 积分项
};

// 串联型位置式PID
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

    void Init(const Config &cfg) { cfg_ = cfg; }

    /**
     * @brief Position PID calculate
     * @param setpoint
     * @param measure
     * @param dt    // the period of Pid loops
     */
    float Calculate(float error);

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
    void Init(const Config &cfg) { cfg_ = cfg; }

    float Calculate(float error);

    void Reset()
    {
        output_     = 0.0f;
        err_last_   = 0.0f;
        err_laster_ = 0.0f;
    }

private:
    Config cfg_{};
    float  output_   = 0.0f;
    float  err_last_ = 0.0f, err_laster_ = 0.0f;
};

#endif // DRIVE_CMAKE_PID_H
