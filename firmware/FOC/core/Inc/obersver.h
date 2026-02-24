//
// Created by 28076 on 26-2-25.
//

#ifndef DRIVE_CMAKE_OBERSVER_H
#define DRIVE_CMAKE_OBERSVER_H
#include "encoder.h"
#include "foc_cfg.h"
#include "util.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

class NonFluxObserver : public AngleProvider, PIController
{
public:
    NonFluxObserver(Config cfg, MotorParam_t motor, float gamma) : PIController(cfg)
    {
        // 预计算
        motor_.ls   = motor.ls / 1000000.0f; // H
        motor_.flux = motor.flux / 1000.0f;  // Wb
        motor_.rs   = motor.rs / 1000.0f;
        gamma_      = gamma;
        ts_         = cfg.dt;
    }

    void Update(const Vector2Df_t &voltage_ab, const Vector2Df_t &current_ab)
    {
        // 1. 计算电压降 (Rs * Is)
        float Vy = voltage_ab.s.alpha - (motor_.rs) * current_ab.s.alpha;
        float Vb = voltage_ab.s.beta - (motor_.rs) * current_ab.s.beta;

        // 2. 计算 L * Is
        float LI_a = motor_.ls * current_ab.s.alpha;
        float LI_b = motor_.ls * current_ab.s.beta;

        // 3. 计算 eta (磁链误差)
        float eta_a = state_.s.alpha - LI_a;
        float eta_b = state_.s.beta - LI_b;

        // 4. 非线性观测器更新
        float eta_sq = motor_.flux * motor_.flux - eta_a * eta_a - eta_b * eta_b;
        float gamma2 = gamma_ * 0.5f;

        state_.s.alpha += ts_ * (Vy + gamma2 * eta_a * eta_sq);
        state_.s.beta += ts_ * (Vb + gamma2 * eta_b * eta_sq);

        // 5. 重新计算 eta 用于 PLL
        eta_a = state_.s.alpha - LI_a;
        eta_b = state_.s.beta - LI_b;

        // 6. PLL 计算角度（error = eta_b * cos(theta) - eta_a * sin(theta)）
        // error = ref - mesures
        float error = (eta_b * cos_f32(theta_elect_) - eta_a * sin_f32(theta_elect_)) / motor_.flux;

        float output = Calculate(error, 0); // 调用父类 Calculate

        theta_elect_ += output * ts_;
        WRAP_0_2PI(theta_elect_);

        velocity_ = output / (float)motor_.pn * RADS_TO_RPM;
    }

private:
    MotorParam_t motor_{};
    Vector2Df_t  state_{0, 0};
    float        gamma_, ts_;
};

#endif // DRIVE_CMAKE_OBERSVER_H
