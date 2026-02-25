//
// Created by 28076 on 26-2-25.
//

#ifndef DRIVE_CMAKE_OBERSVER_H
#define DRIVE_CMAKE_OBERSVER_H
#include "encoder.h"
#include "filters.h"
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
    NonFluxObserver(Config cfg, MotorParam_t motor, float gamma) : motor_(motor), PIController(cfg)
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

        float output = Calculate(error); // 调用父类 Calculate

        theta_elect_ += output * ts_;
        WRAP_0_2PI(theta_elect_);

        velocity_ = lpf_.Update(output) / (float)motor_.pn * RADS_TO_RPM;
    }

private:
    LowPassFilter lpf_ = LowPassFilter(0.1f);
    MotorParam_t motor_;
    Vector2Df_t  state_{0, 0};
    float        gamma_, ts_;
};

class PulsatingHFI : public AngleProvider, PIController
{
public:
    explicit PulsatingHFI(const Config &cfg, const MotorParam_t &motor, uint16_t inject_freq)
        : PIController(cfg), motor_(motor)
    {
        change_cnt_ = 20000 / inject_freq;
        ts_         = cfg.dt;
    }

    void Update(const Vector2Df_t &voltage_ab, const Vector2Df_t &current_ab)
    {
        // 交替注入电压 (每5次采样切换一次符号)
        if (++cnt_ >= change_cnt_) {
            cnt_ = 0;
            sign_ *= -1.0f; // 符号翻转
        } else {
            sign_ = 0.0f;
        }

        Vector2Df_t ab_h;
        ab_h.s.alpha = -(current_ab.s.alpha - 2.0f * iab_last_.s.alpha + iab_laster_.s.alpha) * 0.25f * sign_;
        ab_h.s.beta  = -(current_ab.s.beta - 2.0f * iab_last_.s.beta + iab_laster_.s.beta) * 0.25f * sign_;

        iab_laster_ = iab_last_;
        iab_last_   = current_ab;

        // Calculate the angle
        // error = cos(theta) * beta - sin(theta) * alpha
        float          error = cos_f32(theta_elect_) * ab_h.s.beta;
        -ab_h.s.alpha *sin_f32(theta_elect_);

        // 6. PI控制更新角度
        float pll_output = Calculate(error);

        theta_elect_ += pll_output * ts_;
        WRAP_0_2PI(theta_elect_);

        // 7. 计算速度 (rad/s -> rpm)
        velocity_ = lpf_.Update(pll_output) / motor_.pn * RADS_TO_RPM;
    }

    // 提取高频电流分量 (IdqToIdqH)
    Vector2Df_t GetDqCurrentHigh(const Vector2Df_t &current_dq)
    {
        Vector2Df_t idq_h;
        // idq_h = (idq - 2*idq_last + idq_laster) / 4
        idq_h.r.d = (current_dq.r.d - 2.0f * idq_last_.r.d + idq_laster_.r.d) * 0.25f;
        idq_h.r.q = (current_dq.r.q - 2.0f * idq_last_.r.q + idq_laster_.r.q) * 0.25f;

        idq_laster_ = idq_last_;
        idq_last_   = current_dq;

        return idq_h;
    }

    // 4. 计算高频电流在alpha-beta轴的投影 (HfiAngleCalc)
    Vector2Df_t GetDqCurrentLow(const Vector2Df_t &current_dq)
    {
        Vector2Df_t idq_f;

        // 2. 提取基频电流分量 (IdqToIdqH)
        // idq_f = (idq + 2*idq_last + idq_laster) * 0.25
        idq_f.s.alpha = (current_dq.s.alpha + 2.0f * idq_last_.s.alpha + idq_laster_.s.alpha) * 0.25f;
        idq_f.s.beta = (current_dq.s.beta + 2.0f * idq_last_.s.beta + idq_laster_.s.beta) * 0.25f;

        idq_laster_ = idq_last_;
        idq_last_   = idq_f;

        return idq_f;
    }

private:
    LowPassFilter lpf_ = LowPassFilter(0.1f);
    float         ts_;
    MotorParam_t  motor_;

    // 注入控制
    float   sign_{}; // 当前注入符号 (+1/-1)
    uint8_t cnt_{};  // 计数器
    uint8_t change_cnt_;

    // 电流历史
    Vector2Df_t iab_last_{};   // 上次alpha-beta电流
    Vector2Df_t iab_laster_{}; // 上上次alpha-beta电流

    Vector2Df_t idq_last_{};   // 上次d-q电流
    Vector2Df_t idq_laster_{}; // 上上次d-q电流
};

#endif // DRIVE_CMAKE_OBERSVER_H
