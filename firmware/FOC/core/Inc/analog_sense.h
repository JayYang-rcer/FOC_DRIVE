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

class PhaseSenseBase
{
public:
    struct SenseConfig {
        volatile uint32_t const *addr_u;
        volatile uint32_t const *addr_v;
        volatile uint32_t const *addr_w;
        float                    adc_trans_volt;
        float                    resistance; // power resistance
        float                    gain;       // the gain of IOP
    };

    [[nodiscard]] virtual bool       OffsetCalibrate();
    [[nodiscard]] const Vector2Df_t &GetAlphaBeta() const { return alpha_beta_; }
    [[nodiscard]] const Vector3S_t  &GetCurrents() const { return currents_; }

protected:
    Vector2Df_t alpha_beta_{0};
    Vector3S_t  currents_{0};
    Vector3S_t  offset_{0};

    volatile uint32_t const *addr_u_{nullptr};
    volatile uint32_t const *addr_v_{nullptr};
    volatile uint32_t const *addr_w_{nullptr};
    float                    fcc_   = 0.0f;
    uint16_t                 count_ = 0;
    bool                     offset_init_{false};
    bool                     resource_init_{false};
};

class InlineCurrentSense : public PhaseSenseBase
{
public:
    explicit InlineCurrentSense(const SenseConfig &cfg);
    void Update();
};

class LowsideCurrentSense : public PhaseSenseBase
{
public:
    explicit LowsideCurrentSense(const SenseConfig &cfg);
    void Update(Sector sector_);
};

class PhaseVoltSense
{
public:
    struct SenseConfig {
        volatile uint32_t const *addr_u_;
        volatile uint32_t const *addr_v_;
        volatile uint32_t const *addr_w_;
        float                    fcc_;
    };
    explicit PhaseVoltSense(const SenseConfig &cfg) : cfg_(cfg) {}

    void Update()
    {
        volts.fU = (float)(*cfg_.addr_u_) * cfg_.fcc_;
        volts.fV = (float)(*cfg_.addr_v_) * cfg_.fcc_;
        volts.fW = (float)(*cfg_.addr_w_) * cfg_.fcc_;
    }

    [[nodiscard]] const Vector3S_t &GetPhaseVolts() const { return volts; }

private:
    uint16_t    count_{0};
    Vector3S_t  volts{0};
    SenseConfig cfg_{};
};

class TempSense
{
public:
    struct SenseConfig {
        volatile uint32_t const *addr_temp_;
        float                    temp_R25;
        float                    R_fixed;
        float                    B;
    };
    explicit TempSense(const SenseConfig &cfg) : cfg_(cfg) {}

    [[nodiscard]] float Get_Temperature() const
    {
        // 防呆处理：防止除以0或溢出
        if (cfg_.addr_temp_ == nullptr)
            return -1;
        if (*cfg_.addr_temp_ == 0)
            return -273.15f;
        if (*cfg_.addr_temp_ >= 4095)
            return 150.0f; // 假设上限

        // 计算当前NTC阻值
        // R_ntc = 3300 * (4095 / adc_val - 1)
        float r_ntc = cfg_.R_fixed * (4096.0f / (float)(*cfg_.addr_temp_) - 1.0f);

        // B值公式计算
        // 1/T = 1/T25 + ln(R/R25) / B
        float res = logf(r_ntc / cfg_.temp_R25) / cfg_.B;
        res += 1.0f / T25;
        res = 1.0f / res; // 得到开氏温度

        return res - 273.15f; // 转为摄氏度
    }

private:
    SenseConfig cfg_{};
    const float T25{298.15f}; // 25度时的开氏温度
};

class VoltBusSense
{
public:
    struct SenseConfig {
        volatile uint32_t const *addr_bus_;
        float                    fcc_;
    };
    explicit VoltBusSense(const SenseConfig &cfg) : cfg_(cfg) {}

    [[nodiscard]] float GetBusVolt() const
    {
        return (float)(*cfg_.addr_bus_) * cfg_.fcc_;
    }

private:
    SenseConfig cfg_{};
};

#endif // DRIVE_CMAKE_CURRENT_SENSE_H
