//
// Created by 28076 on 25-6-14.
//
#include "foc_identify.h"
#include "filters.h"
#include "foc_cfg.h"
#include "foc_ctrl.h"
#include "math.h"

LowPassFilter  lpf_ident(0.05f);
volatile float Is_sum   = 0.0f;
foc_identify_t identify = {
    .v_set1           = 0.0f,
    .v_set2           = 0.0f,
    .ident_status_res = IDENT_SET1};

bool FocIdentifyRes(foc_identify_t *identify, FocParam_t *foc, float Is1, float Is2)
{
    switch (identify->ident_status_res) {
    case IDENT_SET1: {
        FocPwmStart(true, false, false, true, false, false);
        // LowPassFilterHandle(&foc->current.fU, &identify_lpf_Is); // 低通滤波处理
        lpf_ident.Update(&foc->current.fU);
        if (foc->current.fU > Is1) {
            identify->ident_status_res = IDENT_GET1;
        } else {
            identify->v_set1 += 0.00001f; // 增加直轴电压
            SET_DTC_A((uint16_t)((float)(identify->v_set1 / foc->vbus) * 4250.f + 17.0f));
            SET_DTC_B(4249);
            SET_DTC_C(0);
        }
        break;
    }
    case IDENT_GET1: {
        static int cnt = 0;
        if (++cnt < 20000) // 等待2000次采样
        {
            lpf_ident.Update(&foc->current.fU);
            Is_sum += fabsf(foc->current.fU); // 累加电流值
        } else {
            identify->ident_status_res = IDENT_SET2;
            identify->i_value1         = Is_sum / 20000.0f; // 计算平均值
            // 清零
            SET_DTC_A(0);
            SET_DTC_B(0);
            SET_DTC_C(0);
            Is_sum = 0.0f;
        }
        break;
    }

    case IDENT_SET2: {
        lpf_ident.Update(&foc->current.fU);
        if (foc->current.fU > Is2) {
            identify->ident_status_res = IDENT_GET2;
        } else {
            identify->v_set2 += 0.00001f; // 增加直轴电压
            SET_DTC_A((uint16_t)((float)(identify->v_set2 / foc->vbus) * 4250.f + 17.5f));
            SET_DTC_B(4249);
            SET_DTC_C(0);
        }
        break;
    }

    case IDENT_GET2: {
        static int cnt = 0;
        if (++cnt < 20000) // 等待2000次采样
        {
            lpf_ident.Update(&foc->current.fU);
            Is_sum += fabsf(foc->current.fU); // 累加电流值
        } else {
            identify->ident_status_res = IDENT_SUCCESS;
            identify->i_value2         = Is_sum / 20000.0f; // 计算平均值
            // 计算相电阻: res = (v2- v1)/((i2 - i1) * 1.5) - 1.5mR MOS内阻
            identify->res = (identify->v_set2 - identify->v_set1) /
                                ((identify->i_value2 - identify->i_value1) * 1.5f) * 1000.f -
                            1.5f;
            // 清零
            SET_DTC_A(0);
            SET_DTC_B(0);
            SET_DTC_C(0);
            Is_sum = 0.0f;
        }
        break;
    }

    case IDENT_SUCCESS: {
        FocPwmStart(true, true, true, true, true, true);
        return true;
    }

    default:
        break;
    }

    return false;
}

bool FocIdentifyInductance(foc_identify_t *ident, FocParam_t *foc)
{
    // 假设电机电阻约为 50mOhm
    static const float motor_rs = 0.05f;

    switch (ident->ident_status_l) {
    case L_IDENT_IDLE: {
        ident->ident_status_l = L_IDENT_ALIGN;
        break;
    }

    case L_IDENT_ALIGN: {
        static uint32_t align_cnt = 0;
        align_cnt++;
        if (align_cnt < 50000) {
            FocVolt(foc->vbus * 0.1f, 0.0f, 0.0f);
        } else {
            align_cnt             = 0;
            ident->ident_status_l = L_IDENT_PULSE_D;
        }
        break;
    }

    case L_IDENT_PULSE_D: {
        // 2. D轴发脉冲测 Ld - 高精度测量
        static uint8_t  pulse_state = 0;
        static float    i_start     = 0.0f;
        static float    i_end       = 0.0f;
        static float    v_pulse     = 0.0f;
        static uint32_t sample_cnt  = 0;

        if (pulse_state == 0) {
            // 提高电压到50%以获得更大电流变化
            v_pulse = foc->vbus * 0.2f;
            FocVolt(v_pulse, 0.0f, 0.0f);
            pulse_state = 1;
            sample_cnt  = 0;
            i_start     = foc->current.fU;
        } else if (pulse_state == 1) {
            sample_cnt++;
            // 缩短到2个周期(100us)捕捉初始斜率
            if (sample_cnt > 2) {
                i_end       = foc->current.fU;
                pulse_state = 2;
            }
        } else {
            // 计算 D 轴电感 - 考虑电阻压降
            float di = i_end - i_start;
            float dt = 2.0f / 20000.0f; // 2个采样周期
            // 补偿电阻压降: L = (V - I*R) * dt / di
            float i_avg       = (i_start + i_end) / 2.0f;
            float v_effective = v_pulse - i_avg * motor_rs;
            if (fabsf(di) > 0.1f) // 放宽阈值
            {
                ident->Ld = v_effective * dt / di * 1000000.0f;
            }
            FocVolt(0, 0.0f, 0.0f);
            pulse_state           = 0;
            ident->ident_status_l = L_IDENT_PULSE_Q;
        }
        break;
    }

    case L_IDENT_PULSE_Q: {
        // 3. Q轴发脉冲测 Lq
        static uint8_t  pulse_state = 0;
        static float    i_start     = 0.0f;
        static float    i_end       = 0.0f;
        static float    v_pulse     = 0.0f;
        static uint32_t sample_cnt  = 0;

        if (pulse_state == 0) {
            // 提高电压到30%
            v_pulse = foc->vbus * 0.3f;
            FocVolt(0.0f, v_pulse, M_PI_2);
            pulse_state = 1;
            sample_cnt  = 0;
            i_start     = foc->current.fU;
        } else if (pulse_state == 1) {
            sample_cnt++;
            if (sample_cnt > 2) {
                i_end       = foc->current.fU;
                pulse_state = 2;
            }
        } else {
            float di          = i_end - i_start;
            float dt          = 2.0f / 20000.0f;
            float i_avg       = (i_start + i_end) / 2.0f;
            float v_effective = v_pulse - i_avg * motor_rs;
            if (fabsf(di) > 0.1f) {
                // 取绝对值，确保结果为正
                ident->Lq = v_effective * dt / fabsf(di) * 1000000.0f;
            }
            FocVolt(0.0f, 0.0f, 0.0f);
            pulse_state           = 0;
            ident->ident_status_l = L_IDENT_FINISH;
        }
        break;
    }

    case L_IDENT_FINISH: {
        FocPwmStart(true, true, true, true, true, true);
        return true;
    }

    default:
        break;
    }

    return false;
}

// 磁链辨识函数 - 通过空载反电动势测量
bool FocIdentifyFlux(foc_identify_t *ident, FocParam_t *foc)
{
    // 需要已知电阻和电感
    static const float Rs         = 0.054f; // 假设电阻 50mOhm (后续可用辨识值)
    static const float Ld_val     = 6.38f; // D轴电感 uH (后续可用辨识值)
    static const float Lq_val     = 8.24f; // Q轴电感 uH
    static const float pole_pairs = 7.0f;  // 极对数
    static const float id_set      = 2.0f;
    static const float target_freq = 30.f;

    switch (ident->ident_status_flux) {
    case FLUX_IDENT_IDLE: {
        ident->ident_status_flux = FLUX_IDENT_ALIGN;
        break;
    }

    case FLUX_IDENT_ALIGN: {
        // 1. D轴对齐
        static uint32_t align_cnt = 0;
        align_cnt++;
        if (align_cnt < 50000) {
            FocCurrent(id_set,0.0f,0.0f);
        } else {
            align_cnt                = 0;
            ident->ident_status_flux = FLUX_IDENT_MEASURE;
        }
        break;
    }

    case FLUX_IDENT_MEASURE: {
        // 2. 施加电角频率，测量反电动势
        static uint32_t measure_cnt   = 0;
        static float    angle         = 0.0f;
        static float    electrical_hz = 0.0f; // 电角频率 Hz

        measure_cnt++;

        // 斜坡加速到目标频率
        if (measure_cnt < 40000) {
            // 逐渐增加频率 (0 -> 30Hz 电角频率)
            electrical_hz = measure_cnt / 40000.0f * target_freq;
        } else if (measure_cnt < 80000) {
            // 保持目标频率 30Hz
            electrical_hz = target_freq;
        } else {
            // 测量完成
            measure_cnt              = 0;
            electrical_hz            = 0.0f;
            angle                    = 0.0f;
            ident->ident_status_flux = FLUX_IDENT_CALC;
            break;
        }

        // 根据频率计算角度增量
        // angle += 2*PI*Hz*dt, dt = 1/20000
        float dt = 1.0f / 20000.0f;
        angle += 2.0f * M_PI * electrical_hz * dt;

        FocCurrent(id_set, 0.0f,angle);
        break;
    }

    case FLUX_IDENT_CALC: {
        // 3. 计算磁链
        // 原理: 在稳态时，Q轴电压方程: Vq = Rs * Iq + j*omega_e*Lq*Iq + j*omega_e * Flux
        static float    vq_measured = 0.0f;
        static float    iq_measured = 0.0f;
        static uint32_t sample_cnt  = 0;

        sample_cnt++;

        if (sample_cnt < 10000) {
            // 采样q轴电压和电流
            vq_measured += foc->vdq.r.q;
            iq_measured += foc->idq.r.q;
        } else if (sample_cnt == 10000) {
            // 计算平均值
            vq_measured /= 10000.0f;
            iq_measured /= 10000.0f;

            // 电角速度 omega = 2*PI*f, f = 150Hz (与测量时一致)
            float electrical_hz = target_freq;
            float omega         = M_2PI * electrical_hz;

            // 补偿电阻压降和电感压降
            // Vq = Rs*Iq + omega*Lq*Iq + omega*Flux
            // Flux = (Vq - Rs*Iq - omega*Lq*Iq) / omega
            float vq_effective = vq_measured - Rs * iq_measured - omega * (Lq_val / 1000000.0f) * iq_measured;

            // 取绝对值确保结果为正
            ident->flux = fabsf(vq_effective / omega * 1000.0f); // 转为 mWb
        } else {
            // 采样完成，直接结束
            ident->ident_status_flux = FLUX_IDENT_FINISH;
        }
        break;
    }

    case FLUX_IDENT_FINISH: {
        FocCurrent(0.0f,0.0f,0.0f);
        return true;
    }

    default:
        break;
    }

    return false;
}
