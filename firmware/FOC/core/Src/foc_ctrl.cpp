#include "foc_ctrl.h"
#include "as5047p.h"
#include "calibration.h"
#include "current_sense.h"
#include "drive_can.h"
#include "encoder.h"
#include "encoder_proc.h"
#include "filter.h"
#include "filters.h"
#include "foc_cfg.h"
#include "foc_math.h"
#include "obersver.h"
#include "tim.h"

#define USE_SPD_PLL          1 // 使用PLL速度估算
#define USE_SPD_DET          0 // 使用微分速度检测
#define USE_POS_PID          0 // 使用位置环
#define USE_ENCODER          1
#define USE_SENSERLESS       1
#define SENSERLESS_MIN_SPEED 500
#define SENSERLESS_MAX_SPEED 10000
#define USE_SLAVE_MODE       0
// #define USE_CURRENTLOOP_FEEDBACK

struct AppConfig {
    MotorParam_t motor = {
        .rs   = 37.5333f, // mOhm
        .ls   = 6.3f,     // uH
        .flux = 1.221f,   // mWb
        .jx   = 0.0001f,
        .pn   = 7,
    };
    PIController::Config pll_observer = {
        .kp           = 1000,
        .ki           = 180000,
        .output_max   = 11000,
        .integral_max = 10000,
        .dt           = 1 / 20000.f, // 20khz
    };
    SpeedPLLMonitor::Config spd_monitor{
        .kp           = 10800.f / 60.f * M_2PI * 0.707f * 2.f,
        .ki           = (10800.f / 60.f * M_2PI) * (10800.f / 60.f * M_2PI),
        .output_max   = 12000,
        .integral_max = 10000,
        .dt           = 1 / 10000.0f,
    };
    InlineCurrentSense::SenseConfig current_sense = {
        .addr_current_u_ = reinterpret_cast<const volatile uint32_t *>(&ADC1->JDR1),
        .addr_current_v_ = reinterpret_cast<const volatile uint32_t *>(&ADC1->JDR2),
        .addr_current_w_ = reinterpret_cast<const volatile uint32_t *>(&ADC1->JDR3),
        .adc_trans_volt_ = 3.3f / 4096.0f,
        .resistance      = 0.001f,
        .gain            = 20.0f,
    };
    AbiEncoder::Config abi_encoder = {
        .cpr        = 4000,
        .pole_pairs = 7,
        .tim_handle = TIM1,
    };
    As5407Encoder::Config as5047_cfg = {
        .cpr                     = 16384,
        .pole_pairs              = 7,
        .As5047RawDataGettingFun = As5047pRead};
    IncrementalPid::Config pid_spdCfg = {
        .kp        = 0.002f,
        .ki        = 0.003f,
        .kd        = 0.0f,
        .out_limit = 15.f,
        .dt        = 1 / 2000.0f,
    };
} app_config;

IncrementalPid     pid_spd(app_config.pid_spdCfg);
SpeedPLLMonitor    spdMonitor(app_config.spd_monitor);
Svpwm              svm(16.0f, 4250);
InlineCurrentSense current_sense(app_config.current_sense);
AbiEncoder         abiEncoder(app_config.abi_encoder);
As5407Encoder      as5047p(app_config.as5047_cfg);
LowPassFilter      lpf_id(0.1f), lpf_iq(0.1f);
LowPassFilter      lpf_speed(0.05f);
NonFluxObserver    nonFluxObserver(app_config.pll_observer, app_config.motor, 10000);

_RAM_FUNC void FocVolt(float vd_ref, float vq_ref, float pos)
{
    foc_param.theta = pos;
    WRAP_0_2PI(foc_param.theta);
    SinCosVal(&foc_param);
    Park(&foc_param);

    foc_param.vdq.r.d = vd_ref;
    foc_param.vdq.r.q = vq_ref;
    InvPark(&foc_param);
}

_RAM_FUNC void FocIFVolt(float id_ref, float pos)
{
    foc_param.theta = pos;
    WRAP_0_2PI(foc_param.theta);
    SinCosVal(&foc_param);
    Park(&foc_param);

    SerialPidCtrl(&id_pi, id_ref, foc_param.idq.r.d);
    foc_param.vdq.r.d = id_pi.out_value;
    foc_param.vdq.r.q = 0;
    InvPark(&foc_param);
}

#ifdef USE_CURRENTLOOP_FEEDBACK
float IqPidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{
    static float uq0;
    pid->target_value = target_value;
    pid->fback_value  = fdback_value;
    pid->error        = pid->target_value - pid->fback_value;
    pid->d_error      = pid->error - pid->pre_error;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error;
    AbsLimit(pid->i_term, pid->i_term_max);

    pid->d_term = pid->d_error * pid->kd;

    uq0 = motor_cfg.rotor_vel * motor_cfg.pn / 60.f * M_2PI * (motor_cfg.ls / 1000 * foc_param.idq.r.d + motor_cfg.flux) / 1000;
    uq0 = AbsLimit(uq0, pid->i_term_max);

    pid->out_value = pid->p_term + pid->i_term + pid->d_term + uq0;
    AbsLimit(pid->out_value, pid->out_max);

    return pid->out_value;
}

float IdPidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{
    static float ud0;
    pid->target_value = target_value;
    pid->fback_value  = fdback_value;
    pid->error        = pid->target_value - pid->fback_value;
    pid->d_error      = pid->error - pid->pre_error;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error;
    AbsLimit(pid->i_term, pid->i_term_max);
    pid->d_term = pid->d_error * pid->kd;

    ud0 = motor_cfg.rotor_vel * motor_cfg.pn / 60.f * M_2PI * motor_cfg.ls / 1000000.f * foc_param.idq.r.q;
    ud0 = AbsLimit(ud0, pid->i_term_max);

    pid->out_value = pid->p_term + pid->i_term + pid->d_term - ud0;
    AbsLimit(pid->out_value, pid->out_max);

    return pid->out_value;
}
#endif

_RAM_FUNC void FocCurrent(float id_set, float iq_set, float pos)
{
    foc_param.theta = pos;
    SinCosVal(&foc_param);
    Park(&foc_param);

#ifdef USE_CURRENTLOOP_FEEDBACK
    IdPidCtrl(&id_pid, id_set, foc_param.i_d);
    foc_param.vdq.r.d = id_pi.out_value;

    IqPidCtrl(&iq_pid, iq_set, foc_param.i_q);
    foc_param.vdq.r.q = iq_pi.out_value;
#endif

    SerialPidCtrl(&id_pi, id_set, foc_param.idq.r.d);
    foc_param.vdq.r.d = id_pi.out_value;

    SerialPidCtrl(&iq_pi, iq_set, foc_param.idq.r.q);
    foc_param.vdq.r.q = iq_pi.out_value;

    InvPark(&foc_param);
}

_RAM_FUNC void HfiVolt(float vd, float vq, float pos)
{
    foc_param.theta = pos;
    SinCosVal(&foc_param);

    static float ud_inject;
    static int   cnt = 0;
    if (++cnt == 5) {
        ud_inject = HfiInjectSign(hfi_param.inject_U);
        cnt       = 0;
    } else {
        hfi_param.sign = 0;
    }
    //    hfi_param.sign   = SIGN(ud_inject);

    foc_param.vdq.r.d = vd + ud_inject;
    foc_param.vdq.r.q = vq;

    InvPark(&foc_param);
}

_RAM_FUNC void HfiCurrent(float id_set, float iq_set, float pos)
{
    foc_param.theta = pos;
    SinCosVal(&foc_param);
    Park(&foc_param);
    IdqToIdqF(&foc_param, &hfi_param);

    static float ud_inject;
    static int   cnt = 0;
    if (++cnt == 5) {
        ud_inject = HfiInjectSign(hfi_param.inject_U);
        cnt       = 0;
    } else {
        hfi_param.sign = 0;
    }
    SerialPidCtrl(&hfi_id_pi, id_set, hfi_param.idq_f.r.d);
    foc_param.vdq.r.d = hfi_id_pi.out_value + ud_inject;

    SerialPidCtrl(&hfi_iq_pi, iq_set, hfi_param.idq_f.r.q);
    foc_param.vdq.r.q = hfi_iq_pi.out_value;

    InvPark(&foc_param);
}

volatile float vbus;

void AnalogSampleUpdate(FocAdcValue_t *adc, FocParam_t *foc)
{
    adc->vbus = ADC2->JDR1;
    vbus      = ((float)adc->vbus) * VBUS_RATIO;
    foc->vbus = ((float)adc->vbus) * VBUS_RATIO;
}

volatile float smo_angle;
lpf_t          lpf_spdpll = {.in_last = 0.0f, .trust = 0.01f}; // PLL低通滤波器
volatile float speed_hz   = 10000;

void EncoderDataCalc(enc_para_t *enc, MotorCfg_t *motor)
{
    static float rotor_vel_last = 0.0f;
#if USE_SPD_DET
    pos_last = pos_now;
    pos_now  = enc_para.raw_data;
    if (pos_now - pos_last < -8192)
        motor.rotor_vel = (pos_now - pos_last + 16383.f) / 16383.f * 60.f * speed_hz;
    else if (pos_now - pos_last > 8192)
        motor.rotor_vel = (pos_now - pos_last - 16383.f) / 16383.f * 60.f * speed_hz;
    else
        motor.rotor_vel = (pos_now - pos_last) / 16383.f * 60.f * speed_hz;
    // 一阶低通滤波
    motor.rotor_vel = (0.05f * motor.rotor_vel + 0.95f * rotor_vel_last);
    rotor_vel_last  = motor.rotor_vel;
    MoveAverageFilter(&speed_maf, &motor.rotor_vel);
#endif

#if USE_SPD_PLL

    //    if(motor_ctrl.mode == FOC_SENSORLESS_CTRL)
    //		motor_cfg.rotor_vel = pll_smo.out_value*60.f/M_2PI/7.f;
    ////使用滑膜速度输出
    //    else
    //    motor->rotor_vel = PllSpeedCtrl(&pll_spd, enc->pos_s); // 编码器速度输出
    motor->rotor_vel = spdMonitor.GetSpeed(enc->pos);
    lpf_speed.Update(&motor->rotor_vel);
//    MoveAverageFilter(&speed_maf, &motor->rotor_vel);
#endif
}

volatile int    spd_cnt = 0, pos_cnt = 0;
int             change_flag = 0;
extern uint16_t can_recieveFlag;
__RAM_FUNC void Encoder_Idle(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    static float pos_last, pos_now = 0.0f;
    static float flag = 0;
    if (flag == 0) {
        SmoParamInit(&smo_param);
        //        pll_hfi.angle_out -= M_2PI;
        flag = 1;
    }
#if USE_SLAVE_MODE
    if (++can_recieveFlag > 10000) {
        motor_ctrl.speed_set = 0;
    }
    uint8_t    data[8]  = {1, 1, 1, 1, 1, 1, 1, 1};
    static int send_cnt = 0;
    if (++send_cnt == 2000) {
        comm_can_transmit_stdid(&hfdcan2, 0x01, data, 8);
        send_cnt = 0;
    }
#endif

    EncoderDataCalc(&enc_para, &motor_cfg);
    //    smo_angle = SmoViewer(&foc_param, &smo_param);
}

void MotorCtrl(MotorCtrl_t *ctrl, FocParam_t *foc)
{
    switch (ctrl->mode) {
    case FOC_IDLE: {
        MotorCtrlReset(ctrl);
        FocVolt(ctrl->vd_set, ctrl->vq_set, foc->theta);
        break;
    }

    case FOC_VF_CTRL: {
        foc->theta += ctrl->epos_acc;
        FocVolt(ctrl->vd_set, ctrl->vq_set, foc->theta);
        break;
    }

    case FOC_IF_CTRL: {
        foc->theta += ctrl->epos_acc;
        FocIFVolt(ctrl->id_set, foc->theta);
        break;
    }

    case FOC_VOLT_CTRL: {
        float fRefSlope = motor_cfg.fRefSlope;
        if (fRefSlope > ctrl->vq_set) {
            fRefSlope -= 0.00005f;
        } else if (fRefSlope < ctrl->vq_set) {
            fRefSlope += 0.00005f;
        } else {
            fRefSlope = ctrl->vq_set;
        }
#if USE_SENSERLESS
        FocVolt(ctrl->vd_set, fRefSlope, nonFlux.theta_e);
#else
        FocVolt(ctrl->vd_set, fRefSlope, enc_para.pos_e);
#endif
        motor_cfg.fRefSlope = fRefSlope;
        break;
    }

    case FOC_CURRENT_CTRL: {
#if USE_SENSERLESS
        FocCurrent(ctrl->id_set, ctrl->iq_set, nonFlux.theta_e);
#else
        FocCurrent(ctrl->id_set, ctrl->iq_set, enc_para.pos_e);
#endif
        break;
    }

    case FOC_SPEED_CTRL: {

        if (++ctrl->spd_cnt == 10) // 20khz/10 = 2khz
        {
#if USE_SENSERLESS
            if (ABS(ctrl->speed_set) < SENSERLESS_MIN_SPEED)
                ctrl->speed_set = 0;
            ctrl->speed_set = AbsLimit(ctrl->speed_set, SENSERLESS_MAX_SPEED);
#endif
            int16_t RefSlope = motor_cfg.RefSlope;
            if (RefSlope > ctrl->speed_set) {
                RefSlope -= 2;
            } else if (RefSlope < ctrl->speed_set) {
                RefSlope += 2;
            } else {
                RefSlope = ctrl->speed_set;
            }
#if USE_SENSERLESS
            //            IncreatParallePidCtrl(&speed_pid, RefSlope, nonFlux.omega);
            speed_pid.out_value = pid_spd.Calculate(RefSlope, nonFlux.omega);
//            IncreatParallePidCtrl(&speed_pid, RefSlope, nonFlux.omega);
#else
            IncreatParallePidCtrl(&speed_pid, RefSlope, motor_cfg.rotor_vel);
#endif
            motor_cfg.RefSlope = RefSlope;
            ctrl->spd_cnt      = 0;
        }
#if USE_SENSERLESS
        FocCurrent(ctrl->id_set, speed_pid.out_value, nonFlux.theta_e);
#else
        FocCurrent(ctrl->id_set, speed_pid.out_value, enc_para.pos_e);
#endif
        break;
    }

    case FOC_POSITION_CTRL: {
        if (++ctrl->pos_cnt == 20) // 20khz/20 = 1khz
        {
            ParallelPidCtrl(&pos_pid, ctrl->pos_set, enc_para.pos_m / M_2PI * 360);
            IncreatParallePidCtrl(&speed_pid, pos_pid.out_value, motor_cfg.rotor_vel);
            ctrl->pos_cnt = 0;
        }
        FocCurrent(ctrl->id_set, speed_pid.out_value, enc_para.pos_e);
        break;
    }

    case FOC_SENSORLESS_CTRL: {
        if (++ctrl->spd_cnt == 10) // 20khz/20 = 1khz
        {
            IncreatParallePidCtrl(&speed_pid, ctrl->speed_set, motor_cfg.rotor_vel);
            ctrl->spd_cnt = 0;
        }
        FocCurrent(ctrl->id_set, speed_pid.out_value, smo_angle);
        break;
    }

    case FOC_HFI: {
        HfiAngleCalc(foc, &hfi_param);
        static bool hfi_init = false;
        if (!hfi_init) {
            hfi_init = HfiNsIdentify(&hfi_param, foc);
        } else {
            // foc->theta += ctrl->epos_acc;
            // HfiVolt(motor_ctrl.vd_set, motor_ctrl.vq_set, foc->theta);
            // HfiVolt(motor_ctrl.vd_set, motor_ctrl.vq_set, hfi_param.theta_e);

            if (++motor_ctrl.spd_cnt == 10) {
                static int pos = 0;
                if (++motor_ctrl.pos_cnt == 2) {
                    ParallelPidCtrl(&pos_pid, ctrl->pos_set, enc_para.pos_m / M_2PI * 360);
                    motor_ctrl.pos_cnt = 0;
                }
                IncreatParallePidCtrl(&HfiSpeed_pid, ctrl->speed_set, hfi_param.omega_e * 60.f / M_2PI / 7.f);
                //       IncreatParallePidCtrl(&HfiSpeed_pid, pos_pid.out_value, hfi_param.omega);
                motor_ctrl.spd_cnt = 0;
            }
            // 高频注入Id偏置，防止电机在速度为0时的观测角度发散
            if (motor_ctrl.speed_set != 0)
                HfiCurrent(5, HfiSpeed_pid.out_value, hfi_param.theta_e);
            else
                HfiCurrent(0, 0, hfi_param.theta_e);
        }
        break;
    }
    default:
        MotorCtrlReset(ctrl);
        FocVolt(ctrl->vd_set, ctrl->vq_set, foc->theta);
        break;
    }
}

_RAM_FUNC void FocHandle(void)
{
    AnalogSampleUpdate(&mc_adc, &foc_param);
#if (USE_SENSERLESS == 0)
    PosCalculate(&enc_para);
#endif
    Clarke(&foc_param);
    non_flux_observer();
    nonFluxObserver.Update(foc_param.vab, foc_param.iab);
    foc_param.vbus = 4.0f * BATTERY_CELL;

#if USE_POS_PID
#if USE_VOLT_POS
    FocVolt(0.f, pos_pid.out_value, enc_para.pos_e);
#else
    FocCurrent(id, speed_pid.out_value, enc_para.pos_e);
#endif
#else
    MotorCtrl(&motor_ctrl, &foc_param);
    abiEncoder.Update();
    // calibrate_mt_encoder(1.0f,0);
    Vector3D_t pwm = svm.GetSvpwmDuty(foc_param.vab);
    SET_DTC_A((uint16_t)pwm.uhU);
    SET_DTC_B((uint16_t)pwm.uhV);
    SET_DTC_C((uint16_t)pwm.uhW);
#endif
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc)
{
    if (hadc->Instance == ADC1) {
        static bool off_init = false, pwm_start = false;
        if (!off_init)
            off_init = current_sense.OffsetCalibrate();
        else {
            if (!pwm_start) {
                FocPwmStart(true, true, true, true, true, true);
                pwm_start = true;
            }
            // as5047p.Update();
            current_sense.Update();
            foc_param.current = current_sense.GetCurrents();
            FocHandle();
        }
    }
}
