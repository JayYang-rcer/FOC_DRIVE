#include "foc_ctrl.h"
#include "analog_sense.h"
#include "as5047p.h"
#include "calibration.h"
#include "drive_can.h"
#include "encoder.h"
#include "encoder_proc.h"
#include "filters.h"
#include "foc_cfg.h"
#include "foc_math.h"
#include "obersver.h"
#include "tim.h"
#include "menu_manager.h"
#include "oled_iic.h"
#include "stm32g4xx_it.h"

#define USE_SPD_PLL          1 // 使用PLL速度估算
#define USE_SPD_DET          0 // 使用微分速度检测
#define USE_POS_PID          0 // 使用位置环
#define USE_SENSERLESS       1
#define SENSERLESS_MIN_SPEED 500
#define SENSERLESS_MAX_SPEED 10000
#define USE_SLAVE_MODE       0
#define CURRENT_LOOP_FREQ    20000.0f // 电流环频率

// #define USE_CURRENTLOOP_FEEDBACK
unsigned char              oled_buffer[SCREEN_PAGE_NUM][SCREEN_COLUMN];
OLED                       oled(&hi2c1, (unsigned char *)(oled_buffer));
extern MenuManager         menuManager;
extern MenuManager::Config config_manager;

struct AppConfig {
    MotorParam_t motor = {
        .rs   = 37.5333f, // mOhm
        .ls   = 6.3f,     // uH
        .flux = 1.221f,   // mWb
        .jx   = 0.0001f,
        .pn   = 7,
    };
    PositionalPid::Config pi_current = {
        .kp           = motor.ls / 1000000.0f * 11000.0f * motor.pn * RPM_TO_RADS * 3.f,
        .ki           = motor.rs / 1000.0f * 11000.0f * motor.pn * RPM_TO_RADS,
        .output_min   = -(BATTERY_CELL * 4.0f) * ONE_BY_SQRT3,
        .output_max   = (BATTERY_CELL * 4.0f) * ONE_BY_SQRT3,
        .integral_max = (BATTERY_CELL * 4.0f) * ONE_BY_SQRT3,
        .anti_coeff   = 0.5f,
        .dt           = 1.0f / CURRENT_LOOP_FREQ,
    };
    PIController::Config pll_nonFlux = {
        .kp           = 1000,
        .ki           = 180000,
        .output_max   = 30000,
        .integral_max = 10000,
        .dt           = 1 / CURRENT_LOOP_FREQ, // 20khz
    };
    PIController::Config pll_pulsating_hfi = {
        .kp           = 1200,
        .ki           = 250000,
        .output_max   = 1000,
        .integral_max = 1000,
        .dt           = 1.0f / CURRENT_LOOP_FREQ, // 20khz
    };
    PIController::Config pll_sildemove = {
        .kp           = 1000,
        .ki           = 180000,
        .output_max   = 30000,
        .integral_max = 10000,
        .dt           = 1 / CURRENT_LOOP_FREQ, // 20khz
    };
    SlideMoveObserver::Config cfg_smo{
        .A   = 0.551139891f,
        .B   = 11.9589844,
        .ksw = 16.83f,
    };
    SpeedPLLMonitor::Config spd_monitor{
        .kp           = 10800.f / 60.f * M_2PI * 0.707f * 2.f,
        .ki           = (10800.f / 60.f * M_2PI) * (10800.f / 60.f * M_2PI),
        .output_max   = 12000,
        .integral_max = 10000,
        .dt           = 1 / 10000.0f,
    };
    InlineCurrentSense::SenseConfig current_sense = {
        .addr_u         = &ADC1->JDR1,
        .addr_v         = &ADC1->JDR2,
        .addr_w         = &ADC1->JDR3,
        .adc_trans_volt = 3.3f / 4096.0f,
        .resistance     = 0.001f,
        .gain           = 20.0f,
    };
    TempSense::SenseConfig temp_sense = {
        .addr_temp_ = &ADC2->JDR2,
        .temp_R25   = 10.f,
        .R_fixed    = 3.3f,
        .B          = 3380.0f,
    };
    VoltBusSense::SenseConfig vbus_sense = {
        .addr_bus_ = &ADC2->JDR1,
        .fcc_      = (6.1f * 3.3f) / 4096.0f,
    };
    AbiEncoder::Config abi_encoder = {
        .cpr        = 4000,
        .pole_pairs = 7,
        .tim_handle = TIM1,
    };
    As5407Encoder::Config as5047_cfg = {
        .cpr                     = 16384,
        .pole_pairs              = 7,
        .As5047RawDataGettingFun = As5047pRead,
    };
    IncrementalPid::Config pid_spdCfg = {
        .kp        = 0.0045f,
        .ki        = 0.0035f,
        .kd        = 0.0f,
        .out_limit = 15.f,
        .dt        = 1 / 2000.0f,
    };
} app_config;

 LowPassFilter      lpf_id(0.1f), lpf_iq(0.1f);
 LowPassFilter      lpf_speed(0.05f);
 Svpwm              svm(16.0f, 4250);
 InlineCurrentSense current_sense(app_config.current_sense);
 TempSense          temp_sense(app_config.temp_sense);
 VoltBusSense       vbus_sense(app_config.vbus_sense);
 IncrementalPid     pid_spd;
 PositionalPid      pi_id, pi_iq;
 SpeedPLLMonitor    spdMonitor;
 AbiEncoder         abiEncoder;
 As5407Encoder      as5047p;
 NonFluxObserver    nonFluxObserver;
 PulsatingHFI       hfiObserver;
 SlideMoveObserver  smoObserver;
 FocController      focController;

void ResourceInit(void)
{
    pid_spd.Init(app_config.pid_spdCfg);
    abiEncoder.Init(app_config.abi_encoder);
    as5047p.Init(app_config.as5047_cfg);
    nonFluxObserver.Init(app_config.pll_nonFlux, app_config.motor, 10000);
    spdMonitor.Init(app_config.spd_monitor);
    hfiObserver.Init(app_config.pll_pulsating_hfi, app_config.motor, 2000);
    smoObserver.Init(app_config.pll_sildemove, app_config.cfg_smo);
    pi_id.Init(app_config.pi_current);
    pi_iq.Init(app_config.pi_current);
    focController.SetCurrentSense(&current_sense);
    oled.Init();
    menuManager.Init(&config_manager);
}

_RAM_FUNC void FocVolt(float vd_ref, float vq_ref, float pos)
{
    foc_param.theta = pos;
    WRAP_0_2PI(foc_param.theta);
    Vector2Df_t SinCos;
    SinCosVal(&SinCos, foc_param.theta);
    Park(&foc_param.idq, &foc_param.iab, &SinCos);

    foc_param.vdq.r.d = vd_ref;
    foc_param.vdq.r.q = vq_ref;
    InvPark(&foc_param.vab, &foc_param.vdq, &SinCos);
}

_RAM_FUNC void FocIFVolt(float id_ref, float pos)
{
    foc_param.theta = pos;
    WRAP_0_2PI(foc_param.theta);
    Vector2Df_t SinCos;
    SinCosVal(&SinCos, foc_param.theta);
    Park(&foc_param.idq, &foc_param.iab, &SinCos);

    float error_id    = id_ref - foc_param.idq.r.d;
    foc_param.vdq.r.d = pi_id.Calculate(error_id);
    foc_param.vdq.r.q = 0;
    InvPark(&foc_param.vab, &foc_param.vdq, &SinCos);
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
    WRAP_0_2PI(foc_param.theta);
    Vector2Df_t SinCos;
    SinCosVal(&SinCos, foc_param.theta);
    Park(&foc_param.idq, &foc_param.iab, &SinCos);

#ifdef USE_CURRENTLOOP_FEEDBACK
    IdPidCtrl(&id_pid, id_set, foc_param.i_d);
    foc_param.vdq.r.d = id_pi.out_value;

    IqPidCtrl(&iq_pid, iq_set, foc_param.i_q);
    foc_param.vdq.r.q = iq_pi.out_value;
#endif
    float error_id    = id_set - foc_param.idq.r.d;
    foc_param.vdq.r.d = pi_id.Calculate(error_id);

    float error_iq    = iq_set - foc_param.idq.r.q;
    foc_param.vdq.r.q = pi_iq.Calculate(error_iq);

    InvPark(&foc_param.vab, &foc_param.vdq, &SinCos);
}

_RAM_FUNC void HfiVolt(float vd, float vq, float pos)
{
    foc_param.theta = pos;
    Vector2Df_t SinCos;
    SinCosVal(&SinCos, foc_param.theta);

    foc_param.vdq.r.d = vd + +hfiObserver.GetInjectVoltage(1.2f);
    foc_param.vdq.r.q = vq;

    InvPark(&foc_param.vab, &foc_param.vdq, &SinCos);
}

_RAM_FUNC void HfiCurrent(float id_set, float iq_set, float pos)
{
    foc_param.theta = pos;
    Vector2Df_t SinCos;
    SinCosVal(&SinCos, foc_param.theta);
    Park(&foc_param.idq, &foc_param.iab, &SinCos);
    Vector2Df_t idq_f = hfiObserver.GetDqCurrentLow(foc_param.idq);
    lpf_id.Update(&idq_f.r.d);
    lpf_iq.Update(&idq_f.r.q);

    float error_id    = id_set - idq_f.r.d;
    foc_param.vdq.r.d = pi_id.Calculate(error_id) + hfiObserver.GetInjectVoltage(1.2f);

    float error_iq    = iq_set - idq_f.r.q;
    foc_param.vdq.r.q = pi_iq.Calculate(error_iq);

    InvPark(&foc_param.vab, &foc_param.vdq, &SinCos);
}

volatile float vbus;
void AnalogSampleUpdate(FocParam_t *foc)
{
    vbus      = vbus_sense.GetBusVolt();
    foc->vbus = vbus;
}

volatile float speed_hz = 10000;
void           EncoderDataCalc(enc_para_t *enc, MotorCfg_t *motor)
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
    motor->rotor_vel = spdMonitor.GetSpeed(enc->pos);
    lpf_speed.Update(&motor->rotor_vel);
#endif
}

uint16_t          ms_cnt      = 0;
extern uint16_t   can_recieveFlag;
__RAM_FUNC void   Encoder_Idle(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_CC1);
    if (++ms_cnt == 200) {
        ms_cnt      = 0;
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
        FocVolt(ctrl->vd_set, fRefSlope, nonFluxObserver.GetElectAngle());
#else
        FocVolt(ctrl->vd_set, fRefSlope, enc_para.pos_e);
#endif
        motor_cfg.fRefSlope = fRefSlope;
        break;
    }

    case FOC_CURRENT_CTRL: {
#if USE_SENSERLESS
        FocCurrent(ctrl->id_set, ctrl->iq_set, nonFluxObserver.GetElectAngle());
#else
        FocCurrent(ctrl->id_set, ctrl->iq_set, enc_para.pos_e);
#endif
        break;
    }

    case FOC_SPEED_CTRL: {
        static float speedloop_out = 0.0f;
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
            float error   = RefSlope - nonFluxObserver.GetVelocity();
            speedloop_out = pid_spd.Calculate(error);
#else
            IncreatParallePidCtrl(&speed_pid, RefSlope, motor_cfg.rotor_vel);
#endif
            motor_cfg.RefSlope = RefSlope;
            ctrl->spd_cnt      = 0;
        }
#if USE_SENSERLESS
        FocCurrent(ctrl->id_set, speedloop_out, nonFluxObserver.GetElectAngle());
#else
        FocCurrent(ctrl->id_set, speedloop_out, enc_para.pos_e);
#endif
        break;
    }

    case FOC_POSITION_CTRL: {
        //        static float i_out = 0.0f;
        //        if (++ctrl->pos_cnt == 20) // 20khz/20 = 1khz
        //        {
        //            ParallelPidCtrl(&pos_pid, ctrl->pos_set, enc_para.pos_m / M_2PI * 360);
        //            ctrl->pos_cnt = 0;
        //        }
        //        if(++ctrl->spd_cnt == 10)
        //        {
        //            i_out = pid_spd.Calculate(pos_pid.out_value - motor_cfg.rotor_vel);
        //            ctrl->spd_cnt = 0;
        //        }
        //        FocCurrent(ctrl->id_set, i_out, enc_para.pos_e);
        break;
    }

    case FOC_SENSORLESS_CTRL: {
        if (++ctrl->spd_cnt == 10) // 20khz/20 = 1khz
        {
            //            IncreatParallePidCtrl(&speed_pid, ctrl->speed_set, motor_cfg.rotor_vel);
            ctrl->spd_cnt = 0;
        }
        break;
    }

    case FOC_HFI: {
        hfiObserver.Update(foc_param.iab);
        static float i_out = 0.0f;
        if (++motor_ctrl.spd_cnt == 10) {
            static int pos     = 0;
            float      error   = ctrl->speed_set - hfiObserver.GetVelocity();
            i_out              = pid_spd.Calculate(error);
            motor_ctrl.spd_cnt = 0;
        }
        // 高频注入Id偏置，防止电机在速度为0时的观测角度发散
        if (motor_ctrl.speed_set != 0) {
            HfiCurrent(6, i_out, hfiObserver.GetElectAngle());
        } else {
            HfiCurrent(0, 0, hfiObserver.GetElectAngle());
        }
    } break;

    default:
        MotorCtrlReset(ctrl);
        FocVolt(ctrl->vd_set, ctrl->vq_set, foc->theta);
        break;
    }
}

_RAM_FUNC void FocHandle(void)
{
    AnalogSampleUpdate(&foc_param);
#if (USE_SENSERLESS == 0)
    PosCalculate(&enc_para);
#endif
    nonFluxObserver.Update(foc_param.vab, foc_param.iab);
//    smoObserver.Update(foc_param.vab, foc_param.iab);
    foc_param.iab  = current_sense.GetAlphaBeta();
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
            // foc_param.current = current_sense.GetCurrents();
            foc_param.current = focController.sense_->GetCurrents();
            FocHandle();
        }
    }
}

void FocController::SetCurrentSense(PhaseSenseBase *sense)
{
    sense_ = sense;
}

void FocController::Run()
{
}
