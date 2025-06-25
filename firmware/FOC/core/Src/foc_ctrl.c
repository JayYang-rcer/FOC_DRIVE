#include "foc_ctrl.h"
#include "foc_math.h"
#include "foc_cfg.h"
#include "vofa.h"
#include "tim.h"
#include "adc.h"
#include "encoder_proc.h"
#include "calibration.h"
#include "filter.h"
#include "foc_identify.h"

#define SPEED_WINDOW_SIZE 16  // 窗口大小
#define CURRENT_WINDOW_SIZE 4  // 窗口大小

#define USE_SPD_PLL 1     // 使用PLL速度估算
#define USE_SPD_DET 0     // 使用微分速度检测
#define USE_POS_PID 0     // 使用位置环
#define USE_ENCODER 1

float speed_buffer[SPEED_WINDOW_SIZE] = {0};  // 存储窗口内的数据
MovingAverage_t speed_maf = {.buffer = speed_buffer, .size = SPEED_WINDOW_SIZE, .index = 0};

lpf_t lpf_iq = {.in_last = 0.0f, .trust = 0.1f}; // iq低通滤波器
lpf_t lpf_id = {.in_last = 0.0f, .trust = 0.1f}; // id低通滤波器

_RAM_FUNC void FocVolt(float vd_ref, float vq_ref, float pos)
{
    foc_param.theta = pos;
    WRAP_0_2PI(foc_param.theta);
    SinCosVal(&foc_param);
    Clarke(&foc_param);
    Park(&foc_param);

    foc_param.v_d = vd_ref;
    foc_param.v_q = vq_ref;
    InvPark(&foc_param);
    SvpwmSector(&foc_param);
}


float IqPidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{
    static float uq0;
    pid->target_value = target_value;
    pid->fback_value = fdback_value;
    pid->error = pid->target_value - pid->fback_value;
    pid->d_error = pid->error - pid->pre_error;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error;
    AbsLimit(pid->i_term, pid->i_term_max);

    pid->d_term = pid->d_error*pid->kd;

    uq0 = motor_cfg.rotor_vel * motor_cfg.pn / 60.f * M_2PI * (motor_cfg.ls / 1000 * foc_param.i_d + motor_cfg.flux) / 1000;
    AbsLimit(uq0, pid->i_term_max);

    pid->out_value = pid->p_term + pid->i_term + pid->d_term + uq0;
    AbsLimit(pid->out_value, pid->out_max);

    return pid->out_value;
}


float IdPidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{
    static float ud0;
    pid->target_value = target_value;
    pid->fback_value = fdback_value;
    pid->error = pid->target_value - pid->fback_value;
    pid->d_error = pid->error - pid->pre_error;

    pid->p_term = pid->kp * pid->error;
    pid->i_term += pid->ki * pid->error;
    AbsLimit(pid->i_term, pid->i_term_max);

    pid->d_term = pid->d_error*pid->kd;

    ud0 = motor_cfg.rotor_vel * motor_cfg.pn / 60.f * M_2PI * motor_cfg.ls / 1000000.f * foc_param.i_q;
    AbsLimit(ud0, pid->i_term_max);

    pid->out_value = pid->p_term + pid->i_term + pid->d_term - ud0;
    AbsLimit(pid->out_value, pid->out_max);

    return pid->out_value;
}


_RAM_FUNC void FocCurrent(float id_set, float iq_set, float pos)
{
    foc_param.theta = pos;
	SinCosVal(&foc_param);

	Clarke(&foc_param);
	Park(&foc_param);

    LowPassFilter(&foc_param.i_d, &lpf_id);
    LowPassFilter(&foc_param.i_q, &lpf_iq);

//    ParallelPidCtrl(&id_pid, id_set, foc_param.i_d);
    IdPidCtrl(&id_pid, id_set, foc_param.i_d);
    foc_param.v_d = id_pid.out_value;

//    ParallelPidCtrl(&iq_pid, iq_set, foc_param.i_q);
    IqPidCtrl(&iq_pid, iq_set, foc_param.i_q);
    foc_param.v_q = iq_pid.out_value;

	InvPark(&foc_param);
    SvpwmSector(&foc_param);
}


_RAM_FUNC void HfiVolt(float vd, float vq, float pos)
{
    foc_param.theta = pos;

    SinCosVal(&foc_param);
    Clarke(&foc_param);

//    LowPassFilter(&foc_param.i_d, &lpf_id);
//    LowPassFilter(&foc_param.i_q, &lpf_iq);

    static int cnt = 0;
    static float ud_inject;
    if(++cnt == 4)
    {
        ud_inject = HfiInjectSign(hfi_param.inject_U);
        hfi_param.sign = SIGN(ud_inject);
        cnt=0;
    }
    foc_param.v_d = vd + ud_inject;
    foc_param.v_q = vq;

    InvPark(&foc_param);
    SvpwmSector(&foc_param);
}


_RAM_FUNC void HfiCurrent(float id_set, float iq_set, float pos)
{
    foc_param.theta = pos;
    SinCosVal(&foc_param);

    Clarke(&foc_param);
    Park(&foc_param);
    IdqToIdqF(&foc_param, &hfi_param);

    static int cnt = 0;
    static float ud_inject;
    if(++cnt == 4)
    {
        ud_inject = HfiInjectSign(hfi_param.inject_U);
        hfi_param.sign = SIGN(ud_inject);
        cnt=0;
    }
//    ParallelPidCtrl(&id_pid, id_set, hfi_param.idq_f.id);
    IdPidCtrl(&id_pid, id_set, foc_param.i_d);
    foc_param.v_d = id_pid.out_value + ud_inject;

//    ParallelPidCtrl(&iq_pid, iq_set, hfi_param.idq_f.iq);
    IqPidCtrl(&iq_pid, iq_set, foc_param.i_q);
    foc_param.v_q = iq_pid.out_value;

    InvPark(&foc_param);
    SvpwmSector(&foc_param);
}


volatile float vbus;
void CurrentUpdate(foc_adc_t* adc, foc_param_t* foc)
{
    adc->ia = ADC1->JDR3;
    adc->ib = ADC1->JDR2;
    adc->ic = ADC1->JDR1;
    adc->va = ADC2->JDR3;
    adc->vb = ADC2->JDR2;
    adc->vc = ADC2->JDR1;
    adc->vbus = ADC2->JDR4;

    foc->v_a = (adc->va)/4096.f * 3.3f* 10.f;
    foc->v_b = (adc->vb)/4096.f * 3.3f* 10.f;
    foc->v_c = (adc->vc)/4096.f * 3.3f* 10.f;

    foc->i_a = ((float)adc->ia - adc->ia_offset)*IRATIO;
    foc->i_b = ((float)adc->ib - adc->ib_offset)*IRATIO;
    foc->i_c = ((float)adc->ic - adc->ic_offset)*IRATIO;

    vbus = ((float)adc->vbus)* VBUS_RATIO;
    foc->vbus = ((float)adc->vbus)*VBUS_RATIO;
}


volatile float pll_lpf_hz = 0.01f,pll_angle;
volatile float speed_hz = 10000;
void EncoderDataCalc(enc_para_t* enc, motor_cfg_t* motor)
{
    static float rotor_vel_last = 0.0f;
#if USE_SPD_DET
    pos_last = pos_now;
    pos_now = enc_para.raw_data;
    if(pos_now - pos_last < -8192)
        motor.rotor_vel = (pos_now - pos_last + 16383.f) / 16383.f * 60.f * speed_hz;
    else if(pos_now - pos_last > 8192)
        motor.rotor_vel = (pos_now - pos_last - 16383.f) / 16383.f * 60.f * speed_hz;
    else
        motor.rotor_vel = (pos_now - pos_last) / 16383.f * 60.f * speed_hz;
    //一阶低通滤波
    motor.rotor_vel = (0.05f * motor.rotor_vel + 0.95f * rotor_vel_last);
    rotor_vel_last = motor.rotor_vel;
    MoveAverageFilter(&speed_maf, &motor.rotor_vel);
#endif

#if USE_SPD_PLL

//    if(motor_ctrl.ctrl_mode == FOC_SENSORLESS_CTRL)
//		motor_cfg.rotor_vel = pll_smo.out_value*60.f/M_2PI/7.f;     //使用滑膜速度输出
//    else
        motor->rotor_vel = PllSpeedCtrl(&pll_spd,enc->pos_s);       //编码器速度输出

    motor->rotor_vel = (1-pll_lpf_hz)*rotor_vel_last+pll_lpf_hz*motor->rotor_vel;
    rotor_vel_last = motor->rotor_vel;
    MoveAverageFilter(&speed_maf, &motor->rotor_vel);
#endif
}


volatile int spd_cnt = 0,pos_cnt=0;
int change_flag=0;
__RAM_FUNC void Encoder_Idle(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
    static float pos_last,pos_now = 0.0f;
    static float flag=0;
    if(flag==0)
    {
        motor_ctrl.speed_set = 500;
        motor_ctrl.pos_set = 300;
        SmoParamInit(&smo_param);
//        pll_hfi.angle_out -= M_2PI;
        flag=1;
    }

    EncoderDataCalc(&enc_para, &motor_cfg);
//    pll_angle = SmoViewer(&foc_param, &smo_param);
}


void MotorCtrl(motor_ctrl_t *ctrl, foc_param_t *foc)
{
    switch (ctrl->ctrl_mode)
    {
        case FOC_VF_CTRL:
        {
            foc->theta += 0.007f;
            FocVolt(ctrl->vd_set, ctrl->vq_set, foc->theta);
            break;
        }

        case FOC_VOLT_CTRL:
        {
            FocVolt(ctrl->vd_set, ctrl->vq_set, enc_para.pos_e);
            break;
        }

        case FOC_CURRENT_CTRL:
        {
            FocCurrent(ctrl->id_set, ctrl->iq_set, enc_para.pos_e);
            break;
        }

        case FOC_SPEED_CTRL:
        {
            if (++ctrl->spd_cnt == 20) // 20khz/20 = 1khz
            {
                IncreatParallePidCtrl(&speed_pid, ctrl->speed_set, motor_cfg.rotor_vel);
                ctrl->spd_cnt = 0;
            }
            FocCurrent(ctrl->id_set, speed_pid.out_value, enc_para.pos_e);
            break;
        }

        case FOC_POSITION_CTRL:
        {
            if (++ctrl->pos_cnt == 20) // 20khz/20 = 1khz
            {
                ParallelPidCtrl(&pos_pid, ctrl->pos_set, enc_para.pos_m / M_2PI * 360);
                IncreatParallePidCtrl(&speed_pid, pos_pid.out_value, motor_cfg.rotor_vel);
                ctrl->pos_cnt = 0;
            }
            FocCurrent(ctrl->id_set, speed_pid.out_value, enc_para.pos_e);
            break;
        }

        case FOC_SENSORLESS_CTRL: 
        {
            if (++ctrl->spd_cnt == 10) // 20khz/20 = 1khz
            {
                IncreatParallePidCtrl(&speed_pid, ctrl->speed_set, motor_cfg.rotor_vel);
                ctrl->spd_cnt = 0;
            }
            FocCurrent(ctrl->id_set,speed_pid.out_value,pll_angle);
            break;
        }

        case FOC_HFI_TEST:
        {
            HfiAngleCalc(foc, &hfi_param);
            if(motor_ctrl.spd_cnt<5000)
                motor_ctrl.spd_cnt++;
            else
            {
                static bool hfi_init=false;
                if(!hfi_init)
                {
                    hfi_init = HfiNsIdentify(&hfi_param,foc);
                }
                else
                {
                    if(motor_ctrl.spd_cnt<20000)
                    {
                        motor_ctrl.spd_cnt++;
                        HfiVolt(0,0,hfi_param.theta_e);
                    }
                    else
                    {
                        static int test=0;

//                        if(test <= 10000)
//                        {
//                            HfiVolt(motor_ctrl.vd_set,motor_ctrl.vq_set,enc_para.pos_e);
//                            test++;
//                        }
//                        else
//                        {
////                            HfiVolt(motor_ctrl.vd_set,motor_ctrl.vq_set,enc_para.pos_e);
//                            HfiVolt(motor_ctrl.vd_set,motor_ctrl.vq_set,hfi_param.theta_e);
//                        }

                        if(++test == 10)
                        {
                            IncreatParallePidCtrl(&speed_pid, ctrl->speed_set, motor_cfg.rotor_vel);
                            test=0;
                        }
                        HfiCurrent(6, speed_pid.out_value, hfi_param.theta_e);
                    }
                }
            }
            break;
        }
    }
}


_RAM_FUNC void FocHandle(void)
{
    VofaStart();
    CurrentUpdate(&mc_adc, &foc_param);
//    foc_param.vbus = 3.7f * 3.f;
//    foc_param.vbus = 12.2f;

#if USE_POS_PID
#if USE_VOLT_POS
		FocVolt(0.f,pos_pid.out_value,enc_para.pos_e);
#else
		FocCurrent(id,speed_pid.out_value,enc_para.pos_e);
#endif
#else
    PosCalculate(&enc_para);
    motor_ctrl.ctrl_mode = FOC_HFI_TEST;
    MotorCtrl(&motor_ctrl, &foc_param);
    //calibrate_mt_encoder(1.0f,0);
    FocPwmRun(&foc_param);
#endif
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if(motor_ctrl.foc_init)
        FocHandle();
}
