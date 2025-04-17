#include "foc_ctrl.h"
#include "foc_cal.h"
#include "foc_cfg.h"
#include "vofa.h"
#include "tim.h"
#include "adc.h"
#include "encoder_proc.h"
#include "calibration.h"
#include "moc_spd.h"
 
#define SPEED_WINDOW_SIZE 16  // 窗口大小
#define USE_SPD_PLL 1     // 使用PLL速度估算
#define USE_SPD_DET 0     // 使用微分速度检测
#define USE_POS_PID 0     // 使用位置环
#define USE_VOLT_POS 0    // 使用电压位置环
#define USE_ENCODER 1

float speed_buffer[SPEED_WINDOW_SIZE] = {0};  // 存储窗口内的数据
MovingAverage_t speed_maf = {.buffer = speed_buffer, .size = SPEED_WINDOW_SIZE, .index = 0};

smo_param_t smo;

void SmoParamInit(smo_param_t *smo)
{
    smo->A = expf(-(motor_cfg.rs/1000)/(motor_cfg.ls/1000000)/10000);
    smo->B = (1 - smo->A)/(motor_cfg.rs/1000);
    smo->ksw = 0.12f;
}

float ualpha;
float ubeta;
void SmoViewer(foc_param_t *foc, smo_param_t *smo)
{
    static float valpha_last, vbeta_last;
    ualpha = (2*foc->v_a-foc->v_b-foc->v_c)/3.f;
    ubeta = (foc->v_b - foc->v_c) * ONE_BY_SQRT3;
    Clarke(foc);

    smo->valpah = smo->ksw*SIGN(smo->ialpha_view - foc->i_alpha);
    smo->vbeta = smo->ksw*SIGN(smo->ibeta_view - foc->i_beta);

    smo->ialpha_view = smo->A * smo->ialpha_view_last + smo->B * (ualpha - smo->valpah);
    smo->ibeta_view = smo->A * smo->ibeta_view_last + smo->B * (ubeta - smo->vbeta);

    //计算滤波后的拓展反电动势
    smo->valpah = 0.1*smo->valpah + 0.9f*valpha_last;
    smo->vbeta = 0.1*smo->vbeta + 0.9f*vbeta_last;

    smo->ialpha_view_last = smo->ialpha_view;
    smo->ibeta_view_last = smo->ibeta_view;
    vbeta_last = smo->vbeta;
    valpha_last = smo->valpah;

    smo->Ealpha = smo->valpah;
    smo->Ebeta = smo->vbeta;
}


_RAM_FUNC void FocVolt(float vd_ref, float vq_ref, float pos)
{
    foc.theta = pos;
    WRAP_0_2PI(foc.theta);
    SinCosVal(&foc);
    Clarke(&foc);
    Park(&foc);

    foc.v_d = vd_ref;
    foc.v_q = vq_ref;
    InvPark(&foc);
    SvpwmSector(&foc);
}


_RAM_FUNC void FocCurrent(float id_set, float iq_set, float pos)
{
	foc.theta = pos;
	SinCosVal(&foc);
	
	Clarke(&foc);
	Park(&foc);

	SerialPidCtrl(&id_pid, id_set, foc.i_d);
	foc.v_d = id_pid.out_value;

	SerialPidCtrl(&iq_pid, iq_set, foc.i_q);
	foc.v_q = iq_pid.out_value;

	InvPark(&foc);
	SvpwmSector(&foc);
}


volatile float id=0,iq=2,uq_set=0.5f;
volatile int spd_cnt = 0,pos_cnt=0;
volatile int spd_set = 500,pos_set = 300;
volatile float speed_hz = 10000;
volatile float vbus;
volatile float pll_lpf_hz = 0.01f;
__RAM_FUNC void Encoder_Idle(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim2,TIM_FLAG_CC1);
    static float pos_last,pos_now = 0.0f;
    static float rotor_vel_last = 0.0f;
    static float flag=0;
    if(flag==0)
    {
#if USE_VOLT_POS
        pos_pid.lpf_d = 0.1f;
			pos_pid.kp = 0.035f;
			pos_pid.kd = 0.0002f;
			pos_pid.out_max = 0.8f;
			pos_pid.out_min = -0.8f;
#else
        pos_pid.kp = 3.5f;
        pos_pid.kd = 0.005f;
        pos_pid.out_max = 300.f;
        pos_pid.out_min = -300.f;
#endif
        SmoParamInit(&smo);
        flag=1;
    }

    SmoViewer(&foc,&smo);
    PosCalculate(&enc_para);
    if(++pos_cnt==10)
    {
        ParallelPidCtrl(&pos_pid, pos_set, enc_para.pos_m/M_2PI*360);
        pos_cnt = 0;
    }

#if USE_SPD_DET
    pos_last = pos_now;
    pos_now = enc_para.raw_data;
    if(pos_now - pos_last < -8192)
        motor_cfg.rotor_vel = (pos_now - pos_last + 16383.f) / 16383.f * 60.f * speed_hz;
    else if(pos_now - pos_last > 8192)
        motor_cfg.rotor_vel = (pos_now - pos_last - 16383.f) / 16383.f * 60.f * speed_hz;
    else
        motor_cfg.rotor_vel = (pos_now - pos_last) / 16383.f * 60.f * speed_hz;
    //一阶低通滤波
    motor_cfg.rotor_vel = (0.05f * motor_cfg.rotor_vel + 0.95f * rotor_vel_last);
    rotor_vel_last = motor_cfg.rotor_vel;
    MoveAverageFilter(&speed_maf, &motor_cfg.rotor_vel);
#endif

#if USE_SPD_PLL
        PosCalculate(&enc_para);
		motor_cfg.rotor_vel = PllSpeedCtrl(&pll_spd,enc_para.pos_s);
		motor_cfg.rotor_vel = (1-pll_lpf_hz)*rotor_vel_last+pll_lpf_hz*motor_cfg.rotor_vel;
		rotor_vel_last = motor_cfg.rotor_vel;
        MoveAverageFilter(&speed_maf, &motor_cfg.rotor_vel);
#endif

#if USE_POS_PID
        IncreatParallePidCtrl(&speed_pid, pos_pid.out_value, motor_cfg.rotor_vel);
#else
        if(++spd_cnt==10)
        {
            IncreatParallePidCtrl(&speed_pid, spd_set, motor_cfg.rotor_vel);
            spd_cnt=0;
        }
#endif
}


volatile int curr=0;
_RAM_FUNC void FocHandle(void)
{
    __HAL_TIM_CLEAR_FLAG(&htim1,TIM_FLAG_CC1);
	 VofaStart();
	mc_adc.ia = ADC1->JDR3;
	mc_adc.ib = ADC1->JDR2;
	mc_adc.ic = ADC1->JDR1;
	mc_adc.va = ADC2->JDR3;
	mc_adc.vb = ADC2->JDR2;
	mc_adc.vc = ADC2->JDR1;
	mc_adc.vbus = ADC2->JDR4;
	foc.v_a = (mc_adc.va)/4096.f * 3.3f* 4.5f;
	foc.v_b = (mc_adc.vb)/4096.f * 3.3f* 4.5f;
	foc.v_c = (mc_adc.vc)/4096.f * 3.3f* 4.5f;


	foc.i_a = ((float)mc_adc.ia - mc_adc.ia_offset)*IRATIO;
	foc.i_b = ((float)mc_adc.ib - mc_adc.ib_offset)*IRATIO;
	foc.i_c = ((float)mc_adc.ic - mc_adc.ic_offset)*IRATIO;
	vbus = 	((float)mc_adc.vbus)* VBUS_RATIO;

//	foc.vbus = ((float)mc_adc.vbus)*VBUS_RATIO;

	foc.vbus = 11.1f;
	foc.inv_vbus = 1.5f/(foc.vbus);

#if USE_POS_PID
#if USE_VOLT_POS
		FocVolt(0.f,pos_pid.out_value,enc_para.pos_e);
#else
		FocCurrent(id,speed_pid.out_value,enc_para.pos_e);
#endif
#else
//		FocCurrent(id,speed_pid.out_value,enc_para.pos_e);
		 FocCurrent(id,iq,enc_para.pos_e);
//        FocVolt(0,uq_set,enc_para.pos_e);
#endif
//        VofaStart();
    //calibrate_mt_encoder(1.0f,0);
    FocPwmRun(&foc);
}

