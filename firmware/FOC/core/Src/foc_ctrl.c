#include "foc_ctrl.h"
#include "foc_cal.h"
#include "foc_cfg.h"
#include "vofa.h"
#include "tim.h"
#include "adc.h"
#include "as5047p.h"
#include "calibration.h"
#include "moc_spd.h"
 
#define SPEED_WINDOW_SIZE 16  // 窗口大小
#define ID_WINDOW_SIZE 8  // 窗口大小
#define IQ_WINDOW_SIZE 8  // 窗口大小
#define USE_SPD_PLL 0     // 使用PLL速度估算
#define USE_SPD_DET 1     // 使用微分速度检测
#define USE_POS_PID 0     // 使用位置环
#define USE_VOLT_POS 0    // 使用电压位置环

 float speed_buffer[SPEED_WINDOW_SIZE] = {0};  // 存储窗口内的数据
 float id_buffer[ID_WINDOW_SIZE] = {0};
 float iq_buffer[IQ_WINDOW_SIZE] = {0};
 volatile int speed_index = 0;    // 当前数据索引
 volatile int id_index = 0;
 volatile int iq_index = 0;
_RAM_FUNC float speed_moving_average_filter(float input, float *buffer, int size) 
{
    float sum = 0;                           // 窗口内数据的和
    static int speed_index;
    // 将新数据存入缓冲区
    buffer[speed_index] = input;

    // 更新索引（循环缓冲区）
    speed_index = (speed_index + 1) % size;

    // 计算窗口内数据的和
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }

    // 返回平均值
    return sum / size;
}


_RAM_FUNC float dcurrent_moving_average_filter(float input, float *buffer, int size)
{
    float sum = 0;                           // 窗口内数据的和
    static int dcurrent_index=0;
    // 将新数据存入缓冲区
    buffer[dcurrent_index] = input;

    // 更新索引（循环缓冲区）
    dcurrent_index = (dcurrent_index + 1) % size;

    // 计算窗口内数据的和
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }

    // 返回平均值
    return sum / size;
}


_RAM_FUNC float qcurrent_moving_average_filter(float input, float *buffer, int size)
{
    float sum = 0;                           // 窗口内数据的和
	static int qcurrent_index=0;
    // 将新数据存入缓冲区
    buffer[qcurrent_index] = input;

    // 更新索引（循环缓冲区）
    qcurrent_index = (qcurrent_index + 1) % size;

    // 计算窗口内数据的和
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }


    // 返回平均值
    return sum / size;
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

	foc.i_d = dcurrent_moving_average_filter(foc.i_d,id_buffer,ID_WINDOW_SIZE);
	foc.i_q = qcurrent_moving_average_filter(foc.i_q,iq_buffer,IQ_WINDOW_SIZE);
	SerialPidCtrl(&id_pid, id_set, foc.i_d);
	foc.v_d = id_pid.out_value;

	SerialPidCtrl(&iq_pid, iq_set, foc.i_q);
	foc.v_q = iq_pid.out_value;

	InvPark(&foc);
	SvpwmSector(&foc);
}


volatile float id=0,iq=2;
volatile int spd_cnt = 0,pos_cnt=0;
volatile int spd_set = 500,pos_set = 300;
volatile float speed_hz = 2000;
volatile float vbus,va_last,vb_last,vc_last;
volatile float pll_lpf_hz = 0.05f;
_RAM_FUNC void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{   
	static float pos_last,pos_now = 0.0f;
    static float rotor_vel_last = 0.0f;
	static float flag=0;
	
    if(htim->Instance == TIM1)
    {
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
            flag=1;
        }

        if(++pos_cnt==20)
        {
            ParallelPidCtrl(&pos_pid, pos_set, enc_para.pos_m/M_2PI*360);
            pos_cnt = 0;
        }

        if(++spd_cnt==10)
        {
#if USE_SPD_DET
            PosCalculate(&enc_para);
            pos_last = pos_now;
            pos_now = enc_para.raw_data;
            if(pos_now - pos_last < -8192)
                motor_cfg.rotor_vel = (pos_now - pos_last + 16383.f) / 16383.f * 60.f * speed_hz;
            else if(pos_now - pos_last > 8192)
                motor_cfg.rotor_vel = (pos_now - pos_last - 16383.f) / 16383.f * 60.f * speed_hz;
            else
                motor_cfg.rotor_vel = (pos_now - pos_last) / 16383.f * 60.f * speed_hz;
            motor_cfg.rotor_vel = speed_moving_average_filter(motor_cfg.rotor_vel,speed_buffer,SPEED_WINDOW_SIZE);

            //一阶低通滤波
            motor_cfg.rotor_vel = (0.05f * motor_cfg.rotor_vel + 0.95f * rotor_vel_last);
            rotor_vel_last = motor_cfg.rotor_vel;
#endif

#if USE_SPD_PLL
            PosCalculate(&enc_para);
			motor_cfg.rotor_vel = PllSpeedCtrl(&pll_spd,enc_para.pos_s);
			motor_cfg.rotor_vel = speed_moving_average_filter(motor_cfg.rotor_vel,speed_buffer,SPEED_WINDOW_SIZE);
			motor_cfg.rotor_vel = (1-pll_lpf_hz)*rotor_vel_last+pll_lpf_hz*motor_cfg.rotor_vel;
			rotor_vel_last = motor_cfg.rotor_vel;
#endif

#if USE_POS_PID
            IncreatParallePidCtrl(&speed_pid, pos_pid.out_value, motor_cfg.rotor_vel);
#else
            IncreatParallePidCtrl(&speed_pid, spd_set, motor_cfg.rotor_vel);
#endif
            spd_cnt = 0;
        }
		// VofaStart();
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

		foc.vbus = ((float)mc_adc.vbus)*VBUS_RATIO;

//		foc.vbus = 11.1f;
		foc.inv_vbus = 1.5f/(foc.vbus);
		
		PosCalculate(&enc_para);
#if USE_POS_PID
#if USE_VOLT_POS
		FocVolt(0.f,pos_pid.out_value,enc_para.pos_e);
#else
		FocCurrent(id,speed_pid.out_value,enc_para.pos_e);
#endif
#else
		FocCurrent(id,speed_pid.out_value,enc_para.pos_e);
//		 FocCurrent(id,iq,enc_para.pos_e);
//        FocVolt(0,0.5f,enc_para.pos_e);
#endif
//        VofaStart();
        //calibrate_mt_encoder(1.0f,0);
        FocPwmRun(&foc);
    }
}

