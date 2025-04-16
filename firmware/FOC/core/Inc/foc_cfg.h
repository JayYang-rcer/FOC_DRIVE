#ifndef __FOC_CFG_H
#define __FOC_CFG_H

#include "pid.h"
#include "moc_spd.h"

#define R_SENSE 0.003f  //采样电阻阻值
#define IOP 20.f //电流采样电阻放大倍数
#define IRATIO (3.3f/4096.f) / R_SENSE / IOP //电流采样电10.1压转换为电流值的系数
#define VBUS_RATIO (6.1f * 3.3f)/4096.0f //母线电压采样电压转换为电压值的系数
//#define VBUS_RATIO 0.0084723f //母线电压采样电压转换为电压值的系数

// Speed PID parameters
#define SPEED_PID_TIME_HZ 5000
#define CURRENT_LOOP_RATE 10000

typedef struct
{
    float rotor_pos;    //转子位置
    float rotor_vel;    //转子转速
    float rotor_rev;    //转子转向
    float rotor_epos;    //转子电角度
    float rotor_evel;    //转子电速度

    float rs;           //相电阻
    float ls;           //相电感
    float flux;         //磁链
    float jx;           //转动惯量
    float pn;           //极对数
    float delta;        //阻尼系数
}motor_cfg_t;

typedef struct
{
    float epos_acc;     //电角加速度
    float vd_set;       //直轴电压设定值
    float vq_set;       //交轴电压设定值
    float id_set;       //直轴电流设定值
    float iq_set;       //交轴电流设定值
    float speed_set;    //速度设定值
    float pos_set;	 	//位置设置
}motor_ctrl_t;

typedef struct 
{
    float ia;           //A相电流
    float ib;           //B相电流
    float ic;           //C相电流
	float va;
	float vb;
	float vc;
    float ia_offset;    //A相电流偏移
    float ib_offset;    //B相电流偏移
    float ic_offset;    //C相电流偏移
    float vbus;         //母线电压

    float temp;         //温度
}foc_adc_t;

typedef struct
{
    float vbus;
	float ibus;
	float inv_vbus; // 母线电压倒数

	float i_abs;

	float theta;   // 角度
	float sin_val; // 此角度对应的正弦值
	float cos_val; // 此角度对应的余弦值

	float i_a; // A 相电流
	float i_b; // B 相电流
	float i_c; // C 相电流

	float v_a; // A 相电压
	float v_b; // B 相电压
	float v_c; // C 相电压

	float i_d; // D 坐标系电流
	float i_q; // Q 坐标系电流


	float v_d; // D 坐标系电压
	float v_q; // Q 坐标系电压

	float i_alpha; // Alpha 坐标系电流
	float i_beta;  // Beta 坐标系电流


	float v_alpha; // Alpha 坐标系电压
	float v_beta;  // Beta 坐标系电压

	float dtc_a; // A 相 PWM 占空比
	float dtc_b; // B 相 PWM 占空比
	float dtc_c; // C 相 PWM 占空比
}foc_param_t;

extern foc_adc_t mc_adc;
extern foc_param_t foc;
extern motor_cfg_t motor_cfg;
extern motor_ctrl_t motor_ctrl;
extern pid_para_t id_pid, iq_pid;
extern pid_para_t speed_pid;
extern pid_para_t pos_pid;
extern pll_t pll_spd;

void GetCurrentOffset(foc_adc_t *mc_adc);
void FocPwmStart(void);
void FocPwmStop(void);
void FocPwmRun(foc_param_t *foc);
void MotorParaInit(void);

#endif
