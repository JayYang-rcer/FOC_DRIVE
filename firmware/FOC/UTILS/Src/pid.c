#include "pid.h"
#include "util.h"
#include "foc_cfg.h"

float PID_Abs_Limit(float a, float ABS_MAX)
{
    if(a > ABS_MAX)
        a = ABS_MAX;
		
    if(a < -ABS_MAX)
        a = -ABS_MAX;
		return a;
}


float IpCtrl()
{
	return 0;
}

float IncreatParallePidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{

	pid->target_value = target_value;
	pid->fback_value = fdback_value;
	pid->error = pid->target_value - pid->fback_value;
	pid->d_error = pid->error - pid->pre_error;
	
	// LowPassFilter(&pid->error, pid->lpf_error);
	// LowPassFilter(&d_error, pid->lpf_d);
	if(ABS(pid->error) < pid->deadband)
		return 0;
	pid->p_term = pid->kp*pid->d_error;
	pid->i_term = PID_Abs_Limit(pid->ki*pid->d_error,pid->i_term_max);

	pid->d_term = pid->d_error*pid->kd * SPEED_PID_TIME_HZ;
	
	if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min) pid->i_term = pid->i_term_min;
	
	pid->out_value = pid->p_term + pid->i_term + pid->d_term;

	if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min) pid->out_value = pid->out_min;
	
	return pid->out_value;
}

float ParallelPidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{
	pid->target_value = target_value;
	pid->fback_value = fdback_value;
	pid->error = pid->target_value - pid->fback_value;
	pid->d_error = pid->error - pid->pre_error;
	 
	// LowPassFilter(&pid->error, pid->lpf_error);
	// LowPassFilter(&d_error, pid->lpf_d);
	if(ABS(pid->error) < pid->deadband)
		return 0;
	
	pid->p_term = pid->kp * pid->error;
	pid->i_term += pid->ki * pid->error;
    if(pid->error==0||pid->target_value==0)
    {                                   /*积分退饱和处理*/
        pid->i_term *= 0.99f;         /*清除累计误差*/
    }
	
	if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min) pid->i_term = pid->i_term_min;
	
	pid->d_term = pid->d_error*pid->kd;

	pid->out_value = pid->p_term + pid->i_term + pid->d_term;

	if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min) pid->out_value = pid->out_min;
	
	return pid->out_value;
}

float SerialPidCtrl(pid_para_t *pid, float target_value, float fdback_value)
{
	pid->target_value = target_value;
	pid->fback_value = fdback_value;
	pid->error = pid->target_value - pid->fback_value;
	
	pid->p_term = pid->kp * pid->error;
	pid->i_term += pid->ki * pid->p_term;
	
	if (pid->i_term > pid->i_term_max) pid->i_term = pid->i_term_max;
	else if (pid->i_term < pid->i_term_min) pid->i_term = pid->i_term_min;
	
	pid->out_value = pid->p_term + pid->i_term;

	if (pid->out_value > pid->out_max) pid->out_value = pid->out_max;
	else if (pid->out_value < pid->out_min) pid->out_value = pid->out_min;
	
	return pid->out_value;
}
