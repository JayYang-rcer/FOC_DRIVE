#include "vofa.h"
#include "foc_cfg.h"
#include "tim.h"
#include "util.h"
#include "as5047p.h"

#define PWM_ARR() __HAL_TIM_GET_AUTORELOAD(&htim8)
#define MAX_BUFFER_SIZE 128
volatile uint8_t send_buf[MAX_BUFFER_SIZE];
volatile uint16_t cnt = 0;
/**
***********************************************************************
* @brief:      vofa_start(void)
* @param:		   void
* @retval:     void
* @details:    发送数据给上位机
***********************************************************************
**/
void VofaStart(void)
{
	// VofaSendData(1,foc.v_alpha);
	// VofaSendData(2,foc.v_beta);
	VofaSendData(1,foc.i_q);
	VofaSendData(2,foc.i_d);
//	VofaSendData(1,foc.i_a);
//	VofaSendData(2,foc.i_b);
//	VofaSendData(3,foc.i_c);


//    static float va_last= 0, vb_last= 0, vc_last= 0;
//    foc.v_a = foc.v_a - va_last;
//    foc.v_b = foc.v_b - vb_last;
//    foc.v_c = foc.v_c - vc_last;
//    foc.v_a = 0.9f*va_last + 0.1f*foc.v_a;
//    foc.v_b = 0.9f*vb_last + 0.1f*foc.v_b;
//    foc.v_c = 0.9f*vc_last + 0.1f*foc.v_c;
//	VofaSendData(1,foc.v_a);
//	VofaSendData(2,foc.v_b);
//	VofaSendData(3,foc.v_c);
//    va_last = foc.v_a;
//    vb_last = foc.v_b;
//    vc_last = foc.v_c;
//
//    static float valpha_last= 0;
//    static float vbeta_last= 0;
//    float valpha = (2*foc.v_a-foc.v_b-foc.v_c)/3.f;
//    float vbeta = (foc.v_b - foc.v_c) * ONE_BY_SQRT3;
//    valpha = 0.9f*valpha_last + 0.1f*valpha;
//    vbeta = 0.9f*vbeta_last + 0.1f*vbeta;
//    VofaSendData(1,valpha);
//    VofaSendData(1,vbeta);
//    valpha_last = valpha;
//    vbeta_last = vbeta;

	VofaSendData(1,enc_para.raw_data);
	VofaSendframetail();
}


/**
***********************************************************************
* @brief:      vofa_transmit(uint8_t* buf, uint16_t len)
* @param:		   void
* @retval:     void
* @details:    修改通信工具，USART或者USB
***********************************************************************
**/
void VofaTransmit(uint8_t* buf, uint16_t len)
{
//	HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 0xFFFF);
	CDC_Transmit_FS((uint8_t *)buf, len);
}


/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据 
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void VofaSendData(uint8_t num, float data) 
{
	send_buf[cnt++] = byte0(data);
	send_buf[cnt++] = byte1(data);
	send_buf[cnt++] = byte2(data);
	send_buf[cnt++] = byte3(data);
}
/**
***********************************************************************
* @brief      vofa_sendframetail(void)
* @param      NULL 
* @retval     void
* @details:   给数据包发送帧尾
***********************************************************************
**/
void VofaSendframetail(void) 
{
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x00;
	send_buf[cnt++] = 0x80;
	send_buf[cnt++] = 0x7f;
	
	/* 将数据和帧尾打包发送 */
	VofaTransmit((uint8_t *)send_buf, cnt);
	cnt = 0;// 每次发送完帧尾都需要清零
}











