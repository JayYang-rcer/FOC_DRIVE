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
void VofaStart(void) {
//	 VofaSendData(1,pll_hfi.error);
//	VofaSendData(1,PWM_ARR());
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
void VofaTransmit(uint8_t *buf, uint16_t len) {
//	HAL_UART_Transmit(&huart3, (uint8_t *)buf, len, 0xFFFF);
    CDC_Transmit_FS((uint8_t *) buf, len);
}


/**
***********************************************************************
* @brief:      vofa_send_data(float data)
* @param[in]:  num: 数据编号 data: 数据 
* @retval:     void
* @details:    将浮点数据拆分成单字节
***********************************************************************
**/
void VofaSendData(uint8_t num, float data) {
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
void VofaSendframetail(void) {
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x00;
    send_buf[cnt++] = 0x80;
    send_buf[cnt++] = 0x7f;

    /* 将数据和帧尾打包发送 */
    VofaTransmit((uint8_t *) send_buf, cnt);
    cnt = 0;// 每次发送完帧尾都需要清零
}











