//
// Created by 28076 on 25-12-8.
//

#include "McuDevicePort.h"
#include "foc_cfg.h"
#include "oled_iic_cfg.h"

// #define DEVICE_ENABLE_ID 0x100
// #define DEVICE_CAN_ID    0x101
#define DEVICE_ENABLE_ID 0x100
#define DEVICE_CAN_ID    0x101
#define USE_CAN_MODE     1

void CanResourceInit(void)
{
    CAN_Filter_Init(&hfdcan2, CanFilter_0, 0x000, 0x000, CAN_FIFO0, CAN_STD);
    CAN_Filter_Init(&hfdcan2, CanFilter_1, 0x000, 0x000, CAN_FIFO1, CAN_STD);
    CAN_Init(&hfdcan2, CAN1_RxCallBack);
}

uint16_t can_recieveFlag = 0;
uint8_t  can_rxData[8];
void     CAN1_RxCallBack(CAN_RxBuffer *CAN_RxBuffer)
{
    for (int i = 0; i < 8; ++i) {
        can_rxData[i] = CAN_RxBuffer->data[i];
    }
#if USE_CAN_MODE
    if (CAN_RxBuffer->header.IdType == FDCAN_STANDARD_ID) {
        if (CAN_RxBuffer->header.Identifier == DEVICE_ENABLE_ID) {
            if (CAN_RxBuffer->data[0] == 1) {
                motor_ctrl.mode = 4;
            } else {
                motor_ctrl.mode = 0;
            }
        }

        if (CAN_RxBuffer->header.Identifier == DEVICE_CAN_ID) {
            motor_ctrl.speed_set = (int16_t)(CAN_RxBuffer->data[0] | CAN_RxBuffer->data[1] << 8);
            can_recieveFlag      = 0;
        }
    }
#endif
}
