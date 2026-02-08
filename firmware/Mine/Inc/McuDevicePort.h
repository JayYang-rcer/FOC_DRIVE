//
// Created by 28076 on 25-12-8.
//

#ifndef DRIVE_CMAKE_MCUDEVICEPORT_H
#define DRIVE_CMAKE_MCUDEVICEPORT_H

#include "drive_can.h"
#ifdef __cplusplus
extern "C" {
#endif
void CAN1_RxCallBack(CAN_RxBuffer *CAN_RxBuffer); // CAN1接收回调函数
void CanResourceInit(void);
#ifdef __cplusplus
}
#endif
#endif // DRIVE_CMAKE_MCUDEVICEPORT_H
