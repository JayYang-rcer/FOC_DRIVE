#ifndef __DRIVE_CAN_H
#define __DRIVE_CAN_H
#include "fdcan.h"

#define CanFilter_0    0
#define CanFilter_1    1
#define CanFilter_2    2
#define CanFilter_3    3
#define CanFilter_4    4
#define CanFilter_5    5
#define CanFilter_6    6
#define CanFilter_7    7
#define CanFilter_8    8
#define CanFilter_9    9
#define CanFilter_10   10
#define CanFilter_11   11
#define CanFilter_12   12
#define CanFilter_13   13
#define CanFilter_14   14
#define CanFilter_15   15
#define CanFilter_16   16
#define CanFilter_17   17
#define CanFilter_18   18
#define CanFilter_19   19
#define CanFilter_20   20
#define CanFilter_21   21
#define CanFilter_22   22
#define CanFilter_23   23
#define CanFilter_24   24
#define CanFilter_25   25
#define CanFilter_26   26
#define CanFilter_27   27

#define CAN_FIFO0        0
#define CAN_FIFO1        1

#define CAN_STD          0
#define CAN_EXT          1


typedef struct CAN_RxMessage {
    FDCAN_RxHeaderTypeDef header;
    uint8_t               data[8];
} CAN_RxBuffer;

#ifdef __cplusplus
extern "C" {
#endif
uint8_t CAN_Init(FDCAN_HandleTypeDef *hfdcan, void (*pFunc)(CAN_RxBuffer *));
void    CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan, uint8_t filterIndex, uint32_t id, uint32_t mask, uint8_t fifo, uint8_t isExtended);
void    comm_can_transmit_extid(FDCAN_HandleTypeDef *hcan, uint32_t ExtId, uint8_t *pdata, uint8_t length); // 拓展帧发送函数
void    comm_can_transmit_stdid(FDCAN_HandleTypeDef *hcan, uint16_t StdId, uint8_t *pdata, uint8_t length); // 标准帧发送函数
#ifdef __cplusplus
}
#endif

#endif
