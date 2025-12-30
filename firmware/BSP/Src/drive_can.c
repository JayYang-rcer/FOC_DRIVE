/**
 * @file drive_can.c
 * @author Yang Jianyi
 * @brief 1)can底层驱动文件，使用该文件，需要在cubeMX中配置好can硬件(参考大疆电机所需的CAN的配置)，并在main.c中调用CAN_Init函数进行初始化
 *        调用CAN_Filter_Init函数进行滤波器配置，comm_can_transmit_extid或comm_can_transmit_stdid函数进行can数据发送
 *
 *        2)该文件使用双fifo的接收方式，当接收到数据时，会调用CAN_RxFifo0MsgPendingCallback或CAN_RxFifo1MsgPendingCallback函数。
 * @version 0.1
 * @date 2024-03-28
 *
 */

#include "drive_can.h"

static void (*pCAN1_RxCpltCallback)(CAN_RxBuffer *);
static void (*pCAN2_RxCpltCallback)(CAN_RxBuffer *);

/**
 * @brief CAN接收滤波器初始化
 *
 * @param hcan can句柄
 * @param object_para 滤波器参数配置，滤波器序号|FIFO绑定|ID类型(帧类型)|数据类型
 * @param Id
 * @param MaskId ID掩码
 */
void CAN_Filter_Init(FDCAN_HandleTypeDef *hfdcan, uint8_t filterIndex, uint32_t id, uint32_t mask, uint8_t fifo, uint8_t isExtended)
{
    FDCAN_FilterTypeDef filter;

    filter.IdType      = isExtended ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
    filter.FilterIndex = filterIndex;
    filter.FilterType  = FDCAN_FILTER_MASK; // ID & MASK 匹配模式

    filter.FilterConfig = (fifo == 0) ? FDCAN_FILTER_TO_RXFIFO0 : FDCAN_FILTER_TO_RXFIFO1;

    filter.FilterID1 = id;   // 标准 11bit 或 扩展 29bit
    filter.FilterID2 = mask; // 掩码（0x7FF 或 0x1FFFFFFF）

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter) != HAL_OK)
        Error_Handler();
}

uint8_t CAN_Init(FDCAN_HandleTypeDef *hfdcan, void (*pFunc)(CAN_RxBuffer *))
{
    assert_param(hfdcan != NULL);

    /* 启动 FDCAN 外设 */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        Error_Handler();
    }

    /* 启动发送完成中断（FIFO 空） */
    //    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_TX_FIFO_EMPTY, 0) != HAL_OK) {
    //        Error_Handler();

    /* FIFO0 收到消息中断 */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    /* FIFO1 收到消息中断 */
    if (HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
    }

    /* 根据实例绑定回调 */
    if (hfdcan->Instance == FDCAN1) {
        pCAN1_RxCpltCallback = pFunc;
        return SUCCESS;
    } else if (hfdcan->Instance == FDCAN2) {
        pCAN2_RxCpltCallback = pFunc;
        return SUCCESS;
    } else {
        return ERROR;
    }
}

/**
 * @brief FDCAN FIFO0 接收回调函数
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    static CAN_RxBuffer CAN_RxBuffer;

    /* 接收到新消息标志 */
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
        return;

    /* FDCAN1 */
    if (hfdcan->Instance == FDCAN1) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_RxBuffer.header, CAN_RxBuffer.data) == HAL_ERROR) {
            // 可加错误处理
        }
        pCAN1_RxCpltCallback(&CAN_RxBuffer);
    }

    /* FDCAN2 */
    if (hfdcan->Instance == FDCAN2) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &CAN_RxBuffer.header, CAN_RxBuffer.data) == HAL_ERROR) {
            // 可加错误处理
        }
        pCAN2_RxCpltCallback(&CAN_RxBuffer);
    }
}

/**
 * @brief HAL FDCAN FIFO1 消息就绪回调
 */
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs)
{
    static CAN_RxBuffer CAN_RxBuffer;

    /* 接收到新消息标志 */
    if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) == 0)
        return;

    /* FDCAN1 */
    if (hfdcan->Instance == FDCAN1) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN_RxBuffer.header, CAN_RxBuffer.data) == HAL_ERROR) {
            // 可加错误处理
        }
        pCAN1_RxCpltCallback(&CAN_RxBuffer);
    }

    /* FDCAN2 */
    if (hfdcan->Instance == FDCAN2) {
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &CAN_RxBuffer.header, CAN_RxBuffer.data) == HAL_ERROR) {
            // 可加错误处理
        }
        pCAN2_RxCpltCallback(&CAN_RxBuffer);
    }
}

void comm_can_transmit_extid(FDCAN_HandleTypeDef *hcan, uint32_t ExtId, uint8_t *pdata, uint8_t length)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    if (length > 8)
        length = 8;

    TxHeader.Identifier          = ExtId;             // 扩展帧 ID
    TxHeader.IdType              = FDCAN_EXTENDED_ID; // 扩展帧
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;  // 数据帧
    TxHeader.DataLength          = length;      // DLC 字段（0..8 字节）
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;     // 不使用快速数据段
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN; // 经典 CAN，不用 CAN-FD
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    TxHeader.MessageMarker       = 0;

    // 发送
    if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, pdata) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief FDCAN 标准帧发送函数
 * @param hfdcan 使用哪个 FDCAN 外设，例如 &hfdcan1 或 &hfdcan2
 * @param StdId 标准帧 ID (11bit)
 * @param pdata 数据指针
 * @param length 数据长度 (最大 8)
 */
void comm_can_transmit_stdid(FDCAN_HandleTypeDef *hfdcan, uint16_t StdId, uint8_t *pdata, uint8_t length)
{
    FDCAN_TxHeaderTypeDef TxHeader;

    if (length > 8)
        length = 8;

    /* 填写 FDCAN 标准帧头 */
    TxHeader.Identifier          = StdId;              // 标准帧 ID
    TxHeader.IdType              = FDCAN_STANDARD_ID;  // 标准帧
    TxHeader.TxFrameType         = FDCAN_DATA_FRAME;   // 数据帧
    TxHeader.DataLength          = length;  // DLC 字节长度
    TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;   // 错误状态指示
    TxHeader.BitRateSwitch       = FDCAN_BRS_OFF;      // 不使用 CAN-FD BRS
    TxHeader.FDFormat            = FDCAN_CLASSIC_CAN;  // 经典 CAN
    TxHeader.TxEventFifoControl  = FDCAN_NO_TX_EVENTS; // 不生成 TxEvent
    TxHeader.MessageMarker       = 0;                  // 可选标记位

    /* 发送消息到 Tx FIFO/Queue */
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &TxHeader, pdata) != HAL_OK) {
        Error_Handler();
    }
}
