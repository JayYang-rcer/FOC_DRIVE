//
// Created by 28076 on 26-1-17.
//

#ifndef DRIVE_CMAKE_KEY_H
#define DRIVE_CMAKE_KEY_H

#include "gpio.h"
#include "stm32g4xx_hal_gpio.h"

typedef enum
{
    KEY_Up,
    KEY_Down,
    KEY_Long,
}KeyState_e;

typedef struct
{
    uint8_t fifo;
    uint8_t num;
    uint8_t sample_cnt;
    GPIO_TypeDef* gpio_port;
    uint16_t gpio_pin;
    KeyState_e status;
}KeyFifo_t;

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

void KEY_FifoScan(void);
void KEY_ProcessHandle(void);

#endif // DRIVE_CMAKE_KEY_H
