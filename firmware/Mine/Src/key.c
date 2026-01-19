//
// Created by 28076 on 26-1-17.
//

#include "key.h"
#include "tim.h"

#define KEY_FIFO_LEN       (5)
#define KEY_FIFO_MASK      ((1 << KEY_FIFO_LEN) - 1)
#define KEY_LONG_PRESS_CNT 50

KeyFifo_t KeyFifo[3] = {
    {
        .fifo      = 0,
        .gpio_port = KEY_MEAU_GPIO_Port,
        .gpio_pin  = KEY_MEAU_Pin,
        .num       = 0,
    },
    {
        .fifo      = 0,
        .gpio_port = KEY_NEXT_GPIO_Port,
        .gpio_pin  = KEY_NEXT_Pin,
        .num       = 1,
    },
    {
        .fifo      = 0,
        .gpio_port = KEY_ENTER_GPIO_Port,
        .gpio_pin  = KEY_ENTER_Pin,
        .num       = 2,
    },
};

// void KEY_Init()

static KeyState_e KEY_GetStatus(uint8_t key_num)
{
    return KeyFifo[key_num - 1].status;
}

static GPIO_PinState KEY_ReadPIn(KeyFifo_t *key)
{
    return HAL_GPIO_ReadPin(key->gpio_port, key->gpio_pin);
}

void KEY_FifoScan(void)
{
    for (int i = 0; i < 3; i++) {
        KeyFifo[i].fifo = ((KeyFifo[i].fifo << 1) | KEY_ReadPIn(&KeyFifo[i]));

        if (KeyFifo[i].fifo & KEY_FIFO_MASK) {
            if (KeyFifo[i].sample_cnt < KEY_LONG_PRESS_CNT) {
                KeyFifo[i].status = KEY_Down;
                KeyFifo[i].sample_cnt++;
            } else {
                KeyFifo[i].status = KEY_Long;
            }
        } else {
            KeyFifo[i].sample_cnt = 0;
            KeyFifo[i].status     = KEY_Up;
        }
    }
}

void KEY_ProcessHandle(void)
{
    if (__HAL_TIM_GET_FLAG(&htim16, TIM_FLAG_CC1)) {
        __HAL_TIM_CLEAR_FLAG(&htim16, TIM_FLAG_CC1);
        KEY_FifoScan();
    }
}