//
// Created by 28076 on 26-1-17.
//

#include "key.h"
#include "tim.h"

struct {
    KEY::Config menu = {
        .port = GPIOB,
        .pin  = GPIO_PIN_0,
        .active_low = false,
        .long_press_ms = 200,
    };
}KeyConfig;
KEY menu(KeyConfig.menu);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM16)
    {
        menu.Tick(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0));
    }
}
