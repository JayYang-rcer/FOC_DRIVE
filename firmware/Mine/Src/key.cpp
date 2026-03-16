//
// Created by 28076 on 26-1-17.
//

#include "key.h"
#include "tim.h"

struct {
    KEY::Config menu = {
        .port = KEY_MEAU_GPIO_Port,
        .pin  = KEY_MEAU_Pin,
        .active_low = false,
        .long_press_ms = 500,
    };
    KEY::Config enter = {
        .port = KEY_ENTER_GPIO_Port,
        .pin  = KEY_ENTER_Pin,
        .active_low = false,
        .long_press_ms = 500,
    };
    KEY::Config next = {
        .port = KEY_NEXT_GPIO_Port,
        .pin  = KEY_NEXT_Pin,
        .active_low = false,
        .long_press_ms = 500,
    };
}KeyConfig;
KEY menu(KeyConfig.menu);
KEY enter(KeyConfig.enter);
KEY next(KeyConfig.next);

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM16)
    {
        menu.Tick(HAL_GPIO_ReadPin(KEY_MEAU_GPIO_Port,KEY_MEAU_Pin));
        enter.Tick(HAL_GPIO_ReadPin(KEY_ENTER_GPIO_Port,KEY_ENTER_Pin));
        next.Tick(HAL_GPIO_ReadPin(KEY_NEXT_GPIO_Port,KEY_NEXT_Pin));
    }
}
