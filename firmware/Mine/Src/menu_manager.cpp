//
// Created by 28076 on 26-3-18.
//

#include "menu_manager.h"

float pid_kp = 0;
MenuItem general_control[] = {
    {
        .label       = "SET:       ",
        .type        = ItemType::VARIABLE,
        .var_ptr     = &pid_kp,
        .var_step    = 0.3f,
    },
    {
        .label       = "SET STEP:  ",
        .type        = ItemType::VARIABLE,
        .var_ptr     = nullptr,
        .var_step    = 0,
    },
    {
        .label       = "START      ",
        .type        = ItemType::FUNCTION,
        .callback    = nullptr,
    },
    {
        .label       = "STOP",
        .type        = ItemType::FUNCTION,
        .callback    = nullptr,
    },
};

MenuItem motor_control[] = {
    {
        .label       = "currentLoop",
        .type        = ItemType::MENU,
        .child       = general_control,
        .child_count = 4,
    },
    {
        .label       = "speedLoop",
        .type        = ItemType::MENU,
        .child       = general_control,
        .child_count = 4,
    },
    {
        .label       = "posLoop",
        .type        = ItemType::MENU,
        .child       = general_control,
        .child_count = 4,
    },
};

MenuItem menu_ctrl_mode[] = {
    {
        .label       = "sensor     ",
        .type        = ItemType::MENU,
        .child       = motor_control,
        .child_count = 3,
    },
    {
        .label       = "senseless  ",
        .type        = ItemType::MENU,
        .child       = motor_control,
        .child_count = 3,
    },
};

MenuItem menu_main_tree[] = {
    {
        .label       = "CONTROL    ",
        .type        = ItemType::MENU,
        .child       = menu_ctrl_mode,
        .child_count = 2,
    },
    {
        .label       = "MOTOR PARAM",
        .type        = ItemType::MENU,
        .child       = nullptr,
        .child_count = 0,
    },
    {
        .label       = "ENCODER    ",
        .type        = ItemType::MENU,
        .child       = nullptr,
        .child_count = 0,
    },
    {
        .label       = "CAN ID     ",
        .type        = ItemType::MENU,
        .child       = nullptr,
        .child_count = 0,
    },
};

MenuItem root = {
    .label       = "root",
    .type        = ItemType::MENU,
    .child       = menu_main_tree,
    .child_count = 4,
};

extern OLED oled;
extern KEY menu;
extern KEY enter;
extern KEY next;

MenuManager::Config config_manager = {
    .main_menu  = &root,
    .key_menu   = menu,
    .key_next   = next,
    .key_enter  = enter,
    .oled       = oled,
    .pin_menu   = {KEY_MEAU_GPIO_Port, KEY_MEAU_Pin},
    .pin_next   = {KEY_NEXT_GPIO_Port, KEY_NEXT_Pin},
    .pin_enter  = {KEY_ENTER_GPIO_Port, KEY_ENTER_Pin},
};
MenuManager menuManager(config_manager);

void task_5ms()
{
    menuManager.KeyScanUpdate();
}

void task_50ms()
{
    menuManager.ManagerUpdate();
}
