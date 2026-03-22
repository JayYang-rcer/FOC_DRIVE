//
// Created by 28076 on 26-3-18.
//

#include "menu_manager.h"
#include "foc_cfg.h"

void MotorStart() {}
void MotorStop() {}
float iq_set = 0, iq_now = 0;
float speed_set = 0, speed_now = 0;
float pos_set = 0, pos_now = 0;
float current_lim = 0.0f;
float bat_cell = 0;
float can_id = 1;

// 状态变量 (0=ERR, 1=OK)
int encoder_status = 0;
int identify_status = 0;

//========================== MOTOR CONTROL ===========================
MenuItem current_control[] = {
    {
        .label    = "SET:       ",
        .type     = ItemType::VARIABLE,
        .var_ptr  = &iq_set,
        .var_step = 0.2f,
    },
    {
        .label    = "NOW:       ",
        .type     = ItemType::DISPLAY,
        .var_ptr  = &iq_now,
    },
    {
        .label    = "START     A",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label    = "STOP       ",
        .type     = ItemType::FUNCTION,
        .callback = MotorStop,
    },
};

MenuItem speed_control[] = {
    {
        .label    = "SET:       ",
        .type     = ItemType::VARIABLE,
        .var_ptr  = &speed_set,
        .var_step = 200.0f,
    },
    {
        .label    = "NOW:       ",
        .type     = ItemType::DISPLAY,
        .var_ptr  = &speed_now,
    },
    {
        .label    = "START   RPM",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label    = "STOP       ",
        .type     = ItemType::FUNCTION,
        .callback = MotorStop,
    },
};

MenuItem pos_control[] = {
    {
        .label    = "SET:       ",
        .type     = ItemType::VARIABLE,
        .var_ptr  = &pos_set,
        .var_step = 15.0f,
    },
    {
        .label    = "NOW:       ",
        .type     = ItemType::DISPLAY,
        .var_ptr  = &pos_now,
    },
    {
        .label    = "START   DEG",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label    = "STOP       ",
        .type     = ItemType::FUNCTION,
        .callback = MotorStop,
    },
};

MenuItem motor_control[] = {
    {
        .label       = "currentLoop",
        .type        = ItemType::MENU,
        .child       = current_control,
        .child_count = 4,
    },
    {
        .label       = "speedLoop  ",
        .type        = ItemType::MENU,
        .child       = speed_control,
        .child_count = 4,
    },
    {
        .label       = "posLoop    ",
        .type        = ItemType::MENU,
        .child       = pos_control,
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
        .child_count = 4,
    },
};
//========================== MOTOR CONTROL ===========================

//========================== MOTOR PARAM ==============================
MenuItem current_limit[] = {
    {
        .label    = "SET:       ",
        .type     = ItemType::VARIABLE,
        .var_ptr  = &current_lim,
        .var_step = 1.5f,
    },
    {
        .label    = "SAVE CHANGE",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label       = "UINT: A    ",
        .type        = ItemType::MENU,
        .child       = nullptr,
        .child_count = 0,
    },
};

MenuItem battery_cell[] = {
    {
        .label       = "SET:       ",
        .type        = ItemType::VARIABLE,
        .var_ptr     = &bat_cell,
        .var_step    = 1,
    },
    {
        .label    = "SAVE CHANGE",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label       = "UINT: S    ",
        .type        = ItemType::MENU,
        .child       = nullptr,
        .child_count = 0,
    },
};

MenuItem motor_identify[] = {
    {
        .label    = "START      ",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label    = "SAVE CHANGE",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label       = "STATUS:    ",
        .type        = ItemType::STATUS,
        .status_ptr  = &identify_status,
    },
};

MenuItem menu_param[] = {
    {
        .label       = "CurrLimit  ",
        .type        = ItemType::MENU,
        .child       = current_limit,
        .child_count = 3,
    },
    {
        .label       = "Bat cells  ",
        .type        = ItemType::MENU,
        .child       = battery_cell,
        .child_count = 3,
    },
    {
        .label       = "MOTOR IDENT",
        .type        = ItemType::MENU,
        .child       = motor_identify,
        .child_count = 3,
    },
};
//========================== MOTOR PARAM ==============================

//========================== ENCODER ==================================
MenuItem encoder_init[] = {
    {
        .label    = "START      ",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label    = "SAVE CHANGE",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label       = "Pole Pairs ",
        .type        = ItemType::DISPLAY,
        .var_ptr     = &pos_set,
    },
    {
        .label       = "STATUS:    ",
        .type        = ItemType::STATUS,
        .status_ptr  = &encoder_status,
    },
};

MenuItem menu_encoder[] = {
    {
        .label       = "AS5047P    ",
        .type        = ItemType::MENU,
        .child       = encoder_init,
        .child_count = 4,
    },
    {
        .label       = "ABI        ",
        .type        = ItemType::MENU,
        .child       = encoder_init,
        .child_count = 4,
    },
    {
        .label       = "HALL       ",
        .type        = ItemType::MENU,
        .child       = encoder_init,
        .child_count = 4,
    },
};
//========================== ENCODER ==================================

//========================== CAN ID ===================================
MenuItem menu_canid[] = {
    {
        .label    = "SET CAN ID ",
        .type     = ItemType::VARIABLE,
        .var_ptr  = &can_id,
        .var_step = 1,
    },
    {
        .label    = "SAVE CHANGE",
        .type     = ItemType::FUNCTION,
        .callback = MotorStart,
    },
    {
        .label       = "STATUS     ",
        .type        = ItemType::DISPLAY,
        .var_ptr     = &can_id,
    },
};
//========================== CAN ID ===================================

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
        .child       = menu_param,
        .child_count = 3,
    },
    {
        .label       = "ENCODER    ",
        .type        = ItemType::MENU,
        .child       = menu_encoder,
        .child_count = 3,
    },
    {
        .label       = "CAN ID     ",
        .type        = ItemType::MENU,
        .child       = menu_canid,
        .child_count = 3,
    },
};

MenuItem root = {
    .label       = "root",
    .type        = ItemType::MENU,
    .child       = menu_main_tree,
    .child_count = 4,
};

extern OLED oled;
extern KEY  menu;
extern KEY  enter;
extern KEY  next;

MenuManager::Config config_manager = {
    .main_menu = &root,
    .key_menu  = menu,
    .key_next  = next,
    .key_enter = enter,
    .oled      = oled,
    .pin_menu  = {KEY_MEAU_GPIO_Port, KEY_MEAU_Pin},
    .pin_next  = {KEY_NEXT_GPIO_Port, KEY_NEXT_Pin},
    .pin_enter = {KEY_ENTER_GPIO_Port, KEY_ENTER_Pin},
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
