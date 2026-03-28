//
// Created by 28076 on 26-3-18.
//

#include "menu_manager.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <cstring>

// 静态成员定义
char MenuManager::display_buffer[12] = {0};

void  MotorStart() {}
void  MotorStop() {}
float iq_set = 0, iq_now = 0;
float speed_set = 0, speed_now = 0;
float pos_set = 0, pos_now = 0;
float current_lim = 0.0f;
float bat_cell    = 0;
float can_id      = 1;

// 状态变量 (0=ERR, 1=OK)
int encoder_status  = 0;
int identify_status = 0;

//========================== MOTOR CONTROL ===========================
MenuItem current_control[] = {
    MenuItem("SET:       ", ItemType::VARIABLE, &iq_set, 0.2f),
    MenuItem("NOW:       ", ItemType::DISPLAY, &iq_now),
    MenuItem("START     A", ItemType::FUNCTION, MotorStart),
    MenuItem("STOP       ", ItemType::FUNCTION, MotorStop),
};

MenuItem speed_control[] = {
    MenuItem("SET:       ", ItemType::VARIABLE, &speed_set, 200.0f),
    MenuItem("NOW:       ", ItemType::DISPLAY, &speed_now),
    MenuItem("START   RPM", ItemType::FUNCTION, MotorStart),
    MenuItem("STOP       ", ItemType::FUNCTION, MotorStop),
};

MenuItem pos_control[] = {
    MenuItem("SET:       ", ItemType::VARIABLE, &pos_set, 15.0f),
    MenuItem("NOW:       ", ItemType::DISPLAY, &pos_now),
    MenuItem("START   DEG", ItemType::FUNCTION, MotorStart),
    MenuItem("STOP       ", ItemType::FUNCTION, MotorStop),
};

MenuItem motor_control[] = {
    MenuItem("currentLoop", ItemType::MENU, current_control, 4),
    MenuItem("speedLoop  ", ItemType::MENU, speed_control, 4),
    MenuItem("posLoop    ", ItemType::MENU, pos_control, 4),
};

MenuItem menu_ctrl_mode[] = {
    MenuItem("sensor     ", ItemType::MENU, motor_control, 3),
    MenuItem("senseless  ", ItemType::MENU, motor_control, 4),
};
//========================== MOTOR CONTROL ===========================

//========================== MOTOR PARAM ==============================
MenuItem current_limit[] = {
    MenuItem("SET:       ", ItemType::VARIABLE, &current_lim, 1.5f),
    MenuItem("SAVE CHANGE", ItemType::FUNCTION, MotorStart),
    MenuItem("UINT: A    ", ItemType::MENU, nullptr, 0),
};

MenuItem battery_cell[] = {
    MenuItem("SET:       ", ItemType::VARIABLE, &bat_cell, 1.0f),
    MenuItem("SAVE CHANGE", ItemType::FUNCTION, MotorStart),
    MenuItem("UINT: S    ", ItemType::MENU, nullptr, 0),
};

MenuItem motor_identify[] = {
    MenuItem("START      ", ItemType::FUNCTION, MotorStart),
    MenuItem("SAVE CHANGE", ItemType::FUNCTION, MotorStart),
    MenuItem("STATUS:    ", ItemType::STATUS, &identify_status),
};

MenuItem menu_param[] = {
    MenuItem("CurrLimit  ", ItemType::MENU, current_limit, 3),
    MenuItem("Bat cells  ", ItemType::MENU, battery_cell, 3),
    MenuItem("MOTOR IDENT", ItemType::MENU, motor_identify, 3),
};
//========================== MOTOR PARAM ==============================

//========================== ENCODER ==================================
MenuItem encoder_init[] = {
    MenuItem("START      ", ItemType::FUNCTION, MotorStart),
    MenuItem("SAVE CHANGE", ItemType::FUNCTION, MotorStart),
    MenuItem("Pole Pairs ", ItemType::DISPLAY, &pos_set),
    MenuItem("STATUS:    ", ItemType::STATUS, &encoder_status),
};

MenuItem menu_encoder[] = {
    MenuItem("AS5047P    ", ItemType::MENU, encoder_init, 4),
    MenuItem("ABI        ", ItemType::MENU, encoder_init, 4),
    MenuItem("HALL       ", ItemType::MENU, encoder_init, 4),
};
//========================== ENCODER ==================================

//========================== CAN ID ===================================
MenuItem menu_canid[] = {
    MenuItem("SET CAN ID ", ItemType::VARIABLE, &can_id, 1.0f),
    MenuItem("SAVE CHANGE", ItemType::FUNCTION, MotorStart),
    MenuItem("STATUS     ", ItemType::DISPLAY, &can_id),
};
//========================== CAN ID ===================================

MenuItem menu_main_tree[] = {
    MenuItem("CONTROL    ", ItemType::MENU, menu_ctrl_mode, 2),
    MenuItem("MOTOR PARAM", ItemType::MENU, menu_param, 3),
    MenuItem("ENCODER    ", ItemType::MENU, menu_encoder, 3),
    MenuItem("CAN ID     ", ItemType::MENU, menu_canid, 3),
};

MenuItem root = MenuItem("root", ItemType::MENU, menu_main_tree, 4);

extern OLED oled;
extern KEY  menu;
extern KEY  enter;
extern KEY  next;

MenuManager::Config config_manager = {
    .main_menu = &root,
    .key_menu  = &menu,
    .key_next  = &next,
    .key_enter = &enter,
    .oled      = &oled,
    .pin_menu  = {KEY_MEAU_GPIO_Port, KEY_MEAU_Pin},
    .pin_next  = {KEY_NEXT_GPIO_Port, KEY_NEXT_Pin},
    .pin_enter = {KEY_ENTER_GPIO_Port, KEY_ENTER_Pin},
};
MenuManager menuManager;

[[noreturn]] void KeyScanTask(void *argument)
{
    for (;;) {
        menuManager.KeyScanUpdate();
        osDelay(5);
    }
}

[[noreturn]] [[maybe_unused]] void oledReflashTask(void *argument)
{
    for (;;) {
        menuManager.ManagerUpdate();
        osDelay(50);
    }
}

#include "analog_sense.h"
extern TempSense temp_sense;
float             temp;
[[noreturn]] [[maybe_unused]] void TempSenseTask(void *argument)
{
    for(;;)
    {
        temp = temp_sense.Get_Temperature();
        osDelay(500);
    }
}
