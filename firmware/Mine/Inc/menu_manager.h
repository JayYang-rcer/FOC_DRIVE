//
// Created by 28076 on 26-3-18.
//

#ifndef DRIVE_CMAKE_MENU_MANAGER_H
#define DRIVE_CMAKE_MENU_MANAGER_H

#include "key.h"
#include "oled_iic.h"
#include <cstdio>

enum class ItemType : uint8_t {
    MENU,     // 下一级菜单
    FUNCTION, // 执行函数
    VARIABLE, // 修改变量 (如 PID 参数)
    DISPLAY   // 显示设定值和当前值
};

struct MenuItem {
    const char *label; // 显示名称
    ItemType    type;  // 类型
    union {
        void (*callback)(void);  // 函数指针 (TYPE_FUNCTION)
        const MenuItem *child;   // 子菜单首地址 (TYPE_MENU)
        float          *var_ptr; // 变量指针 (TYPE_VARIABLE/DISPLAY)
    };
    union {
        int   child_count; // 子菜单条目数 (TYPE_MENU)
        float var_step;    // 修改步长 (TYPE_VARIABLE/DISPLAY)
    };
    float *var_ptr2; // 当前值指针 (TYPE_DISPLAY专用)
};

class MenuManager
{
public:
    struct GPIO_Pin {
        GPIO_TypeDef *port;
        uint16_t      pin;
    };
    struct Config {
        const MenuItem *main_menu;
        KEY            &key_menu;
        KEY            &key_next;
        KEY            &key_enter;
        OLED           &oled;
        GPIO_Pin        pin_menu;  // 菜单/返回键引脚
        GPIO_Pin        pin_next;  // 向下/增加键引脚
        GPIO_Pin        pin_enter; // 确认键引脚
    };
    explicit MenuManager(const Config &cfg) : cfg_(cfg)
    {
        root_    = cfg.main_menu->child;
        current_ = root_;
        size_    = cfg.main_menu->child_count;
    }

    void DrawLine()
    {
        cursor_ = (cursor_ + 1) % size_;
        // 先清除原光标行的编辑标记
        if (current_[last_cursor_].type == ItemType::VARIABLE && current_[last_cursor_].var_ptr != nullptr) {
            int old_cursor = cursor_;
            cursor_        = last_cursor_;
            DrawEditMark(false);
            cursor_ = old_cursor;
        }
        // 新光标行
        if (current_[cursor_].type == ItemType::DISPLAY && current_[cursor_].var_ptr != nullptr) {
            if (current_[cursor_].var_ptr2 != nullptr) {
                DrawDisplayValue(*current_[cursor_].var_ptr, *current_[cursor_].var_ptr2, 2, true); // 双指针
            } else {
                DrawDisplayValue(*current_[cursor_].var_ptr, 2, true); // 单指针（NOW:XX）
            }
        } else if (current_[cursor_].type == ItemType::VARIABLE && current_[cursor_].var_ptr != nullptr) {
            DrawVariableValue(*current_[cursor_].var_ptr, 2, true);
        } else {
            cfg_.oled.OLED_ShowStr(32, 16 * cursor_, current_[cursor_].label, 2, true);
        }
        // 原光标行
        if (current_[last_cursor_].type == ItemType::DISPLAY && current_[last_cursor_].var_ptr != nullptr) {
            int old_cursor = cursor_;
            cursor_        = last_cursor_;
            if (current_[last_cursor_].var_ptr2 != nullptr) {
                DrawDisplayValue(*current_[last_cursor_].var_ptr, *current_[last_cursor_].var_ptr2, 2, false);
            } else {
                DrawDisplayValue(*current_[last_cursor_].var_ptr, 2, false);
            }
            cursor_ = old_cursor;
        } else if (current_[last_cursor_].type == ItemType::VARIABLE && current_[last_cursor_].var_ptr != nullptr) {
            int old_cursor = cursor_;
            cursor_        = last_cursor_;
            DrawVariableValue(*current_[last_cursor_].var_ptr, 2, false);
            cursor_ = old_cursor;
        } else {
            cfg_.oled.OLED_ShowStr(32, 16 * last_cursor_, current_[last_cursor_].label, 2, false);
        }
        last_cursor_ = cursor_;
    }

    void DrawMenu()
    {
        cfg_.oled.OLED_CLS();
        // 第一行（光标行）
        if (current_[0].type == ItemType::DISPLAY && current_[0].var_ptr != nullptr) {
            cursor_ = 0;
            if (current_[0].var_ptr2 != nullptr) {
                DrawDisplayValue(*current_[0].var_ptr, *current_[0].var_ptr2, 2, true);
            } else {
                DrawDisplayValue(*current_[0].var_ptr, 2, true);
            }
        } else if (current_[0].type == ItemType::VARIABLE && current_[0].var_ptr != nullptr) {
            cursor_ = 0;
            DrawVariableValue(*current_[0].var_ptr, 2, true);
        } else {
            cfg_.oled.OLED_ShowStr(32, 0, current_[0].label, 2, true);
        }
        // 其他行
        for (int i = 1; i < size_; i++) {
            if (current_[i].type == ItemType::DISPLAY && current_[i].var_ptr != nullptr) {
                int old_cursor = cursor_;
                cursor_        = i;
                if (current_[i].var_ptr2 != nullptr) {
                    DrawDisplayValue(*current_[i].var_ptr, *current_[i].var_ptr2, 2, false);
                } else {
                    DrawDisplayValue(*current_[i].var_ptr, 2, false);
                }
                cursor_ = old_cursor;
            } else if (current_[i].type == ItemType::VARIABLE && current_[i].var_ptr != nullptr) {
                int old_cursor = cursor_;
                cursor_        = i;
                DrawVariableValue(*current_[i].var_ptr, 2, false);
                cursor_ = old_cursor;
            } else {
                cfg_.oled.OLED_ShowStr(32, 16 * i, current_[i].label, 2, false);
            }
        }
        for (int i = size_; i < 4; i++) {
            cfg_.oled.OLED_ShowStr(32, 16 * i, "           ", 2, false);
        }
        cursor_      = 0;
        last_cursor_ = 0;
    }

    void DrawValue(float val, uint8_t num) // 显示num位有效小数,最多3位
    {
        // 清空buffer
        for (int i = 0; i < 11; i++)
            buffer[i] = ' ';
        // clang-format off
    buffer[0] = 'S'; buffer[1] = 'E'; buffer[2] = 'T'; buffer[3] = ':';
        // clang-format on
        if (val > 199)
            IntegerToBuffer(val, &buffer[4]);
        else
            FloatToBuffer(val, num, &buffer[4]);
        cfg_.oled.OLED_ShowStr(32, cursor_ * 16, buffer, 2, true);
        DrawEditMark(true); // 显示编辑标记
    }

    // 显示变量值（用于非编辑状态）
    void DrawVariableValue(float val, uint8_t num, bool is_highlight)
    {
        for (int i = 0; i < 11; i++)
            buffer[i] = ' ';
        // clang-format off
    buffer[0] = 'S'; buffer[1] = 'E'; buffer[2] = 'T'; buffer[3] = ':';
        // clang-format on
        if (val > 199)
            IntegerToBuffer(val, &buffer[4]);
        else
            FloatToBuffer(val, num, &buffer[4]);
        cfg_.oled.OLED_ShowStr(32, cursor_ * 16, buffer, 2, is_highlight);
        if (is_editing_) {
            DrawEditMark(true); // 编辑状态下显示标记
        }
    }

    // 显示设定值和当前值（DISPLAY类型，双指针）
    void DrawDisplayValue(float set_val, float cur_val, uint8_t num, bool is_highlight)
    {
        // 格式: "SET:XX C:YY" - 2号字体约11字符=88像素，适配128屏幕
        char buf1[16] = "SET:";
        char buf2[16] = "C:";
        FloatToBuffer(set_val, num, &buf1[4]);
        FloatToBuffer(cur_val, num, &buf2[2]);
        // 合并显示到buffer(11字节)
        for (int i = 0; i < 11; i++) buffer[i] = ' ';
        int idx = 0;
        // SET:XX
        for (int i = 0; buf1[i] != '\0' && idx < 7; i++) buffer[idx++] = buf1[i];
        // 空格
        buffer[idx++] = ' ';
        // C:YY
        for (int i = 0; buf2[i] != '\0' && idx < 10; i++) buffer[idx++] = buf2[i];
        buffer[idx] = '\0';
        cfg_.oled.OLED_ShowStr(32, cursor_ * 16, buffer, 2, is_highlight);
    }

    // 显示单值（NOW: XX，DISPLAY类型单指针）
    void DrawDisplayValue(float val, uint8_t num, bool is_highlight)
    {
        // 格式: "NOW:XX"
        for (int i = 0; i < 11; i++) buffer[i] = ' ';
        // clang-format off
        buffer[0] = 'N'; buffer[1] = 'O'; buffer[2] = 'W'; buffer[3] = ':';
        // clang-format on
        if (val > 199)
            IntegerToBuffer(val, &buffer[4]);
        else
            FloatToBuffer(val, num, &buffer[4]);
        cfg_.oled.OLED_ShowStr(32, cursor_ * 16, buffer, 2, is_highlight);
    }

    // 绘制/清除编辑标记 "*"（1号字体，不占用2号字体位置）
    void DrawEditMark(bool show)
    {
        // 2号字体每个字符8像素宽，"SET: XX.XX"约8字符=64像素
        // 屏幕128像素宽，在第100列位置用1号字体(6x8)显示"*"
        if (show) {
            cfg_.oled.OLED_ShowStr(120, cursor_ * 16, "*", 1, false);
        } else {
            cfg_.oled.OLED_ShowStr(120, cursor_ * 16, " ", 1, false);
        }
    }

    static void IntegerToBuffer(int value, char *buffer)
    {
        long temp   = (long)(value < 0 ? -value : value);
        int  digits = 0;
        long t      = temp;
        if (t == 0)
            digits = 1;
        else {
            while (t > 0) {
                t /= 10;
                digits++;
            }
        }

        int offset = 0;
        if (value < 0) {
            buffer[0] = '-';
            offset    = 1;
        }

        for (int i = offset + digits - 1; i >= offset; i--) {
            buffer[i] = (temp % 10) + '0';
            temp /= 10;
        }
    }

    static void FloatToBuffer(float val, int precision, char *buffer)
    {
        float value = val;

        for (int i = 0; i < precision; i++) {
            value *= 10;
        }

        long temp   = (long)(value < 0 ? -value : value);
        int  digits = 0;
        long t      = temp;
        if (t == 0)
            digits = 1;
        else {
            while (t > 0) {
                t /= 10;
                digits++;
            }
        }

        int offset = 0;
        if (value < 0) {
            buffer[0] = '-';
            offset++;
        }
        if (val < 1.0f && val > -1.0f) {
            if (offset == 1)
                buffer[1] = '0';
            if (offset == 0)
                buffer[0] = '0';
            offset++;
        }

        for (int i = offset + digits + 1; i > offset + digits - precision + 1; i--) {
            buffer[i] = (temp % 10) + '0';
            temp /= 10;
        }

        buffer[offset + digits - precision + 1] = '.';

        for (int i = offset + digits - precision; i >= offset + 1; i--) {
            buffer[i] = (temp % 10) + '0';
            temp /= 10;
        }
    }

    void KeyScanUpdate()
    {
        cfg_.key_menu.Tick(HAL_GPIO_ReadPin(cfg_.pin_menu.port, cfg_.pin_menu.pin));
        cfg_.key_next.Tick(HAL_GPIO_ReadPin(cfg_.pin_next.port, cfg_.pin_next.pin));
        cfg_.key_enter.Tick(HAL_GPIO_ReadPin(cfg_.pin_enter.port, cfg_.pin_enter.pin));
    }

    void ManagerUpdate()
    {
        Event key_menu  = cfg_.key_menu.GetEvent();
        Event key_next  = cfg_.key_next.GetEvent();
        Event key_enter = cfg_.key_enter.GetEvent();

        // 菜单键：返回上级菜单或回到根菜单
        if (key_menu == Event::CLICK) {
            if (menu_stack_top_ >= 0) {
                current_ = menu_stack_[menu_stack_top_].menu;
                size_    = menu_stack_[menu_stack_top_].size;
                menu_stack_top_--;
            } else {
                current_ = root_;
                size_    = cfg_.main_menu->child_count;
            }
            is_editing_ = false;
            DrawMenu();
            cfg_.oled.OLED_RefreshRAM();
        }

        // 向下键：移动光标或增加变量值
        if (key_next == Event::CLICK) {
            if (!is_editing_ || (current_[cursor_].type != ItemType::VARIABLE && current_[cursor_].type != ItemType::DISPLAY)) {
                DrawLine();
                cfg_.oled.OLED_RefreshRAM();
            } else {
                *(current_[cursor_].var_ptr) += current_[cursor_].var_step;
                DrawValue(*(current_[cursor_].var_ptr), 2);
                cfg_.oled.OLED_RefreshRAM();
            }
        }

        // 确认键：进入菜单/编辑变量/执行函数
        if (key_enter == Event::CLICK || key_enter == Event::LONG_PRESS) {
            auto &item = current_[cursor_];
            if (item.child != nullptr) {
                switch (item.type) {
                case ItemType::MENU:
                    if (menu_stack_top_ < MAX_MENU_DEPTH - 1) {
                        menu_stack_top_++;
                        menu_stack_[menu_stack_top_] = {current_, size_};
                    }
                    current_ = item.child;
                    size_    = item.child_count;
                    DrawMenu();
                    cfg_.oled.OLED_RefreshRAM();
                    break;

                case ItemType::VARIABLE:
                case ItemType::DISPLAY:
                    is_editing_ = !is_editing_;
                    if (is_editing_) {
                        DrawValue(*item.var_ptr, 2); // 进入编辑，显示*号
                        cfg_.oled.OLED_RefreshRAM();
                    } else {
                        DrawEditMark(false); // 退出编辑，清除*号
                        // 刷新显示当前值
                        if (item.type == ItemType::DISPLAY) {
                            if (item.var_ptr2 != nullptr) {
                                DrawDisplayValue(*item.var_ptr, *item.var_ptr2, 2, true);
                            } else {
                                DrawDisplayValue(*item.var_ptr, 2, true);
                            }
                        }
                        cfg_.oled.OLED_RefreshRAM();
                    }
                    break;

                case ItemType::FUNCTION:
                    if (key_enter == Event::LONG_PRESS) {
                        item.callback();
                    }
                    break;
                default:
                    break;
                }
            }
        }
    }

    // 单独刷新NOW行的函数，由外部定时调用
    void RefreshNowRow()
    {
        if (is_editing_) return;
        for (int i = 0; i < size_; i++) {
            if (current_[i].type == ItemType::DISPLAY && current_[i].var_ptr != nullptr && current_[i].var_ptr2 == nullptr) {
                int old_cursor = cursor_;
                cursor_ = i;
                for (int j = 0; j < 11; j++) buffer[j] = ' ';
                buffer[0] = 'N'; buffer[1] = 'O'; buffer[2] = 'W'; buffer[3] = ':';
                float val = *current_[i].var_ptr;
                if (val > 199)
                    IntegerToBuffer(val, &buffer[4]);
                else
                    FloatToBuffer(val, 2, &buffer[4]);
                cfg_.oled.OLED_ShowStr(32, cursor_ * 16, buffer, 2, false);
                cursor_ = old_cursor;
            }
        }
        cfg_.oled.OLED_RefreshRAM();
    }

private:
    static constexpr int MAX_MENU_DEPTH = 4;
    struct MenuFrame {
        const MenuItem *menu;
        int             size;
    };
    Config          cfg_;
    const MenuItem *root_;            // 顶层菜单（主页）
    const MenuItem *current_;         // 当前显示的菜单数组
    int             size_;            // 当前菜单有多少行
    int             cursor_      = 0; // 光标指向第几行
    int             last_cursor_ = 0;
    bool            is_editing_  = false; // 核心状态：是否正在改参数
    char            buffer[11]{};
    int             menu_stack_top_ = -1;
    MenuFrame       menu_stack_[MAX_MENU_DEPTH]{}; // 菜单栈，支持返回上级
};

void task_5ms();
void task_50ms();

#endif // DRIVE_CMAKE_MENU_MANAGER_H
