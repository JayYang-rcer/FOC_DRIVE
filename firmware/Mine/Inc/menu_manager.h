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
    DISPLAY,  // 显示设定值和当前值
    STATUS    // 显示状态 OK/ERR
};

struct MenuItem {
    const char *label;
    ItemType type;
    union {
        void (*callback)(void);
        const MenuItem *child;
        float *var_ptr;
    };
    union {
        int child_count;
        float var_step;
    };
    float *var_ptr2 = nullptr;
    int *status_ptr = nullptr;

    // 构造函数 - VARIABLE类型 (label, type, ptr, step)
    MenuItem(const char *l, ItemType t, float *ptr, float step) {
        label = l; type = t; var_ptr = ptr; var_step = step; var_ptr2 = nullptr; status_ptr = nullptr;
    }

    // 构造函数 - DISPLAY类型 (label, type, ptr)
    MenuItem(const char *l, ItemType t, float *ptr) {
        label = l; type = t; var_ptr = ptr; var_step = 0; var_ptr2 = nullptr; status_ptr = nullptr;
    }

    // 构造函数 - FUNCTION类型 (label, type, callback)
    MenuItem(const char *l, ItemType t, void (*cb)(void)) {
        label = l; type = t; callback = cb; var_ptr2 = nullptr; status_ptr = nullptr;
    }

    // 构造函数 - MENU类型 (label, type, child, count)
    MenuItem(const char *l, ItemType t, const MenuItem *c, int count) {
        label = l; type = t; child = c; child_count = count; var_ptr2 = nullptr; status_ptr = nullptr;
    }

    // 默认构造函数，支持 designated initializers
    MenuItem() {
        label = nullptr; type = ItemType::MENU; callback = nullptr; child_count = 0; var_ptr2 = nullptr; status_ptr = nullptr;
    }

    // STATUS类型 (label, type, status_ptr)
    MenuItem(const char *l, ItemType t, int *status) {
        label = l; type = t; status_ptr = status;
    }
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
        KEY            *key_menu;
        KEY            *key_next;
        KEY            *key_enter;
        OLED           *oled;
        GPIO_Pin        pin_menu;  // 菜单/返回键引脚
        GPIO_Pin        pin_next;  // 向下/增加键引脚
        GPIO_Pin        pin_enter; // 确认键引脚
    };
    void Init(const Config *cfg)
    {
        cfg_     = *cfg;
        root_    = (*cfg).main_menu->child;
        current_ = root_;
        size_    = (*cfg).main_menu->child_count;
    }
    //    explicit MenuManager(const Config &cfg) : cfg_(cfg)
    //    {
    //        root_    = cfg.main_menu->child;
    //        current_ = root_;
    //        size_    = cfg.main_menu->child_count;
    //    }

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
        if ((current_[cursor_].type == ItemType::DISPLAY || current_[cursor_].type == ItemType::STATUS) ||
            (current_[cursor_].type == ItemType::VARIABLE) && current_[cursor_].var_ptr != nullptr) {
            DrawDisplayValue(*current_[cursor_].var_ptr, 2, true);
        } else {
            cfg_.oled->OLED_ShowStr(32, 16 * cursor_, current_[cursor_].label, 2, true);
        }

        // 原光标行
        if ((current_[last_cursor_].type == ItemType::DISPLAY || current_[cursor_].type == ItemType::STATUS) ||
            (current_[last_cursor_].type == ItemType::VARIABLE) && current_[last_cursor_].var_ptr != nullptr) {
            int old_cursor = cursor_;
            cursor_        = last_cursor_;
            DrawDisplayValue(*current_[last_cursor_].var_ptr, 2, false);
            cursor_ = old_cursor;
        } else {
            cfg_.oled->OLED_ShowStr(32, 16 * last_cursor_, current_[last_cursor_].label, 2, false);
        }
        last_cursor_ = cursor_;
    }

    void DrawMenu()
    {
        cfg_.oled->OLED_CLS();
        // 第一行（光标行）
        if ((current_[0].type == ItemType::DISPLAY || current_[cursor_].type == ItemType::STATUS) || (current_[0].type == ItemType::VARIABLE) && current_[0].var_ptr != nullptr) {
            cursor_ = 0;
            DrawDisplayValue(*current_[0].var_ptr, 2, true);
        } else {
            cfg_.oled->OLED_ShowStr(32, 0, current_[0].label, 2, true);
        }
        // 其他行
        for (int i = 1; i < size_; i++) {
            if ((current_[i].type == ItemType::DISPLAY || current_[cursor_].type == ItemType::STATUS) || (current_[i].type == ItemType::VARIABLE) && current_[i].var_ptr != nullptr) {
                int old_cursor = cursor_;
                cursor_        = i;
                DrawDisplayValue(*current_[i].var_ptr, 2, false);
                cursor_ = old_cursor;
            } else {
                cfg_.oled->OLED_ShowStr(32, 16 * i, current_[i].label, 2, false);
            }
        }
        for (int i = size_; i < 4; i++) {
            cfg_.oled->OLED_ShowStr(32, 16 * i, "           ", 2, false);
        }
        cursor_      = 0;
        last_cursor_ = 0;
    }

    // 显示变量值（用于非编辑状态）
    void DrawDisplayValue(float val, uint8_t num, bool is_highlight)
    {
        // 清空buffer
        const auto &item = current_[cursor_];
        char       *p    = buffer;
        // clang-format off
    for (int i = 0; i < 11; i++) buffer[i] = ' ';

    // 根据类型决定显示内容
    if (item.type == ItemType::STATUS && item.status_ptr != nullptr) {
        // STATUS类型：显示 "STATUS: ERR" 或 "STATUS: OK"
        const char* pre = "STATUS:";
        while (*pre) *p++ = *pre++;
        const char* status = (*item.status_ptr != 0) ? "OK" : "ERR";
        while (*status) *p++ = *status++;
    } else {
        const char* pre = (item.type == ItemType::DISPLAY && !is_editing_) ? "NOW:" : "SET:";
        while (*pre) *p++ = *pre++;
        if (val > 199)
            p = IntToStr(val, p);
        else
            p = FloatToStr(val, p);
    }
        // clang-format on
        cfg_.oled->OLED_ShowStr(32, cursor_ * 16, buffer, 2, is_highlight);
        if (is_editing_) {
            DrawEditMark(true); // 编辑状态下显示标记
        }
    }

    // 绘制/清除编辑标记 "*"（1号字体，不占用2号字体位置）
    void DrawEditMark(bool show)
    {
        // 2号字体每个字符8像素宽，"SET: XX.XX"约8字符=64像素
        // 屏幕128像素宽，在第100列位置用1号字体(6x8)显示"*"
        if (show) {
            cfg_.oled->OLED_ShowStr(120, cursor_ * 16, "*", 1, false);
        } else {
            cfg_.oled->OLED_ShowStr(120, cursor_ * 16, " ", 1, false);
        }
    }

    char *IntToStr(long n, char *s)
    {
        if (n < 0) {
            *s++ = '-';
            n    = -n;
        }
        char *start = s;
        if (n == 0)
            *s++ = '0';
        while (n > 0) {
            *s++ = (n % 10) + '0';
            n /= 10;
        }
        for (char *p1 = start, *p2 = s - 1; p1 < p2; p1++, p2--) {
            char tmp = *p1;
            *p1      = *p2;
            *p2      = tmp;
        }
        return s; // 返回结束符指针
    }

    char *FloatToStr(float f, char *s, int precision = 2)
    {
        long p = 1;
        for (int i = 0; i < precision; i++)
            p *= 10;
        long whole = (long)f;
        long part  = (long)((f > 0 ? f - whole : whole - f) * p + 0.5f); // 取小数部分
        if (whole == 0 && f < 0.0f)
            *s++ = '-';
        s                = IntToStr(whole, s);
        *s++             = '.';
        char *frac_start = s;
        s                = IntToStr(part, s);
        int len          = s - frac_start;
        if (len < precision) { // 补齐前导0，如 0.05
            for (int i = len; i >= 0; i--)
                frac_start[i + (precision - len)] = frac_start[i];
            for (int i = 0; i < (precision - len); i++)
                frac_start[i] = '0';
            s += (precision - len);
        }
        return s;
    }

    void KeyScanUpdate()
    {
        cfg_.key_menu->Tick(HAL_GPIO_ReadPin(cfg_.pin_menu.port, cfg_.pin_menu.pin));
        cfg_.key_next->Tick(HAL_GPIO_ReadPin(cfg_.pin_next.port, cfg_.pin_next.pin));
        cfg_.key_enter->Tick(HAL_GPIO_ReadPin(cfg_.pin_enter.port, cfg_.pin_enter.pin));
    }

    void ManagerUpdate()
    {
        Event key_menu  = cfg_.key_menu->GetEvent();
        Event key_next  = cfg_.key_next->GetEvent();
        Event key_enter = cfg_.key_enter->GetEvent();

        // 菜单键：返回上级菜单或回到根菜单
        if (key_menu == Event::CLICK) {
            if (!is_editing_) {
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
                cfg_.oled->OLED_RefreshRAM();
            } else {
                step_dir_ *= -1;
            }
        }
//
//        // 向下键：移动光标或增加变量值
//        if (key_next == Event::CLICK) {
//            if (!is_editing_ || (current_[cursor_].type != ItemType::VARIABLE && current_[cursor_].type != ItemType::DISPLAY || current_[cursor_].type == ItemType::STATUS)) {
//                DrawLine();
//                cfg_.oled->OLED_RefreshRAM();
//            } else {
//                *(current_[cursor_].var_ptr) += current_[cursor_].var_step * step_dir_;
//                DrawDisplayValue(*(current_[cursor_].var_ptr), 2, true);
//                cfg_.oled->OLED_RefreshRAM();
//            }
//        }
//
//        // 确认键：进入菜单/编辑变量/执行函数
//        if (key_enter == Event::CLICK || key_enter == Event::LONG_PRESS) {
//            auto &item = current_[cursor_];
//            if (item.child != nullptr) {
//                switch (item.type) {
//                case ItemType::MENU:
//                    if (menu_stack_top_ < MAX_MENU_DEPTH - 1) {
//                        menu_stack_top_++;
//                        menu_stack_[menu_stack_top_] = {current_, size_};
//                    }
//                    current_ = item.child;
//                    size_    = item.child_count;
//                    DrawMenu();
//                    cfg_.oled->OLED_RefreshRAM();
//                    break;
//
//                case ItemType::VARIABLE:
//                    is_editing_ = !is_editing_;
//                    if (is_editing_) {
//                        DrawDisplayValue(*item.var_ptr, 2, true); // 进入编辑，显示*号
//                        cfg_.oled->OLED_RefreshRAM();
//                    } else {
//                        DrawEditMark(false); // 退出编辑，清除*号
//                        // 刷新显示当前值
//                        if (item.type == ItemType::DISPLAY || current_[cursor_].type == ItemType::STATUS) {
//                            DrawDisplayValue(*item.var_ptr, 2, true);
//                        }
//                        cfg_.oled->OLED_RefreshRAM();
//                    }
//                    break;
//
//                case ItemType::FUNCTION:
//                    if (key_enter == Event::LONG_PRESS) {
//                        item.callback();
//                    }
//                    break;
//                default:
//                    break;
//                }
//            }
//        }

        if (++now_cnt == 4) {
//            RefreshNowRow();
            now_cnt = 0;
        }
    }

    // 单独刷新NOW行和STATUS行的函数，由外部定时调用
    void RefreshNowRow()
    {
        if (is_editing_)
            return;
        for (int i = 0; i < size_; i++) {
            // 刷新DISPLAY类型（NOW行）
            if (current_[i].type == ItemType::DISPLAY && current_[i].var_ptr != nullptr && current_[i].var_ptr2 == nullptr) {
                int old_cursor = cursor_;
                cursor_        = i;
                // clang-format off
            for (int j = 0; j < 11; j++) buffer[j] = ' ';
            char *p = buffer;
            const char *pre = "NOW:";
            while(*pre) *p++ = *pre++;
                // clang-format on

                float val = *current_[i].var_ptr;
                if (val > 199)
                    p = IntToStr(val, p);
                else
                    p = FloatToStr(val, p);
                if (i == old_cursor)
                    cfg_.oled->OLED_ShowStr(32, cursor_ * 16, buffer, 2, true);
                else
                    cfg_.oled->OLED_ShowStr(32, cursor_ * 16, buffer, 2, false);
                cursor_ = old_cursor;
            }
            // 刷新STATUS类型（实时显示ERR/OK）
            else if (current_[i].type == ItemType::STATUS && current_[i].status_ptr != nullptr) {
                int old_cursor = cursor_;
                cursor_        = i;
                for (int j = 0; j < 11; j++)
                    buffer[j] = ' ';
                char       *p   = buffer;
                const char *pre = "STATUS:";
                while (*pre)
                    *p++ = *pre++;
                const char *status = (*current_[i].status_ptr != 0) ? "OK" : "ERR";
                while (*status)
                    *p++ = *status++;
                if (i == old_cursor)
                    cfg_.oled->OLED_ShowStr(32, cursor_ * 16, buffer, 2, true);
                else
                    cfg_.oled->OLED_ShowStr(32, cursor_ * 16, buffer, 2, false);
                cursor_ = old_cursor;
            }
        }
        cfg_.oled->OLED_RefreshRAM();
    }

private:
    static constexpr int MAX_MENU_DEPTH = 4;
    struct MenuFrame {
        const MenuItem *menu;
        int             size;
    };
    Config          cfg_;
    const MenuItem *root_;    // 顶层菜单（主页）
    const MenuItem *current_; // 当前显示的菜单数组
    uint8_t         now_cnt = 0;
    int             size_;            // 当前菜单有多少行
    int             cursor_      = 0; // 光标指向第几行
    int             last_cursor_ = 0;
    bool            is_editing_  = false; // 核心状态：是否正在改参数
    char            buffer[11]{};
    int             menu_stack_top_ = -1;
    MenuFrame       menu_stack_[MAX_MENU_DEPTH]{}; // 菜单栈，支持返回上级
    int8_t          step_dir_ = 1;
};

void task_5ms();
void task_50ms();

#endif // DRIVE_CMAKE_MENU_MANAGER_H
