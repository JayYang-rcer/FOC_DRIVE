#ifndef _KEY_H
#define _KEY_H

#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif
void KEY_ProcessHandle(void);
[[noreturn]] void KeyScanTask(void *argument);
[[noreturn]] [[maybe_unused]] void oledReflashTask(void *argument);
[[noreturn]] [[maybe_unused]] void TempSenseTask(void *argument);
#ifdef __cplusplus
}
#endif

// 按键事件定义
enum class Event :uint8_t {
    NONE = 0,
    CLICK,        // 单击（按下并快速抬起）
    DOUBLE_CLICK, // 双击（预留）
    LONG_PRESS,   // 长按（按下超过阈值）
};

class KEY
{
public:
    struct Config {
        GPIO_TypeDef *port;
        uint32_t      pin;
        bool          active_low;    // 是否低电平有效（通常上拉按键为true）
        uint16_t      long_press_ms; // 长按判定的毫秒数
    };
    explicit KEY(const Config &cfg) : cfg_(cfg) { cfg_.long_press_ms = cfg.long_press_ms / 5; }
    // 在5ms循环中调用
    void Tick(bool raw_state)
    {
        // 1. 更新位 FIFO
        // 如果是低电平有效，取反逻辑
        bool current_logic_state = cfg_.active_low == !raw_state;
        fifo_                    = (fifo_ << 1) | (current_logic_state ? 1 : 0);

        // 2. 状态消抖与判定
        if ((fifo_ & 0x03FF) == 0x03FF) {
            is_stable_pressed_ = true;
        } else if ((fifo_ & 0x03FF) == 0x0000) {
            is_stable_pressed_ = false;
        }

        // 3. 事件状态机处理
        Update();
    }

    Event GetEvent()
    {
        Event e     = last_event_;
        last_event_ = Event::NONE;
        return e;
    }

private:
    void Update()
    {
        // A. 检测按下瞬间
        if (is_stable_pressed_ && !last_stable_pressed_) {
            press_timer_          = 0;
            long_press_triggered_ = false;
        }
        // B. 持续按下状态
        if (is_stable_pressed_) {
            press_timer_ += 1; // 每次Tick是5ms

            // 判定长按
            if (!long_press_triggered_ && press_timer_ >= cfg_.long_press_ms) {
                last_event_           = Event::LONG_PRESS;
                long_press_triggered_ = true;
            }
        }
        // C. 检测抬起瞬间
        else if (last_stable_pressed_) {
            // 如果抬起时还没有触发长按，则判定为单击
            if (!long_press_triggered_) {
                last_event_ = Event::CLICK;
            }
            press_timer_ = 0;
        }

        last_stable_pressed_ = is_stable_pressed_;
    }

    uint16_t fifo_{};
    uint16_t press_timer_{};
    bool     is_stable_pressed_{};
    bool     last_stable_pressed_{};
    bool     long_press_triggered_{};
    Event    last_event_;
    Config   cfg_;
};

#endif /* _KEY_H */