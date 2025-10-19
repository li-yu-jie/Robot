#include "gpio_control.h"
#include <pigpio.h>
#include <stdio.h>

// 初始化单个GPIO为输出模式
void gpio_init_output(int pin) {
    if (gpioSetMode(pin, PI_OUTPUT) != 0) {
        char msg[100];
        sprintf(msg, "GPIO%d 初始化输出模式失败", pin);
        write_log(LOG_ERROR, msg);
    } else {
        char msg[100];
        sprintf(msg, "GPIO%d 已初始化为输出模式", pin);
        write_log(LOG_INFO, msg);
    }
}

// 设置GPIO电平
void gpio_set_level(int pin, int level) {
    if (level != HIGH && level != LOW) {
        write_log(LOG_ERROR, "电平值必须是HIGH(1)或LOW(0)");
        return;
    }

    // 写入电平并记录日志
    gpioWrite(pin, level);
    char msg[100];
    sprintf(msg, "GPIO%d 已设置为%s电平", pin, level ? "高" : "低");
    write_log(LOG_DEBUG, msg);
}

// 初始化所有需要控制的GPIO（使用GPIO_23/22/24宏）
void gpio_init_all(void) {
    gpio_init_output(Start_Led);  // 替换 Start_Led
    gpio_init_output(Res_Led);  // 替换 Res_Led
    gpio_init_output(Stop_Led);  // 替换 Stop_Led
}