#ifndef GPIO_CONTROL_H
#define GPIO_CONTROL_H

#include "log.h"

// 明确定义GPIO引脚宏（BCM编码）
#define Res_Led 23  // 新增：明确声明GPIO23
#define Start_Led 22  // 新增：明确声明GPIO22
#define Stop_Led 24  // 新增：明确声明GPIO24

// 电平定义
#define HIGH 1
#define LOW 0

// 初始化单个GPIO为输出模式
void gpio_init_output(int pin);

// 设置GPIO电平（HIGH/LOW）
void gpio_set_level(int pin, int level);

// 初始化所有需要控制的GPIO（23、22、24）
void gpio_init_all(void);

#endif  // GPIO_CONTROL_H