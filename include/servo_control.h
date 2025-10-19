#ifndef SERVO_CONTROL_H
#define SERVO_CONTROL_H

#include "log.h"

// 4个舵机的BCM端口（按需求修改）
#define SERVO_1 4    // 舵机1端口
#define SERVO_2 18   // 舵机2端口
#define SERVO_3 17   // 舵机3端口
#define SERVO_4 27   // 舵机4端口

// 舵机脉冲参数（微秒）
#define MIN_PULSE 500   // 0°对应脉冲
#define MAX_PULSE 2500  // 180°对应脉冲

// 设置指定舵机的角度（舵机ID：1-4，角度：0-180）
void set_angle(int servo_id, int angle);

// 初始化舵机控制（返回0表示成功）
int servo_init(void);

// 清理舵机资源（停止所有舵机）
void servo_cleanup(void);

#endif  // SERVO_CONTROL_H