#include "servo_control.h"
#include <pigpio.h>
#include <stdio.h>


static int get_servo_pin(int servo_id) {
    switch(servo_id) {
        case 1: return SERVO_1;
        case 2: return SERVO_2;
        case 3: return SERVO_3;
        case 4: return SERVO_4;
        default: return -1; // 无效ID
    }
}

// 设置舵机角度
void set_angle(int servo_id, int angle) {
    // 校验舵机ID
    int pin = get_servo_pin(servo_id);
    if (pin == -1) {
        write_log(LOG_ERROR, "无效的舵机ID（必须为1-4）");
        return;
    }

    // 限制角度范围（0-180°）
    if (angle < 0) angle = 0;
    if (angle > 300) angle = 300;

    // 计算脉冲宽度（线性映射）
    int pulse = MIN_PULSE + (MAX_PULSE - MIN_PULSE) * angle / 300;
    gpioServo(pin, pulse);

    // 记录日志
    char msg[100];
    sprintf(msg, "舵机%d 已设置到 %d°（脉冲：%dμs）", servo_id, angle, pulse);
    write_log(LOG_DEBUG, msg);
}

// 初始化舵机控制
int servo_init(void) {
    write_log(LOG_INFO, "程序启动，初始化舵机控制");
    // 初始化pigpio库（舵机和GPIO依赖此库）
    if (gpioInitialise() < 0) {
        write_log(LOG_ERROR, "pigpio库初始化失败（请检查是否安装并启动）");
        return -1;
    }
    write_log(LOG_INFO, "pigpio库初始化成功");
    return 0;
}

// 清理舵机资源
void servo_cleanup(void) {
    // 停止所有舵机（发送0μs脉冲）
    gpioServo(SERVO_1, 0);
    gpioServo(SERVO_2, 0);
    gpioServo(SERVO_3, 0);
    gpioServo(SERVO_4, 0);
    write_log(LOG_DEBUG, "所有舵机已停止");

    // 终止pigpio库
    gpioTerminate();
    write_log(LOG_INFO, "程序退出，资源已释放");
}