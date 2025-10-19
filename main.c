#include "servo_control.h"
#include "gpio_control.h"
#include "lidar_control.h"
#include <unistd.h>
#include <stdio.h>
#include <signal.h>
#include <stdlib.h>
#include <stdbool.h>

static volatile bool g_quit = false;

// 信号处理：退出时清理资源
void handle_signal(int signum) {
    printf("\n收到退出信号，清理资源...\n");
    lidar_stop();       // 停止雷达
    servo_cleanup();    // 清理舵机
    exit(0);
}

// 初始化模块（舵机、GPIO、雷达）
bool init_modules() {
    // 初始化舵机（依赖pigpio库）
    if (servo_init() != 0) {
        printf("舵机初始化失败\n");
        return false;
    }

    // 初始化GPIO（LED等外设）
    gpio_init_all();
    printf("GPIO初始化完成\n");

    // 初始化并启动雷达
    if (lidar_init("/dev/ttyUSB0", 115200) != 0 || !lidar_start()) {
        printf("雷达初始化失败\n");
        servo_cleanup();
        return false;
    }

    return true;
}

int main() {
    // 注册信号处理（捕获Ctrl+C和段错误）
    signal(SIGINT, handle_signal);
    signal(SIGSEGV, handle_signal);

    printf("机器人控制系统启动 (按Ctrl+C退出)\n");

    // 初始化所有模块
    if (!init_modules()) {
        return 1;  // 初始化失败则退出
    }

    // 监控两个角度（90°和0°）
    const float angles[] = {90.0f, 0.0f};
    printf("开始监控 %0.1f° 和 %0.1f° 方向\n", angles[0], angles[1]);

    // 主循环：持续获取数据并控制外设
    while (!g_quit) {
        // // 1. 获取两个角度的雷达距离（使用带日志的接口）
        float dist1 = lidar_get_distance(angles[0], 1.0f);  // 90°
        float dist2 = lidar_get_distance(angles[1], 1.0f);  // 0°
        printf("当前距离: %0.3fm (90°), %0.3fm (0°)\n", dist1, dist2);
        
        // 2. 控制GPIO闪烁（22/23/24同时闪烁，周期100ms）
        gpio_set_level(Start_Led, HIGH);
        gpio_set_level(Res_Led, HIGH);
        gpio_set_level(Stop_Led, HIGH);
        usleep(50000);   // 亮50ms
        gpio_set_level(Start_Led, LOW);
        gpio_set_level(Res_Led, LOW);
        gpio_set_level(Stop_Led, LOW);
        usleep(50000);   // 灭50ms
       
        // 3. 控制舵机动作（修正ID错误，舵机ID应为1-4）
        // 第一组动作
        set_angle(1, 300);  // 舵机1转到180°
        set_angle(2, 300);   // 舵机2转到90°
        set_angle(3, 300);  // 舵机3转到180°
        set_angle(4, 200);    // 舵机4转到0°
        usleep(2000000);    // 等待1秒

        // 第二组动作
        set_angle(1, 0);    // 舵机1转到0°
        set_angle(2, 0);    // 舵机2转到0°
        set_angle(3, 100);    // 舵机3转到0°
        set_angle(4, 100);   // 舵机4转到20°
        usleep(2000000);    // 等待1秒

        // 4. 主循环间隔（5秒，避免动作过快）
        usleep(10000);
    }

    // 正常退出（理论上会被信号处理函数提前触发）
    lidar_stop();
    servo_cleanup();
    return 0;
}