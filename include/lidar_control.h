#ifndef LIDAR_CONTROL_H
#define LIDAR_CONTROL_H

#include "log.h"
#include <string>

// 功能：获取指定角度的距离（带日志输出）
// 参数：目标角度（度）、误差容忍（度，默认1度）
// 返回：距离（米，-1表示无效）
float lidar_get_distance(float target_angle_deg, float tolerance_deg = 1.0f);

// 功能：初始化雷达
// 参数：端口号（默认"/dev/ttyUSB0"）、波特率（默认115200）
// 返回：0表示成功，-1表示失败
int lidar_init(const std::string& port = "/dev/ttyUSB0", int baudrate = 115200);

// 功能：启动雷达扫描
// 返回：true表示成功，false表示失败
bool lidar_start();

// 功能：停止雷达并释放资源
void lidar_stop();

#endif  // LIDAR_CONTROL_H