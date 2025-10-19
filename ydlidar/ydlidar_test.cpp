/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2018, EAIBOT, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#include "CYdLidar.h"
#include <iostream>
#include <string>
#include <cmath>
#include <core/base/timer.h>
#include <core/common/ydlidar_help.h>

using namespace std;
using namespace ydlidar;

#if defined(_MSC_VER)
#pragma comment(lib, "ydlidar_sdk.lib")
#endif

int main(int argc, char *argv[])
{
  printf("__   ______  _     ___ ____    _    ____  \n");
  printf("\\ \\ / /  _ \\| |   |_ _|  _ \\  / \\  |  _ \\ \n");
  printf(" \\ V /| | | | |    | || | | |/ _ \\ | |_) | \n");
  printf("  | | | |_| | |___ | || |_| / ___ \\|  _ <  \n");
  printf("  |_| |____/|_____|___|____/_/   \\_\\_| \\_\\ \n");
  printf("\n");
  fflush(stdout);

  // 固定参数（根据日志中的有效配置）
  const std::string port = "/dev/ttyUSB0";  // 固定端口
  const int baudrate = 115200;              // 固定波特率
  const bool isSingleChannel = true;        // 单通道模式
  const float frequency = 5.0f;             // 扫描频率

  // 设置目标角度
  float target_angle_deg;
  printf("请输入需要测量的目标角度(度，例如0表示正前方):");
  std::cin >> target_angle_deg;
  const float target_angle_rad = target_angle_deg * M_PI / 180.0f;
  const float angle_tolerance = 1.0f * M_PI / 180.0f;  // 1度误差容忍

  // 初始化雷达（保留原初始化逻辑）
  ydlidar::os_init();
  CYdLidar laser;

  // 串口设置
  laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
  std::string ignore_array;
  laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

  // 整数参数设置（不修改原逻辑）
  laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
  int optval = TYPE_TRIANGLE;
  laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
  optval = YDLIDAR_TYPE_SERIAL;
  laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
  optval = 5;
  laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
  optval = 4;
  laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));
  optval = 10;
  laser.setlidaropt(LidarPropIntenstiyBit, &optval, sizeof(int));

  // 布尔参数设置（不修改原逻辑）
  bool b_optvalue = false;
  laser.setlidaropt(LidarPropFixedResolution, &b_optvalue, sizeof(bool));
  laser.setlidaropt(LidarPropReversion, &b_optvalue, sizeof(bool));
  laser.setlidaropt(LidarPropInverted, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropAutoReconnect, &b_optvalue, sizeof(bool));
  laser.setlidaropt(LidarPropSingleChannel, &isSingleChannel, sizeof(bool));
  b_optvalue = false;
  laser.setlidaropt(LidarPropIntenstiy, &b_optvalue, sizeof(bool));
  b_optvalue = true;
  laser.setlidaropt(LidarPropSupportMotorDtrCtrl, &b_optvalue, sizeof(bool));
  b_optvalue = false;
  laser.setlidaropt(LidarPropSupportHeartBeat, &b_optvalue, sizeof(bool));

  // 浮点参数设置（不修改原逻辑）
  float f_optvalue = 180.0f;
  laser.setlidaropt(LidarPropMaxAngle, &f_optvalue, sizeof(float));
  f_optvalue = -180.0f;
  laser.setlidaropt(LidarPropMinAngle, &f_optvalue, sizeof(float));
  f_optvalue = 64.f;
  laser.setlidaropt(LidarPropMaxRange, &f_optvalue, sizeof(float));
  f_optvalue = 0.05f;
  laser.setlidaropt(LidarPropMinRange, &f_optvalue, sizeof(float));
  laser.setlidaropt(LidarPropScanFrequency, &frequency, sizeof(float));

  // 禁用噪声过滤（保留原逻辑）
  laser.enableGlassNoise(false);
  laser.enableSunNoise(false);
  laser.setBottomPriority(true);

  // 初始化雷达
  uint32_t t = getms();
  int c = 0;
  bool ret = laser.initialize();
  if (!ret)
  {
    fprintf(stderr, "初始化失败: %s\n", laser.DescribeError());
    return -1;
  }

  // 启动雷达
  ret = laser.turnOn();
  if (!ret)
  {
    fprintf(stderr, "启动失败: %s\n", laser.DescribeError());
    return -1;
  }

  // 打印设备信息（保留原逻辑）
  device_info di;
  memset(&di, 0, DEVICEINFOSIZE);
  if (laser.getDeviceInfo(di, EPT_Module))
    ydlidar::core::common::printfDeviceInfo(di, EPT_Module);
  else
    printf("获取模组信息失败\n");

  if (laser.getDeviceInfo(di, EPT_Base))
    ydlidar::core::common::printfDeviceInfo(di, EPT_Base);
  else
    printf("获取底板信息失败\n");

  // 主循环：获取数据并解析指定角度距离
  LaserScan scan;
  while (ydlidar::os_isOk())
  {
    if (laser.doProcessSimple(scan))
    {
      printf("\n收到扫描数据，共 %llu 个点，扫描频率 %.02f Hz\n",
             scan.points.size(), scan.scanFreq);

      // 查找目标角度的距离
      float target_distance = -1.0f;
      float min_angle_diff = 10000.0f;
      float closest_angle_rad = 0.0f;

      for (const auto& point : scan.points)
      {
        // 计算角度差（处理360度循环）
        float angle_diff = fabsf(point.angle - target_angle_rad);
        angle_diff = fminf(angle_diff, 2 * M_PI - angle_diff);

        if (angle_diff < min_angle_diff)
        {
          min_angle_diff = angle_diff;
          target_distance = point.range;
          closest_angle_rad = point.angle;

          if (min_angle_diff < angle_tolerance)
            break;
        }
      }

      // 输出结果
      if (target_distance >= 0.0f)
      {
        printf("目标角度: %.1f°  最接近角度: %.1f°  距离: %.3f 米\n",
               target_angle_deg,
               closest_angle_rad * 180.0f / M_PI,
               target_distance);
      }
      else
      {
        printf("未找到目标角度 %.1f° 的有效数据\n", target_angle_deg);
      }
    }
    else
    {
      fprintf(stderr, "获取雷达数据失败\n");
    }

    if (!c++)
    {
      printf("初始化到首次解析数据耗时: %u ms\n", getms() - t);
    }
  }

  // 停止雷达并断开连接
  laser.turnOff();
  laser.disconnecting();
  return 0;
}