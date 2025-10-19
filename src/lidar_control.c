#include "lidar_control.h"
#include "CYdLidar.h"
#include <cmath>
#include <string>
#include <stdio.h>
#include <unistd.h>
#include <core/base/timer.h>
#include <core/common/ydlidar_help.h>

using namespace ydlidar;

// 静态变量：雷达实例及状态
static CYdLidar laser;
static bool is_initialized = false;  // 初始化状态
static bool is_running = false;      // 运行状态
static std::string current_port;     // 保存当前端口（用于重连）
static int current_baudrate;         // 保存当前波特率（用于重连）

// 内部函数：雷达重连
static bool lidar_reconnect() {
    write_log(LOG_WARN, "尝试重新连接雷达...");
    lidar_stop();               // 先停止当前连接
    usleep(1000000);            // 等待1秒
    return (lidar_init(current_port, current_baudrate) == 0) && lidar_start();
}

// 内部函数：获取原始距离数据（无日志输出）
static float lidar_get_raw_distance(float target_angle_deg, float tolerance_deg) {
    // 检查雷达状态（用内部变量判断，避免调用不存在的isConnected()）
    if (!is_initialized || !is_running) {
        write_log(LOG_ERROR, "雷达未初始化或未启动");
        return -1.0f;
    }

    // 获取扫描数据
    LaserScan scan;
    static int timeout_count = 0;  // 超时计数器
    if (!laser.doProcessSimple(scan)) {
        timeout_count++;
        char msg[100];
        snprintf(msg, sizeof(msg), "数据获取超时（第%d次）", timeout_count);
        write_log(LOG_ERROR, msg);

        // 连续3次超时触发重连
        if (timeout_count >= 3) {
            if (!lidar_reconnect()) {
                is_running = false;  // 标记为未运行
                return -1.0f;
            }
            timeout_count = 0;  // 重置计数器
        }
        return -1.0f;
    }

    timeout_count = 0;  // 成功获取数据，重置计数器

    // 角度转换（度→弧度）
    const float target_rad = target_angle_deg * M_PI / 180.0f;
    const float tolerance_rad = tolerance_deg * M_PI / 180.0f;

    // 查找最接近目标角度的点
    float min_angle_diff = 10000.0f;  // 最小角度差（初始化为大值）
    float best_distance = -1.0f;      // 最佳距离

    for (const auto& point : scan.points) {
        // 计算角度差（处理360度循环）
        float angle_diff = fabsf(point.angle - target_rad);
        angle_diff = fminf(angle_diff, 2 * M_PI - angle_diff);  // 取最小差值

        if (angle_diff < min_angle_diff) {
            min_angle_diff = angle_diff;
            best_distance = point.range;

            // 若在误差范围内，提前退出
            if (min_angle_diff < tolerance_rad) {
                break;
            }
        }
    }

    // 仅返回误差范围内的有效距离
    return (min_angle_diff < tolerance_rad) ? best_distance : -1.0f;
}

// 初始化雷达（外部接口）
int lidar_init(const std::string& port, int baudrate) {
    // 保存端口和波特率（用于重连）
    current_port = port;
    current_baudrate = baudrate;

    write_log(LOG_INFO, "初始化激光雷达...");
    ydlidar::os_init();  // 初始化系统

    // 配置串口参数
    laser.setlidaropt(LidarPropSerialPort, port.c_str(), port.size());
    std::string ignore_array;  // 忽略角度数组（空）
    laser.setlidaropt(LidarPropIgnoreArray, ignore_array.c_str(), ignore_array.size());

    // 配置整数参数
    laser.setlidaropt(LidarPropSerialBaudrate, &baudrate, sizeof(int));
    int optval = TYPE_TRIANGLE;  // 雷达类型（三角测距）
    laser.setlidaropt(LidarPropLidarType, &optval, sizeof(int));
    optval = YDLIDAR_TYPE_SERIAL;  // 设备类型（串口）
    laser.setlidaropt(LidarPropDeviceType, &optval, sizeof(int));
    optval = 5;  // 采样率
    laser.setlidaropt(LidarPropSampleRate, &optval, sizeof(int));
    optval = 8;  // 异常检查次数（增加容错）
    laser.setlidaropt(LidarPropAbnormalCheckCount, &optval, sizeof(int));

    // 配置布尔参数
    bool b_optval = true;
    laser.setlidaropt(LidarPropAutoReconnect, &b_optval, sizeof(bool));  // 自动重连
    laser.setlidaropt(LidarPropSingleChannel, &b_optval, sizeof(bool));   // 单通道模式

    // 配置浮点参数
    float f_optval = 90.0f;   // 最大扫描角度（度）
    laser.setlidaropt(LidarPropMaxAngle, &f_optval, sizeof(float));
    f_optval = -90.0f;        // 最小扫描角度（度）
    laser.setlidaropt(LidarPropMinAngle, &f_optval, sizeof(float));
    f_optval = 64.0f;         // 最大距离（米）
    laser.setlidaropt(LidarPropMaxRange, &f_optval, sizeof(float));
    f_optval = 0.05f;         // 最小距离（米）
    laser.setlidaropt(LidarPropMinRange, &f_optval, sizeof(float));
    f_optval = 3.0f;          // 扫描频率（Hz，降低频率减少卡顿）
    laser.setlidaropt(LidarPropScanFrequency, &f_optval, sizeof(float));

    // 禁用噪声过滤
    laser.enableGlassNoise(false);
    laser.enableSunNoise(false);
    laser.setBottomPriority(true);

    // 执行初始化
    if (!laser.initialize()) {
        std::string error_msg = "初始化失败: " + std::string(laser.DescribeError());
        write_log(LOG_ERROR, error_msg.c_str());
        return -1;
    }

    // 打印设备信息
    device_info di;
    memset(&di, 0, DEVICEINFOSIZE);
    if (laser.getDeviceInfo(di, EPT_Module)) {
        ydlidar::core::common::printfDeviceInfo(di, EPT_Module);
    } else {
        write_log(LOG_WARN, "获取模组信息失败");
    }

    if (laser.getDeviceInfo(di, EPT_Base)) {
        ydlidar::core::common::printfDeviceInfo(di, EPT_Base);
    } else {
        write_log(LOG_WARN, "获取底板信息失败");
    }

    is_initialized = true;
    write_log(LOG_INFO, "雷达初始化成功");
    return 0;
}

// 启动雷达（外部接口）
bool lidar_start() {
    if (!is_initialized) {
        write_log(LOG_ERROR, "雷达未初始化，无法启动");
        return false;
    }

    // 启动雷达扫描
    if (!laser.turnOn()) {
        std::string error_msg = "启动失败: " + std::string(laser.DescribeError());
        write_log(LOG_ERROR, error_msg.c_str());
        return false;
    }

    // 启动后验证（确保能获取到数据）
    usleep(1000000);  // 等待1秒
    LaserScan test_scan;
    if (!laser.doProcessSimple(test_scan) || test_scan.points.empty()) {
        write_log(LOG_ERROR, "启动后未收到有效数据，可能硬件异常");
        laser.turnOff();  // 关闭雷达
        return false;
    }

    is_running = true;
    write_log(LOG_INFO, "雷达启动成功，开始扫描");
    return true;
}

// 获取距离并输出日志（外部接口）
float lidar_get_distance(float target_angle_deg, float tolerance_deg) {
    // 调用内部函数获取原始距离
    float distance = lidar_get_raw_distance(target_angle_deg, tolerance_deg);

    // 控制台输出和日志记录
    static int log_counter = 0;  // 日志计数器（减少输出频率）
    if (distance >= 0.0f) {
        // 控制台实时输出
        printf("[雷达数据] 角度: %.1f°  距离: %.3f米\n", target_angle_deg, distance);
        
        // 每5次记录一次日志（减少IO操作）
        if (log_counter % 5 == 0) {
            char log_msg[100];
            snprintf(log_msg, sizeof(log_msg), "角度 %.1f° 距离: %.3f米", target_angle_deg, distance);
            write_log(LOG_INFO, log_msg);
        }
        log_counter++;
    } else {
        // 无效数据处理
        printf("[雷达数据] 角度: %.1f°  无有效数据\n", target_angle_deg);
        write_log(LOG_WARN, "未获取到有效雷达数据");
    }

    return distance;
}

// 停止雷达（外部接口）
void lidar_stop() {
    if (is_running) {
        laser.turnOff();  // 停止扫描
        is_running = false;
    }
    if (is_initialized) {
        laser.disconnecting();  // 断开连接
        is_initialized = false;
    }
    write_log(LOG_INFO, "雷达已停止并释放资源");
}