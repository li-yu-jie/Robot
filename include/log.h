#ifndef LOG_H
#define LOG_H

// 日志级别枚举（包含警告级别）
typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARN,  // 警告级别
    LOG_ERROR
} LogLevel;

// 日志写入函数（参数：级别、消息字符串）
void write_log(LogLevel level, const char* message);

#endif  // LOG_H