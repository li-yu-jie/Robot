#include "log.h"
#include <stdio.h>
#include <time.h>
#include <string.h>

#define LOG_FILE "./servo_gpio_log.txt"

static const char* level_str[] = {
    "DEBUG",
    "INFO",
    "ERROR"
};

void write_log(LogLevel level, const char* message) {
    time_t now;
    time(&now);
    struct tm* timeinfo = localtime(&now);
    char time_str[20];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d %H:%M:%S", timeinfo);
    FILE* log_file = fopen(LOG_FILE, "a");
    if (log_file == NULL) {
        fprintf(stderr, "[%s] [ERROR] 无法打开日志文件: %s\n", time_str, LOG_FILE);
        return;
    }

    if (level < LOG_DEBUG || level > LOG_ERROR) {
        level = LOG_INFO;
    }
    fprintf(log_file, "[%s] [%s] %s\n", time_str, level_str[level], message);
    fclose(log_file);
}