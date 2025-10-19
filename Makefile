# 编译器和选项
CC = g++
CFLAGS = -I include -I /usr/local/include/src
LDFLAGS = -lpigpio -lrt -L /usr/local/ydlidar-sdk/lib -lydlidar_sdk  # 确保库路径正确

# 所有源文件
SRCS = main.c \
       src/log.c \
       src/servo_control.c \
       src/gpio_control.c \
       src/lidar_control.c

# 目标文件
OBJS = $(SRCS:.c=.o)

# 可执行文件名
TARGET = main

# 默认目标
all: $(TARGET)

# 链接
$(TARGET): $(OBJS)
	$(CC) $(OBJS) -o $@ $(LDFLAGS)

# 编译
%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# 清理
clean:
	rm -f $(OBJS) $(TARGET)
	@echo "已清理编译产物"

# 运行
run: $(TARGET)
	sudo ./$(TARGET)