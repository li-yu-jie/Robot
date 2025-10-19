cmake_minimum_required(VERSION 2.8)
project(ydlidar_test)

# 启用 C++11 标准
add_compile_options(-std=c++11 -Wall)

# 配置 YDLidar 头文件路径（根据 CYdLidar.h 所在目录）
include_directories(
    /usr/local/include/src  # 关键：指向存放 CYdLidar.h 的目录
)

# 配置 YDLidar 库文件路径（通常 SDK 库会安装在 /usr/local/lib 或 /usr/lib）
link_directories(
    /usr/local/lib  # 若 libydlidar_sdk.so 在此目录
    # /usr/lib      # 若库文件在此目录，取消注释此行
)

# 编译可执行文件
add_executable(ydlidar_test ydlidar_test.cpp)

# 链接 YDLidar SDK 库和系统依赖
target_link_libraries(ydlidar_test
    ydlidar_sdk  # YDLidar 核心库
    pthread      # 线程库
    rt           # 实时库
)