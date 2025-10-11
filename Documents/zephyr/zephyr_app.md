# Zephyr
 zephyr是一个由Linux基金会支持的RTOS，目前对多种架构的多种MCU具有完整的支持。Zephyr的目标是提供一个小型、可扩展、实时的操作系统，适用于资源受限的设备。Zephyr的特点包括使用设备树描述外围硬件等
# A Basic Zephyr Application
## 文件结构
```
app
├── CMakeLists.txt 	\\ CMake构建文件
├── README.rst 		\\ App描述，这是可选的
├── boards								\\ 对应不同开发板
│   └── robomaster_board_c.overlay		\\ 外围设备配置文件
├── prj.conf		\\ Kconfig配置文件
├── sample.yaml		\\ App描述，这是可选的
└── src				\\ 源文件
    └── main.c
```
## CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.20.0)					\\ 指定cmake最低版本
set(CMAKE_CXX_STANDARD 17)								\\ 指定C++标准
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})	\\ 引用Zephyr
project(IMU)											\\ 项目名

FILE(GLOB app_sources src/*.c)						\\ 搜索src目录下的所有.c文件

target_sources(app PRIVATE ${app_sources})			\\ 将src文件添加到app目标
```
## prj.conf
此目录下指定Kconfig设置，如：
```
# 启用dji电机驱动
CONFIG_MOTOR=y
CONFIG_MOTOR_DJI=y
CONFIG_MOTOR_LOG_LEVEL=4
CONFIG_CAN=y
CONFIG_MOTOR_INIT_PRIORITY=90
```
你可以在zephyr官网上和该项目下drivers/*/Kconfig文件中找到更多的配置选项

## boards
详见../motor/devicetree.md
