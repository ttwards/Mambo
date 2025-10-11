# AresPlot 协议示例

这个示例展示了如何使用重构后的 AresPlot 协议与 UART 接口配合工作。

## 功能特性

- **实时数据监控**: 监控多个变量（正弦波、计数器、随机值、布尔标志）
- **UART 通信**: 通过 UART 接口发送数据到上位机
- **自动数据发送**: 定时器驱动的数据采样和发送

## 硬件要求

- 支持的开发板（如 robomaster_board_c）
- UART 连接到上位机

## 构建和运行

```bash
# 构建项目
west build -b robomaster_board_c samples/plotter_demo

# 烧录到开发板
west flash
```

## 使用方法

1. 将开发板通过 UART 连接到上位机
2. 上位机浏览器打开`https://captainkaz.github.io/web-serial-plotter/` (必须使用chromium内核)
3. 选择串口，设置为921600，其余不用修改，然后选择协议为AresPlot
3. 开发板会自动开始发送监控数据
4. 可以将`build/zephyr/zephyr.elf`拖入工具，输入变量名查找变量并重新开始采集，这一操作覆盖之前的正在监视的变量

## 监控的变量

- `sine_wave`: 正弦波值 (float)
- `counter`: 递增计数器 (int32_t)
- `random_value`: 随机值 (int32_t)
- `toggle_flag`: 布尔标志 (bool)

## 协议配置

协议配置可以通过 Kconfig 选项调整：

- `CONFIG_ARESPLOT_MAX_VARS_TO_MONITOR`: 最大监控变量数量
- `CONFIG_ARESPLOT_SHARED_BUFFER_SIZE`: 缓冲区大小
- `CONFIG_ARESPLOT_DEFAULT_SAMPLE_PERIOD_MS`: 默认采样周期

## 代码结构

- `src/main.c`: 主应用程序
- `prj.conf`: 项目配置
- `boards/robomaster_board_c.overlay`: 设备树覆盖文件

## 与原版本的差异

1. **框架集成**: 现在使用 Ares 协议框架
2. **接口抽象**: 支持任何 Ares 接口（UART、USB 等）
3. **更好的错误处理**: 标准化的错误码和处理
4. **线程安全**: 使用互斥锁保护共享数据
5. **定时器驱动**: 使用 Zephyr 内核定时器
