# ARES 双接口通信示例

这个示例演示了如何同时使用 USB Bulk 和 UART 两种接口进行 ARES 协议通信。

## 功能特性

- **双接口支持**: 同时支持 USB Bulk 和 UART 接口
- **ARES 协议**: 使用 ARES dual protocol 进行数据传输
- **交替发送**: 程序会交替通过 USB 和 UART 发送数据
- **回调机制**: 支持函数调用回调和同步数据回调

## 硬件要求

- 支持 USB 设备功能的开发板
- USART6 接口用于 UART 通信
- RoboMaster Board C (或兼容的开发板)

## 配置说明

### prj.conf 主要配置项：
- `CONFIG_USB_BULK_INTERFACE=y`: 启用 USB Bulk 接口
- `CONFIG_UART_INTERFACE=y`: 启用 UART 接口
- `CONFIG_DUAL_PROPOSE_PROTOCOL=y`: 启用双向协议

## 构建和运行

```bash
cd samples/ares_communication
west build -b robomaster_board_c
west flash
```

## 工作原理

1. **初始化阶段**: 程序启动时会初始化 USB Bulk 和 UART 两个接口
2. **协议绑定**: 将双向协议分别绑定到两个接口上
3. **回调设置**: 设置函数调用回调和同步数据回调
4. **数据发送**: 主循环中交替通过两个接口发送数据

## 日志输出

程序运行时会输出以下信息：
- 接口初始化状态
- 数据发送确认
- 回调函数执行情况

## 注意事项

- 确保 USB 和 UART 接口都正确连接
- 如果某个接口初始化失败，程序仍会继续运行，只使用成功初始化的接口

## 接线
### 串口通信
两块C板通过 UART1 进行通信的接线（TX/RX 交叉连接）

### USB通信
一块C板通过USB进行通信的接线

## 上位机测试
在sample_linux下有上位机的协议代码和一个example，可以自行编译或者使用预编译代码
编译：`g++ -std=c++11 -o example src/easy_example.cpp src/ares_protocol.cpp -lusb-1.0 -pthread`
