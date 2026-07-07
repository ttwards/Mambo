# 接口层

ARES 接口层统一承载底层链路，向协议层暴露稳定的发送、缓冲分配和初始化入口。
当前仓库内有两类接口实现：

- UART 接口
- USB 批量传输接口

接口层的公共抽象由 `struct AresInterface` 与 `struct AresInterfaceAPI` 定义。

## `AresInterface`

`include/ares/interface/ares_interface.h` 定义了接口对象：

- `name`: 维护用名称。
- `api`: 具体实现提供的函数表。
- `protocol`: 当前绑定到该接口的协议对象。
- `priv_data`: 具体接口私有状态。

协议层不直接依赖 UART 或 USB 的内部结构，只经由 `api` 访问。

## `AresInterfaceAPI`

接口实现可以提供以下能力。这个结构体是能力表，不是强制每个接口完整实现的 vtable：

- `send()`: 发送 `net_buf`。
- `send_with_callback()`: 发送 `net_buf` 并在完成、abort 或同步失败时通知调用者。
- `send_raw()`: 发送裸字节缓冲，当前由 UART 实现。
- `connect()` / `disconnect()` / `is_connected()`: 连接状态接口，当前 UART/USB 宏未填，属于预留能力。
- `caps()`: 返回 `AresInterfaceCaps` 能力位。
- `mtu()`: 返回接口建议帧长上限。
- `alloc_buf()`: 分配发送缓冲。
- `alloc_buf_with_data()`: 用现有数据块创建发送缓冲，当前由 USB bulk 实现。
- `init()`: 初始化接口实例。

当前宏填充情况：

| 接口宏 | 已填回调 |
| --- | --- |
| `ARES_UART_INTERFACE_DEFINE()` | `init`、`send`、`send_with_callback`、`send_raw`、`caps`、`mtu`、`alloc_buf` |
| `ARES_BULK_INTERFACE_DEFINE()` | `init`、`send`、`send_with_callback`、`caps`、`mtu`、`alloc_buf`、`alloc_buf_with_data` |

协议层在调用可选回调前必须按空指针能力协商。调用 `send()` 或 `send_with_callback()` 后，
`net_buf` 所有权交给接口；当前 UART/USB 实现会在发送完成、发送中止或同步入队失败时释放传入
缓冲，调用者不应再次 `net_buf_unref()`。需要等待 TX 完成的协议应使用 `send_with_callback()`，
并在 `ares_interface_tx_done_cb_t` 里释放自己的在飞状态。

## 绑定顺序

统一绑定入口是 `ares_bind_interface()`，顺序固定为：

1. 写入 `interface->protocol` 与 `protocol->interface`。
2. 初始化接口。
3. 初始化协议。

维护者不应绕过该入口手工调用两侧 `init()`，否则很容易打破依赖顺序。

## UART 接口

### 角色

UART 接口适合：

- 低带宽或中等带宽控制链路
- 逐字节协议解析
- 调试口或外部串口设备

实现位于：

- 头文件：`include/ares/interface/uart/uart.h`
- 实现：`lib/ares/interface/uart/uart.c`

### 公开接口

- `ares_uart_init(struct AresInterface *interface)`
- `ares_uart_send(struct AresInterface *interface, struct net_buf *buf)`
- `ares_uart_send_raw(struct AresInterface *interface, uint8_t *data, uint16_t len)`
- `ares_uart_interface_alloc_buf(struct AresInterface *interface)`
- `ares_uart_init_dev(struct AresInterface *interface, const struct device *uart_dev)`

以及实例定义宏：

- `ARES_UART_INTERFACE_DEFINE(name)`

### 私有状态

`struct AresUartInterface` 维护：

- 接口反向指针
- 原子状态
- RX ring buffer
- UART 设备句柄
- RX 处理信号量与线程
- TX 消息队列、流控信号量和 TX 线程
- 当前在飞的 `net_buf`

这说明 UART 实现是一个收发分离、异步完成的模型，而不是同步阻塞式串口包装。

### 初始化顺序

维护者应按以下顺序使用 UART：

1. 用 `ARES_UART_INTERFACE_DEFINE()` 定义实例。
2. 调用 `ares_uart_init_dev()` 写入 `uart_dev`。
3. 调用 `ares_bind_interface()`。

`ares_uart_init()` 内部会完成：

- `device_is_ready()` 检查
- ring buffer 初始化
- RX 信号量初始化
- `uart_callback_set()`
- 首个 RX slab 缓冲申请
- `uart_rx_enable()`
- RX 处理线程创建
- TX 队列和流控信号量初始化
- TX 线程创建

若设备句柄未事先写入，初始化没有意义；维护时要先查调用点。

### 运行时行为

#### 接收路径

UART 异步回调收到 `UART_RX_RDY` 后：

1. 把新字节写入 ring buffer。
2. 唤醒 RX 处理线程。
3. RX 线程循环读取单字节。
4. 每个字节都转发给 `protocol->api->handle_byte()`。

因此 UART 侧天然适合基于字节状态机的协议，例如双向协议和绘图协议。

#### 发送路径

`ares_uart_send()` 并不直接调用 `uart_tx()`，而是：

1. 把 `net_buf *` 放入 TX 队列。
2. 由专用 TX 线程串行发送。
3. 依靠 `UART_TX_DONE` / `UART_TX_ABORTED` 回调释放在飞缓冲。

这一设计保证了单通道串口发送有统一仲裁点，避免多个上下文直接打到底层驱动。

`ares_uart_send_raw()` 则是例外，它直接调用 `uart_tx()`，更适合 plotter/VOFA 这种自带帧组装、
不依赖 `net_buf` 的快速路径。UART 接口没有实现 `alloc_buf_with_data()`。

### 配置项

`lib/ares/interface/uart/Kconfig` 提供：

- `CONFIG_ARES_UART_LOG_LEVEL`
- `CONFIG_ARES_UART_THREAD_STACK_SIZE`

启用接口本身还需要：

- `CONFIG_UART_INTERFACE`

### 错误边界

UART 接口常见失败点如下：

- TX 队列满：`ares_uart_send()` 返回 `-ENOMEM`，并释放缓冲。
- 设备未 ready：`ares_uart_init()` 返回 `-ENODEV`。
- `uart_callback_set()` 失败：初始化直接返回底层错误码。
- `uart_tx()` 失败：发送线程记录错误并释放当前缓冲。

调用者应将 `send()` 调用视为缓冲所有权转移边界；失败时不要再自行释放已经交给接口的缓冲，避免双重
`unref`。

### 最小入口

```c
ARES_UART_INTERFACE_DEFINE(ctrl_uart_if);

ares_uart_init_dev(&ctrl_uart_if, uart_dev);
ret = ares_bind_interface(&ctrl_uart_if, &ctrl_protocol);
```

## USB 批量传输接口

### 角色

USB bulk 接口适合：

- 上位机大包传输
- 以 `net_buf` 为中心的批量收发
- 需要更高吞吐量的主机链路

实现位于：

- 头文件：`include/ares/interface/usb/usb_bulk.h`
- 实现：`lib/ares/interface/usb/usb_bulk.c`

### 公开接口

- `ares_usbd_init(struct AresInterface *interface)`
- `ares_usbd_write(struct AresInterface *interface, struct net_buf *buf)`
- `ares_usbd_write_with_callback(struct AresInterface *interface, struct net_buf *buf,
  ares_interface_tx_done_cb_t cb, void *user_data)`
- `ares_usbd_caps(struct AresInterface *interface)`
- `ares_usbd_mtu(struct AresInterface *interface)`
- `ares_interface_alloc_buf(struct AresInterface *interface)`
- `ares_interface_alloc_buf_with_data(struct AresInterface *interface, void *data, size_t len)`

以及实例定义宏：

- `ARES_BULK_INTERFACE_DEFINE(name)`

### 数据模型

USB bulk 实现有两个关键方向：

- OUT endpoint: 主机到设备
- IN endpoint: 设备到主机

接收路径采用“中断入队、线程处理”的模式：

1. USBD request handler 收到 OUT 数据。
2. 把 `net_buf` 投递到 `incoming_data_msgq`。
3. 处理线程取出 `net_buf`。
4. 调用 `protocol->api->handle()`，不是 `handle_byte()`。
5. 处理完成后 `net_buf_unref()`。

因此 USB bulk 更偏向帧块处理，而非逐字节中断解析。

### 初始化顺序

对使用者而言，仍然遵循统一顺序：

1. 定义接口实例。
2. 调用 `ares_bind_interface()`。

USB 不需要像 UART 那样先写入一个外部设备句柄；它在内部创建并注册 USBD 设备、描述符、类实例和
处理线程。

### 运行时行为

#### 接收

OUT 端点启用后会持续预投递读请求。收到数据后：

- 若接口仍启用，会尽量重新提交下一次 OUT 请求。
- 若接收队列满，新包会被丢弃并限频告警。

这意味着在高负载下，协议处理线程吞吐跟不上时，USB 层会主动丢包而不是无限堆积。

#### 发送

发送调用 `ares_usbd_write()` 或 `ares_usbd_write_with_callback()`：

1. 检查 USB class 和接口状态。
2. 检查 IN 端点是否已有在飞事务。
3. 在 `net_buf` 用户区写入端点与完成回调信息。
4. 调用 `usbd_ep_enqueue()`。

发送完成后，request handler 清除 IN engaged 状态，调用发送完成回调并释放 `net_buf`。同步入队失败时，
USB 层也会调用完成回调并释放缓冲。

USB bulk 接口没有实现 `send_raw()` 与 `connect()` / `disconnect()` / `is_connected()`。连接状态通过
USBD 消息转换为协议事件，而不是通过 `AresInterfaceAPI` 查询。

### `alloc_buf_with_data()` 的作用

双向协议会优先使用 `alloc_buf_with_data()`。对 USB 来说，这允许用现成帧数据创建发送缓冲，减少协议层
显式拷贝。维护者修改协议发送路径时，应保留这种能力协商，并为没有该能力的接口保留
`alloc_buf()` + `net_buf_add_mem()` fallback。

### 协议事件

USB 配置状态变化会转换成协议事件：

- `ARES_PROTOCOL_EVENT_CONNECTED`
- `ARES_PROTOCOL_EVENT_DISCONNECTED`

这条路径由 USB 层驱动，协议层应在 `event()` 回调中维护在线状态、计时器或清理逻辑。

### 配置项

`lib/ares/interface/usb/Kconfig` 提供：

- `CONFIG_ARES_USB_BULK_LOG_LEVEL`
- `CONFIG_ARES_USB_THREAD_STACK_SIZE`

启用接口本身需要：

- `CONFIG_USB_BULK_INTERFACE`

它还会选择：

- `CONFIG_ARES_COMM_LIB`
- `CONFIG_NET_BUF`
- `CONFIG_USB_DEVICE_STACK_NEXT`
- `CONFIG_UDC_WORKQUEUE`

### 错误边界

USB bulk 常见失败点：

- USB 类尚未初始化：发送返回 `-ENODEV`。
- 接口未 enable：发送返回 `-EPERM`。
- IN 端点忙：发送返回 `-EBUSY`。
- 接收队列满：新收到的 OUT 包被丢弃。
- 端点缓冲分配失败：OUT 请求提交失败并记录错误。

### 最小入口

```c
ARES_BULK_INTERFACE_DEFINE(host_if);

ret = ares_bind_interface(&host_if, &host_protocol);
```

## 维护建议

- 新接口实现应优先复用 `AresInterfaceAPI` 现有能力，不轻易扩展接口表。
- 能用 `handle()` 的批量链路，不要伪装成逐字节链路。
- 能力缺失应通过空函数指针表达，而不是在协议里猜测接口类型。
