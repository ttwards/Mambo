# 公共 API 参考

本文件汇总 Mambo 当前公共接口。接口按调用边界分组：应用层优先使用
`include/zephyr/drivers/` 与 `include/ares/`，驱动内部共享接口放在后半部分。

## Zephyr 驱动 API

### 电机

头文件：`include/zephyr/drivers/motor.h`

Motor API 将不同厂商的旋转电机统一为 `set/get/control` 三类应用操作。
控制器类型、参数、选择与内置计算辅助位于
`include/zephyr/drivers/motor/controller.h`；`motor.h` 会继续包含该头文件以保持兼容。

#### 类型

| 类型 | 说明 |
| --- | --- |
| `enum motor_mode` | 控制模式。`MIT`、`PV`、`VO`。 |
| `enum motor_target` | 目标类型。`TORQUE`、`SPEED`、`POSITION`。 |
| `enum motor_controller_select` | 控制器选择方式。 |
| `enum motor_state_flags` | 控制器声明所需状态量。 |
| `enum motor_output_type` | 控制器输出类型。 |
| `enum motor_cmd` | 电机控制命令。 |
| `motor_setpoint_t` | 应用下发的目标。 |
| `motor_status_t` | 驱动回读的状态。 |
| `motor_controller_info_t` | 控制器描述信息。 |
| `struct motor_controller_params` | 控制器参数。 |
| `struct motor_link_state` | 驱动内部链路状态。 |

#### 应用函数

| 接口 | 说明 | 返回值 |
| --- | --- | --- |
| `motor_set(dev, setpoint)` | 提交目标设定。驱动可缓存目标并按协议周期发送。 | `0` 成功，负 errno 失败。 |
| `motor_get(dev, status)` | 读取状态快照。 | `0` 成功，负 errno 失败。 |
| `motor_control(dev, cmd)` | 执行控制命令。 | 无返回值。 |
| `motor_get_controller_count(dev)` | 返回设备配置的控制器数量。 | 非负数量。 |
| `motor_get_controller_info(dev, index, info)` | 读取控制器描述。 | `0` 成功，`-EINVAL` 表示索引无效。 |
| `motor_resolve_controller(dev, setpoint, info)` | 按 setpoint 选择控制器。 | `0` 成功，`-ENOTSUP` 或 `-EINVAL` 失败。 |

#### 便捷宏

| 宏 | 说明 |
| --- | --- |
| `motor_set_angle(dev, angle)` | 下发位置目标。 |
| `motor_set_rpm(dev, rpm)` | 下发速度目标。 |
| `motor_set_speed(dev, speed)` | 下发速度目标，等同于 `motor_set_rpm()`。 |
| `motor_set_torque(dev, torque)` | 下发扭矩目标。 |
| `motor_set_mit(dev, speed, angle, torque)` | 下发 MIT 目标。 |
| `motor_get_angle(dev)` | 读取当前角度。 |
| `motor_get_rpm(dev)` | 读取当前转速。 |
| `motor_get_torque(dev)` | 读取当前扭矩。 |
| `motor_get_speed(dev)` | 读取当前转速。 |
| `motor_get_mode(dev)` | 读取当前模式。 |

#### 控制器函数

头文件：`include/zephyr/drivers/motor/controller.h`

| 接口 | 说明 |
| --- | --- |
| `motor_controller_update(data, cfg, input, output)` | 执行一次控制器更新。 |
| `motor_controller_get_params(cfg, index, params)` | 读取控制器参数组。 |
| `motor_controller_reset(data, cfg)` | 清空控制器运行状态。 |
| `motor_builtin_controller_update(data, cfg, input, output)` | 内置控制器实现。 |
| `motor_controller_stage_update(data, params, error, timestamp)` | 单级 PID 风格计算。 |
| `motor_controller_clamp(value, min, max)` | 限幅工具。 |

应用通常不直接调用内置控制器函数。自定义驱动或测试可使用这些函数验证控制器行为。

### 线性执行器

头文件：`include/zephyr/drivers/linear_actuator.h`

线性执行器 API 面向直线执行器和电缸，独立于旋转电机 API。

| 接口 | 说明 | 返回值 |
| --- | --- | --- |
| `la_set_position(dev, position)` | 设置目标位置。`position` 范围由驱动约定，Yinshi LA 使用 `0..2000`。 | `0` 成功，负 errno 失败。 |
| `la_get_status(dev, status)` | 读取状态。 | `0` 成功，负 errno 失败。 |
| `la_enable(dev)` | 使能执行器。 | `0` 成功，负 errno 失败。 |
| `la_disable(dev)` | 急停或失能。 | `0` 成功，负 errno 失败。 |
| `la_clear_fault(dev)` | 清除故障。 | `0` 成功，负 errno 失败。 |
| `la_save_params(dev)` | 保存参数到执行器非易失存储。 | `0` 成功，负 errno 失败。 |
| `la_pause(dev)` | 暂停执行器。 | `0` 成功，负 errno 失败。 |
| `la_set_param(dev, index, value)` | 写控制表参数。 | `0` 成功，负 errno 失败。 |

`struct la_status` 包含目标位置、当前位置、温度、电流、力传感器值、故障位和在线状态。

### 底盘（Chassis）

头文件：`include/zephyr/drivers/chassis.h`

Chassis API 面向整车底盘。底层 wheel 数量由 devicetree 的 `wheels` 属性决定。

| 接口 | 说明 |
| --- | --- |
| `chassis_set_speed(dev, speedX, speedY)` | 设置底盘平动速度。 |
| `chassis_set_angle(dev, angle)` | 设置底盘朝向目标。 |
| `chassis_set_gyro(dev, gyro)` | 设置角速度目标。 |
| `chassis_get_status(dev)` | 返回底盘状态指针。 |
| `chassis_set_static(dev, static_angle)` | 设置静态锁角模式。 |
| `chassis_update_sensor(dev, pos_data)` | 更新底盘传感器数据。 |
| `chassis_set_enabled(dev, enabled)` | 启用或关闭底盘输出。 |

`chassis_status_t` 包含 `speedX`、`speedY`、`gyro` 和 `angle`。`struct pos_data`
包含 yaw、加速度和陀螺仪数据。

### 轮组（Wheel）

头文件：`include/zephyr/drivers/wheel.h`

Wheel API 面向单个轮组。

| 接口 | 说明 | 返回值 |
| --- | --- | --- |
| `wheel_set_speed(dev, speed, angle)` | 设置轮速和轮向。 | 无返回值。 |
| `wheel_set_static(dev, angle)` | 锁定到指定角度。 | `0` 成功，负 errno 失败。 |
| `wheel_get_speed(dev)` | 读取当前速度和角度。 | `wheel_status_t *` 或 `NULL`。 |
| `wheel_get_target(dev)` | 读取目标速度和角度。 | `wheel_status_t *` 或 `NULL`。 |
| `wheel_disable(dev)` | 禁用轮组输出。 | 无返回值。 |

`wheel_status_t` 包含 `speed`、`angle` 和 `restricted`。

### SBUS

头文件：`include/zephyr/drivers/sbus.h`

| 接口 | 说明 | 返回值 |
| --- | --- | --- |
| `sbus_get_percent(dev, channelid)` | 读取模拟通道并转换为百分比。500 ms 未收到有效帧后视为离线并清零通道缓存。 | 在线时通常为 `-1.0..1.0`；离线或通道号非法时返回 `0.0` 并记录错误。 |
| `sbus_get_digit(dev, channelid)` | 读取数字通道。500 ms 未收到有效帧后视为离线并清零通道缓存。 | 在线时返回驱动定义的离散值；离线返回 `-ENETDOWN`，通道号非法返回 `-EINVAL`。 |

### PID

头文件：`include/zephyr/drivers/pid.h`

PID 接口是旧版控制器工具。Motor 新 controller 不依赖该公共 API。

| 接口 | 说明 |
| --- | --- |
| `pid_calc(data)` | 使用 `pid_data` 中的引用和反馈指针计算输出。 |
| `pid_calc_in(data, error, deltaT_us)` | 直接输入误差和时间步长，返回输出。 |
| `pid_calc_mit(data, pos_error, vel_error, deltaT_us)` | MIT 风格位置与速度误差计算。 |

`struct pid_config` 来源于 devicetree，包含 `k_p`、`k_i`、`k_d`、积分限幅、输出限幅、
偏置和微分低通参数。

## 电机内部 API

这些接口由 motor 子系统内部共享。应用代码应优先使用 `motor.h`。

### 运行时统计

头文件：`drivers/motor/common/common.h`

| 接口 | 说明 |
| --- | --- |
| `motor_stats_inc(stat)` | 增加一个运行时计数器。 |
| `motor_stats_get(stats)` | 读取运行时计数器快照。 |

`enum motor_runtime_stat` 覆盖不支持的模式、配置错误、CAN filter 错误、未知 RX、驱动错误、限幅和发送错误。

### 链路状态

头文件：`drivers/motor/common/motor_link.h`

| 接口 | 说明 |
| --- | --- |
| `motor_link_request_enable(link)` | 记录用户请求使能。 |
| `motor_link_request_disable(link)` | 记录用户请求失能，并清理链路状态。 |
| `motor_link_mark_online(motor, link, reason)` | 将电机标记为在线。 |
| `motor_link_mark_offline(motor, link, reason)` | 将电机标记为离线。 |
| `motor_link_observe_reply(motor, link)` | 记录一次 request/reply 回复。 |
| `motor_link_note_missed_reply(motor, link, offline_after_misses)` | 记录一次缺失回复，达到阈值后离线。 |
| `motor_link_observe_periodic_report(motor, link)` | 记录一次周期上报。 |
| `motor_link_note_periodic_timeout(motor, link, offline_after_misses)` | 记录一次周期上报超时，达到阈值后离线。 |

返回 `bool` 的接口表示状态是否发生迁移。

### CAN 调度器

头文件：`drivers/motor/common/motor_can_sched.h`

| 接口 | 说明 | 返回值 |
| --- | --- | --- |
| `motor_can_sched_init()` | 初始化 CAN 调度器。 | `0` 成功。 |
| `motor_can_sched_register_can(can_dev)` | 注册一个 CAN 设备。 | `0` 成功，负 errno 失败。 |
| `motor_can_sched_send(can_dev, frame, param, handle_out)` | 通用发送入口，可配置周期、应答追踪与优先级。 | `0` 成功，负 errno 失败。 |
| `motor_can_sched_send_with_priority(can_dev, frame, priority, tag)` | 按明确优先级发送单帧。 | `0` 成功，负 errno 失败。 |
| `motor_can_sched_send_reply(can_dev, frame, reply_id, reply_mask, timeout_ms, tag)` | 发送请求回复帧。 | `0` 成功，负 errno 失败。 |
| `motor_can_sched_update(handle, frame)` | 更新周期帧内容。 | `0` 成功，负 errno 失败。 |
| `motor_can_sched_remove(handle)` | 删除周期帧。 | `0` 成功，负 errno 失败。 |
| `motor_can_sched_report_rx(can_dev, frame)` | 向 CAN 调度器报告 RX 帧。 | 无返回值。 |
| `motor_can_sched_get_stats(can_dev, stats)` | 读取统计。 | `0` 成功，负 errno 失败。 |

`struct motor_can_sched_stats` 统计 TX、RX、ack、drop、retry、timeout、queue peak、按优先级 drop、
窗口占用和 `CONFIG_MOTOR_CAN_SCHED_TX_LATENCY_TRACE` 控制的 TX latency。

### 遥测

头文件：`drivers/motor/common/motor_telemetry.h`

| 接口 | 说明 |
| --- | --- |
| `motor_telemetry_motor_online(motor, reason)` | 输出电机在线事件。 |
| `motor_telemetry_motor_offline(motor, reason, missed)` | 输出电机离线事件。 |
| `motor_telemetry_can_scheduler_health(can_dev, stats)` | 输出 CAN 调度器压力与调试统计。 |

未启用 `CONFIG_MOTOR_TELEMETRY` 时，这些接口编译为空实现。

## ARES 通信 API

### 绑定与协议

头文件：`include/ares/ares_comm.h`

| 接口 | 说明 | 返回值 |
| --- | --- | --- |
| `ares_bind_interface(interface, protocol)` | 绑定接口与协议，并按顺序初始化。 | `0` 成功，负 errno 失败。 |

### 接口层

头文件：`include/ares/interface/ares_interface.h`

`struct AresInterfaceAPI` 是传输层能力表。不是每个接口都会实现所有回调，协议层调用前应检查函数
指针是否为空。`send()` 与 `send_with_callback()` 调用后，`net_buf` 所有权交给接口；接口会在发送
完成、发送中止或同步入队失败时释放缓冲。

| 回调 | 说明 | 当前实现 |
| --- | --- | --- |
| `send(interface, buf)` | 发送一个 `net_buf`，不需要完成通知时使用。 | UART、USB bulk |
| `send_with_callback(interface, buf, cb, user_data)` | 发送 `net_buf` 并在 TX 完成、abort 或同步失败时调用 `ares_interface_tx_done_cb_t`。 | UART、USB bulk |
| `send_raw(interface, data, len)` | 发送原始字节，不使用 `net_buf`。 | UART |
| `connect(interface)` | 建立连接。 | 预留，当前 UART/USB 未填 |
| `disconnect(interface)` | 断开连接。 | 预留，当前 UART/USB 未填 |
| `is_connected(interface)` | 查询连接状态。 | 预留，当前 UART/USB 未填 |
| `caps(interface)` | 返回 `AresInterfaceCaps` 能力位。 | UART、USB bulk |
| `mtu(interface)` | 返回接口建议帧长上限。 | UART、USB bulk |
| `alloc_buf(interface)` | 分配发送缓冲。 | UART、USB bulk |
| `alloc_buf_with_data(interface, data, size)` | 分配并携带现有数据块的发送缓冲。 | USB bulk |
| `init(interface)` | 初始化接口对象。 | UART、USB bulk |

`struct AresInterface` 包含名称、API、绑定的协议和传输私有数据。

### UART 接口

头文件：`include/ares/interface/uart/uart.h`

| 接口 | 说明 |
| --- | --- |
| `ares_uart_init(interface)` | 初始化 UART 接口。 |
| `ares_uart_send(interface, buf)` | 发送 `net_buf`。 |
| `ares_uart_send_with_callback(interface, buf, cb, user_data)` | 发送 `net_buf` 并在 UART TX 完成或失败时通知调用者。 |
| `ares_uart_send_raw(interface, data, len)` | 发送原始字节。 |
| `ares_uart_interface_alloc_buf(interface)` | 分配 UART 发送缓冲。 |
| `ares_uart_caps(interface)` | 返回 UART 接口能力位。 |
| `ares_uart_mtu(interface)` | 返回 UART 建议帧长上限。 |
| `ares_uart_init_dev(interface, uart_dev)` | 绑定 Zephyr UART 设备。 |
| `ARES_UART_INTERFACE_DEFINE(name)` | 定义 UART 接口实例。 |

### USB 批量传输接口

头文件：`include/ares/interface/usb/usb_bulk.h`

| 接口 | 说明 |
| --- | --- |
| `ares_usbd_init(interface)` | 初始化 USB bulk 接口。 |
| `ares_usbd_write(interface, buf)` | 发送 `net_buf`。 |
| `ares_usbd_write_with_callback(interface, buf, cb, user_data)` | 发送 `net_buf` 并在 USB IN 传输完成或失败时通知调用者。 |
| `ares_interface_alloc_buf(interface)` | 分配发送缓冲。 |
| `ares_interface_alloc_buf_with_data(interface, data, len)` | 分配并携带现有数据块的发送缓冲。 |
| `ares_usbd_caps(interface)` | 返回 USB bulk 接口能力位。 |
| `ares_usbd_mtu(interface)` | 返回当前 USB bulk 端点最大包长，未初始化时返回 64。 |
| `ARES_BULK_INTERFACE_DEFINE(name)` | 定义 USB bulk 接口实例。 |

### 协议层

头文件：`include/ares/protocol/ares_protocol.h`

`struct AresProtocolAPI` 是协议层回调表：

| 回调 | 说明 |
| --- | --- |
| `handle(protocol, buf)` | 处理完整缓冲。 |
| `handle_byte(protocol, byte)` | 按字节推进解析。 |
| `event(protocol, event)` | 处理连接事件。 |
| `init(protocol)` | 初始化协议。 |

### 双路协议（Dual Protocol）

头文件：`include/ares/protocol/dual/dual_protocol.h`

| 接口 | 说明 |
| --- | --- |
| `dual_ret_cb_set(protocol, cb)` | 设置函数返回回调。 |
| `dual_func_add(protocol, header, cb)` | 注册远程可调用函数。 |
| `dual_func_remove(protocol, header)` | 移除远程函数。 |
| `dual_func_call(protocol, id, arg1, arg2, arg3)` | 调用远端函数。 |
| `dual_sync_add(protocol, ID, buf, len, cb)` | 注册同步数据块。 |
| `dual_sync_flush(protocol, pack)` | 发送同步数据块。 |
| `DUAL_PROPOSE_PROTOCOL_DEFINE(name)` | 定义 Dual Protocol 实例。 |

Dual Protocol 支持函数帧、同步帧、错误帧与回复帧。协议常量定义在同一头文件中。

### MQTT-like 协议

头文件：`include/ares/protocol/mqttlite/mqttlite_protocol.h`

| 接口 | 说明 |
| --- | --- |
| `ares_mqttlite_register_topic(protocol, topic_id, topic)` | 注册本地 topic id 映射。 |
| `ares_mqttlite_subscribe(protocol, topic_filter, cb, user_data)` | 注册本地 topic 订阅回调。 |
| `ares_mqttlite_subscribe_id(protocol, topic_id, cb, user_data)` | 按已约定 topic id 注册本地订阅回调。 |
| `ares_mqttlite_unsubscribe(protocol, topic_filter)` | 移除本地 topic 订阅。 |
| `ares_mqttlite_unsubscribe_id(protocol, topic_id)` | 移除本地 topic id 订阅。 |
| `ares_mqttlite_publish(protocol, topic, payload, payload_len, qos, cb, user_data)` | 发布消息，QoS1/QoS2 成功时返回 packet id。 |
| `ares_mqttlite_publish_id(protocol, topic_id, payload, payload_len, qos, cb, user_data)` | 使用 2 字节 topic id 发布消息。 |
| `ares_mqttlite_publish_prepare_id(protocol, topic_id, payload_len, pub)` | 准备 QoS0 topic-id 零拷贝发布缓冲。 |
| `ares_mqttlite_publish_commit(protocol, pub)` | 发送已填充的零拷贝发布缓冲。 |
| `ares_mqttlite_publish_abort(pub)` | 释放未提交的零拷贝发布缓冲。 |
| `ares_mqttlite_ping(protocol)` | 发送 PING 帧。 |
| `ARES_MQTTLITE_PROTOCOL_DEFINE(name)` | 定义 MQTT-like 协议实例。 |

协议支持 `ARES_MQTTLITE_QOS0`、`ARES_MQTTLITE_QOS1` 与 `ARES_MQTTLITE_QOS2`。QoS1 使用
`PUBLISH/PUBACK`，QoS2 使用 `PUBLISH/PUBREC/PUBREL/PUBCOMP`。payload 是变长字段，长度由每帧
header 携带，并受 `CONFIG_ARES_MQTTLITE_MAX_PAYLOAD_SIZE` 限制。

### Plotter 协议

头文件：`include/ares/protocol/plotter/aresplot_protocol.h`

| 接口 | 说明 |
| --- | --- |
| `plotter_add_variable(protocol, ptr, type, name)` | 添加可监控变量。 |
| `plotter_remove_variable(protocol, ptr)` | 移除变量。 |
| `plotter_set_sample_rate(protocol, period_ms)` | 设置采样周期。 |
| `aresplot_report_error(protocol, error_code, message, msg_len)` | 启用错误报告时发送错误。 |
| `PLOTTER_PROTOCOL_DEFINE(name)` | 定义 Plotter 协议实例。 |

变量类型由 `aresplot_original_type_t` 描述。

### USB 传输旧接口（Legacy）

头文件：`include/ares/usb_bulk_trans/usb_trans.h`

| 接口 | 说明 |
| --- | --- |
| `ares_usb_transfer_init()` | 初始化旧版 USB transfer。 |
| `usb_trans_func_add(header, cb)` | 注册远程函数。 |
| `usb_trans_func_remove(header)` | 移除远程函数。 |
| `usb_trans_sync_add(data, ID, len, cb)` | 注册同步数据块。 |
| `usb_trans_sync_flush(pack)` | 发送同步数据块。 |
| `usb_trans_call_func(ID, arg1, arg2, arg3, cb)` | 调用远端函数。 |

新代码优先使用 ARES 接口与协议抽象。

### Interboard

头文件：`include/ares/interboard/interboard.h`

| 接口 | 说明 |
| --- | --- |
| `calculate_crc16(data, length)` | 计算帧 CRC16。 |
| `build_txbuf(tx_buffer, msg_type, data, is_retransmit)` | 构造发送帧。 |
| `process_rxbuf(rx_buffer, msg_type, data, is_retransmit)` | 解析接收帧。 |
| `interboard_init(spi_dev)` | 初始化板间 SPI 通信。 |
| `interboard_transceive(tx_data, rx_data)` | 同步收发一帧。 |

工作处理与回调由模块内部注册使用，不建议应用直接调用。

### VOFA JustFloat

头文件：`include/ares/vofa/justfloat.h`

| 接口 | 说明 |
| --- | --- |
| `jf_send_init(uart_dev, delay)` | 初始化 justfloat 输出。 |
| `jf_channel_add(data, value, type)` | 添加输出通道。 |

## ARES 算法 API

### EKF

头文件：

- `include/ares/ekf/QuaternionEKF.h`
- `include/ares/ekf/kalman_filter.h`
- `include/ares/ekf/algorithm.h`
- `include/ares/ekf/imu_task.h`
- `include/ares/ekf/matrix_storage.h`

| 接口 | 说明 |
| --- | --- |
| `IMU_QuaternionEKF_Predict_Update(gx, gy, gz, gyro_dt)` | 推进四元数 EKF。 |
| `CalcBias(q, accel, g, bias)` | 根据姿态和加速度估计偏置。 |
| `Kalman_Filter_Init(kf, xhatSize, uSize, zSize)` | 初始化卡尔曼滤波器。 |
| `Kalman_Filter_Update(kf)` | 执行一次完整更新。 |
| `IMU_temp_read(dev)` | 读取 IMU 温度。 |
| `IMU_Sensor_set_update_cb(cb)` | 设置 IMU 更新回调。 |
| `IMU_Sensor_set_IMU_temp(temp)` | 写入 IMU 温度。 |
| `IMU_Sensor_trig_init(accel_dev, gyro_dev)` | 初始化传感器触发链路。 |
| `matrix_storage_exists()` | 查询标定矩阵是否存在。 |
| `matrix_storage_delete()` | 删除标定矩阵。 |

### Mahony

头文件：

- `include/ares/mahony/MahonyAHRS.h`
- `include/ares/mahony/algorithm.h`
- `include/ares/mahony/imu_task.h`
- `include/ares/mahony/matrix_storage.h`

| 接口 | 说明 |
| --- | --- |
| `MahonyAHRSupdateGyro(gx, gy, gz, gyro_dt)` | 仅用陀螺仪推进 Mahony 姿态。 |
| `IMU_temp_read(dev)` | 读取 IMU 温度。 |
| `IMU_Sensor_set_update_cb(cb)` | 设置 IMU 更新回调。 |
| `IMU_Sensor_set_IMU_temp(temp)` | 写入 IMU 温度。 |
| `matrix_storage_exists()` | 查询标定矩阵是否存在。 |
| `matrix_storage_delete()` | 删除标定矩阵。 |

### 数学工具

`include/ares/ekf/algorithm.h` 与 `include/ares/mahony/algorithm.h` 提供同名数学工具：

| 接口 | 说明 |
| --- | --- |
| `MatInit(m, row, col)` | 初始化矩阵结构。 |
| `zmalloc(size)` | 分配并清零内存。 |
| `Sqrt(x)` | 平方根。 |
| `abs_limit(num, Limit)` | 对称限幅。 |
| `sign(value)` | 符号函数。 |
| `float_deadband(Value, minValue, maxValue)` | 死区处理。 |
| `float_constrain(Value, minValue, maxValue)` | 浮点限幅。 |
| `int16_constrain(Value, minValue, maxValue)` | `int16_t` 限幅。 |
| `loop_float_constrain(Input, minValue, maxValue)` | 环形区间约束。 |
| `theta_format(Ang)` | 角度格式化。 |
| `float_rounding(raw)` | 浮点取整。 |
| `Norm3d(v)` | 三维向量归一化。 |
| `NormOf3d(v)` | 三维向量模长。 |
| `Cross3d(v1, v2, res)` | 三维叉积。 |
| `Dot3d(v1, v2)` | 三维点积。 |
| `AverageFilter(new_data, buf, len)` | 均值滤波。 |
| `GetGroundAccel(q, accel, g, ground_accel)` | 计算地面坐标加速度。 |
| `quaternionToYawPitchRoll(q, yaw, pitch, roll)` | 四元数转欧拉角。 |

## VCAN 内部 API

头文件：`drivers/vcan/spi_can_mfd.h`

| 接口 | 说明 |
| --- | --- |
| `spi_can_mfd_set_bitrate(parent, channel, bitrate)` | 设置远端通道比特率。 |
| `spi_can_mfd_get_core_clock(parent, rate)` | 读取远端 CAN 核心时钟。 |
| `spi_can_mfd_send(parent, channel, frame, timeout, callback, user_data)` | 通过 SPI 桥发送 CAN 帧。 |
| `spi_can_node_handle_rx_frame(dev, frame)` | 父设备向子 CAN node 分发 RX 帧。 |

这些接口仅在 VCAN 父子驱动间使用。
