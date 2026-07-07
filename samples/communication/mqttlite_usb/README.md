# ARES MQTT-like USB Bulk 示例

这个示例演示如何把 MQTT-like 协议绑定到 USB bulk 接口上。

## 功能

- 使用 `ARES_BULK_INTERFACE_DEFINE()` 定义 USB bulk 接口。
- 使用 `ARES_MQTTLITE_PROTOCOL_DEFINE()` 定义协议实例。
- 注册 `robot/mqttlite/heartbeat` 与 `host/command` 的 topic id。
- 按 id 订阅 `host/command`。
- 周期通过 QoS0 zero-copy 路径发布 `robot/mqttlite/heartbeat`。

## 构建

```sh
west build -b dm_mc02 samples/communication/mqttlite_usb --pristine
```

## 上位机约定

设备会发送带 topic-id 标志的 MQTT-like `PUBLISH` 帧，heartbeat 的 topic id 为 `1`，
`host/command` 的 topic id 为 `2`。当前示例 heartbeat 使用 QoS0，不等待上位机确认。
