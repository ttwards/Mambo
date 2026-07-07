# Communication 样例

通信样例分为 `ARES 通信`、`MQTT-like USB` 与 `Plotter 协议` 几条线，覆盖 USB Bulk、UART
接口和日志输出行为。

## 共同边界

- `CONFIG_UART_INTERFACE` 与 `CONFIG_USB_BULK_INTERFACE` 在各样例中的组合不同，须按 `boards/*` 与 `prj.conf` 一起确认。
- 示例流程以 `CONFIG_DUAL_PROPOSE_PROTOCOL`、`CONFIG_ARES_MQTTLITE_PROTOCOL` 或 `CONFIG_PLOTTER` 启用为入口，接口初始化失败时通常会跳过失败通道并继续运行。
- 上位机联调前先确认 `usart6` 与 USB OTG 物理连接。

## samples/communication/ares_communication

- 用途：双接口 ARES 通信演示（USB Bulk + UART），含回调与定时发送逻辑。
- 构建：
  - `west build -b robomaster_board_c samples/communication/ares_communication`
  - `west flash`
- 适用 board：`robomaster_board_c`
- 特殊约束：
  - `samples/communication/ares_communication/boards/dm_mc02.conf` 将 `CONFIG_UART_INTERFACE=n`，在该板上为 USB-only。
- 硬件依赖：
  - USB 设备能力（`samples/communication/ares_communication/README.md` 入口说明）
  - `usart6`（UART 接口）
- 维护规则：
  - 若新增板适配，先确认 `boards/<board>.conf` 是否覆盖了 `CONFIG_UART_INTERFACE`。
  - 回调函数语义（`func_cb` / `sync_cb` / `func_ret_cb`）变化需同步示例文档与接收端联调脚本。

## samples/communication/mqttlite_usb

- 用途：MQTT-like 协议的 USB Bulk 发布/订阅演示，周期通过 topic-id 零拷贝路径发布 QoS0 heartbeat，并按 id 订阅 `host/command`。
- 构建：
  - `west build -b dm_mc02 samples/communication/mqttlite_usb --pristine`
  - `west flash`
- 适用 board：`dm_mc02`
- 硬件依赖：
  - USB 设备能力
  - 兼容 MQTT-like 帧格式的上位机
- 维护规则：
  - QoS 行为变化需同步 `include/ares/protocol/mqttlite/mqttlite_protocol.h` 和协议文档。
  - topic 名称变化需同步上位机接收端。

## samples/communication/plotter_auto

- 用途：自动上报变量的 Plotter 示例。
- 构建：
  - `west build -b robomaster_board_c samples/communication/plotter_auto`
  - `west flash`
- 适用 board：`robomaster_board_c`
- 硬件依赖：
  - UART 通路（日志与协议运行通道）
- 维护规则：
  - 当前采样频率由 `CONFIG_ARESPLOT_FREQ` 与示例逻辑共同影响。
  - 变更采样时序应同步回归说明，避免影响 CI 观测。

## samples/communication/plotter_demo

- 用途：手动配置上报变量的 Plotter 示例，可用于变量级联调。
- 构建：
  - `west build -b robomaster_board_c samples/communication/plotter_demo`
  - `west flash`
- 适用 board：`robomaster_board_c`
- 硬件依赖：
  - `usart6`（当前默认）
  - 兼容的上位机 Plotter 接收端
- 维护规则：
  - 变量名（如 `sine_wave`、`counter`）变更必须同步上位机侧脚本与文档。
  - 配置变更以 `prj.conf` 为准，不在运行代码中临时追加隐式参数。

## 维护建议（通信共用）

- 样例行为以 `prj.conf` + `boards/<board>.conf` + overlay 的组合为准。
- 联调前优先确认 USB 与 UART 的方向与波特率配置，避免通道混用导致误判。
- 如引入新协议变体，补充 `sample.yaml` 并在本文件同步适用 board 与验证入口。
