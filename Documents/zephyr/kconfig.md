# Kconfig

Mambo 的 Kconfig 入口位于仓库根目录 `Kconfig`。Zephyr 将其作为模块配置入口加载，应用通过 `prj.conf` 或 board defconfig 选择需要的驱动和库。

## 入口关系

```text
Kconfig
├── drivers/Kconfig
│   ├── drivers/motor/Kconfig
│   ├── drivers/transfer/Kconfig
│   ├── drivers/chassis/Kconfig
│   ├── drivers/wheel/Kconfig
│   ├── drivers/pid/Kconfig
│   ├── drivers/sensor/Kconfig
│   └── drivers/vcan/Kconfig
└── lib/Kconfig
    └── lib/ares/Kconfig
        ├── lib/ares/interface/Kconfig
        └── lib/ares/protocol/Kconfig
```

`drivers/Kconfig` 只注册驱动菜单。`lib/Kconfig` 注册 ARES 库菜单。模块级选项不应放在应用 `prj.conf` 旁边定义；应用只选择已有选项。

## 驱动选项

主要驱动选项：

| 选项 | 说明 |
| --- | --- |
| `CONFIG_MOTOR` | 启用 motor 驱动菜单。 |
| `CONFIG_MOTOR_COMMON` | 启用 motor 公共层和 CAN 调度器。 |
| `CONFIG_MOTOR_DJI` | 启用 DJI 电机。 |
| `CONFIG_MOTOR_DM` | 启用 DM 电机。 |
| `CONFIG_MOTOR_MI` | 启用 MI 电机。 |
| `CONFIG_MOTOR_RS` | 启用 Robstride 电机。 |
| `CONFIG_MOTOR_LK` | 启用 LK 电机。 |
| `CONFIG_MOTOR_VESC` | 启用 VESC 电机。 |
| `CONFIG_LA_YINSHI` | 启用 Yinshi 线性执行器。 |
| `CONFIG_MOTOR_TELEMETRY` | 启用 motor 在线状态和 CAN 调度器遥测。 |
| `CONFIG_MOTOR_LOG_LEVEL` | motor 子系统日志等级。 |
| `CONFIG_MOTOR_INIT_PRIORITY` | motor device 初始化优先级。 |
| `CONFIG_CAN_COUNT` | motor 代码中预期的 CAN 设备数量。 |

Motor 子系统的调度和节拍选项：

| 选项 | 默认 | 说明 |
| --- | --- | --- |
| `CONFIG_MOTOR_CAN_SCHED_REPLY_GUARD` | `y` | 请求回复帧发送后保留回复窗口。 |
| `CONFIG_MOTOR_CAN_SCHED_REPLY_GUARD_EXTRA_US` | `60` | 回复窗口额外保护时间。 |
| `CONFIG_MOTOR_TELEMETRY_CAN_LOG_INTERVAL_MS` | `2000` | CAN 调度器压力日志最小间隔。 |
| `CONFIG_MOTOR_DM_DEFAULT_FREQ_HZ` | `200` | DM 请求控制频率。 |
| `CONFIG_MOTOR_VESC_CONTROL_FREQ_HZ` | `100` | VESC 最新目标周期发送频率。 |

## ARES 选项

主要库选项：

| 选项 | 说明 |
| --- | --- |
| `CONFIG_ARES` | 启用 ARES 库菜单。 |
| `CONFIG_UART_INTERFACE` | 启用 UART 接口。 |
| `CONFIG_USB_BULK_INTERFACE` | 启用 USB Bulk 接口。 |
| `CONFIG_ARES_MQTTLITE_PROTOCOL` | 启用 MQTT-like 发布/订阅协议。 |
| `CONFIG_ARES_MQTTLITE_MAX_TOPICS` | 设置 MQTT-like topic id 注册数量上限。 |
| `CONFIG_ARES_BOARD_STATUS_LED` | 启用板级状态 LED 服务。 |
| `CONFIG_IMU_PWM_TEMP_CTRL` | 启用 IMU PWM 温控。 |
| `CONFIG_AUTO_PROBE_GYRO_BIAS` | 启用陀螺偏置自动探测。 |
| `CONFIG_EKF_LIB` | 启用 EKF 姿态库。 |
| `CONFIG_MAHONY_LIB` | 启用 Mahony 姿态库。 |
| `CONFIG_VOFA_LIB` | 启用 VOFA justfloat。 |
| `CONFIG_ARES_COMM_LIB` | 启用 ARES 通信抽象。 |

interface 与 protocol 子目录还定义各自的缓冲区、线程、端点和协议参数。应用应只启用实际使用的库，避免无关线程和缓冲占用 RAM。

## 应用配置

应用通过 `prj.conf` 选择功能：

```conf
CONFIG_CAN=y

CONFIG_MOTOR=y
CONFIG_MOTOR_COMMON=y
CONFIG_MOTOR_DJI=y
CONFIG_MOTOR_DM=y
CONFIG_MOTOR_LOG_LEVEL=3
CONFIG_MOTOR_INIT_PRIORITY=90

CONFIG_ZTEST=y
```

board defconfig 仅放置板级固定能力。和某个应用有关的外设、驱动与调试选项应放在应用 `prj.conf`。

## 维护规则

新增 Kconfig 选项时应遵循以下规则：

- 选项名带模块前缀，例如 `MOTOR_`、`ARES_`、`VCAN_`。
- 默认值应保守，不能让未使用模块产生额外线程或硬件访问。
- 依赖关系写在 Kconfig 中，不依赖应用按顺序手动启用。
- help 文本描述行为与单位，不记录特定改动背景。
- 新选项影响 devicetree 或公共 API 时，同步更新 `Documents/devicetree-reference.md` 或 `Documents/api-reference.md`。
