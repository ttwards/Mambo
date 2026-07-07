# Mambo 模块参考

Mambo 是一个 Zephyr 模块。仓库按驱动、库、板级定义、样例和测试分层，应用通过
Zephyr 设备模型、devicetree 与 Kconfig 组合这些模块。

## 顶层目录

| 路径 | 职责 |
| --- | --- |
| `boards/` | 自定义板级定义，包括 DTS、defconfig、runner 和板级初始化。 |
| `dts/bindings/` | 本仓库新增设备与控制器的 devicetree binding。 |
| `drivers/` | Zephyr 驱动实现，包括 motor、chassis、wheel、SBUS、VCAN、传感器和 PID。 |
| `include/zephyr/drivers/` | 面向应用的公共驱动 API。 |
| `include/ares/` | ARES 通信、协议、算法和调试库的公共入口。 |
| `lib/ares/` | ARES 库实现。 |
| `samples/` | 可独立构建的示例程序和上板验证程序。 |
| `tests/` | native_sim 和 ztest 测试。 |
| `.github/workflows/` | GitHub Actions 持续集成、PR 检查和发布构建。 |
| `Documents/` | 架构、接口、设备树和专项设计文档。 |
| `template/` | 新应用模板。 |

## 板级目标

`boards/` 提供 Mambo 支持的板级定义。

| 板级 | 路径 | 说明 |
| --- | --- | --- |
| `dm_mc02` | `boards/damiao/dm_mc02` | 达妙控制板。常用于 DM 电机、VCAN 和机器人下位机应用。 |
| `robomaster_board_a` | `boards/dji/robomaster_board_a` | DJI RoboMaster A 板定义。 |
| `robomaster_board_c` | `boards/dji/robomaster_board_c` | DJI RoboMaster C 板定义。 |

板级代码仅处理硬件启动、供电、LED、时钟、runner 等板子固有职责。业务设备应通过
应用层 overlay 声明，不应直接写入板级 DTS，除非它是板载固定硬件。

## 驱动子系统

### 电机（Motor）

路径：

- `include/zephyr/drivers/motor.h`
- `include/zephyr/drivers/motor/controller.h`
- `drivers/motor/`
- `dts/bindings/motor/`
- `dts/bindings/motor-controller/`

Motor 是 Mambo 的核心执行器接口。它将不同厂商的电机统一成 `motor_set()`、`motor_get()`
和 `motor_control()` 三个应用入口，并在驱动内部复用 controller、link、CAN 调度器
与遥测。

当前支持驱动：

| 驱动 | compatible | 通信 | 反馈模型 |
| --- | --- | --- | --- |
| DJI | `dji,motor` | CAN standard ID | 电机定时上报。 |
| DM | `dm,motor` | CAN standard ID | 主控发送控制帧后电机回复。 |
| MI | `mi,motor` | CAN extended ID | 主控请求与电机反馈。 |
| RS | `rs,motor` | CAN extended ID | 支持请求反馈和可选主动上报。 |
| LK | `lk,motor` | CAN standard ID | 主控发送控制或参数帧后电机回复。 |
| VESC | `vesc,motor` | CAN extended ID | 状态帧与 PING/PONG 共同维持在线状态。 |
| Yinshi LA | `yinshi,la` | UART/RS485 | 线性执行器接口，不挂在旋转电机 API 下。 |

Motor 子系统的公共结构见 [Motor 架构](motor/architecture.md)。

### 底盘（Chassis）

路径：

- `include/zephyr/drivers/chassis.h`
- `drivers/chassis/`
- `dts/bindings/chassis/ares,chassis.yaml`

底盘接收线速度、角速度和姿态目标，并按配置的 wheel 设备拆分为各轮目标。
底盘本身不直接驱动 CAN 或电机协议，只调用 wheel API。

### 轮组（Wheel）

路径：

- `include/zephyr/drivers/wheel.h`
- `drivers/wheel/`
- `dts/bindings/steerwheel/`

Wheel 将单个轮组抽象为速度、角度和静态锁定状态。`ares,mecanum` 表示无转向电机麦轮，
`ares,steerwheel` 表示舵轮。

### SBUS

路径：

- `include/zephyr/drivers/sbus.h`
- `drivers/transfer/ares_sbus.c`
- `dts/bindings/transfer/ares,sbus.yaml`

SBUS 提供遥控器通道读取接口。模拟通道返回归一化百分比，数字通道返回离散档位。

### PID

路径：

- `include/zephyr/drivers/pid.h`
- `drivers/pid/`
- `dts/bindings/pid/`

PID 是旧版控制器接口，仍供 chassis 和部分样例使用。Motor 子系统新的 controller
使用 `motor-controller,*` 兼容节点与 `motor_controller_*()` 接口。

### VCAN

路径：

- `drivers/vcan/`
- `dts/bindings/can/custom,spi-can-mfd.yaml`
- `dts/bindings/can/custom,spi-can-node.yaml`

VCAN 在主机板上暴露标准 Zephyr CAN 设备，实际通过 SPI 访问远端物理 CAN。上层仍使用
`can_send()`、`can_add_rx_filter()` 等 Zephyr CAN API。

### 传感器（Sensor）

路径：

- `drivers/sensor/WIT_HWT906/`
- `drivers/sensor/paw3395/`
- `dts/bindings/sensor/`
- `dts/bindings/imu/`

传感器模块提供 IMU、光流或运动传感器驱动。驱动应尽量遵循 Zephyr 传感器 API，项目特定
标定数据放在对应 binding 中。

### 图形（Graph）

路径：

- `drivers/graph/`

Graph 当前承载 INSLink 相关代码。该目录并非通用图形库，维护时应按通信或姿态子系统分类，
不应在此继续扩展无关功能。

## ARES 库

### 接口层（Interface）

路径：

- `include/ares/interface/`
- `lib/ares/interface/`

接口层是传输层抽象。UART、USB bulk 等具体传输实现提供 `AresInterfaceAPI`，协议
只依赖抽象接口完成发送与缓冲分配。

### 协议层（Protocol）

路径：

- `include/ares/protocol/`
- `lib/ares/protocol/`

协议层是帧解析和业务分发层，目前包含 Dual Protocol、MQTT-like Protocol 与 Plotter Protocol。

### ARES 通信（ARES Comm）

路径：

- `include/ares/ares_comm.h`
- `lib/ares/`

`ares_bind_interface()` 将接口与协议绑定并按顺序初始化。一个接口对象同一时刻只能绑定
一个协议。

### IMU 算法

路径：

- `include/ares/ekf/`
- `include/ares/mahony/`
- `lib/ares/ekf/`
- `lib/ares/mahony/`

提供姿态解算、矩阵工具、Kalman filter、Mahony AHRS 与 IMU 任务辅助函数。EKF 与 Mahony
目录保留一致的工具接口，应用应选择其一作为姿态链路，避免同一任务内混用状态。

### 板间通信（Interboard）

路径：

- `include/ares/interboard/`
- `lib/ares/interboard/`

Interboard 是板间 SPI 帧传输模块，提供固定帧结构、CRC、收发工作线程与同步传输接口。

### VOFA

路径：

- `include/ares/vofa/`
- `lib/ares/vofa/`

VOFA justfloat 用于轻量级调试输出。适合在线观察少量变量，不作为稳定的上位机协议使用。

## 样例（Samples）

样例是可构建的参考应用，不是测试套件。样例应明确标注硬件依赖，避免隐式依赖开发者本地环境。

| 路径 | 说明 |
| --- | --- |
| `samples/motor/*` | 电机驱动和控制器示例。 |
| `samples/chassis` | 底盘与轮组（wheel）组合示例。 |
| `samples/communication/*` | ARES 通信和 plotter 示例。 |
| `samples/vcan/*` | VCAN 主机/从机示例。 |
| `samples/IMU/*` | IMU 姿态和标定示例。 |
| `samples/wit_hwt906_test` | WIT HWT906 传感器验证。 |

## 测试

| 路径 | 说明 |
| --- | --- |
| `tests/native_sim/module_smoke` | 模块加载与基础构建冒烟测试。 |
| `tests/native_sim/pid` | PID 控制器行为测试。 |
| `tests/native_sim/sbus` | SBUS 通道解析与离线清零行为测试。 |
| `tests/native_sim/motor_driver_sim` | motor 驱动、CAN 调度与状态机仿真测试。 |

Native sim 用于验证驱动逻辑、打包、调度与状态转换。它不能替代实车性能测试，也不对真实 CAN 电气层做保证。

## 持续集成

CI 定义位于 `.github/workflows/`：

| 工作流 | 说明 |
| --- | --- |
| `ci.yml` | 格式检查、native_sim 测试、样例构建与静态分析。 |
| `pr-check.yml` | PR 标题、变更规模、SPDX 和关键文件提醒。 |
| `release.yml` | tag、release 或手动触发的固件发布构建。 |

CI 细节见 [测试与 CI](tests/README.md)，native_sim 细节见 [native_sim](tests/native_sim.md)。
