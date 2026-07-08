# native_sim 测试体系

`native_sim` 用于在主机平台验证驱动与控制逻辑，目标是将可复现的时序和协议行为问题尽早固化。本文档覆盖测试范围、运行方式、fake CAN 行为模型、状态机断言、打包与顺序、延迟阈值和边界。

## 测试范围与边界

执行目录：`tests/native_sim/*`

包括但不限于：

- 模块构建入口。
- PID 计算行为。
- 电机驱动的上报/控制链路行为。
- CAN 帧打包、发送顺序、速率和延迟阈值。
- 基础状态机恢复路径。

以下内容不由 native_sim 覆盖：

- 真实总线占用率与竞争。
- CAN 收发器误码、仲裁和总线错误恢复。
- MCU 中断抖动、真实调度器时延、DMA/外设时序。
- 上电、供电、热漂、EMC 等硬件条件下的行为。

## 目录与入口

- `tests/native_sim/module_smoke/`
- `tests/native_sim/pid/`
- `tests/native_sim/sbus/`
- `tests/native_sim/motor_driver_sim/`
- `tests/native_sim/wheel_chassis/`

### 常用命令

全量运行：

```shell
west twister -T tests/native_sim --platform native_sim/native/64 --inline-logs
```

单例运行：

```shell
west twister -T tests/native_sim/module_smoke --platform native_sim/native/64 --inline-logs
west twister -T tests/native_sim/pid --platform native_sim/native/64 --inline-logs
west twister -T tests/native_sim/sbus --platform native_sim/native/64 --inline-logs
west twister -T tests/native_sim/motor_driver_sim --platform native_sim/native/64 --inline-logs
west twister -T tests/native_sim/wheel_chassis --platform native_sim/native/64 --inline-logs
```

单测试构建（快速检查）：

```shell
west build -b native_sim/native/64 tests/native_sim/pid --pristine
```

## module_smoke

测试路径：`tests/native_sim/module_smoke/src/main.c`

- 目标：确认测试模块能在 native_sim 板级环境中启动。
- 关键断言：`CONFIG_BOARD_NATIVE_SIM` 为真。
- testcase：

```text
ares.native_sim.module_smoke
```

- `prj.conf`：`CONFIG_ZTEST=y`
- `testcase.yaml`：tags 包含 `ares`、`native_sim`，平台固定 `native_sim/native/64`。

该用例为最小入口健康检查，主要用于发现 CMake、board 与最小依赖链路问题。

## pid

测试路径：`tests/native_sim/pid/src/main.c`

测试行为：

- 构造 `struct pid_config` 的 fake device，调用 `pid_calc_in` / `pid_calc_mit`。
- 覆盖输出限幅、积分限幅、零时间步、速度误差计算。
- 使用 `FLOAT_TOLERANCE` 比较结果，验证数学边界行为可复用。

测试项：

- `test_pid_calc_in_clamps_output`：`pid_calc_in` 的输出限幅与 `output_limit`/`output_offset` 生效。
- `test_pid_calc_in_clamps_integral`：积分项限幅生效并与速度误差配合计算。
- `test_pid_calc_in_ignores_zero_delta_time`：`delta_time == 0` 不应更新输出，返回 0.
- `test_pid_calc_mit_uses_velocity_error`：`pid_calc_mit` 使用速度误差路径。

`prj.conf`：

```conf
CONFIG_ZTEST=y
CONFIG_CBPRINTF_FP_SUPPORT=y
```

`testcase.yaml` 与 `module_smoke` 一致，平台限定 `native_sim/native/64`。

## sbus

测试路径：`tests/native_sim/sbus/src/main.c` 与 `app.overlay`

测试行为：

- 使用测试专用 UART 设备承载 `sbus,uart` chosen，覆盖 SBUS 驱动初始化路径。
- 注入一帧有效 SBUS 数据，验证在线通道解析与非法通道错误。
- 等待超过 500 ms，验证 `sbus_get_digit()` 返回 `-ENETDOWN`、百分比读取返回安全零值，并确认帧缓存与通道数组被清零。

testcase：

```text
ares.native_sim.sbus
```

`prj.conf` 启用 `CONFIG_ARES_SBUS`、UART async API 与 runtime configure，平台限定 `native_sim/native/64`。

## wheel_chassis

测试路径：`tests/native_sim/wheel_chassis/src/main.c` 与 `app.overlay`

测试行为：

- 使用测试专用 motor 设备承载 `ares,mecanum` 与 `ares,steerwheel`，验证轮级 API 到电机 setpoint 的转换。
- 覆盖麦克纳姆轮速度投影、状态读取和静态角锁存。
- 覆盖舵轮短路径反向驱动、零速停止与禁能扭矩清零。
- 使用测试专用 wheel 设备承载 `ares,chassis`，直接验证底盘公共命令映射、运动解析与静态角下发。

testcase：

```text
ares.native_sim.wheel_chassis
```

`prj.conf` 启用 `CONFIG_MOTOR`、`CONFIG_PID`、`CONFIG_WHEEL`、`CONFIG_CHASSIS` 与 `CONFIG_ZTEST`，平台允许 `native_sim` 与 `native_sim/native/64`。

## motor_driver_sim

测试路径：`tests/native_sim/motor_driver_sim/src/main.c` 与 `app.overlay`

### 覆盖范围

用例节点：

- `dm0`：`dm,motor`
- `mi0`：`mi,motor`
- `rs0`：`rs,motor`
- `lk0`：`lk,motor`
- `dji0`~`dji6`：`dji,motor`

驱动配置包含 `fake_can`，并通过 `fake_can` 通道复用 `can_channel`。

### fake CAN（测试总线）

`fake_can` 由 `compatible = "ares,test-can"` 提供，driver 在测试源码内部实现最小 API：

- `start`/`stop`
- `get_capabilities`/`set_mode`/`set_timing`
- `send`
- `add_rx_filter`
- `get_state`
- `get_core_clock`
- `get_max_filters`

`send` 行为：

- 按 `bitrate` 和经典 CAN 帧格式占用虚拟总线时间
- 标准帧 8 字节按约 111 bit 计算，扩展帧 8 字节按约 131 bit 计算
- 每帧记录到环形缓冲 `SIM_TX_MAX = 512`
- 记录 `can_frame`、`uptime`、`cycle`
- TX 记录时间为帧完成时间，tx 回调在虚拟帧发送完成后触发

`add_rx_filter` 与回调分发行为：

- 过滤注册按 `can_filter` 匹配
- `sim_emit_frame` 先占用虚拟总线时间，再遍历过滤器并触发回调
- 不模拟 bit stuffing、总线仲裁、mailbox 竞争、bus-off、误码状态

该模型适合协议与驱动逻辑验证，不适合硬件级收发行为还原。

### 状态机覆盖

测试集中了两类驱动模型：

- 请求回复型：DM / MI / RS / LK
- 周期上报型：DJI

#### 请求回复型（DM、MI、RS、LK）

流程关键点（`verify_request_reply_motor_drivers`）：

1. 初始 `DISABLE_MOTOR`。
2. `ENABLE_MOTOR` 后发送控制帧。
3. 未反馈时保持在线状态为 false。
4. 注入反馈后转为在线。
5. 停止反馈后等待离线超时：
   - DM：500ms
   - MI：6000ms
   - RS：2000ms
   - LK：3000ms
6. 再次注入反馈，期望恢复时间不超过 `ONLINE_RECOVERY_MS = 30`ms。

#### 周期上报型（DJI）

流程（`test_a_dji_periodic_report_driver`）：

1. 初始 `DISABLE_MOTOR`，在线为 false。
2. 注入上报帧后通过 `service_dji_tx_work` 触发驱动处理。
3. 驱动应在 `ONLINE_RECOVERY_MS = 30`ms 内恢复在线。
4. 注入窗口内不再上报可降为离线。
5. 再次上报后应快速恢复在线。

### 打包、顺序与延迟

#### 发送顺序和打包验证

测试：`test_continuous_command_packing_order_and_latency`

- DM/MI/RS：`motor_set_mit` 的连续 setpoint（10、20、30）；
- LK：`motor_set_speed` 的连续 setpoint（25、50、75）；
- 每次下发前/后都有 `emit_*_feedback` 的配套；
- LK 速度 setpoint 期望直接触发 `0xA2` 控制帧；8ms timer 平常重发当前命令，只有无可用
  控制目标时才 fallback 到 `0x9C` 状态读取。
- 使用期望帧匹配函数比较：
  - 帧 ID（标准/扩展）
  - DLC（固定 8）
  - payload 编码（MIT/速度映射）
- 要求：
  - 匹配顺序不回退（序号严格递增）
  - 延迟不超过阈值（见下）

#### 延迟阈值

- `CONTROL_LATENCY_MS = 50`
- `DJI_CONTROL_LATENCY_MS = 2`

延迟由下发时间戳到匹配控制帧到达时间计算。测试使用 `expect_payload_sequence_step` 在窗口内等待；超时给出完整帧转储用于诊断。

#### 发送速率窗口

测试：`test_control_send_rate_windows`

- 统计窗口长度：
  - DJI：300ms
  - DM：300ms
  - MI：300ms
  - RS：300ms
- 允许帧数范围：
  - DJI：8~100
  - DM：8~50
  - MI：6~80
  - RS：8~70

这是功能性阈值，限制明显过快、过慢或阻塞，不用于性能基准。

#### DJI 压测与公平性

`test_dji_control_not_starved_by_reply_backlog` 构造 DM backlog 后验证 DJI 控制帧仍能发送，确保请求回复流量不会永久压住周期/关键控制链路。

`test_dji_7_motors_on_one_can_keep_950hz_control` 将 7 个 DJI 电机挂在同一个
fake CAN 上，按 `0x200`/`0x1ff` 聚合帧槽位统计每个电机的控制更新频率，
要求每个电机不低于 950Hz。

## 配置与 testcase 约定

### `testcase.yaml`（当前约定）

每个 native_sim 用例目前固定：

```yaml
platform_allow:
  - native_sim/native/64
integration_platforms:
  - native_sim/native/64
```

### `prj.conf`（常见）

- 最小：`CONFIG_ZTEST=y`
- PID 额外：`CONFIG_CBPRINTF_FP_SUPPORT=y`
- motor 仿真：按驱动依赖打开 `CONFIG_CAN`、`CONFIG_MOTOR*` 等，且关闭环回 `CONFIG_CAN_LOOPBACK=n`

## 运行与排障

推荐复现序列：

1. 先单独构建：`west build -b native_sim/native/64 tests/native_sim/motor_driver_sim --pristine`
2. 运行同目录用例：`west twister ... --verbose`
3. 如需保留构建目录：追加 `--no-clean`

失败定位优先关注：

- 期望帧是否在窗口内匹配到；
- `sim_tx_history` 是否记录完整 `uptime_ms`；
- 反馈注入是否在正确路径触发；
- 状态机超时时间是否使用对应电机类型的离线阈值。

## 边界与约束

- 平台固定 `native_sim/native/64`，不对其他 board 平台构建。
- CAN 历史记录为环形缓存（`SIM_TX_MAX = 128`），可用于短时行为验证，不应作为长序列吞吐基准。
- 发送速率窗口采用功能阈值，不代表实车吞吐上限。
- 延迟阈值与超时值以 CI 环境稳定性为准，不等同实车控制实时约束。
