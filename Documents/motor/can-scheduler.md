# 电机 CAN 调度器

`motor_can_sched` 是 motor 子系统的公共 CAN 发送入口。它解决的不是“如何发一帧”，而是“多种 motor 协议同时运行时，如何维持时序、公平性和回复跟踪”。

调度器不解析厂商协议，也不决定电机是否在线。驱动把待发送的 CAN 帧和少量发送元数据交给调度器，调度器负责排队、发送、周期调度和部分统计。

## 设计目标

调度器主要服务四类负载：

- 必须尽快发出的控制帧。
- 周期性重复的目标帧。
- 发送后需要等待回复的请求帧。
- 可以延后的低优先级后台帧。

在同一条总线上，这些负载会彼此竞争。没有公共调度时，单个驱动的高频请求容易压住其它驱动的控制流。`tests/native_sim/motor_driver_sim` 中的 `test_dji_control_not_starved_by_reply_backlog` 就是围绕这个问题建立的。

## 总线注册

每个参与调度的 CAN 设备先调用：

```c
int motor_can_sched_register_can(const struct device *can_dev);
```

注册后，调度器为该总线维护独立的队列、周期表、待回复表和统计窗口。未注册总线上的发送请求应视为配置错误。

驱动初始化时应尽早注册总线；多个设备共享同一 CAN 控制器时，只需要注册同一个 `can_dev`。

## 优先级

调度器提供四级优先级：

| 优先级 | 用途 |
| --- | --- |
| `CRITICAL` | 强实时控制，不能被普通流量阻塞 |
| `HIGH` | 使能、失能、模式切换、清故障、主动探测 |
| `NORMAL` | 常规控制帧、周期目标更新 |
| `LOW` | 后台维护和可延迟负载 |

维护时应保持以下原则：

- 使能、失能、清故障和主动 PING 一类帧通常使用 `HIGH`。
- 周期控制帧默认使用 `NORMAL`。
- 不要把所有帧都提升到高优先级，否则调度器失去意义。

单帧发送应使用 `motor_can_sched_send_with_priority()` 明确声明优先级；需要周期帧或回复跟踪时，使用通用 `motor_can_sched_send()`。

## 发送模型

### 单次发送

适用于一次性控制或命令帧：

```c
int motor_can_sched_send_with_priority(const struct device *can_dev,
				       const struct can_frame *frame,
				       enum motor_can_sched_prio priority,
				       const char *tag);
```

`tag` 主要用于统计和日志定位。

### 周期发送

部分驱动不是“每次 `motor_set()` 立刻发一帧”，而是缓存目标，由自己的工作线程或 timer 按固定频率投递控制帧。若需要由调度器维护周期项，可使用：

- `motor_can_sched_send()` 创建周期项。
- `motor_can_sched_update()` 更新同一周期项的帧内容。
- `motor_can_sched_remove()` 删除周期项。

调度器维护的是发送节奏，不负责生成帧内容。驱动仍然要自己维护“最新目标”。

### 请求-回复发送

请求-回复型协议通过：

```c
int motor_can_sched_send_reply(const struct device *can_dev,
			       const struct can_frame *frame,
			       uint32_t reply_id,
			       uint32_t reply_mask,
			       uint16_t timeout_ms,
			       const char *tag);
```

这一路径会在待回复表中登记一次预期回复，并在超时、重试和放弃时累积统计。它本身不替代驱动的离线逻辑；驱动仍需在超时后调用 `motor_link_note_missed_reply()` 或等价逻辑。

## RX 回报

驱动收到 CAN 帧后，应尽快调用：

```c
void motor_can_sched_report_rx(const struct device *can_dev,
			       const struct can_frame *frame);
```

这一步有三个作用：

- 给回复跟踪器提供匹配机会。
- 更新总线忙碌窗口统计。
- 为遥测层提供调度健康数据。

它不解析帧内容，也不设置电机在线状态。驱动应先后完成两件独立工作：

1. 把原始 RX 帧上报给调度器。
1. 按自身协议更新反馈和链路状态。

## 与各类驱动的关系

调度器面对两种典型驱动模型。

### 周期上报型

DJI 电机自行周期上报反馈，控制帧通常由主控聚合发送。此类驱动更多依赖调度器做发送公平性和队列管理，而不是回复跟踪。

### 请求-回复型

DM、MI、RS 和部分 LK 路径由主控发请求、电机回反馈。此类驱动普遍依赖 `motor_can_sched_send_reply()` 维持发送节奏，并把回复超时交给 `motor_link` 决定是否离线。

VESC 处于中间状态：控制帧是单向的，在线探测使用 `PING/PONG` 和状态帧共同维持。

## 统计与健康信号

调度器维护每条总线的运行统计，至少包括：

- `tx_frames`
- `rx_frames`
- `ack_matches`
- `dropped_frames`
- `retry_frames`
- `ack_timeouts`
- `pending_full`
- `tx_busy`
- `giveups`
- 各优先级队列峰值
- 忙碌窗口和发送延迟窗口

维护者通常不直接关心单次发送结果，而更关心趋势：

- `pending_full` 持续增长，说明回复路径产能不足或超时过长。
- `tx_busy` 持续增长，说明控制频率、总线负载或优先级设置失衡。
- `ack_timeouts` 集中出现在单个驱动，通常意味着 RX filter、reply ID/mask 或电机模式切换有问题。

## 遥测集成

`motor_telemetry_can_scheduler_health()` 负责把统计快照收敛为日志。默认策略是：

- 只有在 `tx_busy`、drop、ack timeout、pending full 或 giveup 增量出现时输出告警。
- `CONFIG_MOTOR_CAN_SCHED_TX_LATENCY_TRACE` 打开时输出发送延迟细节。

这使得调度器日志更像健康信号，而不是逐帧跟踪。

## 维护边界

修改调度器时，应避免越界到驱动策略：

- 不在调度器里硬编码某个电机协议的回复规则。
- 不在调度器里判定电机在线或离线。
- 不在调度器里保存控制目标语义。

调度器只知道“帧”和“发送元数据”。

## 调整时序时的检查项

修改发送频率、超时或优先级时，至少检查：

- `tests/native_sim/motor_driver_sim` 中的发送频率窗口测试是否仍成立。
- DJI 控制帧是否会被请求-回复流量压住。
- 回复超时设置是否与驱动自己的离线阈值一致。
- 同一总线上的不同协议是否仍有公平性。

如果问题表现为偶发离线，而 RX 实际到达，先查回复匹配规则和 filter，再查调度器本身。
