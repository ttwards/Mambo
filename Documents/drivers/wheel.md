# 轮子

## 概览

轮子子系统定义了底盘层消费的单轮抽象。它屏蔽了轮子具体是两驱动机构的可转向模块，还是单电机驱动的麦克纳姆轮两种实现差异。

当前拆分如下：

- `include/zephyr/drivers/wheel.h`
- `drivers/wheel/steerwheel.c`
- `drivers/wheel/mecanum.c`
- `dts/bindings/steerwheel/ares,steerwheel.yaml`
- `dts/bindings/steerwheel/ares,mecanum.yaml`

## 设备模型

公共 API 由 `struct wheel_driver_api` 定义的 Zephyr 子系统。

公共配置数据来源于 `wheel_cfg_t`：

- `angle_offset`
- `wheel_radius`

公共运行状态由 `wheel_status_t` 表示：

- `speed`
- `angle`
- `restricted`

当前提供的实现为：

- `ares,steerwheel`
- `ares,mecanum`

## 公共接口

定义于 `include/zephyr/drivers/wheel.h`：

- `wheel_set_speed(dev, speed, angle)`
- `wheel_set_static(dev, angle)`
- `wheel_get_speed(dev)`
- `wheel_get_target(dev)`
- `wheel_disable(dev)`

该 API 有意保持精简。轮子驱动需在内部将这些请求转换为电机级命令。

### 语义

`wheel_set_speed()` 设定一个轮级平移命令，包括：

- 轮周线速度
- 期望行进方向角

`wheel_set_static()` 请求锁定方向。驱动可将其实现为零驱动速度下的主动转向，也可实现为电机特定的保持行为。

`wheel_get_speed()` 返回当前测量或推导出的轮状态。

`wheel_get_target()` 返回驱动的当前指令目标。

`wheel_disable()` 用于立即抑制输出。

当某驱动未实现对应方法时：

- `wheel_set_speed()` 为空操作
- `wheel_set_static()` 返回 `-ENOSYS`
- `wheel_get_speed()` 与 `wheel_get_target()` 返回 `NULL`
- `wheel_disable()` 为无操作

## Devicetree 绑定

### `ares,steerwheel`

绑定：

- `dts/bindings/steerwheel/ares,steerwheel.yaml`

属性：

- `steer-motor`（必需）
- `wheel-motor`（必需）
- `wheel-radius`（必需，string）
- `angle-offset`（必需，string）
- `inverse-steer`（可选，boolean）
- `inverse-wheel`（可选，boolean）
- `multi_turn`（可选，boolean）

该绑定还定义：

- `wheel-cells = <offset_x offset_y>`

该项被底盘的 `wheels` 属性消费。

### `ares,mecanum`

绑定：

- `dts/bindings/steerwheel/ares,mecanum.yaml`

属性：

- `wheel-motor`（必需）
- `wheel-radius`（必需，string）
- `angle-offset`（必需，string）
- `inverse-wheel`（可选，boolean）
- `free-angle`（可选，string）

此绑定同样定义了 `wheel-cells` 布局。

## 构建与配置入口

Kconfig：

- `drivers/Kconfig`
- `drivers/wheel/Kconfig`

相关符号：

- `CONFIG_WHEEL`

构建入口：

- `drivers/CMakeLists.txt`
- `drivers/wheel/CMakeLists.txt`

当 `CONFIG_WHEEL=y` 时同时编译 `mecanum.c` 与 `steerwheel.c`。

## Steerwheel 驱动

实现文件：

- `drivers/wheel/steerwheel.c`

配置项：

- 转向电机设备
- 驱动电机设备
- 反向标志位
- `multi_turn` 模式

运行时状态：

- 当前状态
- 当前目标
- 计算出的轮子 RPM
- `negative` 标志：用于指示是否反向驱动以减少转向行程

### 运行时行为

`steerwheel_set_speed()`：

- 从电机驱动读取当前转向角与轮速
- 计算当前轮状态
- 当请求速度低于内部 epsilon 时立即停止驱动电机
- 计算目标角以最小化转向行程
- 当翻转 180 度可缩短转向路径时，可选地反向驱动轮子
- 按角度向转向电机下发命令，并向轮电机下发速度命令

启用 `multi_turn` 后，目标角演化以历史目标为基准，而非仅相对当前角度做闭合处理，从而允许转向轴跨圈连续运动。

`steerwheel_set_static()`：

- 强制目标速度为 0
- 使用同样的短路径逻辑解析静态目标角
- 向轮电机发送 `SET_ZERO`
- 命令转向电机到静态目标角
- 返回 `motor_set_angle(cfg->wheel_motor, 0)` 的结果

`steerwheel_disable()` 将两个电机的扭矩清零。

### 边界条件

- 低速请求在不改动转向目标的情况下折叠为零速走常规路径
- 反向标志同时影响状态解读与外发指令
- 多处通过 `fmodf()` 归一化角度，调用方不应假定所有辅助路径都使用同一标准角度区间

## Mecanum 驱动

实现文件：

- `drivers/wheel/mecanum.c`

配置项：

- 轮电机设备
- `wheel_radius`
- `angle_offset`
- `inverse_wheel`
- `free_angle`

运行时状态：

- 当前状态
- 目标状态
- 计算出的 RPM 命令
- 已缓存的静态角

### 麦克纳姆轮示意

俯视单个麦克纳姆轮时，驱动电机提供轮周切向速度，滚子方向提供自由滚动轴。驱动会把底盘请求速度向量投影到自由滚动轴，再折算成电机 RPM。

```text
                 target vector
                      ^
                      |
                      |
        wheel face    |        free rolling axis
      +-------------+ |       /
      |  / / / / /  | |      /  free-angle
      | / / / / /   | |     /
      |/ / / / /    | +----+------> motor tangential axis
      +-------------+

      target projection on free axis
          = target speed * cos(target angle - free axis angle)
```

### 运行时行为

`mecanum_set_speed()` 将请求的底盘运动向量投影到轮子的自由滚动轴，再把投影线速度转换为电机 RPM。

`mecanum_set_static()`：

- 将 RPM 命令置零
- 首次进入时锁存当前轮电机角度
- 通过 `motor_set_angle()` 持有该锁存角

当前代码为兼容性保留了 `angle` 参数，但并未使用该参数来计算麦轮保持方向。

### 边界条件

- `free-angle` 直接影响速度投影和分母 `cosf(free_angle)` 的计算
- 麦轮驱动未实现 `wheel_disable()` 方法，因此底盘禁能时会回退到子系统默认空操作

## 与电机的集成

两种轮子实现依赖以下公共电机 API：

- `motor_get_speed()`
- `motor_get_angle()`
- `motor_set_speed()`
- `motor_set_angle()`
- `motor_set_torque()`
- `motor_control()`

因此，轮子设备更准确地说是连接底盘运动学与电机驱动的胶水层，而非独立的执行机构栈。

## 示例与验证

主要集成示例：

- `samples/chassis`

典型覆盖文件：

- `samples/chassis/boards/robomaster_board_c.overlay`

该覆盖文件实例化三个 `ares,steerwheel` 设备，并通过 `wheels = <...>` phandle 连接到底盘节点。

当前仓内 `tests/` 尚无轮子专用测试。
