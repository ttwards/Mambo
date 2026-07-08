# 驱动文档

本目录记录 Mambo 提供的面向驱动使用方的子系统文档。文档聚焦于集成边界、设备模型、构建入口、运行时语义以及维护约束。驱动内部算法和厂商协议细节保持在源码中说明，除非其直接影响子系统行为。

## 范围

本目录文档覆盖的目录为：

- `drivers/`
- `include/zephyr/drivers/`
- `dts/bindings/`

它们用于补充 `Documents/` 中的模块参考说明，不做重复拷贝。

## 索引

- [底盘(chassis)](chassis.md)
  整车体运动层，将平移与旋转目标解析为每个轮子的命令。
- [轮子(wheel)](wheel.md)
  轮子抽象层，被底盘求解器以及 steerwheel、mecanum 等轮型实现共同使用。
- [SBUS](sbus.md)
  用于 SBUS 遥控输入的异步 UART 接收器。
- [PID](pid.md)
  面向底盘与部分传感器管理代码路径使用的轻量 PID 工具层。
- [传感器(sensor)](sensor.md)
  本仓库当前提供的 Zephyr 传感器驱动。

## 通用约定

该目录下多数子系统遵循标准 Zephyr 拆分方式：

- 公共 API 位于 `include/zephyr/drivers/`
- `drivers/` 下有一个或多个驱动实现
- DTS 绑定位于 `dts/bindings/`
- Kconfig 与 CMake 入口位于 `drivers/`

当某个子系统没有完全遵循该模式时，对应文档会明确指出差异。

## 子系统间关系

- `chassis` 使用 `wheel` API，并在偏航控制中使用一个 PID 实例。
- `wheel` 实现依赖 `motor` API。
- `sbus` 一般由应用或示例直接消费。
- `sensor` 驱动实现标准 Zephyr sensor API，并被 `lib/ares/` 与 `samples/` 中的 IMU 与感知代码使用。

## 维护说明

- 先与导出的头文件保持公共行为一致。
- 不要将私有辅助函数当作稳定接口文档化。
- Devicetree 与 Kconfig 名称按树中实际定义的大小写拼写使用。
- 当子系统未完成或仅部分集成时，记录当前可用状态，不要描述预期设计。
