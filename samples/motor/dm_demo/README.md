# 达妙DM电机控制示例 - 简化版

这个示例演示了达妙DM电机的基本速度控制功能，上电后直接设置电机转速为100 RPM。

## 硬件要求
- RoboMaster Board C (或兼容的STM32F4开发板)
- 达妙DM电机 (如DM4310, DM4340等)
- CAN收发器模块
- 24V电源供应

### CAN总线配置
- 波特率: 1Mbps
- 发送ID: 0x0A
- 接收ID: 0x1A

## 电机参数配置

### 达妙DM电机参数
- 最大速度: 785 rad/s (约7500 RPM)
- 最大扭矩: 5 Nm
- 控制频率: 1000 Hz
- 控制模式: VO (速度-电流双环控制)

## 构建和运行

```bash
# 进入示例目录
cd samples/motor/dm_demo

# 构建项目
west build -b robomaster_board_c

# 烧录程序
west flash
```

## 日志输出示例

```
[00:00:00.100,000] <inf> dm_motor_demo: === 达妙DM电机控制示例 - 简化版 ===
[00:00:00.150,000] <inf> dm_motor_demo: 上电直接设置转速100 RPM
[00:00:00.200,000] <inf> dm_motor_demo: 电机已启用
[00:00:01.250,000] <inf> dm_motor_demo: 电机转速已设置为100 RPM
[00:00:02.300,000] <inf> dm_motor_demo: 电机状态 - 扭矩: 0.12 Nm, 速度: 98.5 RPM, 角度: 15.2°
[00:00:03.300,000] <inf> dm_motor_demo: 电机状态 - 扭矩: 0.15 Nm, 速度: 99.8 RPM, 角度: 45.6°
[00:00:04.300,000] <inf> dm_motor_demo: 电机状态 - 扭矩: 0.14 Nm, 速度: 100.1 RPM, 角度: 75.3°
...
```
