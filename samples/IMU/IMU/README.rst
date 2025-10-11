.. zephyr:code-sample:: imu-quaternion-ekf
   :name: IMU四元数扩展卡尔曼滤波器示例

   使用四元数EKF算法处理IMU传感器数据并估计姿态角度。

概述
****

这个示例应用演示了如何使用ARES框架中的四元数扩展卡尔曼滤波器(Quaternion EKF)来处理IMU传感器数据。该示例集成了BMI08x系列的加速度计和陀螺仪，通过Hongxi Wong的QEKF融合算法实现精确的姿态估计。

传感器配置
==========

该示例使用以下传感器：

* **BMI08x加速度计**: 测量线性加速度，用于重力方向估计
* **BMI08x陀螺仪**: 测量角速度，提供高频率姿态更新

传感器通过SPI接口连接，支持高频率数据采集(最高2000Hz)。

硬件要求
********

* RoboMaster Board C (STM32F407)
* DM MC02开发板

构建和运行
**********

以RoboMaster Board C为例的构建命令:

`west build -b robomaster_board_c samples/IMU`

配置选项
========

主要的Kconfig配置选项：

* ``CONFIG_ARES=y`` - 启用ARES框架
* ``CONFIG_MAHONY_LIB=y`` - 启用Mahony算法库
* ``CONFIG_AUTO_PROBE_GYRO_BIAS=y`` - 启用陀螺仪零偏自动估计
* ``CONFIG_CMSIS_DSP=y`` - 启用CMSIS DSP库用于矩阵运算
* ``CONFIG_PLOTTER=y`` - 启用数据绘图功能

示例输出
========

控制台输出示例:

.. code-block:: console

   *** Booting Zephyr OS build ***
   [00:00:01.000,000] <inf> main: q: 1.000000, 0.000000, 0.000000, 0.000000
   [00:00:01.200,000] <inf> main: Yaw: 0.123456, Pitch: 0.654321, Roll: 0.987654
   [00:00:01.400,000] <inf> main: q: 0.999123, 0.001234, 0.005678, 0.009876
   [00:00:01.600,000] <inf> main: Yaw: 1.234567, Pitch: 0.765432, Roll: 1.098765

输出数据说明:

* **q**: 四元数分量 [q0, q1, q2, q3]，表示当前姿态
* **Yaw/Pitch/Roll**: 欧拉角(度)，分别表示偏航角、俯仰角、横滚角
