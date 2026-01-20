.. zephyr:code-sample:: imu-quaternion-ekf
   :name: IMU Quaternion EKF
   :relevant-api: sensor_interface

   Attitude estimation using Quaternion Extended Kalman Filter (EKF) with IMU data.

Overview
********

This sample demonstrates how to use the Quaternion Extended Kalman Filter (EKF) from the ARES framework to estimate the device's attitude. It fuses data from the BMI08x accelerometer and gyroscope to provide a stable orientation output.

The sample uses the Hongxi Wong's QEKF algorithm to calculate:

*   **Quaternion:** [q0, q1, q2, q3]
*   **Euler Angles:** Yaw, Pitch, Roll

Requirements
************

*   A supported board (e.g., RoboMaster Board C or DM MC02).
*   BMI08x IMU sensor (Accelerometer and Gyroscope) configured in the devicetree.
*   ARES framework and DSP libraries enabled.

Building and Running
********************

Build and flash the sample for the ``robomaster_board_c``:

.. zephyr-app-commands::
   :zephyr-app: samples/IMU/IMU
   :board: robomaster_board_c
   :goals: build flash
   :compact:

Sample Output
*************

The application outputs attitude data to the console:

.. code-block:: console

   [00:00:01.000] <inf> main: q: 1.000000, 0.000000, 0.000000, 0.000000
   [00:00:01.000] <inf> main: Yaw: 0.123456, Pitch: 0.654321, Roll: 0.987654

Usage
*****

1.  Connect the board to a PC via USB/TTL to view console output.
2.  Rotate the board to see the Yaw, Pitch, and Roll angles change in real-time.
