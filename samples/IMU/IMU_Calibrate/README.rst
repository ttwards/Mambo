.. zephyr:code-sample:: imu-calibrate
   :name: IMU Accelerometer Calibration
   :relevant-api: sensor_interface

   Calibrate the accelerometer using Levenberg-Marquardt optimization.

Overview
********

This sample performs a 6-parameter calibration (offset and scale) for the accelerometer. It uses the Levenberg-Marquardt optimization algorithm to find the optimal calibration parameters that minimize the error between the measured acceleration magnitude and gravity (1g) across multiple orientations.

Features:

*   **Data Collection:** Collects accelerometer data in different static orientations.
*   **Optimization:** Runs Levenberg-Marquardt algorithm on-device (using CMSIS-DSP).
*   **Storage:** Saves the calibration matrix to persistent storage.

Requirements
************

*   A supported board (e.g., RoboMaster Board C).
*   IMU sensor (e.g., BMI08x) and a button/GPIO for user interaction (optional).

Building and Running
********************

Build and flash the sample for the ``robomaster_board_c``:

.. zephyr-app-commands::
   :zephyr-app: samples/IMU/IMU_Calibrate
   :board: robomaster_board_c
   :goals: build flash
   :compact:

Usage
*****

1.  **Start Calibration:** The system may auto-start or wait for a button press (depending on config).
2.  **Rotate Board:** Place the board in different static orientations (e.g., Z-up, Z-down, Y-up, Y-down, X-up, X-down).
3.  **Hold Still:** Keep the board still in each orientation until the sample count increments and stabilizes.
4.  **Completion:** Once enough samples are collected, the optimization runs.
5.  **Result:** If successful, the calibration parameters are printed and saved.

Console Output Example:

.. code-block:: console

   [00:00:05.000] <inf> matrix_storage_test: Reading 0 with 100 samples
   ...
   [00:00:10.000] <inf> matrix_storage_test: Calibration successful!
   [00:00:10.000] <inf> matrix_storage_test: Final Beta: [0.012, -0.005, 0.030, 0.998, 1.002, 0.999]
