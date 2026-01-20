.. zephyr:code-sample:: chassis
   :name: Chassis Driver Sample
   :relevant-api: chassis_driver_api

   Control a robot chassis using SBUS input and the chassis subsystem.

Overview
********

This sample application demonstrates how to use the Chassis subsystem API to control a robot chassis.
It reads control signals from an SBUS receiver and maps them to chassis velocity commands:

*   **Channel 3 (Left Stick Vertical):** Controls X-axis speed (Forward/Backward)
*   **Channel 1 (Left Stick Horizontal):** Controls Y-axis speed (Left/Right strafing)
*   **Channel 0 (Right Stick Horizontal):** Controls rotational speed (Yaw rate)

The sample also initializes the IMU sensor to support chassis kinematics that may require orientation feedback.

Requirements
************

*   A board with support for the Chassis subsystem and SBUS (e.g., RoboMaster Board C).
*   A chassis configuration (e.g., Mecanum, Omni, or Steering wheels) defined in the application's devicetree overlay.
*   An SBUS receiver connected to the board's specified UART/SBUS pin.
*   Motors and IMU sensors properly configured in the devicetree.

Building and Running
********************

Build and flash the sample for a supported board (e.g., ``robomaster_board_c``):

.. zephyr-app-commands::
   :zephyr-app: samples/chassis
   :board: robomaster_board_c
   :goals: build flash
   :compact:

Usage
*****

1.  Connect the SBUS receiver and motors to the board.
2.  Power on the board and the remote controller.
3.  The chassis should respond to the joystick movements on the remote controller.

   *   Pushing the left stick up/down moves the chassis forward/backward.
   *   Pushing the left stick left/right moves the chassis left/right.
   *   Pushing the right stick left/right rotates the chassis.
