# pixhawk_mavlink_matlab
This repo has a Matlab script to capture Mavlink messages from the Pixhawk flight controller over Uart and visualize the orientation.

## Steps
Configure the baudrate of the Pixhawk telemetry port over USB (Please refer to: https://qgroundcontrol.com/)
By default, it is set to 921600 in the 'pixhawk_testbench.m' script

An example demonstration:
![til](./assets/pixhawk_mavlink_matlab.gif)
