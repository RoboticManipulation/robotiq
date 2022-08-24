# Robotiq

## Status

This repository (fork of [ros-industrial](https://github.com/ros-industrial/robotiq)) is now ***maintained*** by TAMS group from Univeristy of Hamburg.

## License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## ROS2 Installation

Be sure to only build selected packages, as not all packages have been adjusted for ros2, so a general build command would fail

    colcon build --packages-select robotiq_modbus_rtu robotiq_2f_gripper_control robotiq_two_finger_gripper_actions robotiq_two_finger_gripper_action_server_cpp robotiq_two_finger_gripper_action_client_py
    
## ROS2 Usage
