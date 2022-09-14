# Robotiq

## Status

This repository (fork of [ros-industrial](https://github.com/ros-industrial/robotiq)) is now ***maintained*** by TAMS group from Univeristy of Hamburg.

## License

[![License](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](https://opensource.org/licenses/Apache-2.0) [![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

## ROS2 Installation
Install python package (has to be version 1.3.1)

    pip install pymodbus==1.3.1

Clone the repository and make sure to checkout the branch ros2-galactic.

    git clone git@github.com:RoboticManipulation/robotiq.git -b ros2-galactic

Be sure to only build selected packages, as not all packages have been adjusted for ros2, so a general build command would fail

    colcon build --packages-select robotiq_modbus_rtu robotiq_2f_gripper_control robotiq_two_finger_gripper_actions robotiq_two_finger_gripper_action_server_cpp robotiq_two_finger_gripper_action_client_py
    
## ROS2 Usage

The first thing you need to do is starting the driver, if the connection is setup, the LED on the gripper should turn blue

    #source your workspace
    r2
    
    #find out what port the gripper is connected to
    dmesg | grep tty
    
    #the output should look like this:
    [####.#####] USB 1-2: FTDI USB Serial Device converter now attached to ttyUSB0

    #run the Driver Node, make sure to use the correct tty port number from earlier
    ros2 run robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
    
Now that the driver is running and the conncetion is setup, you can controll the robot with the SimpleController, the ActionServer or your own Nodes.

To use the SimpleController, open a new tab and enter the following

    #source your workspace
    r2
    
    #run the SimpleController
    ros2 run robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
    
To use the Actionserver open two new tabs and enter the following

    #source your workspace in both tabs
    r2
    
    #in tab1 start the server
    ros2 run robotiq_two_finger_gripper_action_server_cpp robotiq_two_finger_gripper_action_server_main
    
    #in tab2 start the client node
    ros2 run robotiq_two_finger_gripper_action_server_cpp robotiq_two_finger_gripper_action_client_main
    
    #alternativly you can use the python client node 
    ros2 run robotiq_two_finger_gripper_action_client_py robotiq_two_finger_gripper_action_client

