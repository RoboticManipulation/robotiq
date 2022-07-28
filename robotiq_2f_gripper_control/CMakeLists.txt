cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_gripper_control)
set(CMAKE_CXX_STANDARD 14)
#find_package(catkin REQUIRED COMPONENTS robotiq_ethercat roscpp rospy message_generation)
find_package(ament_cmake REQUIRED)
#find_package(robotiq_ethercat REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)



#add_message_files(
#  FILES
#  Robotiq2FGripper_robot_input.msg
#  Robotiq2FGripper_robot_output.msg
#)

set(msg_files
  "msg/Robotiq2FGripperRobotInput.msg"
  "msg/Robotiq2FGripperRobotOutput.msg"
)

#catkin_python_setup()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

#catkin_package(
#    CATKIN_DEPENDS rospy message_runtime roscpp robotiq_ethercat
#    INCLUDE_DIRS include
#)

include_directories(include)
include_directories(
  #${catkin_INCLUDE_DIRS}
  include ${Boost_INCLUDE_DIRS}
)

#add_executable(robotiq_2f_gripper_ethercat_node
#  src/robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_node.cpp
#  src/robotiq_2f_gripper_control/robotiq_2f_gripper_ethercat_client.cpp
#)

#target_link_libraries(robotiq_2f_gripper_ethercat_node
#  ${robotiq_ethercat_LIBRARIES}
#  ${catkin_LIBRARIES}
#)

ament_target_dependencies(robotiq_2f_gripper_ethercat_node,
  ${robotiq_ethercat_LIBRARIES}
)
add_dependencies(robotiq_2f_gripper_ethercat_node robotiq_2f_gripper_control_generate_messages_cpp)

install(PROGRAMS nodes/Robotiq2FGripperSimpleController.py nodes/Robotiq2FGripperStatusListener.py
  nodes/Robotiq2FGripperTcpNode.py nodes/Robotiq2FGripperRtuNode.py DESTINATION ~/ros2/${PROJECT_NAME})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
ament_package()
