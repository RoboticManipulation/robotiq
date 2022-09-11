cmake_minimum_required(VERSION 3.5)
project(robotiq_two_finger_gripper_action_server_cpp)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robotiq_two_finger_gripper_actions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robotiq_2f_gripper_control REQUIRED)

include_directories(include)

#add_library(${PROJECT_NAME}_robotiq SHARED
#  src/robotiq_two_finger_gripper_action_client.cpp
#  src/robotiq_two_finger_gripper_action_server.cpp)
#rclcpp_components_register_node(${PROJECT_NAME}_robotiq PLUGIN #"robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionClient" EXECUTABLE #robotiq_two_finger_gripper_action_client)
#rclcpp_components_register_node(${PROJECT_NAME}_robotiq PLUGIN #"robotiq_two_finger_gripper_action_server_cpp::RobotiqTwoFingerGripperActionServer" EXECUTABLE #robotiq_two_finger_gripper_action_server)
#target_compile_definitions(${PROJECT_NAME}_robotiq
#  PRIVATE "ROBOTIQ_TWO_FINGER_GRIPPER_ACTION_SERVER_BUILDING_DLL")
#ament_target_dependencies(${PROJECT_NAME}_robotiq
#  "rclcpp"
#  "rclcpp_action"
#  "rclcpp_components"
#  "robotiq_two_finger_gripper_actions"
#  "robotiq_2f_gripper_control"
#  )
  
add_executable(robotiq_two_finger_gripper_action_server_main src/robotiq_two_finger_gripper_action_server.cpp)
ament_target_dependencies(robotiq_two_finger_gripper_action_server_main
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "robotiq_two_finger_gripper_actions"
  "robotiq_2f_gripper_control")


#add_executable(robotiq_two_finger_gripper_action_client_main src/robotiq_two_finger_gripper_action_client.cpp)
#ament_target_dependencies(robotiq_two_finger_gripper_action_client_main
#  "rclcpp"
#  "rclcpp_action"
#  "rclcpp_components"
#  "robotiq_two_finger_gripper_actions"
#  "robotiq_2f_gripper_control")

add_executable(robotiq_two_finger_gripper_action_client_test src/robotiq_two_finger_gripper_action_client_test.cpp)
ament_target_dependencies(robotiq_two_finger_gripper_action_client_test
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "robotiq_two_finger_gripper_actions"
  "robotiq_2f_gripper_control")

add_executable(robotiq_two_finger_gripper_action_not_composable_client src/robotiq_two_finger_gripper_action_not_composable_client.cpp)
ament_target_dependencies(robotiq_two_finger_gripper_action_not_composable_client
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "robotiq_two_finger_gripper_actions"
  "robotiq_2f_gripper_control")

install(TARGETS
  #robotiq_two_finger_gripper_action_client_main
  robotiq_two_finger_gripper_action_not_composable_client
  robotiq_two_finger_gripper_action_client_test
  robotiq_two_finger_gripper_action_server_main
  DESTINATION lib/${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
