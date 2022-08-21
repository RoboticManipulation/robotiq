cmake_minimum_required(VERSION 3.5)
project(robotiq_2f_gripper_action_server)

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
find_package(robotiq_2f_gripper_actions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(robotiq_2f_gripper_control REQUIRED)

include_directories(include)

add_library(robotiq SHARED
  src/robotiq_2f_gripper_action_client.cpp
  src/robotiq_2f_gripper_action_server.cpp)
rclcpp_components_register_node(robotiq PLUGIN "robotiq_2f_gripper_action_server::Robotiq2FGripperActionClient" EXECUTABLE robotiq_2f_gripper_action_client)
rclcpp_components_register_node(robotiq PLUGIN "robotiq_2f_gripper_action_server::Robotiq2FGripperActionServer" EXECUTABLE robotiq_2f_gripper_action_server)
target_compile_definitions(robotiq
  PRIVATE "ROBOTIQ_2F_GRIPPER_ACTION_SERVER_BUILDING_DLL")
ament_target_dependencies(robotiq
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "robotiq_2f_gripper_actions"
  "robotiq_2f_gripper_control"
  )

install(TARGETS
  robotiq
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
