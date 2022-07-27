# http://ros.org/doc/groovy/api/catkin/html/user_guide/supposed.html
cmake_minimum_required(VERSION 3.5)
project(robotiq_modbus_rtu)
#find_package(catkin REQUIRED COMPONENTS rospy)
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)
#catkin_python_setup()

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)

ament_export_dependencies(rosidl_default_runtime)

#catkin_package(
#    CATKIN_DEPENDS rospy
#)
ament_package()
