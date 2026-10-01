cmake_minimum_required(VERSION 3.16)
project(flight_controller_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(controller_node src/main.cpp)
ament_target_dependencies(controller_node rclcpp msgs std_msgs)
install(TARGETS controller_node DESTINATION lib/${PROJECT_NAME})

ament_package()
