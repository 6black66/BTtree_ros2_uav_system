cmake_minimum_required(VERSION 3.16)
project(uav_ros2_support)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(msgs REQUIRED)
find_package(tinyxml2 REQUIRED)

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_dependencies(rclcpp rclcpp_action msgs tinyxml2)
ament_package()
