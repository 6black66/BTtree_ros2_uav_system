cmake_minimum_required(VERSION 3.16)
project(video_gateway_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(episode_node src/main.cpp)
ament_target_dependencies(episode_node rclcpp std_msgs)
install(TARGETS episode_node DESTINATION lib/${PROJECT_NAME})

ament_package()
