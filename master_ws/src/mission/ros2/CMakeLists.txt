cmake_minimum_required(VERSION 3.16)
project(mission_executor_ros2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core/domain
                 ${CMAKE_CURRENT_BINARY_DIR}/uav_domain)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core
                 ${CMAKE_CURRENT_BINARY_DIR}/mission_core)

add_executable(mission_executor_node src/main.cpp)
ament_target_dependencies(mission_executor_node rclcpp std_msgs)
target_link_libraries(mission_executor_node mission_core)
install(TARGETS mission_executor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
