cmake_minimum_required(VERSION 3.16)
project(executor)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(framework REQUIRED)
find_package(bt_nodes REQUIRED)
find_package(std_msgs REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../core
                 ${CMAKE_CURRENT_BINARY_DIR}/behavior_core)

add_executable(behavior_executor_node src/main.cpp)
ament_target_dependencies(behavior_executor_node
  ament_index_cpp rclcpp behaviortree_cpp bt_nodes std_msgs)
target_link_libraries(behavior_executor_node behavior::core)

install(TARGETS behavior_executor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
