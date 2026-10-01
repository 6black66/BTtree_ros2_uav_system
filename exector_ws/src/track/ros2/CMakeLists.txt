cmake_minimum_required(VERSION 3.16)
project(track_executor_ros2)

find_package(ament_cmake REQUIRED)
find_package(uav_ros2_support REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../master_ws/src/core/domain
                 ${CMAKE_CURRENT_BINARY_DIR}/uav_domain)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core
                 ${CMAKE_CURRENT_BINARY_DIR}/track_core)

add_executable(track_executor_node src/main.cpp)
ament_target_dependencies(track_executor_node uav_ros2_support)
target_link_libraries(track_executor_node track_core)

install(TARGETS track_executor_node DESTINATION lib/track_executor_ros2)

ament_package()
