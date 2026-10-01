cmake_minimum_required(VERSION 3.16)
project(find_executor_ros2)

find_package(ament_cmake REQUIRED)
find_package(uav_ros2_support REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../master_ws/src/core/domain
                 ${CMAKE_CURRENT_BINARY_DIR}/uav_domain)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core
                 ${CMAKE_CURRENT_BINARY_DIR}/find_core)

add_executable(find_executor_node src/main.cpp)
ament_target_dependencies(find_executor_node uav_ros2_support)
target_link_libraries(find_executor_node find_core)

install(TARGETS find_executor_node DESTINATION lib/find_executor_ros2)

ament_package()
