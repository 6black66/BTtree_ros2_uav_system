cmake_minimum_required(VERSION 3.16)
project(refind_executor_ros2)

find_package(ament_cmake REQUIRED)
find_package(uav_ros2_support REQUIRED)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../../../../master_ws/src/core/domain
                 ${CMAKE_CURRENT_BINARY_DIR}/uav_domain)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../core
                 ${CMAKE_CURRENT_BINARY_DIR}/refind_core)

add_executable(refind_executor_node src/main.cpp)
ament_target_dependencies(refind_executor_node uav_ros2_support)
target_link_libraries(refind_executor_node refind_core)

install(TARGETS refind_executor_node DESTINATION lib/${PROJECT_NAME})

ament_package()
