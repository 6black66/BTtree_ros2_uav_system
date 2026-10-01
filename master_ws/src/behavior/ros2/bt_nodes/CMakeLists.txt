cmake_minimum_required(VERSION 3.16)
project(bt_nodes)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(framework REQUIRED)
find_package(msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/node_context.cpp
  src/nav_nodes.cpp
  src/behavior_call.cpp
  src/mission_nodes.cpp
  src/shm_sync.cpp
  src/conditions.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp rclcpp_action behaviortree_cpp msgs)
target_link_libraries(${PROJECT_NAME} framework::framework)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp rclcpp_action behaviortree_cpp framework msgs)
ament_export_include_directories(include)

ament_package()
