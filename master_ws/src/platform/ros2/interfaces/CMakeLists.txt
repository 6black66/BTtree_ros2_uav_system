cmake_minimum_required(VERSION 3.16)
project(msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/NavState.msg
  srv/GoTo.srv
  srv/OrbitCmd.srv
  srv/GimbalCmd.srv
  action/Task.action
  DEPENDENCIES std_msgs
)

ament_package()
