cmake_minimum_required(VERSION 3.16)
project(px4_msgs)
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
rosidl_generate_interfaces(${PROJECT_NAME}
  msg/VehicleStatus.msg
  msg/VehicleLocalPosition.msg
  msg/VehicleGlobalPosition.msg
  msg/VehicleAttitude.msg
  msg/VehicleCommandAck.msg
  msg/VehicleCommand.msg
  msg/TrajectorySetpoint.msg
  msg/OffboardControlMode.msg
  msg/VehicleLandDetected.msg
  DEPENDENCIES builtin_interfaces)
ament_export_dependencies(rosidl_default_runtime)
ament_package()
