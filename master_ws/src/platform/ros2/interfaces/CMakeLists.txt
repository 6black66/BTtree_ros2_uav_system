cmake_minimum_required(VERSION 3.16)
project(msgs)

find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/NavState.msg
  msg/FlightSetpoint.msg
  msg/FlightCommand.msg
  msg/FlightCommandResult.msg
  msg/FlightState.msg
  msg/FlightLinkState.msg
  msg/GimbalCommand.msg
  msg/GimbalState.msg
  msg/GimbalCommandResult.msg
  msg/GimbalLinkState.msg
  msg/AircraftTelemetry.msg
  msg/VideoFrameRef.msg
  msg/VideoStreamState.msg
  msg/ObjectDetection.msg
  msg/Observation.msg
  msg/ObserveTrackRequest.msg
  srv/FlightCommand.srv
  srv/GoTo.srv
  srv/OrbitCmd.srv
  srv/GimbalCmd.srv
  action/Task.action
  DEPENDENCIES std_msgs
)

ament_package()
