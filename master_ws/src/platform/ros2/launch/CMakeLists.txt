cmake_minimum_required(VERSION 3.16)
project(bringup)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(uav_bringup)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY ../../../../../config/ DESTINATION share/${PROJECT_NAME}/config)

ament_package()
