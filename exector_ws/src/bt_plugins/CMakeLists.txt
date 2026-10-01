cmake_minimum_required(VERSION 3.16)
project(bt_capability_plugins)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(bt_nodes REQUIRED)

function(add_capability_plugin capability macro_name)
  set(target bt_${capability}_nodes)
  add_library(${target} SHARED src/capability_plugin.cpp)
  target_compile_definitions(${target} PRIVATE BT_PLUGIN_EXPORT ${macro_name})
  ament_target_dependencies(${target} behaviortree_cpp bt_nodes)
  install(TARGETS ${target} LIBRARY DESTINATION lib)
endfunction()

add_capability_plugin(mission CAPABILITY_MISSION)
add_capability_plugin(navigation CAPABILITY_NAVIGATION)
add_capability_plugin(controller CAPABILITY_CONTROLLER)
add_capability_plugin(scene CAPABILITY_SCENE)
add_capability_plugin(find CAPABILITY_FIND)
add_capability_plugin(observe CAPABILITY_OBSERVE)
add_capability_plugin(track CAPABILITY_TRACK)
add_capability_plugin(report CAPABILITY_REPORT)
add_capability_plugin(refind CAPABILITY_REFIND)

include(CTest)
if(BUILD_TESTING)
  add_executable(capability_plugin_smoke_test tests/phase_plugin_smoke_test.cpp)
  ament_target_dependencies(capability_plugin_smoke_test behaviortree_cpp)
  set(TREE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../../config/behavior_trees)

  add_test(
    NAME capability_tree_standby
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_standby.xml
      $<TARGET_FILE:bt_mission_nodes> $<TARGET_FILE:bt_controller_nodes>
      $<TARGET_FILE:bt_scene_nodes>)
  add_test(
    NAME capability_tree_approch
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_approch.xml
      $<TARGET_FILE:bt_navigation_nodes> $<TARGET_FILE:bt_controller_nodes>
      $<TARGET_FILE:bt_scene_nodes> $<TARGET_FILE:bt_mission_nodes>)
  add_test(
    NAME capability_tree_onstation
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_onstation.xml
      $<TARGET_FILE:bt_controller_nodes> $<TARGET_FILE:bt_scene_nodes>
      $<TARGET_FILE:bt_mission_nodes>)
  add_test(
    NAME capability_tree_find
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_find.xml
      $<TARGET_FILE:bt_find_nodes> $<TARGET_FILE:bt_observe_nodes>
      $<TARGET_FILE:bt_controller_nodes> $<TARGET_FILE:bt_scene_nodes>)
  add_test(
    NAME capability_tree_track
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_track.xml
      $<TARGET_FILE:bt_track_nodes> $<TARGET_FILE:bt_observe_nodes>
      $<TARGET_FILE:bt_controller_nodes> $<TARGET_FILE:bt_scene_nodes>)
  add_test(
    NAME capability_tree_refind
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_refind.xml
      $<TARGET_FILE:bt_refind_nodes> $<TARGET_FILE:bt_observe_nodes>
      $<TARGET_FILE:bt_controller_nodes> $<TARGET_FILE:bt_scene_nodes>)
  add_test(
    NAME capability_tree_report
    COMMAND capability_plugin_smoke_test ${TREE_DIR}/phase_report.xml
      $<TARGET_FILE:bt_report_nodes> $<TARGET_FILE:bt_scene_nodes>
      $<TARGET_FILE:bt_mission_nodes>)
endif()

ament_package()
