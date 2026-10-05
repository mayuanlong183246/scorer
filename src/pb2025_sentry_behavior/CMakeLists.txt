cmake_minimum_required(VERSION 3.16)

if(POLICY CMP0009)
  cmake_policy(SET CMP0009 NEW)
endif()

project(pb2025_sentry_behavior)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## By adding -Wall and -Werror, the compiler does not ignore warnings anymore,
## enforcing cleaner code.
add_definitions(-Wall -Werror)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

#######################
## Find dependencies ##
#######################

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

###########
## Build ##
###########

ament_auto_add_executable(pb2025_sentry_behavior_server src/pb2025_sentry_behavior_server.cpp)

ament_auto_add_executable(tactical_scorer_node src/tactical_scorer_node.cpp)

ament_auto_add_library(action_client SHARED
  src/pb2025_sentry_behavior_client.cpp
)

rclcpp_components_register_node(action_client
  PLUGIN pb2025_sentry_behavior::SentryBehaviorClient
  EXECUTABLE pb2025_sentry_behavior_client
)

####### Conditions #######
ament_auto_add_library(is_base_status_ok SHARED plugins/condition/is_base_status_ok.cpp)
list(APPEND plugin_libs is_base_status_ok)

ament_auto_add_library(is_game_status SHARED plugins/condition/is_game_status.cpp)
list(APPEND plugin_libs is_game_status)

ament_auto_add_library(
  is_mapping_bootstrap_finished SHARED
  plugins/condition/is_mapping_bootstrap_finished.cpp)
list(APPEND plugin_libs is_mapping_bootstrap_finished)

ament_auto_add_library(is_rfid_detected SHARED plugins/condition/is_rfid_detected.cpp)
list(APPEND plugin_libs is_rfid_detected)

ament_auto_add_library(is_status_ok SHARED plugins/condition/is_status_ok.cpp)
list(APPEND plugin_libs is_status_ok)

ament_auto_add_library(is_status_time_ok SHARED plugins/condition/is_status_time_ok.cpp)
list(APPEND plugin_libs is_status_time_ok)

ament_auto_add_library(is_attacked SHARED plugins/condition/is_attacked.cpp)
list(APPEND plugin_libs is_attacked)

ament_auto_add_library(is_detect_enemy SHARED plugins/condition/is_detect_enemy.cpp)
list(APPEND plugin_libs is_detect_enemy)

####### Actions #######
ament_auto_add_library(send_nav2_goal SHARED plugins/action/send_nav2_goal.cpp)
list(APPEND plugin_libs send_nav2_goal)

ament_auto_add_library(pub_nav2_goal SHARED plugins/action/pub_nav2_goal.cpp)
list(APPEND plugin_libs pub_nav2_goal)

ament_auto_add_library(pub_spin_speed SHARED plugins/action/pub_spin_speed.cpp)
list(APPEND plugin_libs pub_spin_speed)

ament_auto_add_library(pub_status SHARED plugins/action/pub_status.cpp)
list(APPEND plugin_libs pub_status)

ament_auto_add_library(status_convert SHARED plugins/action/status_convert.cpp)
list(APPEND plugin_libs status_convert)

ament_auto_add_library(pub_twist SHARED plugins/action/pub_twist.cpp)
list(APPEND plugin_libs pub_twist)

ament_auto_add_library(pub_gimbal_absolute SHARED plugins/action/pub_gimbal_absolute.cpp)
list(APPEND plugin_libs pub_gimbal_absolute)

ament_auto_add_library(pub_gimbal_velocity SHARED plugins/action/pub_gimbal_velocity.cpp)
list(APPEND plugin_libs pub_gimbal_velocity)

ament_auto_add_library(calculate_attack_pose SHARED plugins/action/calculate_attack_pose.cpp)
list(APPEND plugin_libs calculate_attack_pose)

ament_auto_add_library(tactical_nodes SHARED plugins/tactical_nodes.cpp)
list(APPEND plugin_libs tactical_nodes)

ament_auto_add_library(
  tactical_navigate_to_pose SHARED plugins/action/tactical_navigate_to_pose.cpp)
list(APPEND plugin_libs tactical_navigate_to_pose)

ament_auto_add_library(tactical_execution SHARED plugins/tactical_execution.cpp)
list(APPEND plugin_libs tactical_execution)

####### Controls #######
ament_auto_add_library(recovery_node SHARED plugins/condition/control/recovery_node.cpp)
list(APPEND plugin_libs recovery_node)

####### Decorators #######
ament_auto_add_library(rate_controller SHARED plugins/decorator/rate_controller.cpp)
list(APPEND plugin_libs rate_controller)

ament_auto_add_library(tick_after_timeout_node SHARED plugins/decorator/tick_after_timeout_node.cpp)
list(APPEND plugin_libs tick_after_timeout_node)

foreach(bt_plugin ${plugin_libs})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

set(project_targets
  pb2025_sentry_behavior_server
  tactical_scorer_node
  action_client
  ${plugin_libs}
)

foreach(project_target ${project_targets})
  target_include_directories(${project_target} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)
endforeach()

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  ament_add_test(test_referee_gate GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_referee_gate.py
    TIMEOUT 30)
  ament_add_test(test_tactical_server GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_tactical_server.py
      --server $<TARGET_FILE:pb2025_sentry_behavior_server>
      --params ${CMAKE_CURRENT_SOURCE_DIR}/params/sentry_behavior.yaml
    TIMEOUT 30)

  ament_add_gtest(test_tactical_scorer test/test_tactical_scorer.cpp src/tactical_scorer_node.cpp TIMEOUT 30)
  ament_target_dependencies(test_tactical_scorer rclcpp std_msgs auto_aim_interfaces
    pb_rm_interfaces pb2025_sentry_decision_interfaces)
  target_include_directories(test_tactical_scorer PRIVATE include)
  target_compile_definitions(test_tactical_scorer PRIVATE TACTICAL_SCORER_NO_MAIN)
  ament_add_gtest(test_tactical_execution test/test_tactical_execution.cpp TIMEOUT 90)
  ament_target_dependencies(test_tactical_execution rclcpp rclcpp_action nav2_msgs
    geometry_msgs std_msgs pb_rm_interfaces pb2025_sentry_decision_interfaces behaviortree_ros2)
  target_include_directories(test_tactical_execution PRIVATE include)
  target_compile_definitions(test_tactical_execution PRIVATE PLUGIN_DIR="$<TARGET_FILE_DIR:tactical_execution>")
  add_dependencies(test_tactical_execution tactical_execution tactical_nodes tactical_navigate_to_pose)

  set(ament_cmake_clang_format_CONFIG_FILE "${CMAKE_SOURCE_DIR}/.clang-format")
  find_package(ament_lint_auto REQUIRED)
  list(APPEND AMENT_LINT_AUTO_EXCLUDE
    ament_cmake_uncrustify
    ament_cmake_flake8
  )
  ament_lint_auto_find_test_dependencies()
endif()

#############
## Install ##
#############

install(
  TARGETS ${plugin_libs}
  LIBRARY DESTINATION share/${PROJECT_NAME}/bt_plugins
  ARCHIVE DESTINATION share/${PROJECT_NAME}/bt_plugins
  RUNTIME DESTINATION share/${PROJECT_NAME}/bt_plugins
)

install(
  PROGRAMS
    scripts/sentry_referee_simulator.py
    scripts/sentry_tactical_visualizer.py
    scripts/sentry_test_velocity_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(
  INSTALL_TO_SHARE
  behavior_trees
  params
  launch
)
install(FILES REAL_ROBOT_TEST_PLAN.md DESTINATION share/${PROJECT_NAME})
