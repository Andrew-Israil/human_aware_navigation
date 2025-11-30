cmake_minimum_required(VERSION 3.8)
project(nav2_custom_bt_nodes)


if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(conflict_type_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  nav2_msgs
  nav_msgs
  geometry_msgs
  std_msgs
  nav2_util
  conflict_type_msgs
  nav2_behavior_tree
  behaviortree_cpp_v3
)

add_library(path_validity_check_action_bt_node SHARED src/path_validity_check.cpp)
list(APPEND plugin_libs path_validity_check_action_bt_node)

add_library(stop_wait_decorator_bt_node SHARED src/stop_wait.cpp)
list(APPEND plugin_libs stop_wait_decorator_bt_node)

add_library(cancel_control_condition_bt_node SHARED src/cancel_control_condition.cpp)
list(APPEND plugin_libs cancel_control_condition_bt_node)

add_library(dynamic_wait_action_bt_node SHARED src/dynamic_wait_action.cpp)
list(APPEND plugin_libs dynamic_wait_action_bt_node)

add_library(detour_faster_condition_bt_node SHARED src/detour_faster_condition.cpp)
list(APPEND plugin_libs detour_faster_condition_bt_node)

add_library(is_path_empty_condition_bt_node SHARED src/is_path_empty.cpp)
list(APPEND plugin_libs is_path_empty_condition_bt_node)

add_library(is_path_updated_condition_bt_node SHARED src/is_path_updated.cpp)
list(APPEND plugin_libs is_path_updated_condition_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
