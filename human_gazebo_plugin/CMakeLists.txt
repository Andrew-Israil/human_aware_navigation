cmake_minimum_required(VERSION 3.8)
project(human_gazebo_plugin)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo REQUIRED)
find_package(rclcpp REQUIRED)
find_package(people_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(include 
${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${GAZEBO_CXX_FLAGS}")

add_library(ActorPlugin SHARED src/ActorPlugin.cc)
target_link_libraries(ActorPlugin ${GAZEBO_LIBRARIES})

add_library(MoveToVehicle SHARED src/MoveToVehicle.cc)
target_link_libraries(MoveToVehicle ${GAZEBO_LIBRARIES})

add_library(WorkerPlugin SHARED src/WorkerPlugin.cc)
target_link_libraries(WorkerPlugin ${GAZEBO_LIBRARIES})

set(dependencies
  rclcpp
  people_msgs
  nav_msgs
)

ament_target_dependencies(ActorPlugin ${dependencies})
ament_target_dependencies(MoveToVehicle ${dependencies})
ament_target_dependencies(WorkerPlugin ${dependencies})

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

ament_package()
