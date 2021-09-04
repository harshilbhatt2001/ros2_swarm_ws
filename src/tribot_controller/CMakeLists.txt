cmake_minimum_required(VERSION 3.5)
project(tribot_controller)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  geometry_msgs
  sensor_msgs
  nav2_msgs
  behaviortree_cpp_v3
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav2_util
)

add_library(nav2_goal_reached_condition_bt_node SHARED plugins/condition/goal_reached_condition.cpp)
list(APPEND plugin_libs nav2_goal_reached_condition_bt_node)

add_library(nav2_goal_updated_condition_bt_node SHARED plugins/condition/goal_updated_condition.cpp)
list(APPEND plugin_libs nav2_goal_updated_condition_bt_node)

add_library(nav2_is_battery_low_condition_bt_node SHARED plugins/condition/is_battery_low_condition.cpp)
list(APPEND plugin_libs nav2_is_battery_low_condition_bt_node)

add_library(nav2_is_flag_high_condition_bt_node SHARED plugins/condition/is_flag_high_condition.cpp)
list(APPEND plugin_libs nav2_is_flag_high_condition_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

install(TARGETS ${library_name}
                ${plugin_libs}
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
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
