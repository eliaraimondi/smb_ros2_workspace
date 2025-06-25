cmake_minimum_required(VERSION 3.8)
project(mission_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED) 

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

add_executable(mission_switcher src/mission_switcher.cpp)
ament_target_dependencies(mission_switcher rclcpp std_msgs geometry_msgs)
install(TARGETS mission_switcher DESTINATION lib/${PROJECT_NAME})

# add_executable(waypoint_mux_node src/waypoint_mux_node.cpp)
# ament_target_dependencies(waypoint_mux_node rclcpp geometry_msgs)
# install(TARGETS waypoint_mux_node DESTINATION lib/${PROJECT_NAME})
add_executable(waypoint_switcher_node src/waypoint_switcher_node.cpp)
ament_target_dependencies(waypoint_switcher_node
  rclcpp
  geometry_msgs
  nav_msgs
)
install(TARGETS
  waypoint_switcher_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
