cmake_minimum_required(VERSION 3.8)
project(exploration_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Add executables
add_executable(exploration_manager src/exploration_manager.cpp)
add_executable(exploration_manager2 src/exploration_manager2.cpp)

# Specify dependencies
ament_target_dependencies(exploration_manager
  rclcpp 
  geometry_msgs 
  std_msgs
)

ament_target_dependencies(exploration_manager2
  rclcpp   
  geometry_msgs   
  std_msgs
)

# Install targets
install(TARGETS
  exploration_manager
  exploration_manager2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()