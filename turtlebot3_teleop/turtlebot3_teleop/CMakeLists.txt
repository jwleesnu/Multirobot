cmake_minimum_required(VERSION 3.5)
project(turtlebot3_teleop)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Add executable with a different name
add_executable(teleop_keyboard_cpp src/teleop_keyboard.cpp)

# Link libraries
ament_target_dependencies(teleop_keyboard_cpp
  rclcpp
  geometry_msgs
  turtlesim
  std_msgs
  std_srvs
)

# Install
install(TARGETS
  teleop_keyboard_cpp
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package() 