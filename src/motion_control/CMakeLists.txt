cmake_minimum_required(VERSION 3.5)
project(motion_control)

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

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

################################################################################
# Build
################################################################################

include_directories(include)

add_library(controller_lib SHARED
  src/controller/pid.cpp)
# ament_target_dependencies(controller_lib "")

add_library(lane_control SHARED
  src/lane_control.cpp)
target_link_libraries(lane_control controller_lib)
target_compile_definitions(lane_control 
  PRIVATE "MOTION_CONTROL_BUILDING_DLL")
ament_target_dependencies(lane_control
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "geometry_msgs")
rclcpp_components_register_nodes(lane_control "motion_control::LaneControl")

################################################################################
# Install
################################################################################

# add the install(TARGETS…) section so ros2 can find your component
install(TARGETS
  lane_control
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

################################################################################
# TEST
################################################################################

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

################################################################################
# Macro for ament package
################################################################################

ament_package()
