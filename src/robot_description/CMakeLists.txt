cmake_minimum_required(VERSION 3.5.0)
project(robot_description)

# Default to C11
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
endif()
# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT WIN32)
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/launch)
  install(DIRECTORY
    launch
    DESTINATION share/${PROJECT_NAME}/launch)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/config)
  install(DIRECTORY
    config
    DESTINATION share/${PROJECT_NAME}/config)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  install(DIRECTORY
    urdf
    DESTINATION share/${PROJECT_NAME}/urdf)
endif()

if(EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/worlds)
  install(DIRECTORY
    worlds
    DESTINATION share/${PROJECT_NAME}/worlds)
endif()


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## This package only contains resources (URDF, launch, config, worlds).
## No CMake target is defined, so there's nothing to install into
## lib/${PROJECT_NAME}. The following TARGETS install was removed to
## prevent CMake errors during build.

ament_package()
