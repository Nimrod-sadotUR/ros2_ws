cmake_minimum_required(VERSION 3.5)
project(controllers)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(diff_drive_control src/diff_drive_control.cpp)
ament_target_dependencies(diff_drive_control rclcpp std_msgs geometry_msgs)



add_executable(ackermann_control src/ackermann_control.cpp)
ament_target_dependencies(ackermann_control rclcpp std_msgs geometry_msgs)


install(TARGETS
  diff_drive_control
  ackermann_control
  DESTINATION lib/${PROJECT_NAME})

ament_package()