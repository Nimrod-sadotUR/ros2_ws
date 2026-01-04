cmake_minimum_required(VERSION 3.5)
project(cv_engine)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
# find_package(opencv_dnn REQUIRED)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
# Add local 'include' directory so headers in 'src/controllers/include' are found
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

include_directories(
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(cv_engine src/cv_engine.cpp)
ament_target_dependencies(cv_engine rclcpp std_msgs sensor_msgs cv_bridge OpenCV)

install(TARGETS
  cv_engine
  DESTINATION lib/${PROJECT_NAME})

ament_package()