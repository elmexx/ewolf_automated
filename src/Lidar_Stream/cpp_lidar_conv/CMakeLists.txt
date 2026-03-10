cmake_minimum_required(VERSION 3.5)
project(cpp_lidar_conv)

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
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

add_executable(lidar_converter src/Convert_Lidar.cpp)
ament_target_dependencies(lidar_converter rclcpp sensor_msgs cv_bridge pcl_ros pcl_conversions)

install(TARGETS
    lidar_converter
    DESTINATION lib/${PROJECT_NAME})

ament_package()
