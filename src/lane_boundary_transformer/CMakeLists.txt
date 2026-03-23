cmake_minimum_required(VERSION 3.5)
project(lane_boundary_transformer)

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

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(lane_parameter_msg REQUIRED)

add_executable(lane_boundary_transformer_node src/lane_boundary_transformer.cpp)
target_include_directories(lane_boundary_transformer_node PUBLIC include)
ament_target_dependencies(
  lane_boundary_transformer_node
  rclcpp
  tf2_ros
  tf2
  tf2_geometry_msgs
  geometry_msgs
  nav_msgs
  std_msgs
  lane_parameter_msg)

install(TARGETS lane_boundary_transformer_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_path_processing test/test_path_processing.cpp)
  if(TARGET test_path_processing)
    target_include_directories(test_path_processing PUBLIC include)
    ament_target_dependencies(test_path_processing geometry_msgs lane_parameter_msg)
  endif()
endif()

ament_package()
