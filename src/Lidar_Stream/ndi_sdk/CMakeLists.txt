cmake_minimum_required(VERSION 3.5)
project(ndi_sdk)

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

set(NDI_SDK_PATH ${CMAKE_CURRENT_LIST_DIR})

include_directories(
    ${NDI_SDK_PATH}/include
)

add_executable(Send_Lidar src/Send_Lidar.cpp)
add_executable(Send_Kamera src/Send_Kamera.cpp)
ament_target_dependencies(Send_Lidar rclcpp sensor_msgs cv_bridge)
ament_target_dependencies(Send_Kamera rclcpp sensor_msgs cv_bridge)

target_link_libraries(Send_Lidar libndi.so)
target_link_libraries(Send_Kamera libndi.so)

target_include_directories(Send_Lidar PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS Send_Lidar
  DESTINATION lib/${PROJECT_NAME})
  
target_include_directories(Send_Kamera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

install(TARGETS Send_Kamera
  DESTINATION lib/${PROJECT_NAME})
  
install(
    DIRECTORY lib/
    DESTINATION lib
)

ament_package()
