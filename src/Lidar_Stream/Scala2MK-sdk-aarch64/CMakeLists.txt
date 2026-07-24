cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
project(scala_decoder_sdk_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io)
find_package(Threads REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

set(SCALA_DECODER_SDK_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package(ScalaDecoderSdk REQUIRED PATHS ${SCALA_DECODER_SDK_PATH}/cmake)

add_executable(${PROJECT_NAME}_node
  ${SCALA_DECODER_SDK_PATH}/src/${PROJECT_NAME}_node.cpp
)

target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${SCALA_DECODER_SDK_PATH}/include
  ${PCL_INCLUDE_DIRS}
)

ament_target_dependencies(${PROJECT_NAME}_node
  pcl_conversions
  rclcpp
  rclcpp_components
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  Threads::Threads
  ${PCL_LIBRARIES}
  scala_decoder::scala_decoder_sdk
)

set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD_REQUIRED ON)

install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES
  src/test.rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()