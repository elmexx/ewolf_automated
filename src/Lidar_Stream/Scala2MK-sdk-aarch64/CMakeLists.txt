#
# @copyright(c) 2020 This program is the confidential and proprietary product of Valeo
#                    Schalter und Sensoren GmbH (Driving Assistance Research).
#                    All rights reserved.
#
#                    VALEO Schalter und Sensoren GmbH (Driving Assistance Research) will
#                    take no responsibility for any improper behavior of the software. In case of
#                    equipping test vehicles with the sensor kit, VALEO Schalter und Sensoren
#                    GmbH takes no liability on the behavior of the test vehicles equipped with
#                    the sensor kit or any damage caused within or outside to material and
#                    people.
#
# @author Patrick Reichel <patrick.reichel@valeo.com>
#
# @date December 2021
#

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)

project(scala_decoder_sdk_publisher)

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

find_package(PCL REQUIRED COMPONENTS common io)
find_package(Threads REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

#set(SCALA_DECODER_SDK_PATH ${CMAKE_CURRENT_LIST_DIR}/../../../)
set(SCALA_DECODER_SDK_PATH ${CMAKE_CURRENT_LIST_DIR})
find_package(ScalaDecoderSdk REQUIRED PATHS ${SCALA_DECODER_SDK_PATH}/cmake)

include_directories(
    ${SCALA_DECODER_SDK_PATH}/include
    ${PCL_INCLUDE_DIRS}
)

#add_executable(${PROJECT_NAME}_node ${PROJECT_NAME}_node.cpp)
add_executable(${PROJECT_NAME}_node ${SCALA_DECODER_SDK_PATH}/src/${PROJECT_NAME}_node.cpp)
add_executable(${PROJECT_NAME}_node_2 ${SCALA_DECODER_SDK_PATH}/src/${PROJECT_NAME}_node_2.cpp)
add_executable(${PROJECT_NAME}_node_3 ${SCALA_DECODER_SDK_PATH}/src/${PROJECT_NAME}_node_3.cpp)

set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME}_node PROPERTY CXX_STANDARD_REQUIRED ON)

set_property(TARGET ${PROJECT_NAME}_node_2 PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME}_node_2 PROPERTY CXX_STANDARD_REQUIRED ON)

set_property(TARGET ${PROJECT_NAME}_node_3 PROPERTY CXX_STANDARD 14)
set_property(TARGET ${PROJECT_NAME}_node_3 PROPERTY CXX_STANDARD_REQUIRED ON)

ament_target_dependencies(${PROJECT_NAME}_node "pcl_conversions" "rclcpp" "rclcpp_components" "sensor_msgs")
ament_target_dependencies(${PROJECT_NAME}_node_2 "pcl_conversions" "rclcpp" "rclcpp_components" "sensor_msgs")
ament_target_dependencies(${PROJECT_NAME}_node_3 "pcl_conversions" "rclcpp" "rclcpp_components" "sensor_msgs")

target_link_libraries(${PROJECT_NAME}_node
    Threads::Threads
    ${PCL_LIBRARIES}
    scala_decoder::scala_decoder_sdk
)

target_link_libraries(${PROJECT_NAME}_node_2
    Threads::Threads
    ${PCL_LIBRARIES}
    scala_decoder::scala_decoder_sdk
)

target_link_libraries(${PROJECT_NAME}_node_3
    Threads::Threads
    ${PCL_LIBRARIES}
    scala_decoder::scala_decoder_sdk
)

install(TARGETS
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_node_2
    ${PROJECT_NAME}_node_3
    DESTINATION lib/${PROJECT_NAME}
)

install(FILES
    src/run_dashing_launch.py
    DESTINATION share/${PROJECT_NAME}
)
install(FILES
    src/run_eloquent_launch.py
    DESTINATION share/${PROJECT_NAME}
)
install(FILES
    src/run_foxy_launch.py
    DESTINATION share/${PROJECT_NAME}
)
install(FILES
    src/run_galactic_launch.py
    DESTINATION share/${PROJECT_NAME}
)

install(FILES
    src/test.rviz
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
