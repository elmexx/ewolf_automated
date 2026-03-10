## Readme Trajectory Planing

set static TF

~~ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_link_vehicle~~

~~ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link base_link_gps~~


~~ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom odom_gps~~

``
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 odom odom_vehicle
``

``
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link scala_decoder_sdk_lidar
``

``
ros2 launch vehicle_description display.launch.py
``

~~ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 base_link camera_link~~

~~ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 camera_link camera_imu_optical_frame~~

### 1. GPS odom -> base_link_gps
get gps message

``ros2 launch nmea_navsat_driver nmea_socket_driver.launch.py``

caculate gps odom -> base_link_gps

``ros2 run gps_odometry_package gps_odometry_node ``

### 2. Vehicle dynamic odom -> base_link_vehicle

get udp data from speedgoat

``
ros2 run udp_receiver udp_receiver_node
`` 

transform upd date to vehicle dynamic data

``
ros2 run vehicle_dynamic_pkg vehicle_dynamic_node
``

caculate vehicle odom -> base_link_vehicle

``
ros2 run vehicle_odometry_package vehicle_odometry_node 
``

### 3. start camera and imu
 ``ros2 launch realsense2_camera rs_imu_launch.py ``

### 4. transform imu -> imu_enu

``
ros2 run imu_transform_package imu_transform 
``

### 5. Localization with odom_gps and odom_vehicle

``
ros2 launch robot_localization ekf.launch.py
 ``

### 6. run lane detection

`` ros2 run lanedet_ros2 lanedet_node ``

### 7. run lane transform, get lane in odom

``ros2 run lane_boundary_transformer lane_boundary_transformer_node``

### 8. run lane tracking with stanley model or MPC

``ros2 run speedgoat_package speedgoat_node ``

