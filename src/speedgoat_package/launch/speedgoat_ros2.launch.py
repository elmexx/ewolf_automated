import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    return launch.LaunchDescription([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('realsense2_camera'), 'launch'),
            '/rs_imu.launch.py'])),
            
        Node(
            package='lanedet_ros2',
            executable='lanedet_node',
            name='lanedet_ros'),
            
        Node(
            package='speedgoat_package',
            executable='speedgoat_node',
            name='speedgoat_node'),     
  ])
