from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('radar_bev_visualizer')
    config_file = os.path.join(pkg_share, 'config', 'radar_bev_visualizer.yaml')

    return LaunchDescription([
        Node(
            package='radar_bev_visualizer',
            executable='radar_bev_visualizer_node',
            name='radar_bev_visualizer',
            output='screen',
            parameters=[config_file]
        )
    ])
