"""Launch the geographic GNSS position observer."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('geo_map_observer'),
        'config',
        'geo_map_observer.yaml',
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'map_file',
            default_value='',
            description=(
                'Absolute path to an offline .osm XML map '
                '(empty disables loading)'),
        ),
        Node(
            package='geo_map_observer',
            executable='gnss_position_node',
            name='gnss_position_node',
            output='screen',
            parameters=[config, {'map_file': LaunchConfiguration('map_file')}],
        ),
    ])
