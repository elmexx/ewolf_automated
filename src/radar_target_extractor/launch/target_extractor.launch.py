from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='radar_target_extractor',
            executable='target_extractor_node',
            name='target_extractor_node',
            output='screen',
            parameters=[
                {
                    'input_topic': '/radar_detection_pcl_non_infra',
                    'min_x': 1.0,
                    'max_x': 150.0,
                    'max_abs_y': 2.5,
                    'max_abs_azimuth_deg': 10.0,
                    'max_azimuth_std_deg': 10.0,
                    'min_hits_to_validate': 3,
                    'max_misses_to_invalidate': 5,
                    'use_range_instead_of_x': False,
                    'invert_doppler_sign': False,
                }
            ]
        )
    ])