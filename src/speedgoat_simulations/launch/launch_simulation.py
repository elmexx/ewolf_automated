from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='speedgoat_simulations',
            executable='reference_path_slalom',
            name='reference_path',
            output='screen'
        ),
        Node(
            package='speedgoat_simulations',
            executable='simulator_node',
            name='simple_simulator',
            output='screen',
        ),
        Node(
            package='speedgoat_package',
            executable='speedgoat_node',
            name='speedgoat_controller',
            output='screen'
        ),
        Node(
            package='speedgoat_simulations',
            executable='plotter_node',
            name='trajectory_plotter',
            output='screen',
        )
    ])
