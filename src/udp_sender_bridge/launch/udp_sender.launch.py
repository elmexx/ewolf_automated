from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='udp_sender_bridge',
            executable='udp_sender_node',
            name='udp_sender_node',
            output='screen',
            parameters=[
                {
                    'target_host': '10.42.0.10',
                    'target_port': 5500,
                    'dtype': 'float64',
                    'enable_send': True,
                    'send_period_sec': 0.1,
                    'payload_length': 12,

                    'use_speedgoat_topics': True,
                    'use_radar_target_topics': True,

                    'topic_steer': '/steering_angle',
                    'topic_accel': '/acceleration',
                    'topic_dist_rel': '/radar_target/dist_rel',
                    'topic_v_rel': '/radar_target/v_rel',
                    'topic_obj_valid': '/radar_target/obj_valid',

                    'slot_steer': 6,
                    'slot_accel': 7,
                    'slot_dist_rel': 8,
                    'slot_v_rel': 9,
                    'slot_obj_valid': 10,
                    'slot_counter': 11,

                    'default_steer': 0.0,
                    'default_accel': 0.0,
                    'default_dist_rel': 0.0,
                    'default_v_rel': 0.0,
                    'default_obj_valid': 0.0,

                    'timeout_steer_sec': 0.3,
                    'timeout_accel_sec': 0.3,
                    'timeout_dist_rel_sec': 0.3,
                    'timeout_v_rel_sec': 0.3,
                    'timeout_obj_valid_sec': 0.3,
                }
            ]
        )
    ])