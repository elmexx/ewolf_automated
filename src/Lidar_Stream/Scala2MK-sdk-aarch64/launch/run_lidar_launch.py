from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scala_decoder_sdk_publisher',
            executable='scala_decoder_sdk_publisher_node',
            name='scala_decoder_sdk_publisher_left',
            output='screen',
            parameters=[
                {"HostPort": 33336},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.116"},
                {"PointCloudType": "Scala2HI"},
                {"topic_name": "/scala_decoder_sdk_points"},
                {"frame_id": "lidar_left"}
            ]
        ),

        Node(
            package='scala_decoder_sdk_publisher',
            executable='scala_decoder_sdk_publisher_node',
            name='scala_decoder_sdk_publisher_right',
            output='screen',
            parameters=[
                {"HostPort": 33337},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.117"},
                {"PointCloudType": "Scala2HI"},
                {"topic_name": "/scala_decoder_sdk_points_2"},
                {"frame_id": "lidar_right"}
            ]
        ),

        Node(
            package='scala_decoder_sdk_publisher',
            executable='scala_decoder_sdk_publisher_node',
            name='scala_decoder_sdk_publisher_center',
            output='screen',
            parameters=[
                {"HostPort": 33338},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.118"},
                {"PointCloudType": "Scala2HI"},
                {"topic_name": "/scala_decoder_sdk_points_3"},
                {"frame_id": "lidar_center"}
            ]
        ),
    ])