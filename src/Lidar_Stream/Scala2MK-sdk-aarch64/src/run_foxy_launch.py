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

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='scala_decoder_sdk_publisher',
            executable='scala_decoder_sdk_publisher_node',
            parameters=[
                {"HostPort": 33336},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.116"},
                {"PointCloudType": "Scala2HI"} #"unspecified", "Scala2HI", "Scala2LO", "Scala2Single", "Scala1" or "Scala2Double"
            ]
        ),
        Node(
            package='scala_decoder_sdk_publisher',
            executable='scala_decoder_sdk_publisher_node_2',
            parameters=[
                {"HostPort": 33337},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.117"},
                {"PointCloudType": "Scala2HI"} #"unspecified", "Scala2HI", "Scala2LO", "Scala2Single", "Scala1" or "Scala2Double"
            ]
        ),
        Node(
            package='scala_decoder_sdk_publisher',
            executable='scala_decoder_sdk_publisher_node_3',
            parameters=[
                {"HostPort": 33338},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.118"},
                {"PointCloudType": "Scala2HI"} #"unspecified", "Scala2HI", "Scala2LO", "Scala2Single", "Scala1" or "Scala2Double"
            ]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d'+str("/install/scala_decoder_sdk_publisher/share/scala_decoder_sdk_publishertest.rviz")]
        ),
    ])
