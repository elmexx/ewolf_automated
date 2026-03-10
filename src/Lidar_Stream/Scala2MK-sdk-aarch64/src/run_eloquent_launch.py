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
            node_executable='scala_decoder_sdk_publisher_node',
            parameters=[
                {"HostPort": 22017},
                {"HostIP": "192.168.1.123"},
                {"MulticastIP": "224.111.111.111"},
                {"PointCloudType": "Scala2Double"}
            ]
        ),
        Node(
            package='rviz2',
            node_executable='rviz2',
            output='screen',
            arguments=['-d'+str("src/test.rviz")]
        ),
    ])
