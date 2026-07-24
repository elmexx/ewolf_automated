from setuptools import setup

package_name = 'udp_sender_bridge'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/udp_sender.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='ubuntu@example.com',
    description='Reusable ROS2 UDP sender bridge with topic aggregation and timeout handling.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'udp_sender_node = udp_sender_bridge.udp_sender_node:main',
        ],
    },
)