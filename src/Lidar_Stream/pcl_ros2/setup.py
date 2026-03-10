from setuptools import setup

package_name = 'pcl_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ebel',
    maintainer_email='ebel@fkfs.de',
    description='Konvertierung der unorganisierten Punktewolke in ein Bild',
    license='intern',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_converter = pcl_ros2.pcl_to_image:main',
            'lidar_converter_3D = pcl_ros2.pcl_to_image_3D:main'
        ],
    },
)
