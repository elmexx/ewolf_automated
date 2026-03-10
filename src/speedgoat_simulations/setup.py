from setuptools import find_packages, setup

package_name = 'speedgoat_simulations'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/speedgoat_simulations']),
        ('share/speedgoat_simulations', ['package.xml']),
        ('share/speedgoat_simulations/launch', ['launch/launch_simulation.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andreasconfienza',
    maintainer_email='andreasconfienza@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reference_path_leftcurve = speedgoat_simulations.reference_path_leftcurve:main',
            'reference_path_slalom = speedgoat_simulations.reference_path_slalom:main',
            'reference_path_Tjunction = speedgoat_simulations.reference_path_Tjunction:main',
            'reference_path = speedgoat_simulations.reference_path:main',

            'simulator_node = speedgoat_simulations.simulator_node:main',

            'plotter_node = speedgoat_simulations.plotter_node:main',
        ],
    },
)
