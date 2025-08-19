from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'g1pilot'

def expand(patterns):
    files = []
    for p in patterns:
        files.extend(glob(p, recursive=True))
    return files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),

        # Launch Files
        (f'share/{package_name}/launch', [
            'launch/robot_state_launcher.launch.py',
            'launch/controller_launcher.launch.py',
            'launch/teleoperation_launcher.launch.py',
            'launch/navigation_launcher.launch.py'
        ]),

        # URDF / XML
        (f'share/{package_name}/description_files/urdf',
         expand([ 'description_files/urdf/*.urdf', 'description_files/urdf/*.xacro' ])),
        (f'share/{package_name}/description_files/xml',
         expand([ 'description_files/xml/*.xml' ])),

        # Meshes
        (f'share/{package_name}/description_files/meshes',
         expand([
            'description_files/meshes/**/*.STL',
         ])),

        # RViz
        (f'share/{package_name}/rviz', expand(['rviz/*.rviz'])),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clemente Donoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='ROS 2 package for Dynamixel 6-DOF input control',
    license='BSD 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # States Nodes
            'robot_state = g1pilot.state.robot_state:main',
            'state_publisher_29dof = g1pilot.state.state_publisher_29dof:main',

            # Controllers Nodes
            'cartesian_controller = g1pilot.controller.cartesian_controller:main',
            'joint_controller = g1pilot.controller.joint_controller:main',
            'interactive_marker = g1pilot.controller.interactive_marker:main',

            # Teleoperation Nodes
            'joystick = g1pilot.teleoperation.joystick:main',
            'joy_mux = g1pilot.teleoperation.joy_mux:main',

            # Navigation Nodes
            'loco_client = g1pilot.navigation.loco_client:main',
            'dijkstra_planner = g1pilot.navigation.dijkstra_planner:main',
            'nav2point = g1pilot.navigation.navigate_to_point:main',
            'create_map = g1pilot.navigation.create_map:main'
        ],
    },
)
