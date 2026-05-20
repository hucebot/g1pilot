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
            'launch/teleoperation_launcher.launch.py',
            'launch/navigation_launcher.launch.py',
            'launch/mola_launcher.launch.py',
            'launch/livox_launcher.launch.py',
            'launch/manipulation_launcher.launch.py',

            'launch/bringup_launcher.launch.py',
            'launch/bringup_opensot.launch.py',
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

        # Configuration Files
        (f'share/{package_name}/config',
            expand(['config/*.yaml', 'config/*.pgm'])),

        # RViz
        (f'share/{package_name}/rviz', expand(['rviz/*.rviz'])),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Clemente Donoso',
    maintainer_email='clemente.donoso@inria.fr',
    description='ROS 2 package to control the G1 robot',
    license='BSD 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # States Nodes
            'robot_state = g1pilot.state.robot_state:main',

            # Manipulation Nodes
            'interactive_marker = g1pilot.manipulation.interactive_marker:main',
            'dx3_controller = g1pilot.manipulation.dx3_hand:main',
            'opensot_solver = g1pilot.manipulation.opensot_solver:main',

            # Teleoperation Nodes
            'joystick = g1pilot.teleoperation.joystick:main',
            'joy_mux = g1pilot.teleoperation.joy_mux:main',
            'ui_interface = g1pilot.teleoperation.ui_interface:main',

            # Navigation Nodes
            'loco_client = g1pilot.navigation.loco_client:main',
            'dijkstra_planner = g1pilot.navigation.dijkstra_planner:main',
            'nav2point = g1pilot.navigation.nav2point:main',
            'create_map = g1pilot.navigation.create_map:main',
            'mola_fixed = g1pilot.navigation.fix_mola_odometry:main',

            # Bridges
            'web_bridge = g1pilot.scripts.web_bridge:main',
        ],
    },
)
