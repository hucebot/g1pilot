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

        # Launch files
        (f'share/{package_name}/launch', [
            'launch/state_publisher_29dof.launch.py',
            'launch/robot_state_launcher.launch.py',
            'launch/controller_launcher.launch.py',
        ]),

        # URDF / XML
        (f'share/{package_name}/description_files/urdf',
         expand([ 'description_files/urdf/*.urdf', 'description_files/urdf/*.xacro' ])),
        (f'share/{package_name}/description_files/xml',
         expand([ 'description_files/xml/*.xml' ])),

        # Meshes (soporta mayúsculas/minúsculas y subcarpetas)
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
            'cartesian_controller = g1pilot.cartesian_controller:main',
            'joint_controller = g1pilot.joint_controller:main',
            'interactive_marker = g1pilot.interactive_marker:main',
            'robot_state = g1pilot.robot_state:main',
            'loco_client = g1pilot.loco_client:main',
            'joystick = g1pilot.joystick:main',
            'state_publisher_29dof = g1pilot.state_publisher_29dof:main',
        ],
    },
)
