from setuptools import find_packages, setup
from glob import glob

package_name = 'g1pilot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
         (
            "share/" + package_name + "/launch",
            [
                "launch/state_publisher_29dof.launch.py",
                "launch/bridge_launcher.launch.py",
            ],
        ),
        ('share/' + package_name, ['package.xml']),
        (
            "share/" + package_name + "/description_files/urdf",
            glob("description_files/urdf/*.urdf"),
        ),
        (
            "share/" + package_name + "/description_files/xml",
            glob("description_files/xml/*.xml"),
        ),
        # Install meshes
        (
            "share/" + package_name + "/description_files/meshes",
            glob("description_files/meshes/*.STL"),
        ),
        # Install rviz config files
        (
            "share/" + package_name + "/rviz",
            glob("rviz/*.rviz"),
        ),
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
