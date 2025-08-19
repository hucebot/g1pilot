from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1pilot',
            executable='joint_controller',
            name='joint_controller',
            parameters=[
                {
                    'interface': 'eth1',
                    'use_robot': True,
                }],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='cartesian_controller',
            name='cartesian_controller',
            parameters=[{'interface': 'eth1'}],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='interactive_marker',
            name='interactive_marker',
            parameters=[{'interface': 'eth1'}],
            output='screen'
        ),

    ])
