from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Nodo que lee y publica JointStates
        Node(
            package='g1pilot',
            executable='joint_controller',
            name='joint_controller',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='cartesian_controller',
            name='cartesian_controller',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='interactive_marker',
            name='interactive_marker',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

    ])
