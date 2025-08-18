from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='g1pilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

    ])
