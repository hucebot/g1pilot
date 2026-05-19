from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os
import sys


def generate_launch_description():
    if not os.environ.get("G1_INTERFACE"):
        sys.exit("ERROR: G1_INTERFACE environment variable is not set.\n"
                 "Set it to your network interface, e.g.: export G1_INTERFACE=eno2")


    interface = LaunchConfiguration("interface")
    use_robot = LaunchConfiguration("use_robot")
    arm_controlled = LaunchConfiguration("arm_controlled")
    enable_arm_ui = LaunchConfiguration("enable_arm_ui")

    return LaunchDescription([
        DeclareLaunchArgument("interface", default_value=EnvironmentVariable("G1_INTERFACE")),
        DeclareLaunchArgument("use_robot", default_value="true"),
        DeclareLaunchArgument("arm_controlled", default_value="both"),
        DeclareLaunchArgument("enable_arm_ui", default_value="true"),

        # Corrects MOLA odometry and broadcasts TF: map -> pelvis
        Node(
            package='g1pilot',
            executable='mola_fixed',
            name='mola_fixed',
            parameters=[{
                'in_topic': '/lidar_odometry/pose',
                'out_topic': '/lidar_odometry/pose_fixed',
                'map_frame': 'map',
                'base_frame': 'pelvis',
            }],
            output='screen'
        ),

        # Publishes OccupancyGrid on /map
        Node(
            package='g1pilot',
            executable='create_map',
            name='create_map',
            parameters=[{
                'frame_id': 'map',
            }],
            output='screen'
        ),

        # Dijkstra path planner
        Node(
            package='g1pilot',
            executable='dijkstra_planner',
            name='dijkstra_planner',
            parameters=[{
                'map_topic': '/map',
                'odom_topic': '/lidar_odometry/pose_fixed',
                'goal_topic': '/g1pilot/goal',
                'path_topic': '/g1pilot/path',
                'inflation_radius_m': 0.40,
            }],
            output='screen'
        ),

        # Path follower — outputs velocity commands as Joy
        Node(
            package='g1pilot',
            executable='nav2point',
            name='nav2point',
            parameters=[{
                'frame_id': 'map',
            }],
            output='screen'
        ),

        # Locomotion client — sends commands to robot
        Node(
            package='g1pilot',
            executable='loco_client',
            name='loco_client',
            parameters=[{
                'interface': interface,
                'use_robot': ParameterValue(use_robot, value_type=bool),
                'arm_controlled': arm_controlled,
                'enable_arm_ui': ParameterValue(enable_arm_ui, value_type=bool),
            }],
            output='screen'
        ),
    ])
