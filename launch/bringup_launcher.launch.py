from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, EnvironmentVariable
import os
import sys

def generate_launch_description():
    if not os.environ.get("G1_INTERFACE"):
        sys.exit("ERROR: G1_INTERFACE environment variable is not set.\n"
                 "Set it to your network interface, e.g.: export G1_INTERFACE=eno2")

    pkg1_share = FindPackageShare('g1pilot').find('g1pilot')

    interface = LaunchConfiguration("interface")

    navigation_launcher = os.path.join(pkg1_share, 'launch', 'navigation_launcher.launch.py')
    robot_state_launcher = os.path.join(pkg1_share, 'launch', 'robot_state_launcher.launch.py')
    teleoperation_launcher = os.path.join(pkg1_share, 'launch', 'teleoperation_launcher.launch.py')
    manipulation_launcher = os.path.join(pkg1_share, 'launch', 'manipulation_launcher.launch.py')
    livox_launcher = os.path.join(pkg1_share, 'launch', 'livox_launcher.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument("enable_collision_avoidance", default_value="true"),
        DeclareLaunchArgument(
            "interface",
            default_value=EnvironmentVariable("G1_INTERFACE"),
            description="Network interface for Unitree SDK",
        ),
        DeclareLaunchArgument(
            "no_navigation",
            default_value="false",
            description="Set to true to skip launching the navigation stack",
        ),
        DeclareLaunchArgument(
            "no_robot_state",
            default_value="false",
            description="Set to true to skip launching the robot_state stack",
        ),
        DeclareLaunchArgument(
            "no_teleoperation",
            default_value="false",
            description="Set to true to skip launching the teleoperation stack",
        ),
        DeclareLaunchArgument(
            "no_manipulation",
            default_value="false",
            description="Set to true to skip launching the manipulation stack",
        ),
        DeclareLaunchArgument(
            "no_livox",
            default_value="false",
            description="Set to true to skip launching the Livox LiDAR driver",
        ),
        DeclareLaunchArgument(
            "no_web_bridge",
            default_value="false",
            description="Set to true to skip launching the WebSocket bridge",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(livox_launcher),
            condition=UnlessCondition(LaunchConfiguration("no_livox")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(navigation_launcher),
            launch_arguments=[("interface", interface)],
            condition=UnlessCondition(LaunchConfiguration("no_navigation")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_state_launcher),
            launch_arguments={
                'interface': interface,
                'publish_joint_states': 'true',
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("no_robot_state")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(teleoperation_launcher),
            launch_arguments=[("interface", interface)],
            condition=UnlessCondition(LaunchConfiguration("no_teleoperation")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(manipulation_launcher),
            launch_arguments={
                'interface': interface,
                'enable_collision_avoidance': LaunchConfiguration('enable_collision_avoidance'),
                'send_cmds_to_robot': 'true',
                'publish_joint_states_opensot': 'false',
            }.items(),
            condition=UnlessCondition(LaunchConfiguration("no_manipulation")),
        ),
        Node(
            package='g1pilot',
            executable='web_bridge',
            name='web_bridge',
            output='screen',
            emulate_tty=True,
            condition=UnlessCondition(LaunchConfiguration("no_web_bridge")),
        ),
    ])
