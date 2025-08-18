from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

package_name = "g1pilot"
urdf_file_name = "g1_29dof.urdf"
rviz_config_file_name = "g1_29dof.rviz"

def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")

    urdf = os.path.join(
        get_package_share_directory(package_name) + "/description_files/urdf/",
        urdf_file_name,
    )
    with open(urdf, "r") as infp:
        robot_desc = infp.read()


    return LaunchDescription([
        Node(
            package='g1pilot',
            executable='robot_state',
            name='robot_state',
            parameters=[{'interface': 'eth0'}],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mid360_to_livox_tf',
            arguments=[
                '0', '0', '0',
                '0', '0', '3.14159265',
                'mid360_link',
                'livox_frame'
            ]
        ),

        DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation (Gazebo) clock if true",
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[
                    {"use_sim_time": use_sim_time, "robot_description": robot_desc}
                ],
                arguments=[urdf],
            ),
            Node(
                package="rviz2",
                namespace="",
                executable="rviz2",
                name="rviz2",
                arguments=[
                    "-d"
                    + os.path.join("/ros2_ws/src/g1pilot/config/",
                        rviz_config_file_name,
                    )
                ],
            )
    ])
