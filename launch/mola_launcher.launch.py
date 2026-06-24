
# ROS 2 launch file

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.actions import PushRosNamespace
from launch.actions import DeclareLaunchArgument
from launch.actions import SetEnvironmentVariable
from launch.actions import GroupAction
from launch.actions import Shutdown
from ament_index_python import get_package_share_directory
import os


def generate_launch_description():
    myDir = get_package_share_directory("mola_lidar_odometry")

    # -------------------
    #     Arguments
    # -------------------
    # Mandatory
    lidar_topic_name_arg = DeclareLaunchArgument(
        "lidar_topic_name", default_value="/livox/lidar", description="Topic name to listen for PointCloud2 input from the LiDAR (for example '/ouster/points')")
    lidar_topic_env_var = SetEnvironmentVariable(
        name='MOLA_LIDAR_TOPIC', value=LaunchConfiguration('lidar_topic_name'))
    # ~~~~~~~~~~~~
    ignore_lidar_pose_from_tf_arg = DeclareLaunchArgument(
        "ignore_lidar_pose_from_tf", default_value="true", description="If true, the LiDAR pose will be assumed to be at the origin (base_link). Set to false (default) if you want to read the actual sensor pose from /tf")
    fixed_sensorpose_env_var = SetEnvironmentVariable(
        name='MOLA_USE_FIXED_LIDAR_POSE', value=LaunchConfiguration('ignore_lidar_pose_from_tf'))
    # ~~~~~~~~~~~~
    gnss_topic_name_arg = DeclareLaunchArgument(
        "gnss_topic_name", default_value="gps", description="Topic name to listen for NavSatFix input from a GNSS (for example '/gps')")
    gps_topic_env_var = SetEnvironmentVariable(
        name='MOLA_GNSS_TOPIC', value=LaunchConfiguration('gnss_topic_name'))
    # ~~~~~~~~~~~~
    imu_topic_name_arg = DeclareLaunchArgument(
        "imu_topic_name", default_value="/g1pilot/imu", description="Topic name to listen for Imu input (for example '/imu')")
    imu_topic_env_var = SetEnvironmentVariable(
        name='MOLA_IMU_TOPIC', value=LaunchConfiguration('imu_topic_name'))
    # ~~~~~~~~~~~~
    use_rviz = LaunchConfiguration('use_rviz')
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz", default_value="False", description="Whether to launch RViz2 with default lidar-odometry.rviz configuration")
    # ~~~~~~~~~~~~
    use_mola_gui_arg = DeclareLaunchArgument(
        "use_mola_gui", default_value="False", description="Whether to open MolaViz GUI interface for watching live mapping and control UI")
    use_mola_gui_env_var = SetEnvironmentVariable(
        name='MOLA_WITH_GUI', value=LaunchConfiguration('use_mola_gui'))
    # ~~~~~~~~~~~~
    publish_localization_following_rep105_arg = DeclareLaunchArgument(
        "publish_localization_following_rep105", default_value="False", description="Whether to publish localization TFs in between map->odom (true) or directly map->base_link (false)")
    publish_localization_following_rep105_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZ_USE_REP105', value=LaunchConfiguration('publish_localization_following_rep105'))
    # ~~~~~~~~~~~~
    start_mapping_enabled_arg = DeclareLaunchArgument(
        "start_mapping_enabled", default_value="True", description="Whether MOLA-LO should start with map update enabled (true), or in localization-only mode (false)")
    start_mapping_enabled_env_var = SetEnvironmentVariable(
        name='MOLA_MAPPING_ENABLED', value=LaunchConfiguration('start_mapping_enabled'))
    # ~~~~~~~~~~~~
    start_active_arg = DeclareLaunchArgument(
        "start_active", default_value="True", description="Whether MOLA-LO should start active, that is, processing incoming sensor data (true), or ignoring them (false)")
    start_active_env_var = SetEnvironmentVariable(
        name='MOLA_START_ACTIVE', value=LaunchConfiguration('start_active'))
    # ~~~~~~~~~~~~
    mola_lo_reference_frame_arg = DeclareLaunchArgument(
        "mola_lo_reference_frame", default_value="map", description="The /tf frame name to be used for MOLA-LO localization updates")
    mola_lo_reference_frame_env_var = SetEnvironmentVariable(
        name='MOLA_LO_PUBLISH_REF_FRAME', value=LaunchConfiguration('mola_lo_reference_frame'))
    # ~~~~~~~~~~~~
    mola_se_reference_frame_arg = DeclareLaunchArgument(
        "mola_state_estimator_reference_frame", default_value="map", description="The /tf frame name to be used as reference for MOLA State Estimators to publish pose updates")
    mola_tf_map_env_var = SetEnvironmentVariable(
        name='MOLA_TF_MAP', value=LaunchConfiguration('mola_state_estimator_reference_frame'))
    # ~~~~~~~~~~~~
    mola_lo_pipeline_arg = DeclareLaunchArgument(
        "mola_lo_pipeline", default_value="/ros2_ws/src/g1pilot/pipelines/lidar3d.yaml", description="The LiDAR-Odometry pipeline configuration YAML file defining the LO system. Absolute path, or relative to 'mola-cli-launchs/lidar_odometry_ros2.yaml'. Default is the 'lidar3d-default.yaml' system described in the IJRR 2025 paper.")
    mola_lo_pipeline_arg_env_var = SetEnvironmentVariable(
        name='MOLA_ODOMETRY_PIPELINE_YAML', value=LaunchConfiguration('mola_lo_pipeline'))
    # ~~~~~~~~~~~~
    generate_simplemap_arg = DeclareLaunchArgument(
        "generate_simplemap", default_value="False", description="Whether to create a '.simplemap', useful for map post-processing. Refer to online tutorials.")
    generate_simplemap_env_var = SetEnvironmentVariable(
        name='MOLA_GENERATE_SIMPLEMAP', value=LaunchConfiguration('generate_simplemap'))
    # ~~~~~~~~~~~~
    mola_initial_map_mm_file_arg = DeclareLaunchArgument(
        "mola_initial_map_mm_file", default_value="\"\"", description="Can be used to provide a metric map '.mm' file to be loaded as initial map. Refer to online tutorials.")
    mola_initial_map_mm_file_env_var = SetEnvironmentVariable(
        name='MOLA_LOAD_MM', value=LaunchConfiguration('mola_initial_map_mm_file'))
    # ~~~~~~~~~~~~
    mola_initial_map_sm_file_arg = DeclareLaunchArgument(
        "mola_initial_map_sm_file", default_value="\"\"", description="Can be used to provide a keyframes map '.simplemap' file to be loaded as initial map. Refer to online tutorials.")
    mola_initial_map_sm_file_env_var = SetEnvironmentVariable(
        name='MOLA_LOAD_SM', value=LaunchConfiguration('mola_initial_map_sm_file'))
    # ~~~~~~~~~~~~
    mola_tf_footprint_link_arg = DeclareLaunchArgument(
        "mola_tf_footprint_link",
        default_value="",
        description="If not empty, MOLA broadcasts a static /tf from this footprint frame to base_link (e.g. 'base_footprint'). Leave empty to disable footprint TF publishing.")
    mola_tf_footprint_link_env_var = SetEnvironmentVariable(
        name='MOLA_TF_FOOTPRINT_LINK',
        value=LaunchConfiguration('mola_tf_footprint_link'))
    # ~~~~~~~~~~~~
    mola_footprint_to_base_link_tf_arg = DeclareLaunchArgument(
        "mola_footprint_to_base_link_tf", 
        default_value="[0, 0, 0, 0, 0, 0]",
        description="Transformation between base_footprint and base_link, as [x, y, z, yaw_deg, pitch_deg, roll_deg]. Only used when mola_tf_footprint_link is set.")
    mola_footprint_to_base_link_tf_env_var = SetEnvironmentVariable(
        name='MOLA_TF_FOOTPRINT_TO_BASE_LINK',
        value=LaunchConfiguration('mola_footprint_to_base_link_tf'))
    # ~~~~~~~~~~~~
    enforce_planar_motion_arg = DeclareLaunchArgument(
        "enforce_planar_motion", default_value="False", description="Whether to enforce z, pitch, and roll to be zero.")
    enforce_planar_motion_env_var = SetEnvironmentVariable(
        name='MOLA_NAVSTATE_ENFORCE_PLANAR_MOTION', value=LaunchConfiguration('enforce_planar_motion'))
    # ~~~~~~~~~~~~
    forward_ros_tf_odom_to_mola_arg = DeclareLaunchArgument(
        "forward_ros_tf_odom_to_mola", default_value="False", description="Whether to import an existing /tf 'odom'->'base_link' odometry into the MOLA subsystem.")
    forward_ros_tf_odom_to_mola_env_var = SetEnvironmentVariable(
        name='MOLA_FORWARD_ROS_TF_ODOM_TO_MOLA', value=LaunchConfiguration('forward_ros_tf_odom_to_mola'))
    # ~~~~~~~~~~~~
    initial_localization_method_arg = DeclareLaunchArgument(
        "initial_localization_method", default_value="InitLocalization::FixedPose", description="What method to use for initialization. See https://docs.mola-slam.org/latest/ros2api.html#initial-localization")
    initial_localization_method_env_var = SetEnvironmentVariable(
        name='MOLA_LO_INITIAL_LOCALIZATION_METHOD', value=LaunchConfiguration('initial_localization_method'))
    # ~~~~~~~~~~~~
    use_state_estimator_arg = DeclareLaunchArgument(
        "use_state_estimator",
        default_value="False",
        description="If false, the basic state estimator 'mola::state_estimation_simple::StateEstimationSimple' will be used. If true, 'mola::state_estimation_smoother::StateEstimationSmoother' is used instead."
    )
    state_estimator_env_var = SetEnvironmentVariable(
        name='MOLA_STATE_ESTIMATOR', value=PythonExpression(
            ["'mola::state_estimation_smoother::StateEstimationSmoother' if ",
             LaunchConfiguration(
                 'use_state_estimator'),
             " else 'mola::state_estimation_simple::StateEstimationSimple'"
             ]))
    localization_publish_tf_source_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_TF_SOURCE',
        value=PythonExpression([
            "'state_estimator' if ", LaunchConfiguration(
                'use_state_estimator'), " else 'lidar_odometry'"
        ])
    )
    localization_publish_odom_source_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_ODOM_MSGS_SOURCE',
        value=PythonExpression([
            "'state_estimator' if ", LaunchConfiguration(
                'use_state_estimator'), " else 'lidar_odometry'"
        ])
    )
    state_estimator_config_yaml_arg = DeclareLaunchArgument(
        "state_estimator_config_yaml", default_value=PythonExpression(
            ["'../state-estimator-params/state-estimation-smoother.yaml' if ",
             LaunchConfiguration(
                 'use_state_estimator'),
             " else '../state-estimator-params/state-estimation-simple.yaml'"
             ]),
        description="A YAML file with settings for the state estimator. Absolute path or relative to 'mola-cli-launchs/lidar_odometry_ros2.yaml'")
    state_estimator_config_yaml_env_var = SetEnvironmentVariable(
        name='MOLA_STATE_ESTIMATOR_YAML', value=LaunchConfiguration('state_estimator_config_yaml'))
    # ~~~~~~~~~~~~
    lidar_scan_validity_minimum_point_count_arg = DeclareLaunchArgument(
        "lidar_scan_validity_minimum_point_count", default_value="100", description="Minimum number of points in each LiDAR raw scan for it to be considered valid; otherwise, it is ignored.")
    lidar_scan_validity_minimum_point_env_var = SetEnvironmentVariable(
        name='MOLA_OBS_VALIDITY_MIN_POINTS', value=LaunchConfiguration('lidar_scan_validity_minimum_point_count'))
    lidar_scan_validity_enable_env_var = SetEnvironmentVariable(
        name='MOLA_ENABLE_OBS_VALIDITY_FILTER', value='True')
    # ~~~~~~~~~~~~
    mola_precise_deskew_from_imu_arg = DeclareLaunchArgument(
        "mola_precise_deskew_from_imu", default_value="False", description="Whether to use the IMU to deskew the LiDAR scans more precisely")
    mola_precise_deskew_from_imu_env_var = SetEnvironmentVariable(
        name='MOLA_USE_PRECISE_LOCAL_VELOCITIES', value=LaunchConfiguration('mola_precise_deskew_from_imu'))
    
    # Namespace (Based on Nav2's bring-up launch file!)
    # ---------------------------------------------------
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')
    # ~~~~~~~~~~~~
    publish_tf_arg = DeclareLaunchArgument(
        "publish_tf", default_value="False",
        description="Whether MOLA BridgeROS2 should publish localization /tf (map->base_link or REP-105 map->odom)")
    publish_tf_env_var = SetEnvironmentVariable(
        name='MOLA_LOCALIZATION_PUBLISH_TF',
        value=LaunchConfiguration('publish_tf'))

    # Map fully qualified names to relative ones so the node's namespace can be prepended.
    # In case of the transforms (tf), currently, there doesn't seem to be a better alternative
    # https://github.com/ros/geometry2/issues/32
    # https://github.com/ros/robot_state_publisher/pull/30
    #
    # (JLBC further explanation) The problem is the "tf2" library. It's hardcoded to subscribe
    # to "/tf". This remapping allows "/robot/tf" to be seen as "/tf" so tf2_ros (and RViz) can see it.
    #
    tf_remaps = [('/tf', 'tf'),
                 ('/tf_static', 'tf_static')]

    # MOLA subsystem configuration YAML file
    # ------------------------------------------
    mola_system_yaml_file = os.path.join(
        myDir, 'mola-cli-launchs', 'lidar_odometry_ros2.yaml')

    # -------------------
    #        Node
    # -------------------
    node_group = GroupAction([
        PushRosNamespace(
            condition=IfCondition(use_namespace),
            namespace=namespace),

        Node(
            package='mola_launcher',
            executable='mola-cli',
            output='screen',
            remappings=tf_remaps,
            arguments=[mola_system_yaml_file],
            on_exit=Shutdown()
        ),

        Node(
            condition=IfCondition(use_rviz),
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            remappings=tf_remaps,
            arguments=[
                '-d', [os.path.join(myDir, 'rviz2', 'lidar-odometry.rviz')]]
        )
    ])

    return LaunchDescription([
        declare_namespace_cmd,
        declare_use_namespace_cmd,
        lidar_topic_name_arg,
        lidar_topic_env_var,
        ignore_lidar_pose_from_tf_arg,
        fixed_sensorpose_env_var,
        gnss_topic_name_arg,
        gps_topic_env_var,
        imu_topic_name_arg,
        imu_topic_env_var,
        use_mola_gui_arg,
        use_mola_gui_env_var,
        publish_localization_following_rep105_arg,
        publish_localization_following_rep105_env_var,
        start_mapping_enabled_arg,
        start_mapping_enabled_env_var,
        start_active_arg,
        start_active_env_var,
        mola_lo_reference_frame_arg,
        mola_lo_reference_frame_env_var,
        mola_lo_pipeline_arg,
        mola_lo_pipeline_arg_env_var,
        generate_simplemap_arg,
        generate_simplemap_env_var,
        mola_initial_map_mm_file_arg,
        mola_initial_map_mm_file_env_var,
        mola_initial_map_sm_file_arg,
        mola_initial_map_sm_file_env_var,
        mola_tf_footprint_link_arg,
        mola_tf_footprint_link_env_var,
        mola_footprint_to_base_link_tf_arg,
        mola_footprint_to_base_link_tf_env_var,
        enforce_planar_motion_arg,
        enforce_planar_motion_env_var,
        forward_ros_tf_odom_to_mola_arg,
        forward_ros_tf_odom_to_mola_env_var,
        initial_localization_method_arg,
        initial_localization_method_env_var,
        use_state_estimator_arg,
        state_estimator_env_var,
        state_estimator_config_yaml_arg,
        state_estimator_config_yaml_env_var,
        localization_publish_odom_source_env_var,
        localization_publish_tf_source_env_var,
        mola_se_reference_frame_arg,
        mola_tf_map_env_var,
        lidar_scan_validity_minimum_point_count_arg,
        lidar_scan_validity_minimum_point_env_var,
        lidar_scan_validity_enable_env_var,
        mola_precise_deskew_from_imu_arg,
        mola_precise_deskew_from_imu_env_var,
        use_rviz_arg,
        publish_tf_arg,
        publish_tf_env_var,
        node_group,
    ])