#
#   Copyright (c)     
#
#   The Verifiable & Control-Theoretic Robotics (VECTR) Lab
#   University of California, Los Angeles
#
#   Authors: Kenny J. Chen, Ryan Nemiroff, Brett T. Lopez
#   Contact: {kennyjchen, ryguyn, btlopez}@ucla.edu
#

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    current_pkg = FindPackageShare('direct_lidar_inertial_odometry')

    # Set default arguments
    rviz = LaunchConfiguration('rviz', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    offline_replay = LaunchConfiguration('offline_replay', default='false')
    pointcloud_topic = LaunchConfiguration('pointcloud_topic', default='points_raw')
    imu_topic = LaunchConfiguration('imu_topic', default='go2w/imu')
    launch_drivers = LaunchConfiguration('launch_drivers', default='true')
    dlio_config = LaunchConfiguration('dlio_config')
    params_config = LaunchConfiguration('params_config')

    # Define arguments
    declare_rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value=rviz,
        description='Launch RViz'
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value=use_sim_time,
        description='Use simulated time from /clock'
    )
    declare_offline_replay_arg = DeclareLaunchArgument(
        'offline_replay',
        default_value=offline_replay,
        description='Use deterministic, reliable input handling for offline bag replay'
    )
    declare_pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value=pointcloud_topic,
        description='Pointcloud topic name'
    )
    declare_imu_topic_arg = DeclareLaunchArgument(
        'imu_topic',
        default_value=imu_topic,
        description='IMU topic name'
    )
    declare_launch_drivers_arg = DeclareLaunchArgument(
        'launch_drivers',
        default_value=launch_drivers,
        description='Launch live LiDAR and IMU drivers'
    )
    declare_dlio_config_arg = DeclareLaunchArgument(
        'dlio_config',
        default_value=PathJoinSubstitution([current_pkg, 'cfg', 'dlio.yaml']),
        description='D-LIO calibration/intrinsics parameter file'
    )
    declare_params_config_arg = DeclareLaunchArgument(
        'params_config',
        default_value=PathJoinSubstitution([current_pkg, 'cfg', 'params.yaml']),
        description='D-LIO runtime/frame parameter file'
    )

    # Sensor drivers
    hesai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([FindPackageShare('hesai_lidar'), 'launch', 'hesai_lidar_launch.py'])
        ]),
        condition=IfCondition(launch_drivers)
    )
    imu_publisher_node = Node(
        package='go2w_imu_publisher',
        executable='imu_publisher',
        output='screen',
        condition=IfCondition(launch_drivers)
    )

    # DLIO Odometry Node
    dlio_odom_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_odom_node',
        output='screen',
        parameters=[dlio_config, params_config, {
            'use_sim_time': use_sim_time,
            'offline/replay': offline_replay,
        }],
        remappings=[
            ('pointcloud', pointcloud_topic),
            ('imu', imu_topic),
            ('odom', 'dlio/odom_node/odom'),
            ('pose', 'dlio/odom_node/pose'),
            ('path', 'dlio/odom_node/path'),
            ('kf_pose', 'dlio/odom_node/keyframes'),
            ('kf_cloud', 'dlio/odom_node/pointcloud/keyframe'),
            ('deskewed', 'dlio/odom_node/pointcloud/deskewed'),
        ],
    )

    # DLIO Mapping Node
    dlio_map_node = Node(
        package='direct_lidar_inertial_odometry',
        executable='dlio_map_node',
        output='screen',
        parameters=[dlio_config, params_config, {'use_sim_time': use_sim_time}],
        remappings=[
            ('keyframes', 'dlio/odom_node/pointcloud/keyframe'),
        ],
    )

    # RViz node
    rviz_config_path = PathJoinSubstitution([current_pkg, 'launch', 'dlio.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dlio_rviz',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        declare_rviz_arg,
        declare_use_sim_time_arg,
        declare_offline_replay_arg,
        declare_pointcloud_topic_arg,
        declare_imu_topic_arg,
        declare_launch_drivers_arg,
        declare_dlio_config_arg,
        declare_params_config_arg,
        hesai_launch,
        imu_publisher_node,
        dlio_odom_node,
        dlio_map_node,
        rviz_node
    ])
