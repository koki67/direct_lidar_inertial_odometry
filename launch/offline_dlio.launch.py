"""Headless D-LIO processing nodes for a recorded Go2W sensor bag.

Playback, result recording, endpoint barriers, resource sampling, and artifact
generation are owned by scripts/offline/run_dlio_offline.sh. This launch file
deliberately starts no drivers, mapping display node, or RViz process.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _processing_node(context, package_share):
    defaults = {
        "dlio_config": os.path.join(package_share, "cfg", "dlio.yaml"),
        "params_config": os.path.join(package_share, "cfg", "params.yaml"),
        "offline_config": os.path.join(package_share, "cfg", "offline.yaml"),
    }
    paths = {
        name: LaunchConfiguration(name).perform(context) or default
        for name, default in defaults.items()
    }
    for name, path in paths.items():
        if not os.path.isfile(path):
            raise RuntimeError(
                f"D-LIO {name} does not exist: {path}. "
                "Rebuild the selected workspace overlay."
            )

    return [
        Node(
            package="direct_lidar_inertial_odometry",
            executable="dlio_odom_node",
            name="dlio_odom_node",
            output="screen",
            parameters=[
                paths["dlio_config"],
                paths["params_config"],
                paths["offline_config"],
                {
                    "use_sim_time": True,
                    "offline/replay": True,
                },
            ],
            remappings=[
                ("pointcloud", "/points_raw"),
                ("imu", "/go2w/imu"),
                ("odom", "/dlio/odom_node/odom"),
                ("pose", "/dlio/odom_node/pose"),
                ("path", "/dlio/odom_node/path"),
                ("kf_pose", "/dlio/odom_node/keyframes"),
                ("kf_cloud", "/dlio/odom_node/pointcloud/keyframe"),
                ("deskewed", "/dlio/odom_node/pointcloud/deskewed"),
            ],
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory(
        "direct_lidar_inertial_odometry"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "dlio_config",
                default_value="",
                description="Optional calibration/intrinsics YAML override.",
            ),
            DeclareLaunchArgument(
                "params_config",
                default_value="",
                description="Optional D-LIO runtime/tuning YAML override.",
            ),
            DeclareLaunchArgument(
                "offline_config",
                default_value="",
                description="Headless publisher contract YAML.",
            ),
            OpaqueFunction(function=_processing_node, args=[package_share]),
        ]
    )
