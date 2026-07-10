"""TF-only visualization launch for validating D-LIO sensor extrinsics."""

import math
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _ros_parameters(path):
    with Path(path).open(encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}
    return data.get("/**", {}).get("ros__parameters", {})


def _vector(params, key, length):
    values = params.get(key)
    if values is None:
        raise RuntimeError(f"Missing required parameter: {key}")
    if len(values) != length:
        raise RuntimeError(f"Parameter {key} must have {length} values, got {len(values)}")
    return [float(v) for v in values]


def _quaternion_from_matrix(r):
    m00, m01, m02, m10, m11, m12, m20, m21, m22 = r
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * s
        qx = (m21 - m12) / s
        qy = (m02 - m20) / s
        qz = (m10 - m01) / s
    elif m00 > m11 and m00 > m22:
        s = math.sqrt(1.0 + m00 - m11 - m22) * 2.0
        qw = (m21 - m12) / s
        qx = 0.25 * s
        qy = (m01 + m10) / s
        qz = (m02 + m20) / s
    elif m11 > m22:
        s = math.sqrt(1.0 + m11 - m00 - m22) * 2.0
        qw = (m02 - m20) / s
        qx = (m01 + m10) / s
        qy = 0.25 * s
        qz = (m12 + m21) / s
    else:
        s = math.sqrt(1.0 + m22 - m00 - m11) * 2.0
        qw = (m10 - m01) / s
        qx = (m02 + m20) / s
        qy = (m12 + m21) / s
        qz = 0.25 * s

    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    return [qx / norm, qy / norm, qz / norm, qw / norm]


def _fmt(value):
    return f"{value:.9g}"


def _static_transform_node(name, parent_frame, child_frame, translation, quaternion, use_sim_time):
    qx, qy, qz, qw = quaternion
    x, y, z = translation

    return Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name=name,
        arguments=[
            "--x", _fmt(x),
            "--y", _fmt(y),
            "--z", _fmt(z),
            "--qx", _fmt(qx),
            "--qy", _fmt(qy),
            "--qz", _fmt(qz),
            "--qw", _fmt(qw),
            "--frame-id", parent_frame,
            "--child-frame-id", child_frame,
        ],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )


def _resolve_config_path(raw_path, pkg):
    path = Path(raw_path)
    if path.is_absolute():
        return path

    package_relative = pkg / path
    if package_relative.exists():
        return package_relative

    return pkg / "cfg" / path


def _sanitize_node_name(value):
    return "".join(ch if ch.isalnum() else "_" for ch in value).strip("_") or "frame"


def _launch_setup(context, *args, **kwargs):
    pkg = Path(get_package_share_directory("direct_lidar_inertial_odometry"))
    params_path = _resolve_config_path(LaunchConfiguration("params_config").perform(context), pkg)
    dlio_path = _resolve_config_path(LaunchConfiguration("dlio_config").perform(context), pkg)

    params = _ros_parameters(params_path)
    dlio = _ros_parameters(dlio_path)

    base_frame = params.get("frames/baselink", "base_link")
    imu_frame = params.get("frames/imu", "imu")
    lidar_frame = params.get("frames/lidar", "hesai_lidar")

    use_rviz = LaunchConfiguration("use_rviz")
    use_sim_time = LaunchConfiguration("use_sim_time")

    imu_tf = _static_transform_node(
        f"{_sanitize_node_name(base_frame)}_to_{_sanitize_node_name(imu_frame)}",
        base_frame,
        imu_frame,
        _vector(dlio, "extrinsics/baselink2imu/t", 3),
        _quaternion_from_matrix(_vector(dlio, "extrinsics/baselink2imu/R", 9)),
        use_sim_time,
    )
    lidar_tf = _static_transform_node(
        f"{_sanitize_node_name(base_frame)}_to_{_sanitize_node_name(lidar_frame)}",
        base_frame,
        lidar_frame,
        _vector(dlio, "extrinsics/baselink2lidar/t", 3),
        _quaternion_from_matrix(_vector(dlio, "extrinsics/baselink2lidar/R", 9)),
        use_sim_time,
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="check_tf_rviz",
        arguments=["-d", str(pkg / "launch" / "check_tf.rviz")],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(use_rviz),
    )

    return [imu_tf, lidar_tf, rviz]


def generate_launch_description():
    pkg = Path(get_package_share_directory("direct_lidar_inertial_odometry"))

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz2 with the D-LIO check_tf config.",
        ),
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use simulated time for visualization nodes.",
        ),
        DeclareLaunchArgument(
            "dlio_config",
            default_value=str(pkg / "cfg" / "dlio.yaml"),
            description="D-LIO calibration/intrinsics parameter file.",
        ),
        DeclareLaunchArgument(
            "params_config",
            default_value=str(pkg / "cfg" / "params.yaml"),
            description="D-LIO runtime/frame parameter file.",
        ),
        OpaqueFunction(function=_launch_setup),
    ])
