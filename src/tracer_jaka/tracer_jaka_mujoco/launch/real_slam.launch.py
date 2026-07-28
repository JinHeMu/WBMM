#!/usr/bin/env python3
"""Real Tracer + Hipnuc IMU + Lakibeam + EKF + slam_toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("tracer_jaka_mujoco")
    hipnuc_share = get_package_share_directory("hipnuc_imu")

    ekf_config = os.path.join(share, "config", "ekf_real.yaml")
    slam_config = os.path.join(share, "config", "slam_toolbox_real.yaml")
    hipnuc_config = os.path.join(hipnuc_share, "config", "hipnuc_config.yaml")
    # The calibrated tracer_jaka_zu5.urdf is the sensor/TF source of truth for
    # both MuJoCo and the physical robot. Keep the hardware-control-only URDF
    # variant out of this mapping launch so sensor poses cannot drift apart.
    urdf_file = os.path.join(share, "urdf", "tracer_jaka_zu5.urdf")
    rviz_config = os.path.join(share, "rviz", "slam.rviz")

    start_base = LaunchConfiguration("start_base")
    start_rsp = LaunchConfiguration("start_robot_state_publisher")
    start_imu = LaunchConfiguration("start_imu")
    start_lidar = LaunchConfiguration("start_lidar")
    use_rviz = LaunchConfiguration("rviz")

    wheel_odom_topic = LaunchConfiguration("wheel_odom_topic")
    imu_topic = LaunchConfiguration("imu_topic")
    scan_topic = LaunchConfiguration("scan_topic")

    serial_port = LaunchConfiguration("serial_port")
    can_port = LaunchConfiguration("can_port")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_offset = LaunchConfiguration("lidar_angle_offset")
    configure_lidar = LaunchConfiguration("configure_lidar")

    args = [
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("imu_topic", default_value="/IMU_data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("lidar_host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument("lidar_sensor_ip", default_value="192.168.198.2"),
        DeclareLaunchArgument("lidar_port", default_value="2368"),
        DeclareLaunchArgument("lidar_inverted", default_value="false"),
        DeclareLaunchArgument("lidar_angle_offset", default_value="0"),
        DeclareLaunchArgument(
            "configure_lidar",
            default_value="true",
            description="Apply 30Hz, filter=3 and 45..315 degree settings by HTTP",
        ),
    ]

    robot_description = {
        "robot_description": Command([
            FindExecutable(name="xacro"), " ", urdf_file,
        ])
    }

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
            condition=IfCondition(start_rsp),
        ),
        Node(
            package="tracer_base",
            executable="tracer_base_node",
            name="tracer_base_node",
            output="screen",
            parameters=[{
                "port_name": ParameterValue(can_port, value_type=str),
                "odom_frame": "odom",
                "base_frame": "base_footprint",
                "odom_topic_name": ParameterValue(
                    wheel_odom_topic, value_type=str),
                # EKF is the only publisher of odom -> base_footprint.
                "publish_odom_tf": False,
                "is_tracer_mini": False,
                "simulated_robot": False,
                "control_rate": 50,
            }],
            condition=IfCondition(start_base),
        ),
        Node(
            package="hipnuc_imu",
            executable="talker",
            name="IMU_publisher",
            output="screen",
            parameters=[
                hipnuc_config,
                {
                    "serial_port": ParameterValue(serial_port, value_type=str),
                    "frame_id": "imu_link",
                    "imu_topic": ParameterValue(imu_topic, value_type=str),
                },
            ],
            condition=IfCondition(start_imu),
        ),
        Node(
            package="lakibeam1",
            executable="lakibeam1_scan_node",
            name="richbeam_lidar_node0",
            output="screen",
            parameters=[{
                "frame_id": "laser_link",
                "output_topic": ParameterValue(scan_topic, value_type=str),
                "inverted": ParameterValue(lidar_inverted, value_type=bool),
                "hostip": ParameterValue(lidar_host_ip, value_type=str),
                "sensorip": ParameterValue(lidar_sensor_ip, value_type=str),
                "port": ParameterValue(lidar_port, value_type=str),
                "angle_offset": ParameterValue(
                    lidar_angle_offset, value_type=int),
                "scanfreq": "30",
                "filter": "3",
                "laser_enable": "true",
                "scan_range_start": "45",
                "scan_range_stop": "315",
                "configure_sensor": ParameterValue(
                    configure_lidar, value_type=bool),
            }],
            condition=IfCondition(start_lidar),
        ),
        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config],
            remappings=[
                ("/odom", wheel_odom_topic),
                ("/IMU_data", imu_topic),
            ],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
            remappings=[("/scan", scan_topic)],
        ),
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="slam_rviz",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": False}],
                    condition=IfCondition(use_rviz),
                ),
            ],
        ),
    ]

    return LaunchDescription(args + nodes)
