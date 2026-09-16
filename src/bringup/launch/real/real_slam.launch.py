#!/usr/bin/env python3
"""Real Tracer + Hipnuc + Lakibeam + EKF + optional slam_toolbox.

This is a functional composition of ``hardware_interface.launch.py`` and
``localization.launch.py``. It does not start OCS2, REMANI, AMCL, or motion
command publishers.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("tracer_jaka_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_arm_pose", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument(
            "start_jaka_hardware", default_value="false",
            description=(
                "Start JAKA ros2_control in read-only telemetry mode.")),
        DeclareLaunchArgument("start_jaka_fts", default_value="true"),
        DeclareLaunchArgument("jaka_robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("jaka_local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("imu_topic", default_value="/IMU_data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("lidar_host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument(
            "lidar_sensor_ip", default_value="192.168.198.2"),
        DeclareLaunchArgument("lidar_port", default_value="2368"),
        DeclareLaunchArgument("lidar_inverted", default_value="false"),
        DeclareLaunchArgument("lidar_angle_offset", default_value="0"),
        DeclareLaunchArgument("configure_lidar", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, "launch", "hardware_interface.launch.py")),
            launch_arguments={
                "start_base": LaunchConfiguration("start_base"),
                "start_robot_state_publisher": LaunchConfiguration(
                    "start_robot_state_publisher"),
                "start_arm_pose": LaunchConfiguration("start_arm_pose"),
                "start_imu": LaunchConfiguration("start_imu"),
                "start_lidar": LaunchConfiguration("start_lidar"),
                "start_jaka_hardware": LaunchConfiguration(
                    "start_jaka_hardware"),
                "start_jaka_fts": LaunchConfiguration("start_jaka_fts"),
                "jaka_robot_ip": LaunchConfiguration("jaka_robot_ip"),
                "jaka_local_ip": LaunchConfiguration("jaka_local_ip"),
                "can_port": LaunchConfiguration("can_port"),
                "serial_port": LaunchConfiguration("serial_port"),
                "publish_odom_tf": "false",
                "wheel_odom_topic": LaunchConfiguration(
                    "wheel_odom_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
                "lidar_host_ip": LaunchConfiguration("lidar_host_ip"),
                "lidar_sensor_ip": LaunchConfiguration("lidar_sensor_ip"),
                "lidar_port": LaunchConfiguration("lidar_port"),
                "lidar_inverted": LaunchConfiguration("lidar_inverted"),
                "lidar_angle_offset": LaunchConfiguration(
                    "lidar_angle_offset"),
                "configure_lidar": LaunchConfiguration("configure_lidar"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, "launch", "localization.launch.py")),
            launch_arguments={
                "start_ekf": "true",
                "start_slam": LaunchConfiguration("start_slam"),
                "use_sim_time": "false",
                "ekf_config": os.path.join(
                    bringup_share, "config", "real", "ekf_real.yaml"),
                "slam_config": os.path.join(
                    bringup_share, "config", "real", "slam_toolbox_real.yaml"),
                "wheel_odom_topic": LaunchConfiguration("wheel_odom_topic"),
                "imu_topic": LaunchConfiguration("imu_topic"),
                "scan_topic": LaunchConfiguration("scan_topic"),
            }.items(),
        ),
        TimerAction(
            period=4.0,
            actions=[Node(
                package="rviz2",
                executable="rviz2",
                name="slam_rviz",
                output="screen",
                arguments=["-d", os.path.join(
                    bringup_share, "rviz", "slam.rviz")],
                parameters=[{"use_sim_time": False}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )],
        ),
    ])
