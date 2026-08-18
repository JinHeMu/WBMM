#!/usr/bin/env python3
"""Reusable wheel odometry + IMU + 2D laser mapping stack.

This launch file intentionally does not start MuJoCo.  Use it on the real robot
and remap the three sensor inputs with launch arguments.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("tracer_jaka_mujoco")
    ekf_config = os.path.join(share, "config", "ekf_real.yaml")
    slam_config = os.path.join(share, "config", "slam_toolbox_real.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")
    wheel_odom = LaunchConfiguration("wheel_odom_topic")
    imu = LaunchConfiguration("imu_topic")
    scan = LaunchConfiguration("scan_topic")
    sim_time_param = ParameterValue(use_sim_time, value_type=bool)

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/odom"),
        DeclareLaunchArgument("imu_topic", default_value="/IMU_data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": sim_time_param}],
            remappings=[
                ("/odom", wheel_odom),
                ("/IMU_data", imu),
            ],
        ),
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config, {"use_sim_time": sim_time_param}],
            remappings=[("/scan", scan)],
        ),
    ])
