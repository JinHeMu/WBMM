#!/usr/bin/env python3
"""MuJoCo + wheel/IMU EKF + slam_toolbox mapping pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tracer_jaka_mujoco")
    bridge_launch = os.path.join(share, "launch", "bridge.launch.py")
    ekf_config = os.path.join(share, "config", "ekf.yaml")
    slam_config = os.path.join(share, "config", "slam_toolbox.yaml")
    rviz_config = os.path.join(share, "rviz", "slam.rviz")

    viewer = LaunchConfiguration("viewer")
    rviz = LaunchConfiguration("rviz")
    camera = LaunchConfiguration("camera")
    camera_rate = LaunchConfiguration("camera_rate")

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("camera_rate", default_value="30.0"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={
                "viewer": viewer,
                "camera": camera,
                "camera_rate": camera_rate,
            }.items(),
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
        ),

        # Let the bridge, EKF and slam_toolbox establish the complete TF tree
        # before RViz starts requesting transforms at interpolated sim time.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="slam_rviz",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": True}],
                    condition=IfCondition(rviz),
                ),
            ],
        ),
    ])
