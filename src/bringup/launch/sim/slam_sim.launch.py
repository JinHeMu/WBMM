#!/usr/bin/env python3
"""Pure MuJoCo + EKF + slam_toolbox mapping composition.

This is a functional composition of ``sim.launch.py`` and
``localization.launch.py``. No OCS2/REMANI nodes are started.
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
    mujoco_share = get_package_share_directory("tracer_jaka_mujoco")

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("camera_rate", default_value="30.0"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument(
            "model",
            default_value=os.path.join(
                mujoco_share, "models", "scene.xml")),
        DeclareLaunchArgument("init_keyframe", default_value="home"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, "launch", "sim.launch.py")),
            launch_arguments={
                "viewer": LaunchConfiguration("viewer"),
                "camera": LaunchConfiguration("camera"),
                "camera_rate": LaunchConfiguration("camera_rate"),
                "camera_width": LaunchConfiguration("camera_width"),
                "camera_height": LaunchConfiguration("camera_height"),
                "model": LaunchConfiguration("model"),
                "init_keyframe": LaunchConfiguration("init_keyframe"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                bringup_share, "launch", "localization.launch.py")),
            launch_arguments={
                "start_ekf": "true",
                "start_slam": "true",
                "use_sim_time": "true",
                "wheel_odom_topic": "/wheel/odometry",
                "imu_topic": "/imu/data",
                "ekf_config": os.path.join(
                    bringup_share, "config", "sim", "ekf_sim.yaml"),
                "slam_config": os.path.join(
                    bringup_share, "config", "sim", "slam_toolbox_sim.yaml"),
            }.items(),
        ),
        TimerAction(
            period=5.0,
            actions=[Node(
                package="rviz2",
                executable="rviz2",
                name="slam_rviz",
                output="screen",
                arguments=["-d", os.path.join(
                    bringup_share, "rviz", "slam.rviz")],
                parameters=[{"use_sim_time": True}],
                condition=IfCondition(LaunchConfiguration("rviz")),
            )],
        ),
    ])
