#!/usr/bin/env python3
"""Run the real robot with OCS2 end-effector holding.

The intended command path is:

    external base teleop -> /cmd_vel -> tracer_base_node
    OCS2 MRT             -> /arm_controller/commands -> JAKA arm

The OCS2 base command is remapped to a sink in this deployment.  This keeps
the external base teleop as the only base command owner while OCS2 observes
the measured base motion and compensates with the arm.

Safety defaults are fail-closed:

    hardware_write:=false
    initial_task_phase:=0

``hardware_write`` is passed to both the JAKA hardware interface and the OCS2
MRT output gate.  Set it to true only after the real-robot state, TF and
controller checks have been completed manually.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _include(path, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    bringup_share = get_package_share_directory("tracer_jaka_bringup")
    description_share = get_package_share_directory("tracer_jaka_description")

    hardware_launch = os.path.join(
        bringup_share, "launch", "wbmm_hardware_interface.launch.py")
    ocs2_launch = os.path.join(
        bringup_share, "launch", "ocs2.launch.py")

    default_base_config = os.path.join(
        bringup_share, "config", "common", "ocs2.yaml")
    default_task_file = os.path.join(
        bringup_share, "config", "real", "task.info")
    default_urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")

    hardware = _include(hardware_launch, {
        "hardware_write": LaunchConfiguration("hardware_write"),
        "jaka_robot_ip": LaunchConfiguration("robot_ip"),
        "jaka_local_ip": LaunchConfiguration("local_ip"),
        "can_port": LaunchConfiguration("can_port"),
        "serial_port": LaunchConfiguration("serial_port"),
        "start_base": LaunchConfiguration("start_base"),
        "start_robot_state_publisher": "true",
        "start_arm_pose": "false",
        "start_imu": LaunchConfiguration("start_imu"),
        "start_lidar": LaunchConfiguration("start_lidar"),
        "start_jaka_hardware": "true",
        "start_jaka_fts": LaunchConfiguration("start_jaka_fts"),
        "start_arm_controller": "true",
        "arm_controller_name": "arm_controller",
        "wheel_odom_topic": LaunchConfiguration("odom_topic"),
        "publish_odom_tf": LaunchConfiguration("publish_odom_tf"),
        "imu_topic": "/imu/data",
        "scan_topic": "/scan",
        "lidar_host_ip": LaunchConfiguration("lidar_host_ip"),
        "lidar_sensor_ip": LaunchConfiguration("lidar_sensor_ip"),
    })

    # Keep /cmd_vel owned by the external base teleop.  The remap is scoped to
    # the OCS2 include, so tracer_base_node still receives /cmd_vel normally.
    ocs2 = GroupAction(actions=[
        SetRemap(
            src="/cmd_vel",
            dst="/ocs2/disabled_base_cmd",
        ),
        _include(ocs2_launch, {
            "base_config_file": LaunchConfiguration("base_config_file"),
            "config_file": LaunchConfiguration("ocs2_config"),
            "task_file": LaunchConfiguration("task_file"),
            "urdf_file": LaunchConfiguration("urdf_file"),
            "lib_folder": LaunchConfiguration("lib_folder"),
            "use_sim_time": "false",
            "use_target": LaunchConfiguration("use_target"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "initial_task_phase": LaunchConfiguration("initial_task_phase"),
            "command_output_enabled": LaunchConfiguration("hardware_write"),
            "odom_topic": LaunchConfiguration("odom_topic"),
        }),
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            "hardware_write",
            default_value="false",
            description=(
                "Real-motion gate. false is telemetry/dry-run; true enables "
                "JAKA writes and OCS2 arm command output.")),
        DeclareLaunchArgument(
            "initial_task_phase",
            default_value="0",
            description=(
                "Initial phase: 0=Navigation, 1=Transition, "
                "2=Execution, 3=Retract.")),
        DeclareLaunchArgument(
            "use_target",
            default_value="true",
            description="Start the OCS2 interactive end-effector target node."),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start the OCS2 RViz configuration."),
        DeclareLaunchArgument(
            "lib_folder",
            default_value="/tmp/wbmm_ocs2_real_ee_hold",
            description="Directory for generated OCS2 MPC/MRT libraries."),
        DeclareLaunchArgument(
            "base_config_file",
            default_value=default_base_config,
            description="Common OCS2 ROS parameter file."),
        DeclareLaunchArgument(
            "ocs2_config",
            default_value="",
            description="Optional OCS2 override; empty uses common defaults."),
        DeclareLaunchArgument(
            "task_file",
            default_value=default_task_file,
            description="Real OCS2 task.info file."),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=default_urdf_file,
            description="Canonical robot URDF used by OCS2."),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument(
            "odom_topic",
            default_value="/wheel/odometry",
            description=(
                "Odometry consumed by OCS2. Use /odometry/filtered only "
                "when an EKF is started separately.")),
        DeclareLaunchArgument(
            "publish_odom_tf",
            default_value="true",
            description=(
                "Publish odom -> base_footprint from tracer_base. Set false "
                "when robot_localization owns this TF.")),
        DeclareLaunchArgument(
            "start_base",
            default_value="true",
            description="Start the Tracer base driver."),
        DeclareLaunchArgument(
            "start_imu",
            default_value="false",
            description="Start the Hipnuc IMU driver."),
        DeclareLaunchArgument(
            "start_lidar",
            default_value="false",
            description="Start the LiDAR driver."),
        DeclareLaunchArgument(
            "start_jaka_fts",
            default_value="false",
            description="Start the real F/T broadcaster."),
        DeclareLaunchArgument("lidar_host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument(
            "lidar_sensor_ip", default_value="192.168.198.2"),
        hardware,
        ocs2,
    ])
