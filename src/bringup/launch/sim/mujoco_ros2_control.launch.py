#!/usr/bin/env python3
"""
标准 ros2_control 栈（可接 OCS2 / diff_drive_controller / nav2）。

启动:
  ros2 launch tracer_jaka_mujoco ros2_control.launch.py
控制:
  底盘: ros2 topic pub /base_controller/cmd_vel_unstamped geometry_msgs/Twist "{linear:{x:0.3}, angular:{z:0.5}}"
  机械臂: 通过 /arm_controller 发 JointTrajectory，或接 MoveIt
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.substitutions import Command, FindExecutable
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.actions import ExecuteProcess, TimerAction


def generate_launch_description():
    pkg = "tracer_jaka_mujoco"
    share = get_package_share_directory(pkg)
    description_share = get_package_share_directory("tracer_jaka_description")
    urdf = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.controlled.urdf.xacro")
    model = os.path.join(share, "models", "tracer_jaka_zu5_robot.xml")
    ctrl = os.path.join(
        description_share, "config", "ros2_controllers.yaml")

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", urdf, " ",
                "control_backend:=mujoco",
            ]),
            value_type=str,
        )
    }

    rsp = Node(package="robot_state_publisher", executable="robot_state_publisher",
               output="screen",
               parameters=[robot_description, {"use_sim_time": True}])

    mujoco_node = Node(
        package="mujoco_ros2_control", executable="mujoco_ros2_control",
        output="screen",
        parameters=[robot_description, ctrl,
                    {"mujoco_model_path": model},
                    {"use_sim_time": True, "update_rate": 200}])


    jsb = Node(package="controller_manager", executable="spawner",
               arguments=["joint_state_broadcaster", "-c", "/controller_manager"])
    arm = Node(package="controller_manager", executable="spawner",
               arguments=["arm_controller", "-c", "/controller_manager"])
    base = Node(package="controller_manager", executable="spawner",
                arguments=["base_controller", "-c", "/controller_manager"])

    return LaunchDescription([
        rsp, mujoco_node, jsb,
        RegisterEventHandler(OnProcessExit(target_action=jsb, on_exit=[arm, base])),
    ])
