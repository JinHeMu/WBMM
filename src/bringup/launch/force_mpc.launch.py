#!/usr/bin/env python3
"""Integrated F/T -> admittance -> EE reference -> OCS2 execution.

Uses the existing WBMM hardware backends. Simulation can use a zero-force
C++ virtual sensor; real execution defaults to telemetry-only and disabled
admittance. All dynamics and calibration values remain in YAML files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
    OpaqueFunction, RegisterEventHandler, TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _value(context, name):
    return LaunchConfiguration(name).perform(context).strip()


def _bool(value):
    return value.lower() in ("true", "1", "yes", "on")


def _merge(base, override):
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(base.get(key), dict):
            _merge(base[key], value)
        else:
            base[key] = value


def _make_nodes(context):
    bringup = get_package_share_directory("tracer_jaka_bringup")
    description = get_package_share_directory("tracer_jaka_description")
    backend = _value(context, "backend")
    sim = backend == "sim"
    hardware_write = _bool(_value(context, "hardware_write"))
    output_enabled = sim or hardware_write
    enable = _value(context, "admittance.enable")
    enable = sim if enable == "auto" else _bool(enable)
    fake_wrench = _bool(_value(context, "fake_wrench"))
    keyboard_wrench = _bool(_value(context, "keyboard_wrench"))
    if keyboard_wrench and (not sim or not fake_wrench):
        raise RuntimeError("keyboard_wrench requires backend:=sim fake_wrench:=true")
    if fake_wrench and not sim:
        raise RuntimeError("fake_wrench is available only with backend:=sim")

    force_profile = _value(context, "force_params_file")
    calibration = _value(context, "calibration_file")
    if not calibration and not sim:
        calibration = os.path.join(bringup, "config", "real", "force_mpc_calibration.yaml")
    if sim and calibration:
        raise RuntimeError("Real payload calibration must not be applied to simulation")

    force_layers = [
        os.path.join(bringup, "config", "common", "force_control.yaml"),
        os.path.join(bringup, "config", backend, "force_control.yaml"),
        force_profile,
    ]
    for path in [*force_layers, *([calibration] if calibration else [])]:
        if not os.path.isfile(path):
            raise RuntimeError(f"Force parameter file does not exist: {path!r}")

    # Read only interface names; numerical control configuration is consumed
    # directly by the C++ nodes using the same ordered parameter-file layers.
    control = {}
    sensor = {}
    for path in force_layers:
        with open(path, encoding="utf-8") as stream:
            values = yaml.safe_load(stream) or {}
        _merge(control, values.get("whole_body_force_control", {}).get("ros__parameters", {}))
        _merge(sensor, values.get("force_sensor_processor", {}).get("ros__parameters", {}))
    state_frame = control.get("state_frame", "odom")
    if state_frame != "odom":
        raise RuntimeError("force_mpc uses wheel odometry; state_frame must be odom")
    tcp_frame = control.get("force_sensor", {}).get("tcp_frame", "tool0")
    state_topic = control.get("topics", {}).get("states", "/whole_body_force_control/states")
    robot_name = control.get("robot_name", "mobile_manipulator")
    urdf = os.path.join(description, "urdf", "tracer_jaka_zu5.urdf")
    task = _value(context, "task_file") or os.path.join(bringup, "config", backend, "task.info")
    if not os.path.isfile(task):
        raise RuntimeError(f"OCS2 task file does not exist: {task!r}")
    ocs2_layers = [os.path.join(bringup, "config", "common", "ocs2.yaml")]
    if sim:
        ocs2_layers.append(os.path.join(bringup, "config", "sim", "ocs2.yaml"))
    common = {"taskFile": task, "urdfFile": urdf, "use_sim_time": sim,
              "world_frame": state_frame, "initial_task_phase": 2, "robot_name": robot_name}
    library_root = _value(context, "lib_folder")
    mrt_force_layers = [force_profile]
    if sim:
        mrt_force_layers.append(os.path.join(bringup, "config", "sim", "force_mpc.yaml"))
    sensor_overrides = {
        "use_sim_time": sim,
        "force_sensor.tcp_frame": tcp_frame,
        "force_sensor.require_stamped_wrench": not sim,
    }
    tare = _value(context, "tare")
    if tare != "auto":
        sensor_overrides["force_sensor.tare_after_compensation"] = _bool(tare)
        sensor_overrides["force_sensor.tare_on_start"] = _bool(tare)
    raw_topic = _value(context, "raw_wrench_topic")
    if raw_topic:
        sensor_overrides["topics.raw_wrench"] = raw_topic
    elif sim:
        sensor_overrides["topics.raw_wrench"] = (
            "/whole_body_force_control/fake_wrench" if fake_wrench
            else "/fts_broadcaster/wrench_raw")
    nodes = [
        Node(package="wbmm_ocs2_ros", executable="wbmm_mpc_node", name="wbmm_mpc_node",
             output="screen", parameters=[*ocs2_layers, {
                 **common, "libFolder": os.path.join(library_root, "mpc"),
                 "ee_target_topic": control.get("topics", {}).get("ee_target", "/mobile_manipulator_ee_target"),
             }]),
        Node(package="wbmm_ocs2_ros", executable="wbmm_mrt_node", name="wbmm_mrt_node",
             output="screen", parameters=[*ocs2_layers, *mrt_force_layers, {
                 **common, "libFolder": os.path.join(library_root, "mrt"),
                 "ee_frame": tcp_frame, "odom_topic": "/wheel/odometry",
                 "command_output_enabled": output_enabled,
                 "force_gate.require_active": True, "force_gate.state_topic": state_topic,
             }]),
        Node(package="whole_body_force_control", executable="force_sensor_processor_node",
             name="force_sensor_processor", output="screen",
             parameters=[*force_layers, *([calibration] if calibration else []), sensor_overrides]),
        Node(package="whole_body_force_control", executable="whole_body_force_control_node",
             name="whole_body_force_control", output="screen",
             parameters=[*force_layers, {
                 "urdf_file": urdf, "use_sim_time": sim,
                 "admittance.enable": enable, "admittance.output": output_enabled,
             }]),
    ]
    if fake_wrench:
        nodes.append(Node(package="whole_body_force_control", executable="virtual_wrench_node",
                          name="virtual_force_publisher", output="screen",
                          parameters=[force_profile, {
                              "frame_id": sensor.get("force_sensor", {}).get("sensor_frame", "jk_se_vi_200_link"),
                              "topic": sensor_overrides["topics.raw_wrench"],
                              "command_frame": tcp_frame,
                          }]))
    if keyboard_wrench:
        nodes.append(Node(package="whole_body_force_control", executable="keyboard_wrench_node",
                          name="keyboard_force_publisher", output="screen",
                          parameters=[force_profile, {"frame_id": tcp_frame}]))
    actions = []
    if _bool(_value(context, "start_backend")):
        if sim:
            filename = "mujoco_hardware_interface.launch.py"
            args = {"scene": "force_follow_infinite", "viewer": _value(context, "viewer"),
                    "arm_servo_config": _value(context, "arm_servo_config"),
                    "arm_bias_compensation": _value(context, "arm_bias_compensation"),
                    "publish_odom_tf": "true", "start_fts": str(not fake_wrench).lower(),
                    "start_camera": "false", "start_lidar": "false", "start_imu": "false"}
        else:
            filename = "wbmm_hardware_interface.launch.py"
            args = {"hardware_write": str(hardware_write).lower(),
                    "start_arm_controller": str(hardware_write).lower(),
                    "start_arm_pose": "false", "start_jaka_fts": "true",
                    "torque_sensor_mode": "1", "publish_odom_tf": "true",
                    "start_lidar": "false", "start_imu": "false",
                    "jaka_robot_ip": _value(context, "jaka_robot_ip"),
                    "jaka_local_ip": _value(context, "jaka_local_ip"),
                    "can_port": _value(context, "can_port")}
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(bringup, "launch", filename)),
            launch_arguments=args.items()))
    if _bool(_value(context, "use_rviz")):
        nodes.append(Node(package="rviz2", executable="rviz2", output="screen",
                          arguments=["-d", os.path.join(bringup, "rviz", "whole_body_force_control_sim.rviz")],
                          parameters=[{"use_sim_time": sim}]))
    # Give the surviving MRT heartbeat watchdog time to publish its stop/hold
    # before shutting down the backend after a force or MPC process exits.
    for node in nodes[:4]:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=node,
            on_exit=[TimerAction(period=0.5, actions=[EmitEvent(event=Shutdown(
                reason="A force/MPC control process exited"))])],
        )))
    return [*actions, *nodes]


def generate_launch_description():
    bringup = get_package_share_directory("tracer_jaka_bringup")
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="sim", choices=["sim", "real"]),
        DeclareLaunchArgument("start_backend", default_value="true"),
        DeclareLaunchArgument("hardware_write", default_value="false"),
        DeclareLaunchArgument("admittance.enable", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("tare", default_value="auto", choices=["auto", "true", "false"]),
        DeclareLaunchArgument("fake_wrench", default_value="false"),
        DeclareLaunchArgument("keyboard_wrench", default_value="false"),
        DeclareLaunchArgument("raw_wrench_topic", default_value=""),
        DeclareLaunchArgument("calibration_file", default_value=""),
        DeclareLaunchArgument("force_params_file", default_value=os.path.join(
            bringup, "config", "common", "force_mpc.yaml")),
        DeclareLaunchArgument("task_file", default_value=""),
        DeclareLaunchArgument("lib_folder", default_value="/tmp/wbmm_force_mpc/auto_generated"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("arm_servo_config", default_value=os.path.join(
            get_package_share_directory("tracer_jaka_mujoco"), "config", "arm_servo.yaml")),
        DeclareLaunchArgument("arm_bias_compensation", default_value="auto",
                             choices=["auto", "true", "false"]),
        DeclareLaunchArgument("jaka_robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("jaka_local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        OpaqueFunction(function=_make_nodes),
    ])
