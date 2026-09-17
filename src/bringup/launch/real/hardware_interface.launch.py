#!/usr/bin/env python3
"""Pure real-robot hardware-interface bringup for Tracer + JAKA.

Owns the real drivers, robot_state_publisher and the single ros2_control
``controller_manager`` process. Deployment launches decide which controllers
are spawned and whether OCS2 / MoveIt / REMANI consumes the state.

Safety:
  ``hardware_write`` is the only real-motion gate. ``true`` allows the JAKA
  hardware interface to write commands. ``false`` keeps it in telemetry-only
  mode. This launch itself does not create motion targets.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    description_share = get_package_share_directory("tracer_jaka_description")
    hipnuc_share = get_package_share_directory("hipnuc_imu")
    jaka_controllers = os.path.join(
        description_share, "config", "ros2_controllers.yaml")
    urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")
    jaka_urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.controlled.urdf.xacro")

    start_base = LaunchConfiguration("start_base")
    start_rsp = LaunchConfiguration("start_robot_state_publisher")
    start_arm_pose = LaunchConfiguration("start_arm_pose")
    start_imu = LaunchConfiguration("start_imu")
    start_lidar = LaunchConfiguration("start_lidar")
    start_jaka_hardware = LaunchConfiguration("start_jaka_hardware")
    start_jaka_fts = LaunchConfiguration("start_jaka_fts")
    start_arm_controller = LaunchConfiguration("start_arm_controller")
    arm_controller_name = LaunchConfiguration("arm_controller_name")
    controller_manager_timeout = LaunchConfiguration(
        "controller_manager_timeout")
    hardware_write = LaunchConfiguration("hardware_write")

    can_port = LaunchConfiguration("can_port")
    serial_port = LaunchConfiguration("serial_port")
    wheel_odom_topic = LaunchConfiguration("wheel_odom_topic")
    publish_odom_tf = LaunchConfiguration("publish_odom_tf")
    imu_topic = LaunchConfiguration("imu_topic")
    scan_topic = LaunchConfiguration("scan_topic")
    lidar_host_ip = LaunchConfiguration("lidar_host_ip")
    lidar_sensor_ip = LaunchConfiguration("lidar_sensor_ip")
    lidar_port = LaunchConfiguration("lidar_port")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_offset = LaunchConfiguration("lidar_angle_offset")
    configure_lidar = LaunchConfiguration("configure_lidar")
    jaka_robot_ip = LaunchConfiguration("jaka_robot_ip")
    jaka_local_ip = LaunchConfiguration("jaka_local_ip")
    hipnuc_config = os.path.join(hipnuc_share, "config", "hipnuc_config.yaml")

    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", urdf_file]),
            value_type=str,
        )
    }

    jaka_robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", jaka_urdf_file, " ",
                "control_backend:=real ",
                "robot_ip:=", jaka_robot_ip, " ",
                "local_ip:=", jaka_local_ip, " ",
                "hardware_write:=", hardware_write,
            ]),
            value_type=str,
        )
    }

    jaka_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        # Keep the executable's default node name.  A __node remap here is
        # inherited by dynamically loaded controllers and would rename every
        # controller to "controller_manager", preventing their named sections
        # in ros2_controllers.yaml from being applied.
        output="screen",
        parameters=[
            jaka_robot_description,
            jaka_controllers,
            {"use_sim_time": False},
        ],
        remappings=[
            ("/controller_manager/wrench", "/fts_broadcaster/wrench"),
        ],
        condition=IfCondition(start_jaka_hardware),
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
        ],
        condition=IfCondition(start_jaka_hardware),
    )

    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="arm_controller_spawner",
        output="screen",
        arguments=[
            arm_controller_name,
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
        ],
        condition=IfCondition(PythonExpression([
            "'", start_jaka_hardware, "'.lower() == 'true' and '",
            start_arm_controller, "'.lower() == 'true'",
        ])),
    )

    fts_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="fts_broadcaster_spawner",
        output="screen",
        arguments=[
            "fts_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout", controller_manager_timeout,
            "--param-file", jaka_controllers,
        ],
        condition=IfCondition(PythonExpression([
            "'", start_jaka_hardware, "'.lower() == 'true' and '",
            start_jaka_fts, "'.lower() == 'true'",
        ])),
    )

    joint_state_broadcaster_delayed = TimerAction(
        period=2.0,
        actions=[joint_state_broadcaster],
        condition=IfCondition(start_jaka_hardware),
    )

    # Serialize controller loading: joint_state -> optional arm controller ->
    # fts. When no arm controller is requested, load fts directly after JSB.
    spawn_arm_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[arm_controller],
        ),
        condition=IfCondition(PythonExpression([
            "'", start_jaka_hardware, "'.lower() == 'true' and '",
            start_arm_controller, "'.lower() == 'true'",
        ])),
    )
    spawn_fts_after_arm = RegisterEventHandler(
        OnProcessExit(
            target_action=arm_controller,
            on_exit=[fts_broadcaster],
        ),
        condition=IfCondition(PythonExpression([
            "'", start_jaka_hardware, "'.lower() == 'true' and '",
            start_arm_controller, "'.lower() == 'true' and '",
            start_jaka_fts, "'.lower() == 'true'",
        ])),
    )
    spawn_fts_after_jsb = RegisterEventHandler(
        OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[fts_broadcaster],
        ),
        condition=IfCondition(PythonExpression([
            "'", start_jaka_hardware, "'.lower() == 'true' and '",
            start_arm_controller, "'.lower() != 'true' and '",
            start_jaka_fts, "'.lower() == 'true'",
        ])),
    )

    return LaunchDescription([
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument(
            "start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_arm_pose", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument(
            "start_jaka_hardware", default_value="true",
            description="Start JAKA ros2_control."),
        DeclareLaunchArgument(
            "start_jaka_fts", default_value="true",
            description="Spawn the JAKA force/torque broadcaster."),
        DeclareLaunchArgument(
            "start_arm_controller", default_value="false",
            description=(
                "Spawn an arm controller after joint_state_broadcaster. "
                "Deployment launches choose forward or trajectory controller.")),
        DeclareLaunchArgument(
            "arm_controller_name", default_value="arm_controller",
            description="Controller name to spawn when start_arm_controller."),
        DeclareLaunchArgument(
            "controller_manager_timeout", default_value="30.0"),
        DeclareLaunchArgument(
            "hardware_write", default_value="false",
            description=(
                "Only real-motion gate: true allows JAKA writes, false keeps "
                "JAKA telemetry-only.")),
        DeclareLaunchArgument("jaka_robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("jaka_local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument("can_port", default_value="can0"),
        DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("wheel_odom_topic", default_value="/odom"),
        DeclareLaunchArgument(
            "publish_odom_tf", default_value="false",
            description=(
                "Publish odom -> base_footprint from tracer_base. Keep false "
                "when robot_localization owns that transform.")),
        DeclareLaunchArgument("imu_topic", default_value="/IMU_data"),
        DeclareLaunchArgument("scan_topic", default_value="/scan"),
        DeclareLaunchArgument("lidar_host_ip", default_value="0.0.0.0"),
        DeclareLaunchArgument(
            "lidar_sensor_ip", default_value="192.168.198.2"),
        DeclareLaunchArgument("lidar_port", default_value="2368"),
        DeclareLaunchArgument("lidar_inverted", default_value="false"),
        DeclareLaunchArgument("lidar_angle_offset", default_value="0"),
        DeclareLaunchArgument(
            "configure_lidar", default_value="false",
            description=(
                "Apply 30 Hz / filter=3 / 45..315 deg lidar settings via "
                "HTTP. Keep false unless the lidar needs reconfiguration.")),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
            condition=IfCondition(start_rsp),
        ),
        Node(
            package="tracer_jaka_bringup",
            executable="arm_pose_publisher.py",
            name="arm_pose_publisher",
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(start_arm_pose),
        ),
        jaka_controller_manager,
        joint_state_broadcaster_delayed,
        spawn_arm_after_jsb,
        spawn_fts_after_arm,
        spawn_fts_after_jsb,
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
                "publish_odom_tf": ParameterValue(
                    publish_odom_tf, value_type=bool),
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
    ])
