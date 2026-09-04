#!/usr/bin/env python3
"""Real Tracer + Hipnuc IMU + Lakibeam + EKF + slam_toolbox."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory("tracer_jaka_bringup")
    description_share = get_package_share_directory("tracer_jaka_description")
    hipnuc_share = get_package_share_directory("hipnuc_imu")

    ekf_config = os.path.join(bringup_share, "config", "ekf_real.yaml")
    slam_config = os.path.join(
        bringup_share, "config", "slam_toolbox_real.yaml")
    hipnuc_config = os.path.join(hipnuc_share, "config", "hipnuc_config.yaml")
    jaka_controllers = os.path.join(
        description_share, "config", "ros2_controllers.yaml")
    jaka_fts_params = os.path.join(
        description_share, "config", "fts_broadcaster_humble.yaml")
    # The calibrated tracer_jaka_zu5.urdf is the sensor/TF source of truth for
    # both MuJoCo and the physical robot. Keep the hardware-control-only URDF
    # variant out of this mapping launch so sensor poses cannot drift apart.
    urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")
    jaka_urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.controlled.urdf.xacro")
    rviz_config = os.path.join(bringup_share, "rviz", "slam.rviz")

    start_base = LaunchConfiguration("start_base")
    start_slam = LaunchConfiguration("start_slam")
    start_rsp = LaunchConfiguration("start_robot_state_publisher")
    start_arm_pose = LaunchConfiguration("start_arm_pose")
    start_imu = LaunchConfiguration("start_imu")
    start_lidar = LaunchConfiguration("start_lidar")
    start_jaka_hardware = LaunchConfiguration("start_jaka_hardware")
    start_jaka_fts = LaunchConfiguration("start_jaka_fts")
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
    jaka_robot_ip = LaunchConfiguration("jaka_robot_ip")
    jaka_local_ip = LaunchConfiguration("jaka_local_ip")

    args = [
        DeclareLaunchArgument("start_base", default_value="true"),
        DeclareLaunchArgument(
            "start_slam",
            default_value="true",
            description=(
                "Start slam_toolbox in mapping mode and publish map->odom. "
                "Set false when a separate AMCL/localization node provides "
                "map->odom.")),
        DeclareLaunchArgument("start_robot_state_publisher", default_value="true"),
        DeclareLaunchArgument("start_arm_pose", default_value="true"),
        DeclareLaunchArgument("start_imu", default_value="true"),
        DeclareLaunchArgument("start_lidar", default_value="true"),
        DeclareLaunchArgument(
            "start_jaka_hardware",
            default_value="false",
            description=(
                "Start the real JAKA ros2_control hardware interface in "
                "read-only mode and publish actual joint/FT states.")),
        DeclareLaunchArgument(
            "start_jaka_fts",
            default_value="true",
            description="Publish the JAKA force/torque sensor state."),
        DeclareLaunchArgument("jaka_robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("jaka_local_ip", default_value="10.5.5.127"),
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
            default_value="false",
            description="Apply 30Hz, filter=3 and 45..315 degree settings by HTTP (set to true if lidar needs reconfiguration)",
        ),
    ]

    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", urdf_file,
            ]),
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
                "jaka_read_only:=true",
            ]),
            value_type=str,
        )
    }

    jaka_controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        output="screen",
        parameters=[
            jaka_robot_description,
            jaka_controllers,
            {"use_sim_time": False},
        ],
        # In the Humble controller_manager the in-process FTS controller uses
        # the manager node name, so its private wrench topic resolves to
        # /controller_manager/wrench.  Keep the project's public topic stable.
        remappings=[
            ("/controller_manager/wrench", "/fts_broadcaster/wrench"),
        ],
        condition=IfCondition(start_jaka_hardware),
    )

    jaka_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="jaka_joint_state_broadcaster_spawner",
        output="screen",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    fts_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        name="fts_broadcaster_spawner",
        output="screen",
        arguments=[
            "fts_broadcaster",
            "--controller-manager",
            "/controller_manager",
            # frame_id is a required parameter of the Humble FTS broadcaster.
            # Controller-specific sections in the manager's parameter file are
            # not forwarded reliably while loading a controller, so give the
            # spawner the file explicitly.
            "--param-file",
            jaka_fts_params,
        ],
        condition=IfCondition(start_jaka_fts),
    )

    jaka_state_broadcaster_delayed = TimerAction(
        period=2.0,
        actions=[jaka_state_broadcaster],
        condition=IfCondition(start_jaka_hardware),
    )

    # Loading both controllers concurrently can race inside the Humble
    # controller_manager.  Load the FTS broadcaster only after the joint-state
    # spawner has completed.
    fts_after_joint_state = RegisterEventHandler(
        OnProcessExit(
            target_action=jaka_state_broadcaster,
            on_exit=[fts_broadcaster],
        ),
        condition=IfCondition(start_jaka_hardware),
    )

    nodes = [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": False}],
            condition=IfCondition(start_rsp),
        ),
        # Publish a default arm-up joint state so the full TF tree (including
        # every link of the JAKA arm) is available for RViz and slam_toolbox.
        # On the real robot the JAKA driver publishes the actual joint states
        # and this node can be stopped with start_arm_pose:=false.
        Node(
            package="tracer_jaka_bringup",
            executable="arm_pose_publisher.py",
            name="arm_pose_publisher",
            parameters=[{"use_sim_time": False}],
            condition=IfCondition(start_arm_pose),
        ),
        jaka_controller_manager,
        jaka_state_broadcaster_delayed,
        fts_after_joint_state,
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
        # Delay EKF 2 s so base + IMU drivers are already publishing data.
        TimerAction(
            period=2.0,
            actions=[
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
            ],
        ),
        # Delay SLAM 4 s so EKF has time to converge and publish odom TF.
        TimerAction(
            period=4.0,
            actions=[
                Node(
                    package="slam_toolbox",
                    executable="async_slam_toolbox_node",
                    name="slam_toolbox",
                    output="screen",
                    parameters=[slam_config],
                    remappings=[("/scan", scan_topic)],
                    condition=IfCondition(start_slam),
                ),
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
