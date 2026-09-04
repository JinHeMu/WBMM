#!/usr/bin/env python3
"""Unified MoveIt bringup for the real JAKA arm and MuJoCo simulation."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _load_yaml(path):
    with open(path, "r", encoding="utf-8") as stream:
        return yaml.safe_load(stream)


def _validate_arguments(context):
    backend = LaunchConfiguration("backend").perform(context).lower()
    allow_execution = LaunchConfiguration(
        "allow_trajectory_execution").perform(context).lower()
    jaka_read_only = LaunchConfiguration("jaka_read_only").perform(
        context).lower()
    use_servo = LaunchConfiguration("use_servo").perform(context).lower()
    use_joy = LaunchConfiguration("use_joy").perform(context).lower()

    if backend not in ("sim", "real"):
        raise RuntimeError(
            f"Unsupported backend '{backend}'. Expected backend:=sim or backend:=real."
        )
    if allow_execution not in ("auto", "true", "false"):
        raise RuntimeError(
            "allow_trajectory_execution must be auto, true, or false."
        )
    if allow_execution == "true" and use_servo == "true":
        raise RuntimeError(
            "MoveIt trajectory execution and MoveIt Servo cannot own the arm "
            "command interface together. Choose use_servo:=false for "
            "trajectories or leave allow_trajectory_execution:=auto for Servo."
        )
    if (backend == "real" and allow_execution == "true" and
            jaka_read_only != "false"):
        raise RuntimeError(
            "Real trajectory execution requires jaka_read_only:=false."
        )

    actions = [LogInfo(msg=f"MoveIt backend selected: {backend}")]
    if backend == "real":
        if use_servo == "true":
            actions.append(LogInfo(msg=(
                "[SAFETY WARNING] REAL Servo backend selected. The safe "
                "default keeps JAKA read-only; real Servo motion requires "
                "jaka_read_only:=false. MoveIt trajectory execution remains "
                "disabled while Servo owns the arm controller."
            )))
        else:
            actions.append(LogInfo(msg=(
                "[SAFETY WARNING] REAL backend selected. The safe defaults "
                "keep JAKA read-only and disable MoveIt trajectory execution. "
                "Real trajectory motion requires both jaka_read_only:=false "
                "and allow_trajectory_execution:=true."
            )))
    if use_joy == "true" and use_servo != "true":
        actions.append(LogInfo(msg=(
            "[CONFIG WARNING] use_joy:=true is ignored unless "
            "use_servo:=true."
        )))
    return actions


def generate_launch_description():
    bringup_share = get_package_share_directory("tracer_jaka_bringup")
    description_share = get_package_share_directory("tracer_jaka_description")
    moveit_share = get_package_share_directory("tracer_jaka_moveit_config")
    mujoco_share = get_package_share_directory("tracer_jaka_mujoco")

    backend = LaunchConfiguration("backend")
    allow_trajectory_execution = LaunchConfiguration(
        "allow_trajectory_execution")
    use_rviz = LaunchConfiguration("use_rviz")
    use_servo = LaunchConfiguration("use_servo")
    use_joy = LaunchConfiguration("use_joy")
    use_gripper = LaunchConfiguration("use_gripper")
    robot_ip = LaunchConfiguration("robot_ip")
    local_ip = LaunchConfiguration("local_ip")
    jaka_read_only = LaunchConfiguration("jaka_read_only")

    is_real = PythonExpression(["'", backend, "'.lower() == 'real'"])
    is_sim = PythonExpression(["'", backend, "'.lower() == 'sim'"])
    servo_and_joy = PythonExpression([
        "'", use_servo, "'.lower() == 'true' and '", use_joy,
        "'.lower() == 'true'",
    ])
    real_gripper = PythonExpression([
        "'", backend, "'.lower() == 'real' and '", use_gripper,
        "'.lower() == 'true'",
    ])
    use_sim_time = ParameterValue(is_sim, value_type=bool)
    resolved_allow_execution = ParameterValue(
        PythonExpression([
            "('", allow_trajectory_execution,
            "'.lower() == 'true') if '", allow_trajectory_execution,
            "'.lower() in ('true', 'false') else ('", backend,
            "'.lower() == 'sim' and '", use_servo,
            "'.lower() != 'true')",
        ]),
        value_type=bool,
    )
    control_backend = PythonExpression([
        "'real' if '", backend, "'.lower() == 'real' else 'mock'",
    ])

    controlled_urdf = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.controlled.urdf.xacro")
    moveit_config = (
        MoveItConfigsBuilder(
            robot_name="tracer_jaka",
            package_name="tracer_jaka_moveit_config",
        )
        .robot_description(
            file_path=controlled_urdf,
            mappings={
                "control_backend": control_backend,
                "robot_ip": robot_ip,
                "local_ip": local_ip,
                "jaka_read_only": jaka_read_only,
            },
        )
        .robot_description_semantic(
            file_path="config/tracer_jaka_zu5.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "use_sim_time": use_sim_time,
                "allow_trajectory_execution": resolved_allow_execution,
            },
        ],
        arguments=["--ros-args", "--log-level", "info"],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        condition=IfCondition(use_rviz),
        arguments=["-d", os.path.join(moveit_share, "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        output="log",
    )

    # The MuJoCo bridge owns simulated state, /joint_states and the canonical
    # FollowJointTrajectory action used by MoveIt. No controller_manager is
    # started in this branch.
    mujoco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mujoco_share, "launch", "bridge.launch.py")),
        condition=IfCondition(is_sim),
        launch_arguments={
            "model": LaunchConfiguration("sim_model"),
            "viewer": LaunchConfiguration("sim_viewer"),
            "camera": LaunchConfiguration("sim_camera"),
            "camera_rate": LaunchConfiguration("sim_camera_rate"),
            "camera_width": LaunchConfiguration("sim_camera_width"),
            "camera_height": LaunchConfiguration("sim_camera_height"),
            "init_keyframe": LaunchConfiguration("sim_init_keyframe"),
            "fts_enable": LaunchConfiguration("sim_fts_enable"),
            "fts_zero_on_start": LaunchConfiguration(
                "sim_fts_zero_on_start"),
        }.items(),
    )

    # The real branch owns robot_state_publisher and ros2_control. Controller
    # spawners are serialized so Humble does not race controller_manager.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        condition=IfCondition(is_real),
        parameters=[moveit_config.robot_description, {"use_sim_time": False}],
        output="screen",
    )
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        name="controller_manager",
        condition=IfCondition(is_real),
        parameters=[
            moveit_config.robot_description,
            os.path.join(
                description_share, "config", "ros2_controllers.yaml"),
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="screen",
    )
    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(is_real),
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
        ],
        output="screen",
    )
    trajectory_controller = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(PythonExpression([
            "'", backend, "'.lower() == 'real' and '", use_servo,
            "'.lower() != 'true'",
        ])),
        arguments=[
            "arm_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
        ],
        output="screen",
    )
    forward_controller = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(PythonExpression([
            "'", backend, "'.lower() == 'real' and '", use_servo,
            "'.lower() == 'true'",
        ])),
        arguments=[
            "arm_controller",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
        ],
        output="screen",
    )
    fts_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        condition=IfCondition(is_real),
        arguments=[
            "fts_broadcaster",
            "--controller-manager", "/controller_manager",
            "--controller-manager-timeout",
            LaunchConfiguration("controller_manager_timeout"),
            "--param-file",
            os.path.join(
                description_share, "config", "fts_broadcaster_humble.yaml"),
        ],
        output="screen",
    )
    spawn_trajectory_after_jsb = RegisterEventHandler(
        condition=IfCondition(is_real),
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster,
            on_exit=[trajectory_controller, forward_controller],
        ),
    )
    spawn_fts_after_arm = RegisterEventHandler(
        condition=IfCondition(is_real),
        event_handler=OnProcessExit(
            target_action=trajectory_controller,
            on_exit=[fts_broadcaster],
        ),
    )
    spawn_fts_after_servo_arm = RegisterEventHandler(
        condition=IfCondition(is_real),
        event_handler=OnProcessExit(
            target_action=forward_controller,
            on_exit=[fts_broadcaster],
        ),
    )

    servo_parameters = {
        "moveit_servo": _load_yaml(
            os.path.join(bringup_share, "config", "moveit_servo.yaml"))
    }
    servo = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        condition=IfCondition(use_servo),
        parameters=[
            servo_parameters,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )
    joy_to_servo = ComposableNodeContainer(
        name="moveit_servo_joy_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        condition=IfCondition(servo_and_joy),
        composable_node_descriptions=[
            ComposableNode(
                package="jaka_driver",
                plugin="moveit_servo::JoyToServoPub",
                name="controller_to_servo_node",
            ),
            ComposableNode(
                package="joy",
                plugin="joy::Joy",
                name="joy_node",
            ),
        ],
        output="screen",
    )

    gripper = Node(
        package="dh_gripper_driver",
        executable="dh_ag95_driver",
        name="dh_ag95_driver",
        condition=IfCondition(real_gripper),
        parameters=[{
            "device_port": LaunchConfiguration("gripper_device_port"),
            "baudrate": LaunchConfiguration("gripper_baudrate"),
            "gripper_id": LaunchConfiguration("gripper_id"),
            "max_position": 100.0,
            "max_force": 100.0,
        }],
        output="screen",
    )

    arguments = [
        DeclareLaunchArgument(
            "backend", default_value="sim",
            description="Execution backend: sim (MuJoCo) or real (JAKA)."),
        DeclareLaunchArgument(
            "allow_trajectory_execution", default_value="auto",
            description=(
                "auto enables execution only for sim; use true explicitly for "
                "real robot trajectory execution.")),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_servo", default_value="false"),
        DeclareLaunchArgument(
            "use_joy", default_value="false",
            description="Start joystick conversion; requires use_servo:=true."),
        DeclareLaunchArgument(
            "use_gripper", default_value="false",
            description="Start the real DH AG95 driver (real backend only)."),
        DeclareLaunchArgument("robot_ip", default_value="10.5.5.100"),
        DeclareLaunchArgument("local_ip", default_value="10.5.5.127"),
        DeclareLaunchArgument(
            "jaka_read_only", default_value="true",
            description=(
                "Read JAKA state without enabling servo mode or sending arm "
                "commands. Set false only for intended real motion.")),
        DeclareLaunchArgument(
            "controller_manager_timeout", default_value="30.0"),
        DeclareLaunchArgument(
            "sim_model",
            default_value=os.path.join(mujoco_share, "models", "scene.xml")),
        DeclareLaunchArgument("sim_viewer", default_value="true"),
        DeclareLaunchArgument("sim_camera", default_value="false"),
        DeclareLaunchArgument("sim_camera_rate", default_value="30.0"),
        DeclareLaunchArgument("sim_camera_width", default_value="640"),
        DeclareLaunchArgument("sim_camera_height", default_value="480"),
        DeclareLaunchArgument("sim_init_keyframe", default_value="home"),
        DeclareLaunchArgument("sim_fts_enable", default_value="true"),
        DeclareLaunchArgument(
            "sim_fts_zero_on_start", default_value="true"),
        DeclareLaunchArgument(
            "gripper_device_port", default_value="/dev/ttyUSB0"),
        DeclareLaunchArgument("gripper_baudrate", default_value="115200"),
        DeclareLaunchArgument("gripper_id", default_value="1"),
    ]

    return LaunchDescription(arguments + [
        OpaqueFunction(function=_validate_arguments),
        mujoco,
        robot_state_publisher,
        controller_manager,
        spawn_trajectory_after_jsb,
        spawn_fts_after_arm,
        spawn_fts_after_servo_arm,
        joint_state_broadcaster,
        move_group,
        servo,
        joy_to_servo,
        gripper,
        rviz,
    ])
