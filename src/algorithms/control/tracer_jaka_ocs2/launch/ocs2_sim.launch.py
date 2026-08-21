#!/usr/bin/env python3
# =============================================================================
#  ocs2_mujoco_sim.launch.py
#
#  MuJoCo + ros2_control + OCS2 集成 launch:
#    1. 准备 OCS2 需要的 URDF 文件路径
#       - 如果输入是 .xacro，则编译成 /tmp/.../tracer_jaka.urdf
#       - 如果输入是 .urdf，则直接复制到 /tmp/.../tracer_jaka.urdf
#    2. 启动 tracer_jaka_mujoco/ros2_control.launch.py
#       - robot_state_publisher
#       - mujoco_ros2_control
#       - joint_state_broadcaster
#       - arm_controller
#       - base_controller
#    3. 等 MuJoCo 和控制器就绪后，启动 OCS2:
#       - tracer_jaka_mpc_node
#       - tracer_jaka_mrt_node
#       - tracer_jaka_target_node
#       - 可选 joy 控制目标点
#    4. 可选 RViz2
# =============================================================================

import os
import shutil

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _ensure_urdf(context, *args, **kwargs):
    """
    给 OCS2 准备一个实际存在的 URDF 文件路径。

    OCS2 通常需要 urdfFile 是一个真实 .urdf 路径，
    不能直接吃 xacro。
    """
    src = context.perform_substitution(LaunchConfiguration("xacro_file"))
    dst = context.perform_substitution(LaunchConfiguration("urdf_file"))

    os.makedirs(os.path.dirname(dst), exist_ok=True)

    # 如果是 xacro，就编译
    if src.endswith(".xacro"):
        return [
            LogInfo(msg=f"[ocs2_mujoco] xacro -> urdf: {src} -> {dst}"),
            ExecuteProcess(
                cmd=[
                    "xacro",
                    src,
                    "sim_mode:=true",
                    "-o",
                    dst,
                ],
                output="screen",
                shell=False,
            ),
        ]

    # 如果已经是 urdf，就直接复制到 /tmp 给 OCS2 用
    if os.path.abspath(src) != os.path.abspath(dst):
        shutil.copyfile(src, dst)

    return [
        LogInfo(msg=f"[ocs2_mujoco] use urdf: {dst}")
    ]


def generate_launch_description():
    pkg_ocs2 = FindPackageShare("tracer_jaka_ocs2")
    pkg_mujoco = FindPackageShare("tracer_jaka_mujoco")
    pkg_remani = FindPackageShare("remani_planner")
    pkg_grid_map = FindPackageShare("grid_map")

    # -------------------------------------------------------------------------
    # Launch 参数
    # -------------------------------------------------------------------------
    use_sim_time = LaunchConfiguration("use_sim_time")
    viewer = LaunchConfiguration("viewer")
    use_rviz = LaunchConfiguration("use_rviz")
    rviz_config = LaunchConfiguration("rviz_config")
    use_joy = LaunchConfiguration("use_joy")
    use_csv_target = LaunchConfiguration("use_csv_target")
    start_slam = LaunchConfiguration("start_slam")
    start_remani = LaunchConfiguration("start_remani")
    start_remani_bridge = LaunchConfiguration("start_remani_bridge")
    remani_launch_file = LaunchConfiguration("remani_launch_file")
    mrt_odom_topic = LaunchConfiguration("mrt_odom_topic")
    remani_static_esdf_file = LaunchConfiguration(
        "remani_static_esdf_file"
    )
    remani_static_esdf_offset_x = LaunchConfiguration(
        "remani_static_esdf_offset_x"
    )
    remani_static_esdf_offset_y = LaunchConfiguration(
        "remani_static_esdf_offset_y"
    )
    remani_static_esdf_offset_z = LaunchConfiguration(
        "remani_static_esdf_offset_z"
    )
    mujoco_model = LaunchConfiguration("mujoco_model")
    map_to_odom_x = LaunchConfiguration("map_to_odom_x")
    remani_manipulator_max_vel = LaunchConfiguration(
        "remani_manipulator_max_vel")
    remani_manipulator_max_acc = LaunchConfiguration(
        "remani_manipulator_max_acc")
    remani_freeze_manipulator = LaunchConfiguration(
        "remani_freeze_manipulator")
    remani_tracking_error_replan_enabled = LaunchConfiguration(
        "remani_tracking_error_replan_enabled")
    remani_planner_frame = LaunchConfiguration("remani_planner_frame")
    remani_target_frame = LaunchConfiguration("remani_target_frame")
    remani_use_tf_transform = LaunchConfiguration("remani_use_tf_transform")
    mrt_traj_horizon = LaunchConfiguration("mrt_traj_horizon")

    task_file = LaunchConfiguration("task_file")
    xacro_file = LaunchConfiguration("xacro_file")
    urdf_file = LaunchConfiguration("urdf_file")
    lib_folder = LaunchConfiguration("lib_folder")

    declare_args = [
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "viewer",
            default_value="true",
            description="Open the MuJoCo native viewer.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "mrt_traj_horizon",
            default_value="1.0",
            description=(
                "Seconds ahead in the MPC policy used for arm position "
                "commands. Contact tasks should use a short horizon."
            ),
        ),
        DeclareLaunchArgument(
            "arm_use_velocity_integrator",
            default_value="false",
            description="Integrate MPC joint velocity into bounded position commands.",
        ),
        DeclareLaunchArgument(
            "arm_max_command_velocity",
            default_value="0.5",
            description="Per-joint velocity limit for the position-command integrator.",
        ),
        DeclareLaunchArgument(
            "arm_contact_command_velocity",
            default_value="0.10",
            description="Per-joint slew limit for guarded/contact position references.",
        ),
        DeclareLaunchArgument(
            "arm_max_delta_per_step",
            default_value="0.50",
            description=(
                "Maximum position-command lead relative to the measured joint "
                "state. Contact pipelines should keep this small."
            ),
        ),
        DeclareLaunchArgument(
            "arm_contact_max_delta_per_step",
            default_value="0.10",
            description=(
                "Maximum position-command lead while force contact is active."
            ),
        ),
        DeclareLaunchArgument(
            "force_control_state_topic",
            default_value="",
            description=(
                "Optional std_msgs/String state used to enable the contact "
                "command-lead limit."
            ),
        ),
        DeclareLaunchArgument(
            "contact_arm_reference_topic",
            default_value="",
            description=(
                "Optional six-joint reference used while force contact is active."
            ),
        ),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution(
                [pkg_ocs2, "rviz", "tracer_jaka_ocs2.rviz"]
            ),
            description="RViz configuration file.",
        ),
        DeclareLaunchArgument(
            "use_joy",
            default_value="true",
        ),
        DeclareLaunchArgument(
            "use_csv_target",
            default_value="false",
            description=(
                "Start the legacy CSV target publisher. Keep false when "
                "using the REMANI-to-OCS2 bridge."
            ),
        ),
        DeclareLaunchArgument(
            "start_slam",
            default_value="true",
            description=(
                "Start robot_localization and slam_toolbox. The EKF always "
                "provides odom -> base_footprint; when false a static "
                "map -> odom transform is used only for visualization."
            ),
        ),
        DeclareLaunchArgument(
            "start_remani",
            default_value="true",
            description=(
                "Start REMANI whole-body planner and the REMANI-to-OCS2 "
                "reference bridge."
            ),
        ),
        DeclareLaunchArgument(
            "start_remani_bridge",
            default_value="true",
            description=(
                "Start the legacy direct REMANI-to-OCS2 bridge. Set false "
                "when another planner such as wipe_planner owns the MPC target."
            ),
        ),
        DeclareLaunchArgument(
            "remani_launch_file",
            default_value=PathJoinSubstitution([
                pkg_remani,
                "launch",
                "remani_mpc_tracking.launch.py",
            ]),
            description=(
                "Absolute REMANI launch file. Pipelines can override this "
                "to prevent an older AMENT overlay from selecting a stale "
                "remani_planner installation."
            ),
        ),
        DeclareLaunchArgument(
            "mrt_odom_topic",
            default_value="/odometry/filtered",
            description=(
                "Odometry consumed by the OCS2 MRT state estimator. The "
                "default is the wheel-odom + IMU EKF output."
            ),
        ),
        DeclareLaunchArgument(
            "remani_static_esdf_file",
            default_value=PathJoinSubstitution([
                pkg_grid_map,
                "maps",
                "tracer_jaka_zu5_scene_esdf.npz",
            ]),
            description="Static ESDF NPZ consumed by REMANI GridMap.",
        ),
        # The current MuJoCo robot starts at world x=-2 while odometry starts
        # at x=0, hence x_odom=x_mujoco+2 for the generated scene ESDF.
        DeclareLaunchArgument(
            "remani_static_esdf_offset_x",
            default_value="2.0",
        ),
        DeclareLaunchArgument(
            "remani_static_esdf_offset_y",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "remani_static_esdf_offset_z",
            default_value="0.0",
        ),
        DeclareLaunchArgument(
            "mujoco_model",
            default_value=PathJoinSubstitution([
                pkg_mujoco, "models", "scene.xml",
            ]),
            description="MuJoCo scene XML used by the bridge.",
        ),
        DeclareLaunchArgument(
            "map_to_odom_x",
            default_value="-2.0",
            description=(
                "Static map->odom x translation used only when SLAM is off."),
        ),
        DeclareLaunchArgument(
            "remani_manipulator_max_vel",
            default_value="1.57",
            description="REMANI arm velocity limit in rad/s.",
        ),
        DeclareLaunchArgument(
            "remani_manipulator_max_acc",
            default_value="3.14",
            description="REMANI arm acceleration limit in rad/s^2.",
        ),
        DeclareLaunchArgument(
            "remani_freeze_manipulator",
            default_value="false",
            description="Keep the measured arm posture in REMANI front-end.",
        ),
        DeclareLaunchArgument(
            "remani_tracking_error_replan_enabled",
            default_value="true",
            description="Enable REMANI tracking-error replans.",
        ),
        DeclareLaunchArgument(
            "remani_planner_frame",
            default_value="odom",
            description=(
                "Frame in which REMANI plans. Set map to validate the "
                "persistent-map pipeline in simulation.")),
        DeclareLaunchArgument(
            "remani_target_frame",
            default_value="odom",
            description="Frame in which the REMANI->OCS2 bridge publishes."),
        DeclareLaunchArgument(
            "remani_use_tf_transform",
            default_value="false",
            description=(
                "Use dynamic TF map->odom in the bridge when planner_frame "
                "differs from target_frame.")),
        DeclareLaunchArgument(
            "task_file",
            default_value=PathJoinSubstitution(
                [pkg_ocs2, "config", "task.info"]
            ),
        ),
        DeclareLaunchArgument(
            # 名字沿用你原来的 xacro_file，但现在默认指向 MuJoCo 包里的 .urdf
            "xacro_file",
            default_value=PathJoinSubstitution(
                [pkg_mujoco, "urdf", "tracer_jaka_zu5.urdf"]
            ),
        ),
        DeclareLaunchArgument(
            "urdf_file",
            default_value="/tmp/ocs2_tracer_jaka/tracer_jaka.urdf",
        ),
        DeclareLaunchArgument(
            "lib_folder",
            default_value="/tmp/ocs2_tracer_jaka/auto_generated",
        ),
    ]

    # -------------------------------------------------------------------------
    # Step 1: 准备 OCS2 需要的 URDF 文件
    # -------------------------------------------------------------------------
    prepare_urdf = OpaqueFunction(function=_ensure_urdf)

    # -------------------------------------------------------------------------
    # Step 2: 启动 MuJoCo ros2_control 仿真
    #
    # 这个 launch 里已经包含:
    #   - robot_state_publisher
    #   - mujoco_ros2_control
    #   - joint_state_broadcaster
    #   - arm_controller
    #   - base_controller
    # -------------------------------------------------------------------------
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [pkg_mujoco, "launch", "bridge.launch.py"]
            )
        ),
            launch_arguments={
                "viewer": viewer,
                "model": mujoco_model,
            }.items()
    )

    # -------------------------------------------------------------------------
    # Step 3: wheel odom + IMU localization and 2D SLAM
    #
    # TF ownership:
    #   slam_toolbox       map  -> odom
    #   robot_localization odom -> base_footprint
    #   robot_state_publisher   base_footprint -> robot/sensor links
    #
    # The MuJoCo bridge publishes /wheel/odometry and /imu/data, but its
    # publish_odom_tf parameter is false, so the EKF must be present.
    # -------------------------------------------------------------------------
    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[
            PathJoinSubstitution([pkg_mujoco, "config", "ekf.yaml"]),
            {"use_sim_time": use_sim_time},
        ],
        remappings=[
            ("odometry/filtered", "/odometry/filtered"),
        ],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [pkg_mujoco, "config", "slam_toolbox.yaml"]
            ),
            {"use_sim_time": use_sim_time},
        ],
        condition=IfCondition(start_slam),
    )

    # -------------------------------------------------------------------------
    # Step 4: OCS2 三件套
    # -------------------------------------------------------------------------
    mpc_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_mpc_node",
        name="tracer_jaka_mpc_node",
        output="screen",
        parameters=[
            {
                "taskFile": task_file,
                "urdfFile": urdf_file,
                "libFolder": lib_folder,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    mrt_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_mrt_node",
        name="tracer_jaka_mrt_node",
        output="screen",
        parameters=[
            {
                "taskFile": task_file,
                "urdfFile": urdf_file,
                "libFolder": lib_folder,

                "mrt_loop_rate": 125.0,
                "traj_horizon": ParameterValue(
                    mrt_traj_horizon, value_type=float),
                "arm_use_velocity_integrator": ParameterValue(
                    LaunchConfiguration("arm_use_velocity_integrator"),
                    value_type=bool),
                "arm_max_command_velocity": ParameterValue(
                    LaunchConfiguration("arm_max_command_velocity"),
                    value_type=float),
                "arm_contact_command_velocity": ParameterValue(
                    LaunchConfiguration("arm_contact_command_velocity"),
                    value_type=float),
                "arm_max_delta_per_step": ParameterValue(
                    LaunchConfiguration("arm_max_delta_per_step"),
                    value_type=float),
                "arm_contact_max_delta_per_step": ParameterValue(
                    LaunchConfiguration("arm_contact_max_delta_per_step"),
                    value_type=float),
                "force_control_state_topic": LaunchConfiguration(
                    "force_control_state_topic"),
                "contact_arm_reference_topic": LaunchConfiguration(
                    "contact_arm_reference_topic"),
                "traj_num_points": 5,

                "use_stamped_cmd": False,
                
                # wheel odom + IMU 经 robot_localization 融合后的状态
                "odom_topic": mrt_odom_topic,

                # 来自 joint_state_broadcaster
                "joint_state_topic": "/joint_states",

                # 关键修改：
                # controllers.yaml 里 base_controller:
                #   use_stamped_vel: false
                # 所以命令话题是 /base_controller/cmd_vel_unstamped
                "base_cmd_topic": "/base_controller/cmd_vel",

                # MRT 输出 std_msgs/Float64MultiArray。JointTrajectoryController
                # 的逐关节位置命令入口是 /commands；/joint_trajectory 需要
                # trajectory_msgs/JointTrajectory，不能混用。
                "arm_cmd_topic": "/arm_controller/commands",

                "arm_joint_names": [
                    "joint_1",
                    "joint_2",
                    "joint_3",
                    "joint_4",
                    "joint_5",
                    "joint_6",
                ],

                "base_frame": "base_footprint",
                "use_sim_time": use_sim_time,
            }
        ],
    )
    whole_body_trajectory_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_whole_body_trajectory_node',
        name='whole_body_trajectory_target_node',
        output='screen',
        parameters=[{
            'csv_file': '/home/a/WBMM/tools/examples/whole_body_path_example.csv',
            'robot_name': 'mobile_manipulator',
            'world_frame': 'odom',
            'state_dim': 9,
            'input_dim': 8,
            'base_dim': 3,
            'arm_dim': 6,
            # CSV 没有 time 列时才用到:
            'linear_speed': 0.15,
            'angular_speed': 0.30,
            'joint_speed': 0.50,
            'min_dt': 0.10,
            'time_scale': 1.0,       # >1 = 整体放慢
            'start_lead': 1.0,
            'auto_publish': True,
            'auto_publish_delay': 1.0,
            'prepend_current_state': True,
            'hold_time_at_end': 3.0,
        }
        ],
        # REMANI and the CSV player must never publish OCS2 targets together.
        condition=IfCondition(PythonExpression([
            "'", use_csv_target, "'.lower() == 'true' and '",
            start_remani, "'.lower() == 'false'",
        ])),
    )

    target_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_target_node",
        name="tracer_jaka_target_node",
        output="screen",
        parameters=[
            {
                "robot_name": "mobile_manipulator",
                "marker_frame": "odom",
                "ee_frame": "tool0",
                "marker_scale": 0.3,
                "input_dim": 8,
                "use_sim_time": use_sim_time,
            }
        ],
    )

    # -------------------------------------------------------------------------
    # 可选：手柄驱动
    # -------------------------------------------------------------------------
    joy_driver = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_joy),
        output="screen",
    )

    joy_target_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_joy_target_node",
        name="tracer_jaka_joy_target_node",
        output="screen",
        parameters=[
            {
                "robot_name": "mobile_manipulator",
                "marker_frame": "odom",

                # 建议和 target_node 保持一致。
                # 如果你的 URDF 里实际末端 frame 叫 gripper_center_link，
                # 再改回 gripper_center_link。
                "ee_frame": "tool0",

                "input_dim": 8,
                "joy_topic": "/joy",
                "publish_rate": 50.0,

                "linear_speed": 0.15,
                "angular_speed": 0.6,
                "deadzone": 0.10,

                "use_sim_time": use_sim_time,

                # PS4 / PS5 手柄可以在这里覆盖映射
                # "axis_x": 1,
                # "axis_y": 0,
                # "axis_z": 4,
                # "axis_yaw": 3,
                # "button_deadman": 4,
                # "button_reset": 0,
                # "button_home": 1,
            }
        ],
        condition=IfCondition(use_joy),
    )

    # 手柄控制底盘全身轨迹 (左摇杆前进/后退, 右摇杆转向)
    # "胡萝卜" 模式: 目标 = 当前位置 + 速度 * lookahead_time
    # 松开 LB: 底盘保持位置, 手臂→home (MPC 主动对抗重力)
    joy_whole_body_node = Node(
        package="tracer_jaka_ocs2",
        executable="tracer_jaka_joy_whole_body_node",
        name="tracer_jaka_joy_whole_body_node",
        output="screen",
        parameters=[
            {
                "robot_name": "mobile_manipulator",
                "world_frame": "odom",
                "publish_rate": 50.0,
                "linear_speed_max": 0.4,       # 最大线速度 [m/s]
                "angular_speed_max": 1.0,      # 最大角速度 [rad/s]
                "deadzone": 0.10,
                "lookahead_time": 1.5,          # 胡萝卜前视距离 [s]
                "trajectory_horizon": 2.0,      # 轨迹总时长 [s]
                "lead_time": 0.05,
                "num_waypoints": 5,             # 航点数 (显式编码速度)
                "state_dim": 9,
                "input_dim": 8,
                "base_dim": 3,
                "arm_dim": 6,
                "joy_topic": "/joy",
                # 手柄映射
                "axis_linear": 1,       # 左摇杆 Y → 前进/后退
                "axis_angular": 3,      # 右摇杆 X → 转向
                "button_deadman": 4,     # LB → 安全开关
                "button_arm_home": 0,    # A → 臂归 home
                "button_arm_hold": 1,    # B → 臂保持当前构型
                # 机械臂 home 位姿
                "arm_home": [-0.515, 1.5707, -1.5707, 1.5707, 1.5707, 0.254],
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_joy),
    )

    # -------------------------------------------------------------------------
    # Step 5: RViz2
    # -------------------------------------------------------------------------
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=[
            "-d",
            rviz_config,
        ],
        parameters=[
            {
                "use_sim_time": use_sim_time,
            }
        ],
        condition=IfCondition(use_rviz),
        output="screen",
    )


    map_to_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="map_to_odom_static_tf",
        arguments=[
            map_to_odom_x, "0", "0",     # x y z
            "0", "0", "0",      # yaw pitch roll
            "map",
            "odom",
        ],
        output="screen",
        # slam_toolbox owns map -> odom while mapping is enabled.
        condition=UnlessCondition(start_slam),
    )

    # -------------------------------------------------------------------------
    # 启动时序
    #
    # MuJoCo 和 controller_manager 需要一点时间启动；
    # OCS2 MPC codegen / solver 初始化也比较吃时间。
    # -------------------------------------------------------------------------
    rviz_delayed = TimerAction(
        period=4.0,
        actions=[rviz_node],
    )

    ocs2_delayed = TimerAction(
        period=8.0,
        actions=[
            mpc_node,
            mrt_node,
            #target_node,
            #joy_driver,
            #joy_target_node,
            #joy_whole_body_node,
        ],
    )

    # REMANI consumes the EKF odometry and joint states, publishes
    # /planning/trajectory, then the bridge converts it to OCS2 targets.
    # Start it after MuJoCo, localization and OCS2 have had time to initialize.
    remani_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            remani_launch_file
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "start_planner": "true",
            "start_bridge": start_remani_bridge,
            "urdf_file": urdf_file,
            "static_esdf_file": remani_static_esdf_file,
            "static_esdf_offset_x": remani_static_esdf_offset_x,
            "static_esdf_offset_y": remani_static_esdf_offset_y,
            "static_esdf_offset_z": remani_static_esdf_offset_z,
            "odom_topic": mrt_odom_topic,
            "joint_state_topic": "/joint_states",
            "planner_to_ocs2_x": "0.0",
            "planner_to_ocs2_y": "0.0",
            "planner_to_ocs2_yaw": "0.0",
            "manipulator_max_vel": remani_manipulator_max_vel,
            "manipulator_max_acc": remani_manipulator_max_acc,
            "freeze_manipulator": remani_freeze_manipulator,
            "tracking_error_replan_enabled":
                remani_tracking_error_replan_enabled,
            "planner_frame": remani_planner_frame,
            "target_frame": remani_target_frame,
            "use_tf_transform": remani_use_tf_transform,
        }.items(),
    )

    remani_delayed = TimerAction(
        period=12.0,
        actions=[remani_launch],
        condition=IfCondition(start_remani),
    )

    return LaunchDescription(
        declare_args
        + [
            prepare_urdf,
            mujoco_launch,
            ekf_node,
            slam_node,
            ocs2_delayed,
            remani_delayed,
            map_to_odom_tf,
            rviz_delayed,
            whole_body_trajectory_node,
        ]
    )
