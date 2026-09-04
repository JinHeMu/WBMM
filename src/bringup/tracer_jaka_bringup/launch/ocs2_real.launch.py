#!/usr/bin/env python3
# =============================================================================
#  ocs2_real.launch.py
#
#  实机 launch (替换原 ocs2_sim.launch.py 中的 Gazebo 部分):
#    1. xacro -> /tmp/.../tracer_jaka_real.urdf  (real_robot:=true 把 ros2_control
#       插件块切到 jaka_hardware_interface)
#    2. robot_state_publisher
#    3. tracer_base (CAN 节点, 接 /cmd_vel, 发 /odom 与 odom->base_link TF)
#    4. ros2_control_node + joint_state_broadcaster + arm_controller
#    5. OCS2 三件套 (mpc / mrt / target):
#         use_sim_time          : false
#         use_stamped_cmd       : false      ← 让 MRT 发 Twist 给 tracer_base
#         base_cmd_topic        : /cmd_vel
#         odom_topic            : /odom
#         base_frame            : base_footprint
#    6. RViz2
# =============================================================================

import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import (
    DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.substitutions import (
    Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _enforce_safety_gate(context):
    release = LaunchConfiguration('safety_release').perform(context).lower()
    read_only = LaunchConfiguration('jaka_read_only').perform(context).lower()
    command_output = LaunchConfiguration(
        'command_output_enabled').perform(context).lower()
    released = release in ('1', 'true', 'yes', 'on')
    arm_writable = read_only not in ('1', 'true', 'yes', 'on')
    commands_enabled = command_output in ('1', 'true', 'yes', 'on')
    if (arm_writable or commands_enabled) and not released:
        raise RuntimeError(
            'Real motion gates require safety_release:=true in addition to '
            'jaka_read_only:=false/command_output_enabled:=true')
    if commands_enabled and not arm_writable:
        raise RuntimeError(
            'command_output_enabled:=true is inconsistent with '
            'jaka_read_only:=true')
    return []



def generate_launch_description():
    pkg_ocs2     = FindPackageShare('tracer_jaka_ocs2')
    pkg_description = FindPackageShare('tracer_jaka_description')
    pkg_tracer   = FindPackageShare('tracer_base')

    # --------- 参数 ---------
    use_rviz   = LaunchConfiguration('use_rviz')
    start_base = LaunchConfiguration('start_base')
    start_rsp  = LaunchConfiguration('start_robot_state_publisher')
    mrt_odom_topic = LaunchConfiguration('mrt_odom_topic')
    task_file  = LaunchConfiguration('task_file')
    urdf_file  = LaunchConfiguration('urdf_file')
    control_urdf_file = LaunchConfiguration('control_urdf_file')
    lib_folder = LaunchConfiguration('lib_folder')
    can_port   = LaunchConfiguration('can_port')
    publish_odom_tf = LaunchConfiguration('publish_odom_tf')
    robot_ip   = LaunchConfiguration('robot_ip')
    local_ip   = LaunchConfiguration('local_ip')
    jaka_read_only = LaunchConfiguration('jaka_read_only')
    command_output_enabled = LaunchConfiguration('command_output_enabled')
    start_ocs2 = LaunchConfiguration('start_ocs2')
    rviz_config = LaunchConfiguration('rviz_config')
    arm_max_delta_per_step = LaunchConfiguration('arm_max_delta_per_step')
    arm_max_command_velocity = LaunchConfiguration('arm_max_command_velocity')


    declare_args = [
        DeclareLaunchArgument('use_rviz',  default_value='true'),
        DeclareLaunchArgument('start_base', default_value='true'),
        DeclareLaunchArgument(
            'start_robot_state_publisher', default_value='true'),
        DeclareLaunchArgument(
            'mrt_odom_topic', default_value='/odom',
            description='Odometry consumed by MRT; use /odometry/filtered when EKF is running'),
        DeclareLaunchArgument('can_port',  default_value='can0'),
        DeclareLaunchArgument(
            'publish_odom_tf',
            default_value='true',
            description='Set false when robot_localization publishes odom TF'),
        DeclareLaunchArgument('robot_ip',  default_value='10.5.5.100'),
        DeclareLaunchArgument('local_ip',  default_value='10.5.5.127'),
        DeclareLaunchArgument(
            'jaka_read_only', default_value='true',
            description=(
                'Keep the JAKA hardware interface in telemetry-only mode. '
                'Set false only after the dry-run checks pass.')),
        DeclareLaunchArgument(
            'command_output_enabled', default_value='false',
            description=(
                'Create and use MRT base/arm command publishers. This is the '
                'second real-motion safety gate.')),
        DeclareLaunchArgument(
            'safety_release', default_value='false',
            description=(
                'Explicit third opt-in required before any real command '
                'output or writable arm interface.')),
        DeclareLaunchArgument(
            'start_ocs2', default_value='true',
            description='Start MPC/MRT after the hardware feedback is ready.'),
        DeclareLaunchArgument(
            'task_file',
            default_value=PathJoinSubstitution(
                [pkg_ocs2, 'config', 'task_real.info'])),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution(
                [pkg_description, "urdf", "tracer_jaka_zu5.urdf"]
            ),
            description='Canonical control-free URDF consumed by OCS2/Pinocchio.',
        ),
        DeclareLaunchArgument(
            "control_urdf_file",
            default_value=PathJoinSubstitution(
                [pkg_description, "urdf", "tracer_jaka_zu5.controlled.urdf.xacro"]
            ),
            description='Canonical xacro used only to create robot_description.',
        ),
        DeclareLaunchArgument(
            'lib_folder',
            default_value='/tmp/ocs2_tracer_jaka_real/auto_generated'),
        DeclareLaunchArgument('use_joy',    default_value='false'),
        DeclareLaunchArgument('joy_device', default_value='/dev/input/js0'),
        DeclareLaunchArgument(
            'arm_max_delta_per_step', default_value='0.05',
            description='Maximum arm position-command lead over measurement [rad].'),
        DeclareLaunchArgument(
            'arm_max_command_velocity', default_value='0.15',
            description='MRT arm command slew-rate limit [rad/s].'),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [pkg_ocs2, 'rviz', 'tracer_jaka_ocs2.rviz'])),
    ]


    # --------- 2. robot_state_publisher ---------
    robot_description = {
        'robot_description': Command([
            FindExecutable(name='xacro'), ' ',
            control_urdf_file, ' ',
            'control_backend:=real ',
            'robot_ip:=', robot_ip, ' ',
            'local_ip:=', local_ip, ' ',
            'jaka_read_only:=', jaka_read_only, ' ',
        ])
    }
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': False}],
        condition=IfCondition(start_rsp),
    )

    # --------- 3. tracer 底盘 (CAN) ---------
    tracer_base = Node(
        package='tracer_base',
        executable='tracer_base_node',     # 视你的 CMakeLists 实际可执行名替换
        name='tracer_base_node',
        output='screen',
        parameters=[{
            'port_name':       can_port,
            'odom_frame':      'odom',
            'base_frame':      'base_footprint',  # 与 OCS2 / URDF 对齐
            'odom_topic_name': 'odom',
            'publish_odom_tf': publish_odom_tf,
            'is_tracer_mini':  False,
            'simulated_robot': False,
            'control_rate':    50,
        }],
        condition=IfCondition(start_base),
    )

    # --------- 4. controller_manager + spawners ---------
    controllers_yaml = PathJoinSubstitution(
        [pkg_description, 'config', 'ros2_controllers.yaml'])

    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml,
                    {'use_sim_time': False}],
        output='screen',
        remappings=[
            # Humble FTS broadcaster publishes relative to controller_manager.
            ('/controller_manager/wrench', '/fts_broadcaster/wrench'),
        ],
    )

    spawn_jsb = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )
    spawn_jtc = Node(
        package='controller_manager', executable='spawner',
        arguments=['arm_controller',
                   '--controller-manager', '/controller_manager'],
        output='screen',
    )

    fts_params = PathJoinSubstitution(
        [pkg_description, 'config', 'fts_broadcaster_humble.yaml'])
    spawn_fts = Node(
        package='controller_manager', executable='spawner',
        name='fts_broadcaster_spawner',
        arguments=[
            'fts_broadcaster',
            '--controller-manager', '/controller_manager',
            '--param-file', fts_params,
        ],
        output='screen',
    )

    # 串行加载可避免 Humble controller_manager 内的并发加载竞争：
    # joint_state_broadcaster -> force_torque_sensor_broadcaster -> forward controller.
    spawn_fts_after_jsb = RegisterEventHandler(
        OnProcessExit(target_action=spawn_jsb, on_exit=[spawn_fts]))
    spawn_jtc_after_fts = RegisterEventHandler(
        OnProcessExit(target_action=spawn_fts, on_exit=[spawn_jtc]))

    # --------- 5. OCS2 三件套 ---------
    common_ocs2 = {
        'taskFile':     task_file,
        'urdfFile':     urdf_file,
        'libFolder':    lib_folder,
        'use_sim_time': False,
    }

    mpc_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mpc_node',
        name='tracer_jaka_mpc_node',
        output='screen',
        parameters=[common_ocs2],
    )

    mrt_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_mrt_node',
        output='screen',
        parameters=[{
            **common_ocs2,
            'mrt_loop_rate':     100.0,
            'traj_horizon':      0.10,                 # 实机给宽一点
            'use_stamped_cmd':   False,                # *** 关键: 发 Twist ***
            'base_cmd_topic':    '/cmd_vel',           # *** tracer_base 订这个 ***
            'odom_topic':        mrt_odom_topic,       # *** 默认 /odom，EKF 时用 /odometry/filtered ***
            'joint_state_topic': '/joint_states',
            'arm_cmd_topic':     '/arm_controller/commands',
            'command_output_enabled': ParameterValue(
                command_output_enabled, value_type=bool),
            'arm_max_delta_per_step': ParameterValue(
                arm_max_delta_per_step, value_type=float),
            'arm_max_command_velocity': ParameterValue(
                arm_max_command_velocity, value_type=float),
            'arm_joint_names':   ['joint_1','joint_2','joint_3',
                                  'joint_4','joint_5','joint_6'],
            'base_frame':        'base_footprint',
            'world_frame':       'odom',
            'ee_frame':          'tool0',
        }],
    )

    target_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_target_node',
        name='tracer_jaka_target_node',
        output='screen',
        parameters=[{
            'robot_name':    'mobile_manipulator',
            'marker_frame':  'odom',
            'ee_frame':      'tool0',
            'marker_scale':  0.3,
            'input_dim':     8,
            'use_sim_time':  False,
        }],
    )

    # --------- 6. RViz2 ---------
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    use_joy    = LaunchConfiguration('use_joy')
    joy_device = LaunchConfiguration('joy_device')

    # 手柄驱动: 读 /dev/input/jsX, 出 /joy
    joy_driver = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            {
                "device_id": 0,
                "deadzone": 0.05,
                "autorepeat_rate": 20.0,
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(use_joy),
        output="screen",
    )


    # 手柄 -> OCS2 目标位姿节点
    joy_target_node = Node(
        package='tracer_jaka_ocs2',
        executable='tracer_jaka_joy_target_node',
        name='tracer_jaka_joy_target_node',
        output='screen',
        parameters=[{
            'robot_name':    'mobile_manipulator',
            'marker_frame':  'odom',
            'ee_frame':      'tool0',
            'input_dim':     8,
            'joy_topic':     '/joy',
            'publish_rate':  50.0,
            'linear_speed':  0.15,
            'angular_speed': 0.6,
            'deadzone':      0.10,
            'use_sim_time':  False,           # sim launch 里改成 use_sim_time
            # 如果是 PS4/PS5 手柄, 这里覆盖默认的轴/键映射即可
            # 'axis_x': 1, 'axis_y': 0, 'axis_z': 4, 'axis_yaw': 3,
            # 'button_deadman': 4, 'button_reset': 0, 'button_home': 1,
        }],
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
                "arm_home": [0.0, 1.5707,-1.57, 1.5707, 1.57, 0.785398],
                "use_sim_time": False,
            }
        ],
        condition=IfCondition(use_joy),
    )

    # arm_home": [0.0, 1.5707,0.0, 1.5707, 3.14159, 0.785398],



    # --------- 时序 ---------
    # 时序原则:
    #   t=0    : xacro, rsp, tracer_base, controller_manager 同时起
    #   t=2s   : spawn jsb (要等 ros2_control_node 先起来)
    #   jsb退出: spawn jtc (RegisterEventHandler 串起来)
    #   t=4s   : rviz
    #   t=10s  : 等 odom + joint_states + JTC 都活了, 再起 OCS2 三件套
    rviz_delayed = TimerAction(period=4.0, actions=[rviz_node])
    spawn_delayed = TimerAction(period=2.0, actions=[spawn_jsb])
    ocs2_delayed = TimerAction(
        period=10.0,
        actions=[mpc_node, mrt_node, joy_driver, joy_whole_body_node],
        condition=IfCondition(start_ocs2))

    return LaunchDescription(declare_args + [
        OpaqueFunction(function=_enforce_safety_gate),
        rsp,
        tracer_base,
        controller_manager,
        spawn_delayed,
        spawn_fts_after_jsb,
        spawn_jtc_after_fts,
        ocs2_delayed,

        rviz_delayed,
    ])
