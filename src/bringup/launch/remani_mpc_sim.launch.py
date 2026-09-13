#!/usr/bin/env python3
# =============================================================================
#  remani_mpc_sim.launch.py
#
#  REMANI -> OCS2 MPC/MRT -> MuJoCo 的单入口仿真 launch。
#
#  数据流：
#    MuJoCo + EKF
#      -> REMANI 全身规划器 (静态 ESDF, odom 坐标系)
#      -> remani_to_ocs2_reference_bridge
#      -> wbmm_ocs2_ros MPC/MRT
#      -> MuJoCo 底盘 + 机械臂
#
#  本文件不重复实现仿真管线，只对 bringup/ocs2_sim.launch.py 做 REMANI 场景的
#  默认参数封装，并可可选地自动发送一个 2D demo goal，便于一条命令验证全链路。
#
#  常用法：
#    # 可视化仿真 + 自动发送 demo goal
#    ros2 launch tracer_jaka_bringup remani_mpc_sim.launch.py
#
#    # 无头验证
#    ros2 launch tracer_jaka_bringup remani_mpc_sim.launch.py \
#        viewer:=false use_rviz:=false
#
#    # 交互模式（RViz 里点 2D Goal / 或自己发 /goal_pose）
#    ros2 launch tracer_jaka_bringup remani_mpc_sim.launch.py \
#        publish_demo_goal:=false
# =============================================================================

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('tracer_jaka_bringup')

    remani_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [bringup_share, 'launch', 'ocs2_sim.launch.py'])
        ),
        launch_arguments={
            # 可视化/系统组合
            'viewer': LaunchConfiguration('viewer'),
            'use_rviz': LaunchConfiguration('use_rviz'),
            # REMANI 默认在 odom 系规划，不需要 SLAM；EKF 仍提供 odom->base。
            'start_slam': LaunchConfiguration('start_slam'),
            # REMANI 拥有 MPC target，不能再启动 CSV target。
            'start_remani': 'true',
            'start_remani_bridge': 'true',
            'remani_planner_frame': 'odom',
            'remani_target_frame': 'odom',
            'remani_use_tf_transform': 'false',
            'map_to_odom_x': '0.0',
        }.items(),
    )

    # 可选：REMANI 进入 WAIT_TARGET 后发布一次固定 goal。
    # 默认 (0.0, 1.2, 0.0) 是当前 scene.xml + 静态 ESDF 下已验证可行的短距离目标。
    demo_goal = TimerAction(
        period=LaunchConfiguration('goal_delay'),
        condition=IfCondition(LaunchConfiguration('publish_demo_goal')),
        actions=[
            Node(
                package='tracer_jaka_mujoco',
                executable='demo_goal_publisher',
                name='remani_mpc_sim_demo_goal',
                output='screen',
                parameters=[{
                    'goal_x': ParameterValue(
                        LaunchConfiguration('goal_x'), value_type=float),
                    'goal_y': ParameterValue(
                        LaunchConfiguration('goal_y'), value_type=float),
                    'goal_yaw': ParameterValue(
                        LaunchConfiguration('goal_yaw'), value_type=float),
                    'frame_id': 'odom',
                    'use_sim_time': True,
                }],
            ),
        ],
    )

    declared_arguments = [
        DeclareLaunchArgument(
            'viewer', default_value='true',
            description='Open the MuJoCo native viewer.'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Start RViz2 for target/status visualization.'),
        DeclareLaunchArgument(
            'start_slam', default_value='false',
            description='Start slam_toolbox. REMANI in odom mode does not need it.'),
        DeclareLaunchArgument(
            'publish_demo_goal', default_value='true',
            description='Publish one deterministic /goal_pose after startup.'),
        DeclareLaunchArgument(
            'goal_x', default_value='0.0',
            description='Demo goal x in odom.'),
        DeclareLaunchArgument(
            'goal_y', default_value='1.2',
            description='Demo goal y in odom.'),
        DeclareLaunchArgument(
            'goal_yaw', default_value='0.0',
            description='Demo goal yaw in odom (rad).'),
        DeclareLaunchArgument(
            'goal_delay', default_value='30.0',
            description=(
                'Seconds after launch before publishing the demo goal; '
                'REMANI starts around t=12 s and needs a moment to load ESDF.')),
    ]

    return LaunchDescription(
        declared_arguments + [remani_sim, demo_goal])
