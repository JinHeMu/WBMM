#!/usr/bin/env python3
"""Run REMANI for Tracer+JAKA and bridge its polynomial output to OCS2.

The OCS2 MPC/MRT nodes and the robot drivers are intentionally not included;
start them with tracer_jaka_ocs2/ocs2_sim.launch.py or ocs2_real.launch.py.
"""

import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _merged_planner_parameters():
    share = get_package_share_directory('remani_planner')
    helper_path = os.path.join(share, 'launch', 'sim_example.py')
    spec = importlib.util.spec_from_file_location(
        'remani_sim_example_parameters', helper_path)
    helper = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(helper)
    return helper.merged_parameters('exp0')


def generate_launch_description():
    tracer_share = get_package_share_directory('tracer_jaka_mujoco')
    default_urdf = os.path.join(
        tracer_share, 'urdf', 'tracer_jaka_zu5_real.urdf')
    grid_map_share = get_package_share_directory('grid_map')
    default_esdf = os.path.join(
        grid_map_share, 'maps', 'tracer_jaka_zu5_scene_esdf.npz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_planner = LaunchConfiguration('start_planner')
    start_bridge = LaunchConfiguration('start_bridge')
    urdf_file = LaunchConfiguration('urdf_file')
    static_esdf_file = LaunchConfiguration('static_esdf_file')
    static_esdf_offset_x = LaunchConfiguration('static_esdf_offset_x')
    static_esdf_offset_y = LaunchConfiguration('static_esdf_offset_y')
    static_esdf_offset_z = LaunchConfiguration('static_esdf_offset_z')
    odom_topic = LaunchConfiguration('odom_topic')
    joint_state_topic = LaunchConfiguration('joint_state_topic')
    planner_to_ocs2_x = LaunchConfiguration('planner_to_ocs2_x')
    planner_to_ocs2_y = LaunchConfiguration('planner_to_ocs2_y')
    planner_to_ocs2_yaw = LaunchConfiguration('planner_to_ocs2_yaw')

    planner_parameters = _merged_planner_parameters()

    planner = Node(
        package='remani_planner',
        executable='remani_planner_node',
        name='remani_planner_node',
        output='screen',
        condition=IfCondition(start_planner),
        parameters=[
            planner_parameters,
            {
                'use_sim_time': use_sim_time,
                'mm.use_urdf_model': True,
                'mm.urdf_file': ParameterValue(urdf_file, value_type=str),
                'mm.urdf_base_link': 'base_footprint',
                'mm.urdf_mobile_base_collision_link': 'base_link',
                'mm.urdf_joint_names': [
                    'joint_1', 'joint_2', 'joint_3',
                    'joint_4', 'joint_5', 'joint_6',
                ],
                # Use the ESDF generated directly from the MuJoCo scene.
                # No depth image, point cloud, occupancy fusion or EDT
                # reconstruction is active in this mode.
                'grid_map.use_static_esdf': True,
                'grid_map.static_esdf_file': ParameterValue(
                    static_esdf_file, value_type=str),
                'grid_map.static_esdf_offset_x': ParameterValue(
                    static_esdf_offset_x, value_type=float),
                'grid_map.static_esdf_offset_y': ParameterValue(
                    static_esdf_offset_y, value_type=float),
                'grid_map.static_esdf_offset_z': ParameterValue(
                    static_esdf_offset_z, value_type=float),
                'grid_map.use_load_map': False,
                'grid_map.use_global_map': True,
                'grid_map.use_tf_cloud_transform': False,
                'grid_map.frame_id': 'odom',
                'fsm.global_plan': True,
                'fsm.target_type': 1,
            },
        ],
        remappings=[
            ('odom_world', odom_topic),
            ('joint_state', joint_state_topic),
            ('grid_map/odom', odom_topic),
            ('planning/trajectory', '/planning/trajectory'),
        ],
    )

    bridge = Node(
        package='tracer_jaka_ocs2',
        executable='remani_to_ocs2_reference_bridge',
        name='remani_to_ocs2_reference_bridge',
        output='screen',
        condition=IfCondition(start_bridge),
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_name': 'mobile_manipulator',
            'trajectory_topic': '/planning/trajectory',
            'state_dim': 9,
            'input_dim': 8,
            'arm_dim': 6,
            'sample_dt': 0.04,
            'reference_horizon': 3.0,
            'start_lead': 0.05,
            'publish_rate': 20.0,
            'assembly_timeout': 0.04,
            'zero_velocity_threshold': 1.0e-4,
            'hold_at_end': 2.0,
            'planner_to_ocs2_x': ParameterValue(
                planner_to_ocs2_x, value_type=float),
            'planner_to_ocs2_y': ParameterValue(
                planner_to_ocs2_y, value_type=float),
            'planner_to_ocs2_yaw': ParameterValue(
                planner_to_ocs2_yaw, value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_planner', default_value='true'),
        DeclareLaunchArgument('start_bridge', default_value='true'),
        DeclareLaunchArgument('urdf_file', default_value=default_urdf),
        DeclareLaunchArgument(
            'static_esdf_file', default_value=default_esdf),
        # In the current MJCF, base_footprint is fixed at world x=-2 while
        # its planar joints publish odom x=0. Therefore x_odom=x_mujoco+2.
        DeclareLaunchArgument('static_esdf_offset_x', default_value='2.0'),
        DeclareLaunchArgument('static_esdf_offset_y', default_value='0.0'),
        DeclareLaunchArgument('static_esdf_offset_z', default_value='0.0'),
        DeclareLaunchArgument(
            'odom_topic', default_value='/base_controller/odom'),
        DeclareLaunchArgument(
            'joint_state_topic', default_value='/joint_states'),
        DeclareLaunchArgument('planner_to_ocs2_x', default_value='0.0'),
        DeclareLaunchArgument('planner_to_ocs2_y', default_value='0.0'),
        DeclareLaunchArgument('planner_to_ocs2_yaw', default_value='0.0'),
        planner,
        bridge,
    ])
