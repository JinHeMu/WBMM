#!/usr/bin/env python3
"""ROS 2 Humble launch helpers shared by exp0 and exp1."""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def merged_parameters(experiment):
    share = get_package_share_directory('remani_planner')
    result = {}

    def merge(destination, source):
        for key, value in source.items():
            if isinstance(value, dict) and isinstance(destination.get(key), dict):
                merge(destination[key], value)
            else:
                destination[key] = value

    for filename in ('mm_param.yaml', 'remani_planner_param.yaml',
                     f'{experiment}_param.yaml'):
        with open(os.path.join(share, 'config', filename), encoding='utf-8') as stream:
            merge(result, yaml.safe_load(stream))

    result.setdefault('fsm', {}).update({
        'thresh_no_replan_meter': 2.5,
        'fail_safe': True,
    })
    result.setdefault('grid_map', {}).update({
        'resolution': 0.05,
        'map_size_x': 8.0,
        'map_size_y': 8.0,
        'map_size_z': 3.0,
        'local_update_range_x': 17.0,
        'local_update_range_y': 17.0,
        'local_update_range_z': 3.0,
        'obstacles_inflation': 0.0,
        'local_map_margin': 15,
        'ground_height': -0.01,
        'cx': 321.04638671875,
        'cy': 243.44969177246094,
        'fx': 387.229248046875,
        'fy': 387.229248046875,
        'use_depth_filter': True,
        'depth_filter_tolerance': 0.15,
        'depth_filter_maxdist': 5.0,
        'depth_filter_mindist': 0.2,
        'depth_filter_margin': 2,
        'k_depth_scaling_factor': 1000.0,
        'skip_pixel': 2,
        'p_hit': 0.65,
        'p_miss': 0.35,
        'p_min': 0.12,
        'p_max': 0.90,
        'p_occ': 0.80,
        'min_ray_length': 0.1,
        'max_ray_length': 4.5,
        'virtual_ceil_height': 4.0,
        'visualization_truncate_height': 2.8,
        'show_occ_time': False,
        'pose_type': 1,
        'frame_id': 'world',
        'local_bound_inflate': 0.0,
        'show_esdf_time': False,
        'esdf_slice_height': -0.1,
    })
    result.setdefault('manager', {}).update({
        'control_points_distance': 0.4,
        'polyTraj_piece_length': 2.0,
        'feasibility_tolerance': 0.05,
        'drone_id': -1,
    })
    return result


def generate_example_launch_description(experiment):
    params = merged_parameters(experiment)
    share = get_package_share_directory('remani_planner')
    rviz = LaunchConfiguration('rviz')
    auto_start = LaunchConfiguration('auto_start')
    target_type = LaunchConfiguration('target_type')
    return LaunchDescription([
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='Start RViz2 with the example configuration'),
        DeclareLaunchArgument(
            'auto_start', default_value='false',
            description='Automatically trigger planning (normally used with target_type:=2)'),
        DeclareLaunchArgument(
            'target_type', default_value='1',
            description='Goal mode: 1 uses the RViz2 2D Goal Pose, 2 uses preset waypoints'),
        Node(
            package='remani_planner',
            executable='remani_simulator',
            name='remani_simulator',
            output='screen',
            parameters=[params, {
                'map.x_size': 8.0,
                'map.y_size': 8.0,
                'map.z_size': 3.0,
                'map.resolution': 0.05,
                'map.seed': 30,
                'auto_start': ParameterValue(auto_start, value_type=bool),
            }],
        ),
        Node(
            package='remani_planner',
            executable='remani_planner_node',
            name='remani_planner_node',
            output='screen',
            parameters=[params, {
                'fsm.target_type': ParameterValue(target_type, value_type=int),
            }],
            remappings=[
                ('odom_world', '/mm/car/odom'),
                ('joint_state', '/mm/mani/joint_state'),
                ('gripper_state', '/mm/mani/gripper_state'),
                ('gripper_cmd', '/mm_controller_node/gripper_cmd'),
                ('grid_map/odom', '/mm/car/odom'),
                ('planning/trajectory', '/planning/trajectory'),
            ],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(share, 'launch', f'{experiment}.rviz')],
            condition=IfCondition(rviz),
            output='screen',
        ),
    ])
