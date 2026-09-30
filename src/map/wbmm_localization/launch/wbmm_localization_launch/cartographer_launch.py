"""Shared Cartographer launch implementation; never starts hardware or EKF."""

import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from .launch_support import guard_localization


def value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def validate_inputs(mode, config, state, save, resolution=0.05, readiness=None):
    """Check inputs before a composing launch starts any of its processes."""
    config = Path(config).resolve()
    if not config.is_file():
        raise RuntimeError(f'Cartographer Lua config does not exist: {config}')
    if mode == 'mapping' and state:
        raise RuntimeError('Fresh mapping does not load state_file. '
                           'Use cartographer_localization to load an existing map.')
    if mode == 'localization' and (
            not state or not Path(state).is_file() or
            Path(state).suffix != '.pbstream'):
        raise RuntimeError('Cartographer localization requires an existing state_file (.pbstream).')
    if save and (Path(save).suffix != '.pbstream' or
                 not Path(save).resolve().parent.is_dir()):
        raise RuntimeError('save_state_file requires a .pbstream suffix and an existing parent directory.')
    if mode == 'mapping' and save and Path(save).exists():
        raise RuntimeError('save_state_file already exists. Choose a new filename to preserve existing maps.')
    resolution = float(resolution)
    if not math.isfinite(resolution) or resolution <= 0.0:
        raise RuntimeError('resolution must be finite and positive.')
    if readiness is not None and not Path(readiness).is_file():
        raise RuntimeError(f'readiness_config does not exist: {readiness}')
    return config, resolution


def make_nodes(context, mode):
    share = Path(get_package_share_directory('wbmm_localization'))
    state = value(context, 'state_file')
    save = value(context, 'save_state_file')
    readiness = (value(context, 'readiness_config') if mode == 'localization' and
                 value(context, 'start_readiness').lower() == 'true' else None)
    config, resolution = validate_inputs(
        mode, value(context, 'cartographer_config') or
        share / 'config' / f'cartographer_{mode}.lua', state, save,
        value(context, 'resolution'), readiness)
    use_sim_time = value(context, 'use_sim_time').lower() == 'true'
    args = ['-configuration_directory', str(config.parent),
            '-configuration_basename', config.name,
            '-collect_metrics=true']
    if mode == 'localization':
        args += ['-load_state_filename', str(Path(state).resolve()),
                 '-load_frozen_state=true']
    elif save:
        args += ['-save_state_filename', str(Path(save).resolve())]
    nodes = [Node(
        package='cartographer_ros', executable='cartographer_node',
        name='cartographer_node', output='screen', arguments=args,
        parameters=[{'use_sim_time': use_sim_time}],
        remappings=[('scan', value(context, 'scan_topic')),
                    ('imu', value(context, 'imu_topic')),
                    ('odom', value(context, 'odom_topic'))]),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', output='screen',
             arguments=['-resolution', str(resolution),
                        '-include_frozen_submaps=true',
                        f'-include_unfrozen_submaps={str(mode == "mapping").lower()}'],
             parameters=[{'use_sim_time': use_sim_time}],
             remappings=[('map', '/cartographer/map')]),
        Node(package='wbmm_localization', executable='cartographer_ground_map',
             name='cartographer_ground_map', output='screen',
             parameters=[{'use_sim_time': use_sim_time}])]
    if readiness is not None:
        nodes.append(Node(
            package='wbmm_localization', executable='localization_readiness',
            name='localization_readiness', output='screen',
            parameters=[readiness, {'use_sim_time': use_sim_time,
                        'backend': 'cartographer_localization',
                        'scan_topic': value(context, 'scan_topic'),
                        'imu_topic': value(context, 'imu_topic'),
                        'odom_topic': value(context, 'odom_topic')}]))
    return nodes


def description(mode):
    share = Path(get_package_share_directory('wbmm_localization'))
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('scan_topic', default_value='/scan'),
        DeclareLaunchArgument('imu_topic', default_value='/imu/data'),
        DeclareLaunchArgument('odom_topic', default_value='/odometry/filtered'),
        DeclareLaunchArgument('state_file', default_value=''),
        DeclareLaunchArgument('save_state_file', default_value=''),
        DeclareLaunchArgument('cartographer_config', default_value=''),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('start_readiness', default_value='true'),
        DeclareLaunchArgument('readiness_config', default_value=str(
            share / 'config' / 'readiness.yaml')),
        OpaqueFunction(function=lambda context: guard_localization(
            make_nodes(context, mode), value(context, 'use_sim_time').lower() == 'true',
            False, True, value(context, 'odom_topic'))),
    ])
