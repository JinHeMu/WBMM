"""Daily-use entries; keep deployment choices out of algorithm launches."""

import os
from pathlib import Path

import numpy as np
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _text(value):
    return str(value).lower() if isinstance(value, bool) else str(value)


def _value(context, key):
    return LaunchConfiguration(key).perform(context).strip()


def _file(path, label, suffix=None):
    if not path or not Path(path).is_file() or (suffix and not path.endswith(suffix)):
        raise RuntimeError(f'{label} requires an existing {suffix or "file"}: {path!r}')


def _saved_map(arguments):
    backend = arguments.get('localization_backend', 'cartographer_localization')
    if backend == 'cartographer_localization':
        _file(arguments.get('state_file', ''), 'state_file', '.pbstream')
    elif backend == 'amcl':
        _file(arguments.get('map_file', ''), 'map_file', '.yaml')
    else:
        raise RuntimeError('Saved-map localization requires cartographer_localization or amcl')


def _compose(context, function):
    backend = _value(context, 'backend')
    if backend not in ('real', 'sim'):
        raise RuntimeError('backend must be real or sim')
    share = get_package_share_directory('tracer_jaka_bringup')
    config_path = _value(context, 'config_file') or os.path.join(
        share, 'config', backend, 'startup.yaml')
    with open(config_path, encoding='utf-8') as stream:
        config = yaml.safe_load(stream)
    if not isinstance(config, dict):
        raise RuntimeError('startup config must be a YAML mapping')
    # Paths in startup.yaml may refer to installed package resources or to the
    # workspace containing the original YAML; no workstation-specific defaults.
    candidates = Path(share, 'config', backend, 'startup.yaml').resolve().parents
    workspace = os.environ.get('WBMM_WS') or next((str(path) for path in candidates
        if (path / 'src' / 'bringup' / 'CMakeLists.txt').is_file()), '')

    def expand(value):
        if isinstance(value, str):
            if '{workspace}' in value and not workspace:
                raise RuntimeError('Set WBMM_WS or provide esdf_file in startup.yaml')
            return value.replace('{bringup}', share).replace('{workspace}', workspace)
        return value

    settings = {key: expand(value) for key, value in config.get(function, {}).items()}
    hardware = {key: expand(value) for key, value in config.get('hardware', {}).items()}
    write = _value(context, 'hardware_write')
    if write == 'auto':
        write = 'true' if backend == 'sim' else 'false'
    sim_time = backend == 'sim'
    gui = _value(context, 'use_rviz')
    viewer = _value(context, 'viewer')

    file_keys = {'mapping': ('save_state_file',),
                 'localization': ('state_file', 'map_file'),
                 'navigation': ('state_file', 'map_file', 'esdf_file')}.get(function, ())
    for key in file_keys:
        if key in context.launch_configurations and _value(context, key):
            settings[key] = _value(context, key)

    def include(name, arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(share, 'launch', name)),
            launch_arguments={key: _text(value) for key, value in arguments.items()}.items())

    if function in ('mapping', 'localization') or (function == 'navigation' and not sim_time):
        localization_backend = settings.get('localization_backend', '')
        if localization_backend.startswith('cartographer_'):
            from wbmm_localization_launch.cartographer_launch import validate_inputs
            localization_share = get_package_share_directory('wbmm_localization')
            mode = localization_backend.removeprefix('cartographer_')
            validate_inputs(mode, settings.get('cartographer_config') or os.path.join(
                localization_share, 'config', localization_backend + '.lua'),
                settings.get('state_file', ''), settings.get('save_state_file', ''),
                readiness=(settings.get('readiness_config') or os.path.join(
                    localization_share, 'config', 'readiness.yaml'))
                if mode == 'localization' else None)

    # These entries already compose a full backend and must never get another.
    if function == 'end_effector':
        settings.update(hardware_write=write, use_rviz=gui)
        if sim_time:
            settings['command_output_enabled'] = settings.pop('hardware_write')
            settings['viewer'] = viewer
        else:
            for env, key in (('JAKA_IP', 'robot_ip'), ('JAKA_LOCAL_IP', 'local_ip'),
                             ('CAN_IFACE', 'can_port')):
                settings[key] = os.environ.get(env, hardware.get(
                    {'robot_ip': 'jaka_robot_ip', 'local_ip': 'jaka_local_ip'}.get(key, key), ''))
        return [include('mujoco_ocs2_ee_hold.launch.py' if sim_time
                        else 'real_ocs2_ee_hold.launch.py', settings)]

    if function == 'navigation':
        esdf = settings.pop('esdf_file', '')
        _file(esdf, 'esdf_file', '.npz')
        with np.load(esdf, allow_pickle=False) as archive:
            frame = archive['frame_id'] if 'frame_id' in archive else None
            if frame is None or frame.shape != () or str(frame.item()) != 'map':
                raise RuntimeError('navigation requires scalar ESDF frame_id=map')
        settings.update(hardware_write=write, use_rviz=gui)
        if sim_time:
            if settings.get('state_file') or settings.get('map_file'):
                raise RuntimeError('Simulation navigation uses the virtual ESDF demo; '
                                   'use robot_localization for saved-map localization')
            settings.pop('state_file', None)
            settings.pop('map_file', None)
            settings.update(esdf_file=esdf, viewer=viewer)
            return [include('wbmm_tracking.launch.py', settings)]
        _saved_map(settings)
        settings.update(static_esdf_file=esdf, use_sim_time=False)
        # OCS2 tracks in odom; the planner alone consumes the map-frame ESDF.
        # Do not inject this map-frame ESDF into an odom-frame OCS2 collision term.
        algorithm = include('remani_mpc_localized.launch.py', settings)
    elif function in ('mapping', 'localization'):
        if function == 'localization':
            _saved_map(settings)
        settings.update(start_ekf=True, start_slam=function == 'mapping',
                        use_sim_time=sim_time, use_rviz=gui)
        algorithm = include('localization.launch.py', settings)
    elif function == 'moveit':
        settings.update(use_sim_time=sim_time, use_rviz=gui, hardware_write=write)
        algorithm = include('moveit.launch.py', settings)
    elif function == 'hardware':
        algorithm = None
    else:
        raise RuntimeError(f'Unknown function: {function}')

    # EKF owns odom->base during localization. Otherwise the backend owns it.
    hardware['publish_odom_tf'] = function in ('hardware', 'moveit')
    if sim_time:
        hardware['viewer'] = viewer
        hardware_launch = 'mujoco_hardware_interface.launch.py'
    else:
        hardware.update(hardware_write=write, start_arm_controller=True,
                        arm_controller_name='arm_trajectory_controller'
                        if function == 'moveit' else 'arm_controller')
        for env, key in (('JAKA_IP', 'jaka_robot_ip'), ('JAKA_LOCAL_IP', 'jaka_local_ip'),
                         ('CAN_IFACE', 'can_port')):
            if env in os.environ:
                hardware[key] = os.environ[env]
        hardware_launch = 'wbmm_hardware_interface.launch.py'
    actions = [include(hardware_launch, hardware)]
    if algorithm is not None:
        actions.append(algorithm)
    return actions


def generate(function):
    """Expose only deployment/runtime switches; tuning stays in config files."""
    arguments = [
        DeclareLaunchArgument('backend', default_value='sim', choices=['real', 'sim']),
        DeclareLaunchArgument('config_file', default_value='',
                              description='Startup YAML; default config/<backend>/startup.yaml'),
        DeclareLaunchArgument('hardware_write', default_value='auto',
                              choices=['auto', 'true', 'false'],
                              description='auto: simulation output on, real output off'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('viewer', default_value='true', description='MuJoCo viewer'),
    ]
    if function in ('localization', 'navigation'):
        arguments += [DeclareLaunchArgument('state_file', default_value=''),
                      DeclareLaunchArgument('map_file', default_value='')]
    if function == 'navigation':
        arguments.append(DeclareLaunchArgument('esdf_file', default_value=''))
    if function == 'mapping':
        arguments.append(DeclareLaunchArgument('save_state_file', default_value=''))
    return LaunchDescription(arguments + [OpaqueFunction(
        function=lambda context: _compose(context, function))])
