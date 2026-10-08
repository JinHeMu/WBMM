"""Resolve complete entries without executing drivers or ROS nodes."""

import importlib.util
from pathlib import Path

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from wbmm_bringup_launch import entrypoints
from test_common_launch import describe_actions


ROOT = Path(__file__).resolve().parents[1]
FUNCTIONS = ('hardware', 'mapping', 'localization', 'navigation', 'moveit', 'end_effector')


def defaults(function, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in entrypoints.generate(function).entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def includes(actions, context):
    result = {}
    for action in actions:
        description = action.launch_description_source.get_launch_description(context)
        declared = {a.name for a in description.entities if isinstance(a, DeclareLaunchArgument)}
        arguments = {key: perform_substitutions(context, normalize_to_list_of_substitutions(value))
                     for key, value in action.launch_arguments}
        assert set(arguments) <= declared, (action.launch_description_source.location,
                                            set(arguments) - declared)
        result[Path(action.launch_description_source.location).name] = arguments
    return result


@pytest.fixture
def maps(tmp_path):
    state = tmp_path / 'site.pbstream'
    state.write_bytes(b'test: launch validates paths, not map contents')
    esdf = tmp_path / 'site.npz'
    np.savez(esdf, frame_id=np.str_('map'))
    return {'state_file': str(state), 'esdf_file': str(esdf)}


@pytest.mark.parametrize('backend', ('sim', 'real'))
@pytest.mark.parametrize('function', FUNCTIONS)
def test_complete_entries_forward_declared_arguments_and_motion_gate(function, backend, maps):
    overrides = {'backend': backend, 'viewer': 'false', 'use_rviz': 'false'}
    if function == 'localization' or (function == 'navigation' and backend == 'real'):
        overrides['state_file'] = maps['state_file']
    if function == 'navigation':
        overrides['esdf_file'] = maps['esdf_file']
    context = defaults(function, **overrides)
    entries = includes(entrypoints._compose(context, function), context)
    assert len(entries) == (1 if function in ('hardware', 'end_effector') or
                            (function == 'navigation' and backend == 'sim') else 2)
    for name, arguments in entries.items():
        if 'hardware_write' in arguments:
            assert arguments['hardware_write'] == ('true' if backend == 'sim' else 'false')
        if 'use_sim_time' in arguments:
            assert arguments['use_sim_time'] == str(backend == 'sim').lower()
    hardware_name = ('mujoco_hardware_interface.launch.py' if backend == 'sim'
                     else 'wbmm_hardware_interface.launch.py')
    if hardware_name in entries:
        assert entries[hardware_name]['publish_odom_tf'] == str(
            function in ('hardware', 'moveit')).lower()
    if function == 'moveit' and backend == 'real':
        assert entries[hardware_name]['arm_controller_name'] == 'arm_trajectory_controller'
    if function == 'end_effector' and backend == 'sim':
        assert entries['mujoco_ocs2_ee_hold.launch.py']['command_output_enabled'] == 'true'
    if function == 'navigation' and backend == 'real':
        assert entries['remani_mpc_localized.launch.py']['wait_for_localization'] == 'true'
        assert entries['remani_mpc_localized.launch.py']['world_frame'] == 'odom'
        assert 'esdf_file' not in entries['remani_mpc_localized.launch.py']

    # Follow the includes and algorithm OpaqueFunctions, never execute nodes.
    # This catches broken config/model resources and downstream forwarding too.
    visited = set()
    describe_actions(entrypoints._compose(context, function), context, visited)
    if function in ('mapping', 'localization'):
        assert 'localization.launch.py' in visited
    if function == 'navigation':
        assert 'wbmm_planning.launch.py' in visited
        assert 'ocs2.launch.py' in visited


@pytest.mark.parametrize('function', ('moveit', 'end_effector', 'navigation'))
def test_explicit_real_write_gate_reaches_composed_controller(function, maps):
    context = defaults(function, backend='real', hardware_write='true', **maps)
    entries = includes(entrypoints._compose(context, function), context)
    assert all(arguments['hardware_write'] == 'true' for arguments in entries.values())


@pytest.mark.parametrize('frame', ('odom', ['map'], None))
def test_real_navigation_rejects_wrong_esdf_frame_before_backend(frame, maps, tmp_path):
    esdf = tmp_path / 'bad.npz'
    np.savez(esdf, **({} if frame is None else {'frame_id': frame}))
    context = defaults('navigation', backend='real', state_file=maps['state_file'],
                       esdf_file=str(esdf))
    with pytest.raises(RuntimeError, match='frame_id=map'):
        entrypoints._compose(context, 'navigation')


def test_mapping_preserves_existing_map_and_localization_requires_map(tmp_path):
    saved = tmp_path / 'existing.pbstream'
    saved.write_bytes(b'existing')
    with pytest.raises(RuntimeError, match='already exists'):
        entrypoints._compose(defaults('mapping', backend='real', save_state_file=str(saved)),
                             'mapping')
    assert saved.read_bytes() == b'existing'
    with pytest.raises(RuntimeError, match='state_file'):
        entrypoints._compose(defaults('localization', backend='real'), 'localization')


def test_deployment_environment_reaches_hardware(monkeypatch):
    monkeypatch.setenv('JAKA_IP', '10.10.10.2')
    monkeypatch.setenv('CAN_IFACE', 'can1')
    context = defaults('moveit', backend='real')
    arguments = includes(entrypoints._compose(context, 'moveit'), context)[
        'wbmm_hardware_interface.launch.py']
    assert arguments['jaka_robot_ip'] == '10.10.10.2'
    assert arguments['can_port'] == 'can1'


def test_planner_tuning_changes_speed_without_overriding_frame_or_files(tmp_path, maps):
    path = ROOT / 'launch' / 'wbmm_planning.launch.py'
    spec = importlib.util.spec_from_file_location('planner_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    context = LaunchContext()
    context.launch_configurations.update(esdf_file=maps['esdf_file'], world_frame='map',
        urdf_file=str(Path(get_package_share_directory('tracer_jaka_description')) /
                      'urdf' / 'tracer_jaka_zu5.urdf'),
        planner_config=str(ROOT / 'config' / 'real' / 'planner.yaml'))
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    captured = []
    module.Node = lambda **kwargs: captured.append(kwargs)
    module._make_nodes(context)
    parameters = captured[0]['parameters'][0]
    assert parameters['max_linear_velocity'] == 0.1
    assert parameters['world_frame'] == 'map'
    assert parameters['esdf_file'] == maps['esdf_file']
    config = tmp_path / 'override.yaml'
    config.write_text(yaml.safe_dump({'wbmm_planner_node': {
        'ros__parameters': {'world_frame': 'odom'}}}))
    context.launch_configurations['planner_config'] = str(config)
    with pytest.raises(RuntimeError, match='only planner tuning'):
        module._make_nodes(context)
