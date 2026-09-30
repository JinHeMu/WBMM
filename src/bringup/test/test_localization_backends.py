"""Backend exclusivity, frame routing and startup gating without running nodes."""

import importlib.util
from pathlib import Path

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions

from wbmm_localization_launch import launch_support


BRINGUP = Path(__file__).resolve().parents[1]
LOCALIZATION = BRINGUP.parent / 'map' / 'wbmm_localization'


def load(name, monkeypatch):
    spec = importlib.util.spec_from_file_location(name, BRINGUP / 'launch' / name)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    monkeypatch.setattr(module, 'get_package_share_directory', lambda package:
                        str(LOCALIZATION if package == 'wbmm_localization' else BRINGUP))
    return module


def context_for(module, **overrides):
    context = LaunchContext()
    context.launch_configurations.update(overrides)
    for action in module.generate_launch_description().entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def includes(actions, context):
    result = {}
    for action in actions:
        if isinstance(action, IncludeLaunchDescription):
            action.launch_description_source.get_launch_description(context)
            name = Path(action.launch_description_source.location).name
            result[name] = {key: perform_substitutions(context,
                           normalize_to_list_of_substitutions(value))
                           for key, value in action.launch_arguments}
    return result


@pytest.mark.parametrize('backend,entry', [
    ('amcl', 'amcl_localization.launch.py'),
    ('cartographer_mapping', 'cartographer_mapping.launch.py'),
    ('cartographer_localization', 'cartographer_localization.launch.py'),
    ('none', None),
    ('slam_toolbox', None),
])
def test_only_selected_global_backend_is_active(backend, entry, monkeypatch):
    module = load('localization.launch.py', monkeypatch)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = context_for(module, localization_backend=backend, start_slam='true')
    actions = module._make_nodes(context)
    assert actions[0]['condition'].evaluate(context)
    assert actions[1]['condition'].evaluate(context) == (backend == 'slam_toolbox')
    selected = includes(actions, context)
    assert list(selected) == ([entry] if entry else [])
    if entry:
        assert selected[entry]['odom_topic'] == '/odometry/filtered'


@pytest.mark.parametrize('backend', ['amcl', 'cartographer_localization'])
def test_saved_map_routes_planner_into_map_and_enables_gate(backend, monkeypatch):
    module = load('wbmm.launch.py', monkeypatch)
    import launch_ros.actions
    monkeypatch.setattr(launch_ros.actions, 'Node', lambda **kwargs: kwargs)
    context = context_for(module, start_localization='true', start_planning='true',
                          start_ocs2='true', localization_backend=backend,
                          odom_topic='/custom/controller_odom')
    actions = module._make_actions(context)
    selected = includes(actions, context)
    planner = selected['wbmm_planning.launch.py']
    assert planner['world_frame'] == 'map'
    assert planner['bridge_world_frame'] == 'odom'
    assert planner['odom_topic'] == '/odometry/filtered_map'
    assert planner['wait_for_localization'] == 'true'
    assert planner['localization_backend'] == backend
    assert selected['ocs2.launch.py']['odom_topic'] == '/custom/controller_odom'
    relay = next(action for action in actions if isinstance(action, dict))
    assert relay['parameters'][0]['odom_topic'] == '/odometry/filtered'
    assert relay['parameters'][0]['use_sim_time'] is False


def test_explicit_skip_wait_and_mapping_do_not_add_saved_map_gate(monkeypatch):
    module = load('wbmm.launch.py', monkeypatch)
    import launch_ros.actions
    monkeypatch.setattr(launch_ros.actions, 'Node', lambda **kwargs: kwargs)
    for backend, wait_setting in [('cartographer_mapping', 'auto'), ('amcl', 'false')]:
        context = context_for(module, start_localization='true', start_planning='true',
                              localization_backend=backend, wait_for_localization=wait_setting)
        selected = includes(module._make_actions(context), context)
        assert selected['wbmm_planning.launch.py']['wait_for_localization'] == 'false'


def test_planner_gate_receives_both_actions_and_expected_backend(monkeypatch, tmp_path):
    module = load('wbmm_planning.launch.py', monkeypatch)
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    urdf, esdf = tmp_path / 'robot.urdf', tmp_path / 'map.npz'
    urdf.touch()
    esdf.touch()
    captured = []
    monkeypatch.setattr(launch_support, 'gate_until_ready',
                        lambda *args: captured.append(args) or ['gate'])
    context = context_for(module, urdf_file=str(urdf), esdf_file=str(esdf),
                          wait_for_localization='true',
                          localization_backend='cartographer_localization')
    assert module._make_nodes(context) == ['gate']
    actions, use_sim_time, backend, timeout, topic = captured[0]
    assert [action['package'] for action in actions] == ['wbmm_planner_ros', 'wbmm_trajectory_to_mpc']
    assert (use_sim_time, backend, timeout, topic) == (
        False, 'cartographer_localization', 30.0, '/localization/status')
