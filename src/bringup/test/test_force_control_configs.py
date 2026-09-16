"""Check the two force-control configs and resolve profiles without motion."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
FORCE_CONTROL = BRINGUP.parent / 'control' / 'whole_body_force_control'
CONFIG = FORCE_CONTROL / 'config'


def parameters(filename):
    return yaml.safe_load((CONFIG / filename).read_text(encoding='utf-8'))[
        'whole_body_force_control']['ros__parameters']


def load_launch(deployment):
    path = (BRINGUP / 'launch' / deployment
            / f'whole_body_force_control_{deployment}.launch.py')
    spec = importlib.util.spec_from_file_location(f'force_{deployment}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_only_two_deployment_configs_exist():
    assert {path.name for path in CONFIG.iterdir()} == {
        'force_follow_sim.yaml', 'force_follow_real.yaml'}
    for filename in ('force_follow_sim.yaml', 'force_follow_real.yaml'):
        assert parameters(filename)['control_mode'] == 'force_follow'


@pytest.mark.parametrize('profile, velocity_mode, max_offset, base_limit', [
    ('infinite', True, 1000000.0, 1000000.0),
    ('20s', False, 5.20, 5.10),
])
def test_sim_profiles_share_config_and_keep_behavior(
        profile, velocity_mode, max_offset, base_limit, monkeypatch):
    module = load_launch('sim')
    original_share = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name:
                        str(FORCE_CONTROL) if name == 'whole_body_force_control'
                        else original_share(name))
    # Return declarations as data; never construct or execute a ROS process.
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'profile': profile, 'viewer': 'false', 'use_rviz': 'false'})
    nodes = module._launch_nodes(context)
    controller = next(node for node in nodes
                      if node['package'] == 'whole_body_force_control')
    assert controller['parameters'][0] == str(CONFIG / 'force_follow_sim.yaml')
    effective = {}
    for entry in controller['parameters']:
        if isinstance(entry, str):
            effective.update(parameters(Path(entry).name))
        else:
            effective.update(entry)
    assert effective['force_velocity_mode'] is velocity_mode
    assert effective['force_deadband'] == (0.5 if velocity_mode else 0.0)
    assert effective['max_offset'] == max_offset
    assert effective['max_base_delta'] == base_limit
    assert effective['max_velocity'] == 0.25
    assert effective['base_share'] == 0.98
    assert effective['force_axis'] == 'x'
    assert effective['use_sim_time'] is True


def test_real_config_preserves_closed_gates_and_z_test_limits():
    config = parameters('force_follow_real.yaml')
    assert config['armed'] is False
    assert config['reference_output_enabled'] is False
    assert config['enforce_single_target_owner'] is True
    assert config['require_wrench_frame'] is True
    assert config['tare_samples'] == 100
    assert config['force_velocity_mode'] is False
    assert config['force_axis'] == 'z'
    assert [config[f'response_body_{axis}'] for axis in 'xyz'] == [0.0, 0.0, 1.0]
    assert config['base_share'] == config['max_base_delta'] == 0.0
    assert config['max_offset'] == 0.020
    assert config['max_velocity'] == 0.010
    assert config['max_joint_delta'] == 0.050
    assert config['hard_wrench_limit'] == [15.0, 15.0, 15.0, 2.0, 2.0, 2.0]
    assert config['max_wrench_rate'] == [20.0, 20.0, 20.0, 5.0, 5.0, 5.0]


def test_real_launch_defaults_remain_disabled():
    description = load_launch('real').generate_launch_description()
    arguments = {
        action.name: action for action in description.entities
        if isinstance(action, DeclareLaunchArgument)}
    context = LaunchContext()
    for action in arguments.values():
        action.execute(context)
    values = context.launch_configurations
    assert values['hardware_write'] == 'false'
    assert values['force_control_armed'] == 'false'
    assert values['force_reference_output_enabled'] == 'false'
    assert Path(values['force_params_file']).name == 'force_follow_real.yaml'


def test_launches_do_not_reference_deleted_configs():
    obsolete = {
        'force_follow_20s_sim.yaml', 'force_follow_infinite_sim.yaml',
        'force_follow_infinite_real.yaml', 'force_follow_real_z_test.yaml',
        'six_axis_example.yaml'}
    for path in (BRINGUP / 'launch').rglob('*.launch.py'):
        text = path.read_text(encoding='utf-8')
        assert not any(filename in text for filename in obsolete), str(path)
