"""Check force-control base/override configs and launch profiles."""

import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
import pytest
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
FORCE_CONTROL = BRINGUP.parent / 'control' / 'whole_body_force_control'
COMMON_CONFIG = BRINGUP / 'config' / 'common'
REAL_CONFIG = BRINGUP / 'config' / 'real'
SIM_CONFIG = BRINGUP / 'config' / 'sim'


def load_yaml(path):
    return yaml.safe_load(path.read_text(encoding='utf-8')) or {}


def deep_merge(base, override):
    result = dict(base)
    for key, value in override.items():
        if (key in result and isinstance(result[key], dict)
                and isinstance(value, dict)):
            result[key] = deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_force_params(layers, node_name='whole_body_force_control'):
    data = {}
    for layer in layers:
        if isinstance(layer, (str, Path)):
            layer = load_yaml(Path(layer))
        data = deep_merge(data, layer)
    return data[node_name]['ros__parameters']


def flatten(values, prefix=''):
    result = {}
    for key, value in values.items():
        full_key = f'{prefix}.{key}' if prefix else key
        if isinstance(value, dict):
            result.update(flatten(value, full_key))
        else:
            result[full_key] = value
    return result


def load_launch(profile):
    if profile == 'real':
        filename = 'whole_body_force_control.launch.py'
    elif profile == 'sim':
        filename = 'whole_body_force_control_profiles.launch.py'
    else:
        raise ValueError(profile)
    path = BRINGUP / 'launch' / filename
    spec = importlib.util.spec_from_file_location(f'force_{profile}', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_force_control_config_layers_are_centralized():
    assert (COMMON_CONFIG / 'force_control.yaml').is_file()
    assert (REAL_CONFIG / 'force_control.yaml').is_file()
    assert (SIM_CONFIG / 'force_control.yaml').is_file()

    for paths in (
        (COMMON_CONFIG / 'force_control.yaml',
         SIM_CONFIG / 'force_control.yaml'),
        (COMMON_CONFIG / 'force_control.yaml',
         REAL_CONFIG / 'force_control.yaml'),
    ):
        config = flatten(load_force_params(paths))
        assert len(config['admittance.selected_axes']) == 6
        assert all(isinstance(value, bool)
                   for value in config['admittance.selected_axes'])
        assert config['state_frame'] == 'odom'
        assert 'control_mode' not in config
        assert 'admittance_axes' not in config
        assert 'admittance.max_offset' not in config
        assert 'whole_body.max_base_delta' not in config
        assert 'whole_body.max_joint_delta' not in config
        assert 'force_sensor.max_wrench_rate' not in config


@pytest.mark.parametrize('profile, stiffness_x, damping_x', [
    ('infinite', 0.0, 28.0),
    ('20s', 1.0, 2.0),
])
def test_sim_profiles_use_admittance_limits(
        profile, stiffness_x, damping_x, monkeypatch):
    module = load_launch('sim')
    original_share = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name:
                        str(BRINGUP) if name == 'tracer_jaka_bringup'
                        else original_share(name))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'profile': profile, 'viewer': 'false', 'use_rviz': 'false'})
    nodes = module._launch_nodes(context)
    controller = next(
        node for node in nodes
        if node.get('executable') == 'whole_body_force_control_node')
    assert controller['parameters'][0] == str(
        COMMON_CONFIG / 'force_control.yaml')
    assert controller['parameters'][1] == str(SIM_CONFIG / 'force_control.yaml')
    effective = flatten(deep_merge(
        load_yaml(COMMON_CONFIG / 'force_control.yaml'),
        load_yaml(SIM_CONFIG / 'force_control.yaml'))
        ['whole_body_force_control']['ros__parameters'])
    for entry in controller['parameters'][2:]:
        effective.update(entry)
    assert effective['admittance.selected_axes'] == [
        True, False, False, False, False, False]
    assert effective['admittance.stiffness'][0] == stiffness_x
    assert effective['admittance.damping'][0] == damping_x
    assert effective['admittance.max_velocity'][0] == 0.25
    assert effective['whole_body.base_share'] == 0.98
    assert 'admittance.max_offset' not in effective
    assert 'whole_body.max_base_delta' not in effective
    assert 'whole_body.max_joint_delta' not in effective
    assert effective['use_sim_time'] is True

    processor = next(
        node for node in nodes
        if node.get('executable') == 'force_sensor_processor_node')
    sensor = flatten(load_force_params(
        [COMMON_CONFIG / 'force_control.yaml',
         SIM_CONFIG / 'force_control.yaml'],
        'force_sensor_processor'))
    for entry in processor['parameters'][2:]:
        sensor.update(entry)
    assert sensor['force_sensor.sensor_frame'] == 'jk_se_vi_200_link'
    assert sensor['topics.processed_wrench'] == effective['topics.wrench']


@pytest.mark.parametrize('profile, stiffness, damping, base_share', [
    ('three_axis_admittance',
     [150.0, 150.0, 150.0, 0.0, 0.0, 0.0],
     [45.0, 45.0, 45.0, 4.5, 4.5, 4.5],
     0.40),
    ('three_axis_follow',
     [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
     [50.0, 50.0, 50.0, 4.5, 4.5, 4.5],
     0.80),
])
def test_three_axis_example_profiles(
        profile, stiffness, damping, base_share, monkeypatch):
    module = load_launch('sim')
    original_share = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name:
                        str(BRINGUP) if name == 'tracer_jaka_bringup'
                        else original_share(name))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'profile': profile, 'viewer': 'false', 'use_rviz': 'false'})
    nodes = module._launch_nodes(context)
    controller = next(
        node for node in nodes
        if node.get('executable') == 'whole_body_force_control_node')
    effective = flatten(deep_merge(
        load_yaml(COMMON_CONFIG / 'force_control.yaml'),
        load_yaml(SIM_CONFIG / 'force_control.yaml'))
        ['whole_body_force_control']['ros__parameters'])
    for entry in controller['parameters'][2:]:
        effective.update(entry)
    assert effective['admittance.selected_axes'] == [
        True, True, True, False, False, False]
    assert effective['admittance.stiffness'] == stiffness
    assert effective['admittance.damping'] == damping
    assert effective['whole_body.base_share'] == base_share


def test_sensor_z_sim_uses_world_frame_translation_admittance(monkeypatch):
    module = load_launch('sim')
    original_share = module.get_package_share_directory
    monkeypatch.setattr(module, 'get_package_share_directory', lambda name:
                        str(BRINGUP) if name == 'tracer_jaka_bringup'
                        else original_share(name))
    monkeypatch.setattr(module, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    context.launch_configurations.update({
        'profile': 'sensor_z', 'viewer': 'false', 'use_rviz': 'false'})
    nodes = module._launch_nodes(context)
    mrt = next(node for node in nodes if node.get('name') == 'wbmm_mrt_node')
    assert Path(mrt['parameters'][0]) == COMMON_CONFIG / 'ocs2.yaml'
    assert Path(mrt['parameters'][1]) == SIM_CONFIG / 'ocs2.yaml'
    config = deep_merge(load_yaml(Path(mrt['parameters'][0])),
                        load_yaml(Path(mrt['parameters'][1])))
    mrt_params = config['wbmm_mrt_node']['ros__parameters']
    assert mrt_params['command_output_enabled'] is True
    assert mrt['parameters'][2]['odom_topic'] == '/wheel/odometry'

    force = flatten(deep_merge(
        load_yaml(COMMON_CONFIG / 'force_control.yaml'),
        load_yaml(SIM_CONFIG / 'force_control.yaml'))
        ['whole_body_force_control']['ros__parameters'])
    force_controller = next(
        node for node in nodes
        if node.get('executable') == 'whole_body_force_control_node')
    force.update(force_controller['parameters'][2])
    assert force['force_sensor.tcp_frame'] == 'tool0'
    assert force['admittance.selected_axes'] == [
        True, True, True, False, False, False]
    assert force['admittance.stiffness'][2] == 150.0
    assert force['whole_body.base_share'] == 0.40
    assert 'admittance.max_offset' not in force
    assert 'whole_body.max_base_delta' not in force
    assert 'whole_body.max_joint_delta' not in force
    assert force['whole_body.max_base_velocity'] == 0.50
    assert force['whole_body.max_joint_velocity'] == 0.50
    sensor = flatten(load_force_params(
        [COMMON_CONFIG / 'force_control.yaml',
         SIM_CONFIG / 'force_control.yaml'],
        'force_sensor_processor'))
    assert sensor['force_sensor.sensor_frame'] == 'jk_se_vi_200_link'
    assert sensor['force_sensor.tcp_frame'] == 'tool0'
    assert sensor['force_sensor.hard_force_norm_limit'] == 20.0
    assert sensor['force_sensor.hard_wrench_limit'][:3] == [20.0, 20.0, 20.0]
    assert sensor['topics.processed_wrench'] == force['topics.wrench']
    assert all(node.get('package') != 'tracer_jaka_mujoco' for node in nodes)


def test_real_config_preserves_closed_gates_and_admittance_limits():
    config = flatten(load_force_params([
        COMMON_CONFIG / 'force_control.yaml',
        REAL_CONFIG / 'force_control.yaml',
    ]))
    assert config['admittance.enable'] is False
    assert config['admittance.output'] is False
    assert config['safety.enforce_single_target_owner'] is True
    assert config['force_sensor.tcp_frame'] == 'tool0'
    assert len(config['admittance.selected_axes']) == 6
    assert config['admittance.mass'] == [3.0, 3.0, 3.0, 0.3, 0.3, 0.3]
    assert config['admittance.damping'] == [45.0, 45.0, 45.0, 4.5, 4.5, 4.5]
    assert config['admittance.stiffness'] == [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    assert config['whole_body.base_share'] == 0.6
    assert 'whole_body.max_base_delta' not in config
    assert 'whole_body.max_joint_delta' not in config
    assert 'admittance.max_offset' not in config
    assert 'force_sensor.max_wrench_rate' not in config
    assert config['whole_body.max_base_velocity'] == 0.20
    assert config['whole_body.max_joint_velocity'] == 0.50
    sensor = flatten(load_force_params([
        COMMON_CONFIG / 'force_control.yaml',
        REAL_CONFIG / 'force_control.yaml',
    ], 'force_sensor_processor'))
    assert sensor['force_sensor.sensor_frame'] == 'jk_se_vi_200_link'
    assert sensor['force_sensor.tcp_frame'] == 'tool0'
    assert sensor['force_sensor.tf_fallback_to_latest'] is False
    assert sensor['force_sensor.hard_force_norm_limit'] == 50.0
    assert sensor['force_sensor.hard_wrench_limit'] == [
        50.0, 50.0, 50.0, 2.0, 2.0, 2.0]
    assert sensor['topics.processed_wrench'] == config['topics.wrench']


def test_fts_broadcaster_and_compliance_use_actual_sensor_link():
    description = BRINGUP.parent / 'robotics' / 'tracer_jaka_description'
    controllers = yaml.safe_load(
        (description / 'config' / 'ros2_controllers.yaml').read_text())
    frame = controllers['fts_broadcaster']['ros__parameters']['frame_id']
    urdf = ET.parse(description / 'urdf' / 'tracer_jaka_zu5.urdf')
    assert frame == flatten(load_force_params([
        COMMON_CONFIG / 'force_control.yaml',
        REAL_CONFIG / 'force_control.yaml',
    ], 'force_sensor_processor'))['force_sensor.sensor_frame']
    assert frame in {link.attrib['name'] for link in urdf.findall('link')}


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
    assert values['admittance.enable'] == 'false'
    assert values['admittance.output'] == 'false'
    force_params = Path(values['force_params_file'])
    assert force_params.name == 'force_control.yaml'
    assert force_params.parent.name == 'real'
    force_base = Path(values['force_base_params_file'])
    assert force_base.name == 'force_control.yaml'
    assert force_base.parent.name == 'common'


def test_launches_do_not_reference_deleted_configs():
    obsolete = {
        'force_follow_20s_sim.yaml', 'force_follow_infinite_sim.yaml',
        'force_follow_infinite_real.yaml', 'force_follow_real_z_test.yaml',
        'six_axis_example.yaml', 'force_follow_sim.yaml',
        'force_follow_real.yaml', 'ocs2_sim.yaml',
    }
    for path in (BRINGUP / 'launch').rglob('*.launch.py'):
        text = path.read_text(encoding='utf-8')
        assert not any(filename in text for filename in obsolete), str(path)
