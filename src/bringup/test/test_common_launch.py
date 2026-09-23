"""Static composition checks for the backend-agnostic WBMM launch tree."""

import ast
import importlib.util
from pathlib import Path

from launch import LaunchContext
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction,
)
from launch.utilities import normalize_to_list_of_substitutions, perform_substitutions
import numpy as np
import pytest
import yaml


BRINGUP = Path(__file__).resolve().parents[1]
LAUNCH_DIR = BRINGUP / 'launch'
LAUNCH_FILES = {path.name: path for path in LAUNCH_DIR.glob('*.launch.py')}

CORE_ALGORITHM_LAUNCHES = {
    'localization.launch.py',
    'moveit.launch.py',
    'ocs2.launch.py',
    'remani.launch.py',
    'whole_body_force_control.launch.py',
    'remani_mpc.launch.py',
    'remani_mpc_localized.launch.py',
}

BACKEND_LAUNCHES = {
    'wbmm_hardware_interface.launch.py',
    'mujoco_hardware_interface.launch.py',
}


def load_module(path):
    spec = importlib.util.spec_from_file_location(path.stem.replace('.', '_'), path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def launch_arguments(path):
    tree = ast.parse(path.read_text(encoding='utf-8'))
    return {
        call.args[0].value
        for call in ast.walk(tree)
        if isinstance(call, ast.Call)
        and isinstance(call.func, ast.Name)
        and call.func.id == 'DeclareLaunchArgument'
    }


def test_core_algorithm_launches_do_not_start_backends():
    forbidden_packages = {
        'controller_manager', 'tracer_base', 'hipnuc_imu', 'lakibeam1',
        'realsense2_camera', 'tracer_jaka_mujoco', 'dh_gripper_driver',
    }
    forbidden_launch_includes = {
        'wbmm_hardware_interface.launch.py',
        'mujoco_hardware_interface.launch.py',
        }
    for name in CORE_ALGORITHM_LAUNCHES:
        path = LAUNCH_FILES[name]
        assert 'backend' not in launch_arguments(path)
        content = path.read_text(encoding='utf-8')
        assert not any(include in content for include in forbidden_launch_includes)
        tree = ast.parse(content)
        packages = {
            keyword.value.value
            for call in ast.walk(tree)
            if isinstance(call, ast.Call)
            for keyword in call.keywords
            if keyword.arg == 'package'
            and isinstance(keyword.value, ast.Constant)
        }
        assert not packages.intersection(forbidden_packages), name


def test_wbmm_top_level_defaults_to_no_backend():
    path = LAUNCH_FILES['wbmm.launch.py']
    content = path.read_text(encoding='utf-8')
    assert 'default_value="none"' in content
    assert 'start_localization", default_value="false"' in content
    assert 'start_ocs2", default_value="false"' in content
    assert 'start_remani", default_value="false"' in content
    assert 'start_force_control", default_value="false"' in content
    assert 'start_moveit", default_value="false"' in content


def test_removed_entry_and_offset_arguments_have_no_bringup_references():
    assert not (LAUNCH_DIR / 'bringup.launch.py').exists()
    for path in LAUNCH_FILES.values():
        content = path.read_text(encoding='utf-8')
        assert 'bringup.launch.py' not in content
        assert 'static_esdf_offset_' not in content
        assert 'planner_to_ocs2_' not in content


def test_remani_requires_tf_between_different_frames():
    module = load_module(LAUNCH_FILES['remani.launch.py'])
    context = LaunchContext()
    context.launch_configurations.update({
        'planner_frame': 'map', 'target_frame': 'odom',
        'use_tf_transform': 'false',
    })
    with pytest.raises(RuntimeError, match='use_tf_transform'):
        module._make_include(context)


def test_moveit_uses_only_hardware_write_as_motion_gate():
    args = launch_arguments(LAUNCH_FILES['moveit.launch.py'])
    assert 'allow_trajectory_execution' not in args
    assert 'safety_release' not in args
    assert 'use_servo' not in args
    assert 'use_joy' not in args
    assert 'hardware_write' in args


def test_real_ros2_control_keeps_controller_names_and_uses_canonical_config():
    path = LAUNCH_FILES['wbmm_hardware_interface.launch.py']
    content = path.read_text(encoding='utf-8')
    tree = ast.parse(content)
    manager_nodes = []
    for call in ast.walk(tree):
        if not (isinstance(call, ast.Call) and
                isinstance(call.func, ast.Name) and call.func.id == 'Node'):
            continue
        keywords = {keyword.arg: keyword.value for keyword in call.keywords}
        package = keywords.get('package')
        executable = keywords.get('executable')
        if (isinstance(package, ast.Constant) and
                package.value == 'controller_manager' and
                isinstance(executable, ast.Constant) and
                executable.value == 'ros2_control_node'):
            manager_nodes.append(keywords)

    assert len(manager_nodes) == 1
    assert 'name' not in manager_nodes[0]
    assert '"--param-file", jaka_controllers' in content


def describe_actions(actions, context, visited):
    """Resolve launch composition, but never execute Node/Process actions."""
    for action in actions:
        if action.condition is not None and not action.condition.evaluate(context):
            continue
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
        elif isinstance(action, OpaqueFunction):
            describe_actions(action.execute(context) or [], context, visited)
        elif isinstance(action, TimerAction):
            describe_actions(action.actions, context, visited)
        elif isinstance(action, IncludeLaunchDescription):
            action.launch_description_source.get_launch_description(context)
            location = action.launch_description_source.location
            assert Path(location).is_file(), location
            name = Path(location).name
            if name not in LAUNCH_FILES:
                continue
            child = LaunchContext()
            child.launch_configurations.update(context.launch_configurations)
            declared = launch_arguments(LAUNCH_FILES[name])
            for key, value in action.launch_arguments:
                assert key in declared, f'{name}: undeclared forwarded argument {key}'
                child.launch_configurations[key] = perform_substitutions(
                    context, normalize_to_list_of_substitutions(value))
            visited.add(name)
            module = load_module(LAUNCH_FILES[name])
            describe_actions(
                module.generate_launch_description().entities, child, visited)


@pytest.mark.parametrize('entry', [
    'wbmm_hardware_interface.launch.py',
    'mujoco_hardware_interface.launch.py',
    'wbmm.launch.py',
    'whole_body_force_control.launch.py',
    'remani_mpc.launch.py',
    'remani_mpc_localized.launch.py',
])
def test_deployment_composition_resolves_without_starting_nodes(entry, tmp_path):
    map_file = tmp_path / 'site.yaml'
    map_file.write_text('image: site.pgm\n', encoding='utf-8')
    esdf_file = tmp_path / 'site.npz'
    np.savez_compressed(esdf_file, frame_id=np.str_('map'))
    context = LaunchContext()
    context.launch_configurations.update({
        'map_file': str(map_file),
        'static_esdf_file': str(esdf_file),
        'esdf_file': str(esdf_file),
        'map2d_yaml': str(map_file),
        'publish_ply_mesh': 'false',
    })
    module = load_module(LAUNCH_FILES[entry])
    visited = set()
    describe_actions(module.generate_launch_description().entities, context, visited)
    assert 'bringup.launch.py' not in visited


def test_canonical_interface_contract_matches_real_hardware_defaults():
    contract = yaml.safe_load(
        (BRINGUP / 'config' / 'common' / 'interface.yaml').read_text(
            encoding='utf-8'))['wbmm_ros_interface']
    topics = contract['topics']

    real_launch = (
        LAUNCH_DIR / 'wbmm_hardware_interface.launch.py').read_text(
        encoding='utf-8')
    mujoco_launch = (
        LAUNCH_DIR / 'mujoco_hardware_interface.launch.py').read_text(
        encoding='utf-8')

    assert topics['wheel_odometry']['name'] == '/wheel/odometry'
    assert topics['imu']['name'] == '/imu/data'
    for content in (real_launch, mujoco_launch):
        assert topics['wheel_odometry']['name'] in content
        assert topics['imu']['name'] in content
        assert topics['scan']['name'] in content
        assert topics['fts_wrench']['name'] in content
    assert '/IMU_data' not in real_launch
    assert 'default_value="/odom"' not in real_launch

    imu_config = yaml.safe_load(
        (BRINGUP.parent / 'drivers' / 'sensors' / 'hipnuc_imu' / 'config'
         / 'hipnuc_config.yaml').read_text(encoding='utf-8'))
    assert (imu_config['IMU_publisher']['ros__parameters']['imu_topic']
            == topics['imu']['name'])

    ekf_common = yaml.safe_load(
        (BRINGUP / 'config' / 'common' / 'ekf.yaml').read_text(
            encoding='utf-8'))['ekf_filter_node']['ros__parameters']
    assert ekf_common['odom0'] == topics['wheel_odometry']['name']
    assert ekf_common['imu0'] == topics['imu']['name']


@pytest.mark.parametrize('backend, expected_launch', [
    ('real', 'wbmm_hardware_interface.launch.py'),
    ('mujoco', 'mujoco_hardware_interface.launch.py'),
])
def test_wbmm_backend_composition_declares_all_forwarded_args(
        backend, expected_launch):
    context = LaunchContext()
    context.launch_configurations.update({'hardware_backend': backend})
    module = load_module(LAUNCH_FILES['wbmm.launch.py'])
    visited = set()
    describe_actions(module.generate_launch_description().entities, context, visited)
    assert expected_launch in visited


def test_esdf_validation_launch_and_task_are_wired():
    launch_path = LAUNCH_DIR / 'ocs2_esdf_validation.launch.py'
    assert launch_path.is_file()

    ocs2_args = launch_arguments(LAUNCH_FILES['ocs2.launch.py'])
    assert 'esdf_file' in ocs2_args
    assert 'world_frame' in ocs2_args

    wbmm_args = launch_arguments(LAUNCH_FILES['wbmm.launch.py'])
    assert 'esdf_file' in wbmm_args
    assert 'world_frame' in wbmm_args

    task_file = BRINGUP / 'config' / 'sim' / 'task_esdf.info'
    assert task_file.is_file()
    text = task_file.read_text(encoding='utf-8')
    assert 'backend esdf' in text
    assert 'activate            true' in text


def test_remani_tracking_launch_wires_esdf_and_phase_bridge():
    path = LAUNCH_DIR / 'remani_tracking.launch.py'
    assert path.is_file()
    content = path.read_text(encoding='utf-8')
    assert '"start_ocs2": "true"' in content
    assert '"start_remani": "true"' in content
    assert '"use_target": "true"' in content
    assert 'remani_phase_bridge.py' in content

    declared = launch_arguments(path)
    assert {
        'task_file', 'static_esdf_file', 'esdf_file', 'remani_config',
    }.issubset(declared)

    task_file = BRINGUP / 'config' / 'sim' / 'task_esdf_tracking.info'
    assert task_file.is_file()
    task = task_file.read_text(encoding='utf-8')
    assert 'modeSwitch' in task
    assert 'initialPhase 0' in task
    assert ('backend esdf' in task or 'backend             esdf' in task)
    assert 'activate            true' in task
    assert 'frame "map"' in task

    assert 'default_value="map"' in content
    assert '/home/a/WBMM/maps/map1/site_remani.npz' in content
    assert 'remani_planner_frame": "map"' in content
    assert 'publish_map_odom_tf' in content

    rviz_config = BRINGUP / 'rviz' / 'remani_tracking_map1.rviz'
    assert rviz_config.is_file()
    assert 'Fixed Frame: map' in rviz_config.read_text(encoding='utf-8')

    remani_profile = BRINGUP / 'config' / 'sim' / 'remani_tracking.yaml'
    assert remani_profile.is_file()
    assert 'tracking_error_replan_enabled: false' in remani_profile.read_text(
        encoding='utf-8')


def test_localized_launch_defaults_to_map1():
    content = (
        LAUNCH_DIR / 'remani_mpc_localized.launch.py').read_text(
            encoding='utf-8')
    assert '/home/a/WBMM/maps/map1/site_remani.npz' in content
    assert '/home/a/WBMM/maps/map1/site_2d.yaml' in content
