"""Check the algorithm-only common launches without starting any ROS node."""

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


BRINGUP = Path(__file__).resolve().parents[1]
LAUNCH_FILES = {path.name: path for path in (BRINGUP / 'launch').rglob('*.launch.py')}


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


def test_common_launches_are_algorithm_only():
    forbidden_packages = {'controller_manager', 'tracer_base', 'hipnuc_imu',
                          'lakibeam1', 'tracer_jaka_mujoco', 'dh_gripper_driver'}
    for path in (BRINGUP / 'launch' / 'common').glob('*.launch.py'):
        assert 'backend' not in launch_arguments(path)
        tree = ast.parse(path.read_text(encoding='utf-8'))
        packages = {
            keyword.value.value
            for call in ast.walk(tree)
            if isinstance(call, ast.Call)
            for keyword in call.keywords
            if keyword.arg == 'package' and isinstance(keyword.value, ast.Constant)
        }
        assert not packages.intersection(forbidden_packages)


def test_removed_entry_and_offset_arguments_have_no_bringup_references():
    assert not (BRINGUP / 'launch' / 'common' / 'bringup.launch.py').exists()
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
    common_args = launch_arguments(LAUNCH_FILES['moveit.launch.py'])
    real_args = launch_arguments(LAUNCH_FILES['moveit_real.launch.py'])
    for args in (common_args, real_args):
        assert 'allow_trajectory_execution' not in args
        assert 'safety_release' not in args
        assert 'hardware_write' in args


def describe_actions(actions, context, visited):
    """Resolve bringup composition, but never execute Node/Process actions."""
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
            describe_actions(module.generate_launch_description().entities, child, visited)


@pytest.mark.parametrize('entry', [
    'ocs2_sim.launch.py', 'remani_mpc_sim.launch.py',
    'ocs2_esdf_validation.launch.py', 'ocs2_real.launch.py',
    'remani_mpc_real.launch.py', 'remani_mpc_localized_real.launch.py',
    'slam_sim.launch.py', 'real_slam.launch.py',
    'moveit_sim.launch.py', 'moveit_real.launch.py', 'servo.launch.py',
])
def test_deployment_composition_resolves_without_starting_nodes(entry, tmp_path):
    map_file = tmp_path / 'site.yaml'
    map_file.write_text('image: site.pgm\n', encoding='utf-8')
    esdf_file = tmp_path / 'site.npz'
    np.savez_compressed(esdf_file, frame_id=np.str_('map'))
    context = LaunchContext()
    context.launch_configurations.update({
        'map_file': str(map_file), 'static_esdf_file': str(esdf_file),
        'esdf_file': str(esdf_file), 'map2d_yaml': str(map_file),
        'publish_ply_mesh': 'false',
    })
    module = load_module(LAUNCH_FILES[entry])
    visited = set()
    describe_actions(module.generate_launch_description().entities, context, visited)
    assert 'bringup.launch.py' not in visited
