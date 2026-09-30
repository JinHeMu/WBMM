from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo

from wbmm_localization_launch import cartographer_launch
from wbmm_localization_launch.launch_support import resolve_backend, release_on_success


PACKAGE = Path(__file__).resolve().parents[1]


@pytest.fixture
def context(monkeypatch):
    monkeypatch.setattr(cartographer_launch, 'get_package_share_directory', lambda _: str(PACKAGE))
    monkeypatch.setattr(cartographer_launch, 'Node', lambda **kwargs: kwargs)
    context = LaunchContext()
    for action in cartographer_launch.description('mapping').entities:
        if isinstance(action, DeclareLaunchArgument):
            action.execute(context)
    return context


def test_mapping_does_not_load_map_or_start_readiness(context, tmp_path):
    output = tmp_path / 'site.pbstream'
    context.launch_configurations['save_state_file'] = str(output)
    cartographer, occupancy = cartographer_launch.make_nodes(context, 'mapping')
    assert '-save_state_filename' in cartographer['arguments']
    assert '-load_state_filename' not in cartographer['arguments']
    assert '-include_unfrozen_submaps=true' in occupancy['arguments']
    assert ('odom', '/odometry/filtered') in cartographer['remappings']
    assert cartographer['parameters'] == [{'use_sim_time': False}]


def test_localization_freezes_existing_state_and_only_displays_frozen_map(context, tmp_path):
    state = tmp_path / 'site.pbstream'
    state.write_bytes(b'path-only test fixture; not a real pbstream')
    context.launch_configurations.update(state_file=str(state), use_sim_time='true')
    cartographer, occupancy, readiness = cartographer_launch.make_nodes(context, 'localization')
    assert '-load_frozen_state=true' in cartographer['arguments']
    assert '-save_state_filename' not in cartographer['arguments']
    assert '-include_unfrozen_submaps=false' in occupancy['arguments']
    assert readiness['package'] == 'wbmm_localization'
    assert readiness['parameters'][1]['backend'] == 'cartographer_localization'
    assert readiness['parameters'][1]['use_sim_time'] is True


def test_localization_rejects_missing_map_before_returning_nodes(context):
    with pytest.raises(RuntimeError, match='state_file'):
        cartographer_launch.make_nodes(context, 'localization')


def test_rejects_bad_lua_and_output_path(context, tmp_path):
    context.launch_configurations['cartographer_config'] = str(tmp_path / 'missing.lua')
    with pytest.raises(RuntimeError, match='Lua config'):
        cartographer_launch.make_nodes(context, 'mapping')
    context.launch_configurations['cartographer_config'] = ''
    context.launch_configurations['save_state_file'] = str(tmp_path / 'absent' / 'site.pbstream')
    with pytest.raises(RuntimeError, match='save_state_file'):
        cartographer_launch.make_nodes(context, 'mapping')


def test_explicit_backend_wins_legacy_start_slam_and_unknown_backend_is_error():
    assert resolve_backend('auto', True) == 'slam_toolbox'
    assert resolve_backend('auto', False) == 'none'
    assert resolve_backend('cartographer_localization', True) == 'cartographer_localization'
    assert resolve_backend('amcl', True) == 'amcl'
    with pytest.raises(ValueError):
        resolve_backend('cartograher', True)


def test_failed_waiter_never_releases_planner_actions():
    actions = [object(), object()]
    assert release_on_success(actions, SimpleNamespace(returncode=0)) is actions
    for code in (1, -2):
        returned = release_on_success(actions, SimpleNamespace(returncode=code))
        assert len(returned) == 1 and isinstance(returned[0], LogInfo)
