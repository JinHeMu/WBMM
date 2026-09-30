from pathlib import Path
from types import SimpleNamespace

import pytest
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument, LogInfo

from wbmm_localization_launch import cartographer_launch
from wbmm_localization_launch.launch_support import preflight_exit, resolve_backend, release_on_success


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
    cartographer, occupancy, ground = cartographer_launch.make_nodes(context, 'mapping')
    assert '-save_state_filename' in cartographer['arguments']
    assert '-load_state_filename' not in cartographer['arguments']
    assert '-include_unfrozen_submaps=true' in occupancy['arguments']
    assert '-map_frame' not in occupancy['arguments']
    assert occupancy['remappings'] == [('map', '/cartographer/map')]
    assert ground['executable'] == 'cartographer_ground_map'
    assert ground['parameters'] == [{'use_sim_time': False}]
    assert ('odom', '/odometry/filtered') in cartographer['remappings']
    assert cartographer['parameters'] == [{'use_sim_time': False}]


def test_localization_freezes_existing_state_and_only_displays_frozen_map(context, tmp_path):
    state = tmp_path / 'site.pbstream'
    state.write_bytes(b'path-only test fixture; not a real pbstream')
    context.launch_configurations.update(state_file=str(state), use_sim_time='true')
    cartographer, occupancy, ground, readiness = cartographer_launch.make_nodes(context, 'localization')
    assert '-load_frozen_state=true' in cartographer['arguments']
    assert '-save_state_filename' not in cartographer['arguments']
    assert '-include_unfrozen_submaps=false' in occupancy['arguments']
    assert occupancy['remappings'] == [('map', '/cartographer/map')]
    assert ground['executable'] == 'cartographer_ground_map'
    assert ground['parameters'] == [{'use_sim_time': True}]
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


def test_cartographer_tf_ownership_and_imu_tracking():
    config = (PACKAGE / 'config/cartographer_mapping.lua').read_text()
    assert 'map_frame = "cartographer_map"' in config
    assert 'tracking_frame = "imu_link"' in config
    assert 'published_frame = "odom"' in config
    assert 'provide_odom_frame = false' in config
    assert 'publish_frame_projected_to_2d = true' in config
    assert 'include "cartographer_mapping.lua"' in (
        PACKAGE / 'config/cartographer_localization.lua').read_text()


def test_localization_without_readiness_still_has_ground_bridge(context, tmp_path):
    state = tmp_path / 'fixture.pbstream'
    state.touch()
    context.launch_configurations.update(state_file=str(state), start_readiness='false')
    nodes = cartographer_launch.make_nodes(context, 'localization')
    assert len(nodes) == 3
    assert nodes[-1]['executable'] == 'cartographer_ground_map'


def test_mapping_rejects_state_file_instead_of_showing_an_old_map(context, tmp_path):
    context.launch_configurations['state_file'] = str(tmp_path / 'old.pbstream')
    with pytest.raises(RuntimeError, match='Fresh mapping'):
        cartographer_launch.make_nodes(context, 'mapping')


def test_mapping_preserves_existing_output(context, tmp_path):
    saved = tmp_path / 'existing.pbstream'
    saved.write_bytes(b'existing map')
    context.launch_configurations['save_state_file'] = str(saved)
    with pytest.raises(RuntimeError, match='already exists'):
        cartographer_launch.make_nodes(context, 'mapping')
    assert saved.read_bytes() == b'existing map'


def test_failed_publisher_check_releases_no_localization_nodes():
    from launch.actions import EmitEvent
    actions = [object()]
    assert preflight_exit(actions, SimpleNamespace(returncode=0)) is actions
    rejected = preflight_exit(actions, SimpleNamespace(returncode=1))
    assert len(rejected) == 2
    assert isinstance(rejected[0], LogInfo)
    assert isinstance(rejected[1], EmitEvent)
