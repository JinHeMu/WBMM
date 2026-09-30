"""Backend policy and a launch gate that releases actions only on success."""

from launch.actions import EmitEvent, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node


BACKENDS = ('auto', 'none', 'slam_toolbox', 'amcl',
            'cartographer_mapping', 'cartographer_localization')
SAVED_MAP_BACKENDS = ('amcl', 'cartographer_localization')


def resolve_backend(backend, start_slam):
    if backend not in BACKENDS:
        raise ValueError(f'Unsupported localization_backend: {backend!r}')
    if backend == 'auto':
        return 'slam_toolbox' if start_slam else 'none'
    return backend


def release_on_success(actions, event):
    if event.returncode == 0:
        return actions
    return [LogInfo(msg='Localization did not become ready; planner was not started.')]


def gate_until_ready(actions, use_sim_time, backend, timeout, status_topic):
    waiter = Node(
        package='wbmm_localization', executable='wait_for_localization',
        name='wait_for_localization', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'backend': backend,
                     'timeout': timeout, 'status_topic': status_topic}])
    handler = RegisterEventHandler(OnProcessExit(
        target_action=waiter,
        on_exit=lambda event, context: release_on_success(actions, event)))
    return [handler, waiter]


def preflight_exit(actions, event):
    if event.returncode == 0:
        return actions
    return [LogInfo(msg='Localization startup cancelled due to conflicting publishers. '
                         'See localization_preflight above.'),
            EmitEvent(event=Shutdown(reason='Conflicting localization publishers'))]


def guard_localization(actions, use_sim_time, start_ekf, global_backend, odom_topic):
    checker = Node(
        package='wbmm_localization', executable='localization_preflight',
        name='localization_preflight', output='screen',
        parameters=[{'use_sim_time': use_sim_time, 'start_ekf': start_ekf,
                     'global_backend': global_backend, 'odom_topic': odom_topic}])
    handler = RegisterEventHandler(OnProcessExit(
        target_action=checker,
        on_exit=lambda event, context: preflight_exit(actions, event)))
    return [handler, checker]
