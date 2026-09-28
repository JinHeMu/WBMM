"""Display existing WBMM planner/MPC output; never starts planning or control."""
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _nodes(context):
    def value(name):
        return LaunchConfiguration(name).perform(context)
    with open(value('config_file'), encoding='utf-8') as stream:
        settings = yaml.safe_load(stream) or {}
    configured = settings.get('wbmm_visualization', {}).get('ros__parameters', {})
    overrides = {
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
    }
    urdf = value('urdf_file') or configured.get('urdf_file', '')
    if not urdf and not configured.get('robot_description'):
        try:
            urdf = str(Path(get_package_share_directory('tracer_jaka_description')) /
                       'urdf/tracer_jaka_zu5.urdf')
        except PackageNotFoundError as error:
            raise RuntimeError('Provide urdf_file for the robot to display') from error
    if urdf:
        if not Path(urdf).is_file():
            raise RuntimeError('URDF file does not exist: ' + urdf)
        overrides['urdf_file'] = urdf
        # An explicit launch argument takes precedence over XML in a custom config.
        if value('urdf_file'):
            overrides['robot_description'] = ''
    if value('mpc_frame'):
        overrides['mpc_frame'] = value('mpc_frame')
    actions = [Node(
        package='wbmm_visualization', executable='trajectory_visualizer',
        name='wbmm_visualization', output='screen',
        parameters=[value('config_file'), overrides])]
    if value('use_rviz').lower() == 'true':
        actions.append(Node(
            package='rviz2', executable='rviz2', name='wbmm_trajectory_rviz',
            arguments=['-d', value('rviz_config'), '-f', value('fixed_frame')],
            parameters=[{'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
            output='screen'))
    return actions


def generate_launch_description():
    share = Path(get_package_share_directory('wbmm_visualization'))
    return LaunchDescription([
        DeclareLaunchArgument('urdf_file', default_value='', description='Expanded URDF; empty uses the installed WBMM robot description.'),
        DeclareLaunchArgument('config_file', default_value=str(share / 'config/visualization.yaml')),
        DeclareLaunchArgument('mpc_frame', default_value='', description='Override config MPC frame (default odom); must match OCS2 world_frame.'),
        DeclareLaunchArgument('fixed_frame', default_value='map', description='RViz fixed frame; requires TF when input frames differ.'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=str(share / 'rviz/whole_body_trajectories.rviz')),
        OpaqueFunction(function=_nodes),
    ])
