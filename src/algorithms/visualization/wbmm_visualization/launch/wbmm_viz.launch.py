"""Standalone wbmm_viz_node launch.

Starts only the unified visualization node plus optional RViz. Feed the data
contract with remaps, e.g.:

    ros2 launch wbmm_visualization wbmm_viz.launch.py use_rviz:=true \
      trajectory_topic:=/ta_wbmp/whole_body_trajectory \
      phase_schedule_topic:=/ta_wbmp/phase_schedule
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    mujoco_share = get_package_share_directory('tracer_jaka_description')
    default_urdf = os.path.join(mujoco_share, 'urdf', 'tracer_jaka_zu5.urdf')

    return LaunchDescription([
        DeclareLaunchArgument('urdf_file', default_value=default_urdf,
                              description='Whole-body URDF for mesh display'),
        DeclareLaunchArgument('use_rviz', default_value='false'),
        DeclareLaunchArgument('trajectory_topic',
                              default_value='/wbmm/whole_body_trajectory'),
        DeclareLaunchArgument('phase_schedule_topic',
                              default_value='/wbmm/phase_schedule'),

        Node(package='wbmm_visualization', executable='wbmm_viz_node',
             name='wbmm_viz', output='screen',
             parameters=[{
                 'urdf_file': LaunchConfiguration('urdf_file'),
                 'ee_frame': 'tool0',
                 'time_segment_duration': 15.0,
                 'segment_snapshots': 2,
                 'playback_enabled': True,
                 'playback_rate': 5.0,
                 'playback_period': 0.10,
                 'playback_loop': True,
                 'robot_mesh_rate': 30.0,
             }],
             remappings=[
                 ('/wbmm/whole_body_trajectory',
                  LaunchConfiguration('trajectory_topic')),
                 ('/wbmm/phase_schedule',
                  LaunchConfiguration('phase_schedule_topic')),
             ]),
        Node(package='rviz2', executable='rviz2', name='rviz2',
             output='screen',
             arguments=['-d', os.path.join(
                 get_package_share_directory('wbmm_visualization'),
                 'rviz', 'wbmm_viz.rviz')],
             condition=IfCondition(LaunchConfiguration('use_rviz'))),
    ])
