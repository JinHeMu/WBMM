# Copyright 2026 WBMM Contributors
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    arm_controller_name = LaunchConfiguration("arm_controller_name")
    share = get_package_share_directory("tracer_jaka_description")
    xacro_file = os.path.join(
        share, "urdf", "tracer_jaka_zu5.controlled.urdf.xacro"
    )
    controllers = os.path.join(share, "config", "ros2_controllers.yaml")
    robot_description = {
        "robot_description": ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ", xacro_file, " ",
                "control_backend:=mock",
            ]),
            value_type=str,
        )
    }
    manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, controllers],
        output="screen",
    )
    state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
        output="screen",
    )
    arm_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[arm_controller_name, "-c", "/controller_manager"],
        output="screen",
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            "arm_controller_name",
            default_value="arm_controller",
            description=(
                "Use arm_controller for OCS2/Servo or "
                "arm_trajectory_controller for MoveIt."
            ),
        ),
        manager,
        TimerAction(period=1.0, actions=[state_spawner]),
        RegisterEventHandler(
            OnProcessExit(target_action=state_spawner, on_exit=[arm_spawner])
        ),
    ])
