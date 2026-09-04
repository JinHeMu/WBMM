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
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("tracer_jaka_description")
    urdf = os.path.join(share, "urdf", "tracer_jaka_zu5.urdf")
    rviz = os.path.join(share, "config", "jaka_zu5_urdf.rviz")
    with open(urdf, encoding="utf-8") as stream:
        robot_description = {"robot_description": stream.read()}

    return LaunchDescription([
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[robot_description],
            remappings=[("/joint_states", "robot_jog_command")],
        ),
        Node(
            package="jaka_jog_panel",
            executable="jakajogpanel",
            name="jaka_jog_panel",
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz",
            arguments=["-d", rviz],
        ),
    ])
