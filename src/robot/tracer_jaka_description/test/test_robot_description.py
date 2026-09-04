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

import subprocess
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


PACKAGE = Path(__file__).resolve().parents[1]
URDF = PACKAGE / "urdf" / "tracer_jaka_zu5.urdf"
CONTROLLED_XACRO = (
    PACKAGE / "urdf" / "tracer_jaka_zu5.controlled.urdf.xacro"
)


def _expand(backend: str) -> ET.Element:
    result = subprocess.run(
        [
            "xacro",
            str(CONTROLLED_XACRO),
            f"control_backend:={backend}",
            "robot_ip:=192.0.2.10",
            "local_ip:=192.0.2.20",
            "jaka_read_only:=true",
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    return ET.fromstring(result.stdout)


def _kinematic_signature(robot: ET.Element):
    def normalized(element):
        if element is None:
            return None
        return (
            element.tag,
            tuple(sorted(element.attrib.items())),
            (element.text or "").strip(),
            tuple(normalized(child) for child in element),
        )

    links = sorted(link.attrib["name"] for link in robot.findall("link"))
    joints = []
    for joint in robot.findall("joint"):
        joints.append(
            (
                joint.attrib["name"],
                joint.attrib["type"],
                joint.find("parent").attrib["link"],
                joint.find("child").attrib["link"],
                normalized(joint.find("origin")),
                normalized(joint.find("axis")),
                normalized(joint.find("limit")),
            )
        )
    collisions = sorted(
        (
            link.attrib["name"],
            normalized(collision),
        )
        for link in robot.findall("link")
        for collision in link.findall("collision")
    )
    return links, sorted(joints), collisions


class RobotDescriptionTest(unittest.TestCase):
    def test_control_backends_preserve_geometry_frames_and_limits(self):
        canonical = ET.parse(URDF).getroot()
        self.assertIsNone(canonical.find("ros2_control"))
        expected = _kinematic_signature(canonical)
        self.assertEqual(_kinematic_signature(_expand("mujoco")), expected)
        self.assertEqual(_kinematic_signature(_expand("real")), expected)
        self.assertEqual(_kinematic_signature(_expand("mock")), expected)

    def test_required_frames_and_collision_envelope_are_present(self):
        canonical = ET.parse(URDF).getroot()
        link_names = {link.attrib["name"] for link in canonical.findall("link")}
        required = {"base_footprint", "laser_link", "imu_link", "d455_link", "tool0"}
        self.assertTrue(required <= link_names)
        base_link = canonical.find("link[@name='base_link']")
        collision_names = {
            collision.attrib["name"] for collision in base_link.findall("collision")
        }
        self.assertEqual(
            collision_names,
            {f"base_collision_{index}" for index in range(1, 7)},
        )

    def test_backend_plugins_and_interfaces_are_isolated(self):
        mujoco = _expand("mujoco")
        real = _expand("real")
        self.assertEqual(
            mujoco.findtext("ros2_control/hardware/plugin"),
            "mujoco_ros2_control/MujocoSystem",
        )
        self.assertEqual(
            real.findtext("ros2_control/hardware/plugin"),
            "jaka_hardware_interface/JakaHardwareInterface",
        )
        real_params = {
            param.attrib["name"]: param.text
            for param in real.findall("ros2_control/hardware/param")
        }
        self.assertEqual(real_params["robot_ip"], "192.0.2.10")
        self.assertEqual(real_params["local_ip"], "192.0.2.20")
        self.assertEqual(real_params["read_only"].lower(), "true")
        self.assertIsNotNone(
            real.find("ros2_control/sensor[@name='tcp_fts_sensor']")
        )
        self.assertIsNone(mujoco.find("ros2_control/sensor"))
        self.assertIsNotNone(
            mujoco.find("ros2_control/joint[@name='left_wheel']")
        )
        self.assertIsNone(real.find("ros2_control/joint[@name='left_wheel']"))

    def test_meshes_are_owned_by_description_package(self):
        canonical = ET.parse(URDF).getroot()
        prefix = "package://tracer_jaka_description/"
        for mesh in canonical.findall(".//mesh"):
            uri = mesh.attrib["filename"]
            self.assertTrue(uri.startswith(prefix), uri)
            self.assertTrue((PACKAGE / uri.removeprefix(prefix)).is_file(), uri)

    def test_controller_names_are_backend_independent(self):
        config = (PACKAGE / "config" / "ros2_controllers.yaml").read_text()
        for name in (
            "joint_state_broadcaster",
            "arm_controller",
            "arm_trajectory_controller",
            "base_controller",
            "fts_broadcaster",
        ):
            self.assertIn(f"{name}:", config)
        self.assertNotIn("jaka_forward_controller", config)
        self.assertNotIn("jaka_arm_controller", config)


if __name__ == "__main__":
    unittest.main()
