"""Regression tests for the independent real force-control motion gate."""

import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext


def load_launch_module():
    launch_file = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "whole_body_force_control.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "whole_body_force_control", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_context(hardware_write, reference_output):
    context = LaunchContext()
    context.launch_configurations.update({
        "hardware_write": str(hardware_write).lower(),
        "admittance.output": str(reference_output).lower(),
    })
    return context


class ForceControlRealSafetyGateTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.module = load_launch_module()

    def test_default_shadow_mode_is_accepted(self):
        self.assertEqual(
            self.module._enforce_force_motion_gate(
                make_context(False, False)),
            [],
        )

    def test_reference_output_requires_hardware_write(self):
        self.assertEqual(
            self.module._enforce_force_motion_gate(
                make_context(True, True)),
            [],
        )
        with self.assertRaises(RuntimeError):
            self.module._enforce_force_motion_gate(
                make_context(False, True))


if __name__ == "__main__":
    unittest.main()
