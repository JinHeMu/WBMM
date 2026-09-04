"""Regression tests for the independent real force-control motion gate."""

import importlib.util
from pathlib import Path
import unittest

from launch import LaunchContext


def load_launch_module():
    launch_file = (
        Path(__file__).resolve().parents[1]
        / "launch"
        / "whole_body_force_control_real.launch.py"
    )
    spec = importlib.util.spec_from_file_location(
        "whole_body_force_control_real", launch_file)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_context(read_only, command_output, safety_release, reference_output):
    context = LaunchContext()
    context.launch_configurations.update({
        "jaka_read_only": str(read_only).lower(),
        "command_output_enabled": str(command_output).lower(),
        "safety_release": str(safety_release).lower(),
        "force_reference_output_enabled": str(reference_output).lower(),
    })
    return context


class ForceControlRealSafetyGateTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        cls.module = load_launch_module()

    def test_default_shadow_mode_is_accepted(self):
        self.assertEqual(
            self.module._enforce_force_motion_gate(
                make_context(True, False, False, False)),
            [],
        )

    def test_reference_output_requires_all_real_motion_gates(self):
        self.assertEqual(
            self.module._enforce_force_motion_gate(
                make_context(False, True, True, True)),
            [],
        )

        unsafe = [
            make_context(True, True, True, True),
            make_context(False, False, True, True),
            make_context(False, True, False, True),
        ]
        for context in unsafe:
            with self.subTest(context=context):
                with self.assertRaises(RuntimeError):
                    self.module._enforce_force_motion_gate(context)


if __name__ == "__main__":
    unittest.main()
