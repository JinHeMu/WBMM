#!/usr/bin/env python3
"""Start the complete end_effector stack; select backend:=sim or backend:=real."""

from wbmm_bringup_launch.entrypoints import generate


def generate_launch_description():
    return generate("end_effector")
