#!/usr/bin/env python3
"""Start the complete hardware stack; select backend:=sim or backend:=real."""

from wbmm_bringup_launch.entrypoints import generate


def generate_launch_description():
    return generate("hardware")
