#!/usr/bin/env python3

import importlib.util
import os

_helper_path = os.path.join(os.path.dirname(__file__), 'sim_example.py')
_spec = importlib.util.spec_from_file_location('remani_sim_example', _helper_path)
_helper = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_helper)


def generate_launch_description():
    return _helper.generate_example_launch_description('exp1')
