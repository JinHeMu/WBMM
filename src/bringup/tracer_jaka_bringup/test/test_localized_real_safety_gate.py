"""Regression tests for the localized real-robot safety gate."""

import importlib.util
from pathlib import Path

from launch import LaunchContext
import numpy as np
import pytest


def load_launch_module():
    """Load the launch module without depending on an installed workspace."""
    launch_file = (
        Path(__file__).resolve().parents[1]
        / 'launch'
        / 'remani_mpc_localized_real.launch.py'
    )
    spec = importlib.util.spec_from_file_location(
        'remani_mpc_localized_real', launch_file
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def make_context(map_file, esdf_file):
    """Construct the safe read-only launch context used by L5 checks."""
    context = LaunchContext()
    context.launch_configurations.update({
        'safety_release': 'false',
        'jaka_read_only': 'true',
        'command_output_enabled': 'false',
        'map_file': str(map_file),
        'start_remani': 'true',
        'static_esdf_file': str(esdf_file),
    })
    return context


def write_contract(path, frame_id=None):
    """Write the minimal NPZ metadata needed by the top-level gate."""
    values = {'sentinel': np.uint8(0)}
    if frame_id is not None:
        values['frame_id'] = np.str_(frame_id)
    np.savez_compressed(path, **values)


def test_localized_real_accepts_map_esdf(tmp_path):
    """Allow a readable ESDF explicitly expressed in map."""
    module = load_launch_module()
    map_file = tmp_path / 'site.yaml'
    map_file.write_text('image: site.pgm\n', encoding='utf-8')
    esdf_file = tmp_path / 'site.npz'
    write_contract(esdf_file, 'map')

    assert module._enforce_safety_gate(
        make_context(map_file, esdf_file)) == []


@pytest.mark.parametrize('frame_id', ['odom', None])
def test_localized_real_rejects_invalid_esdf_frame(tmp_path, frame_id):
    """Reject a mismatched or missing ESDF frame before starting hardware."""
    module = load_launch_module()
    map_file = tmp_path / 'site.yaml'
    map_file.write_text('image: site.pgm\n', encoding='utf-8')
    esdf_file = tmp_path / 'site.npz'
    write_contract(esdf_file, frame_id)

    with pytest.raises(RuntimeError):
        module._enforce_safety_gate(make_context(map_file, esdf_file))
