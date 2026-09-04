"""Regression tests for obstacle-free MuJoCo ESDF generation."""

from pathlib import Path

import numpy as np

from grid_map.mjcf_to_esdf import build_esdf_from_mjcf


def test_obstacle_free_scene_produces_all_free_esdf(tmp_path):
    """Ensure an empty static scene stays free throughout the ESDF volume."""
    scene = (
        Path(__file__).resolve().parents[3]
        / 'simulation'
        / 'tracer_jaka_mujoco'
        / 'models'
        / 'scene_esdf_validation.xml'
    ).resolve()
    output = tmp_path / 'empty_esdf.npz'

    esdf, occupancy, origin, voxel_size = build_esdf_from_mjcf(
        xml_path=str(scene),
        voxel_size=0.20,
        bounds_min=(-1.0, -1.0, 0.0),
        bounds_max=(1.0, 1.0, 1.0),
        output_path=output,
    )

    assert esdf.shape == (10, 10, 5)
    assert not np.any(occupancy)
    assert np.all(esdf == np.float32(100.0))
    assert np.allclose(origin, (-1.0, -1.0, 0.0))
    assert voxel_size == 0.20

    archive = np.load(output)
    assert not np.any(archive['occupancy'])
    assert np.all(archive['esdf'] == np.float32(100.0))
    assert archive['frame_id'].item() == 'odom'
