import importlib
from pathlib import Path

import numpy as np
import pytest

from my_course_pkg.grasp.candidate_ranker import (
    load_grasp_candidates,
    rank_grasp_candidates,
)


def pose_with_tool_z(z_axis):
    matrix = np.eye(4, dtype=np.float32)
    z_axis = np.asarray(z_axis, dtype=np.float32)
    z_axis /= np.linalg.norm(z_axis)
    reference = np.array([1.0, 0.0, 0.0])
    if abs(float(z_axis @ reference)) > 0.9:
        reference = np.array([0.0, 1.0, 0.0])
    x_axis = np.cross(reference, z_axis)
    x_axis /= np.linalg.norm(x_axis)
    matrix[:3, 0] = x_axis
    matrix[:3, 1] = np.cross(z_axis, x_axis)
    matrix[:3, 2] = z_axis
    return matrix


def test_load_grasp_candidates_concatenates_sorted_archives(tmp_path):
    first = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], 2, axis=0)
    second = np.repeat(np.eye(4, dtype=np.float32)[None, :, :], 3, axis=0)
    np.savez(tmp_path / "002.npz", poses=second)
    np.savez(tmp_path / "001.npz", poses=first)

    result = load_grasp_candidates(tmp_path)

    assert result.shape == (5, 4, 4)
    np.testing.assert_allclose(result[0], first[0])
    np.testing.assert_allclose(result[-1], second[-1])


def test_load_grasp_candidates_rejects_archive_without_poses(tmp_path):
    np.savez(tmp_path / "bad.npz", transforms=np.eye(4, dtype=np.float32))

    with pytest.raises(ValueError, match="poses"):
        load_grasp_candidates(tmp_path)


def test_rank_grasp_candidates_prefers_world_down_tool_z():
    down = pose_with_tool_z([0.0, 0.0, -1.0])
    side = pose_with_tool_z([1.0, 0.0, 0.0])

    ranked = rank_grasp_candidates(np.eye(4), np.stack([side, down]))

    assert [candidate.index for candidate in ranked] == [1, 0]
    assert ranked[0].approach_angle_deg == pytest.approx(0.0)
    np.testing.assert_allclose(ranked[0].T_world_grasp, down)


def test_rank_grasp_candidates_returns_stable_top_k_for_equal_scores():
    down = pose_with_tool_z([0.0, 0.0, -1.0])

    ranked = rank_grasp_candidates(np.eye(4), np.stack([down, down]), top_k=1)

    assert len(ranked) == 1
    assert ranked[0].index == 0


def test_ranked_candidate_contains_object_and_world_transforms():
    candidate = pose_with_tool_z([0.0, 0.0, -1.0])
    T_world_object = np.eye(4)
    T_world_object[:3, 3] = [0.2, -0.1, 0.4]

    ranked = rank_grasp_candidates(T_world_object, np.stack([candidate]))

    np.testing.assert_allclose(ranked[0].T_object_grasp, candidate)
    np.testing.assert_allclose(
        ranked[0].T_world_grasp[:3, 3],
        [0.2, -0.1, 0.4],
    )


def test_grasp_root_uses_environment_override(monkeypatch):
    monkeypatch.setenv("GRASP_ROOT", "C:/test-data/grasps")

    import my_course_pkg.grasp.config as config

    config = importlib.reload(config)

    assert config.GRASP_ROOT == Path("C:/test-data/grasps")
