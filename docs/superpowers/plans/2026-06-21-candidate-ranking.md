# Candidate Ranking Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Load every stable grasp archive for one object and deterministically return the top-K world-frame poses whose approach direction is closest to world down.

**Architecture:** Add a pure NumPy `candidate_ranker` module that owns archive validation, homogeneous-transform validation, world-frame conversion, and ranking. Keep `grasp_selector` as the ROS-facing compatibility layer; it delegates its existing top-down choice to the new module without changing the caller interface.

**Tech Stack:** Python 3.10, NumPy, pytest, existing ROS2 package layout.

---

## File structure

- Create: `src/my_course_pkg/my_course_pkg/grasp/candidate_ranker.py` — pure candidate loading, validation, and ranking.
- Create: `src/my_course_pkg/test/test_candidate_ranker.py` — unit tests that run without ROS, MoveIt, or MuJoCo.
- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py` — configurable grasp-root directory.
- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py` — delegate archive loading and top-down selection to `candidate_ranker`.

## Task 1: Specify candidate loading and ranking with failing tests

**Files:**
- Create: `src/my_course_pkg/test/test_candidate_ranker.py`

- [ ] **Step 1: Write the failing tests**

```python
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
```

- [ ] **Step 2: Run the tests to verify they fail**

Run from the repository root:

```powershell
$env:PYTHONPATH = "src/my_course_pkg"
python -m pytest src/my_course_pkg/test/test_candidate_ranker.py -v
```

Expected: collection fails because `my_course_pkg.grasp.candidate_ranker` does not exist.

- [ ] **Step 3: Commit the failing tests**

```powershell
git add src/my_course_pkg/test/test_candidate_ranker.py
git commit -m "test: specify grasp candidate ranking"
```

## Task 2: Implement pure archive loading and deterministic ranking

**Files:**
- Create: `src/my_course_pkg/my_course_pkg/grasp/candidate_ranker.py`
- Test: `src/my_course_pkg/test/test_candidate_ranker.py`

- [ ] **Step 1: Implement the smallest passing module**

```python
from dataclasses import dataclass
from pathlib import Path

import numpy as np


WORLD_DOWN = np.array([0.0, 0.0, -1.0], dtype=float)


@dataclass(frozen=True)
class RankedGrasp:
    index: int
    T_object_grasp: np.ndarray
    T_world_grasp: np.ndarray
    approach_angle_deg: float


def _validate_pose_batch(poses, source):
    poses = np.asarray(poses, dtype=float)
    if poses.ndim != 3 or poses.shape[1:] != (4, 4):
        raise ValueError(f"{source} poses must have shape (N, 4, 4), got {poses.shape}.")
    if poses.shape[0] == 0:
        raise ValueError(f"{source} contains no poses.")
    if not np.isfinite(poses).all():
        raise ValueError(f"{source} poses contain non-finite values.")
    if not np.allclose(poses[:, 3, :], [0.0, 0.0, 0.0, 1.0]):
        raise ValueError(f"{source} poses must be homogeneous transforms.")
    return poses


def load_grasp_candidates(grasp_dir):
    grasp_dir = Path(grasp_dir)
    archives = sorted(grasp_dir.glob("*.npz"))
    if not archives:
        raise FileNotFoundError(f"No .npz grasp archives found in {grasp_dir}.")

    batches = []
    for archive_path in archives:
        with np.load(archive_path) as archive:
            if "poses" not in archive.files:
                raise ValueError(f"{archive_path} does not contain a 'poses' array.")
            batches.append(_validate_pose_batch(archive["poses"], archive_path))
    return np.concatenate(batches, axis=0)


def rank_grasp_candidates(T_world_object, T_object_grasps, top_k=None):
    T_world_object = _validate_pose_batch(
        np.asarray(T_world_object, dtype=float)[None, :, :], "T_world_object"
    )[0]
    T_object_grasps = _validate_pose_batch(T_object_grasps, "T_object_grasps")
    if top_k is not None and top_k <= 0:
        raise ValueError("top_k must be positive when provided.")

    T_world_grasps = T_world_object @ T_object_grasps
    tool_z_axes = T_world_grasps[:, :3, 2]
    norms = np.linalg.norm(tool_z_axes, axis=1)
    if np.any(norms == 0.0):
        raise ValueError("A grasp pose has a zero-length tool Z axis.")
    cosines = np.clip((tool_z_axes / norms[:, None]) @ WORLD_DOWN, -1.0, 1.0)
    angles = np.degrees(np.arccos(cosines))
    ordered_indices = np.argsort(angles, kind="stable")
    if top_k is not None:
        ordered_indices = ordered_indices[:top_k]

    return [
        RankedGrasp(
            index=int(index),
            T_object_grasp=T_object_grasps[index].copy(),
            T_world_grasp=T_world_grasps[index].copy(),
            approach_angle_deg=float(angles[index]),
        )
        for index in ordered_indices
    ]
```

- [ ] **Step 2: Run the new unit tests**

```powershell
$env:PYTHONPATH = "src/my_course_pkg"
python -m pytest src/my_course_pkg/test/test_candidate_ranker.py -v
```

Expected: 5 passed.

- [ ] **Step 3: Commit the module**

```powershell
git add src/my_course_pkg/my_course_pkg/grasp/candidate_ranker.py src/my_course_pkg/test/test_candidate_ranker.py
git commit -m "feat: rank grasp candidates by approach"
```

## Task 3: Make the existing selector use the ranked candidate list

**Files:**
- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py`
- Modify: `src/my_course_pkg/test/test_candidate_ranker.py`

- [ ] **Step 1: Configure the grasp root and delegate selection**

In `config.py`, add this import and constant after the existing imports:

```python
from pathlib import Path

GRASP_ROOT = Path(
    os.environ.get("GRASP_ROOT", str(Path.home() / "Data" / "grasps"))
).expanduser()
```

In `grasp_selector.py`, replace the current `load_grasps_for_object` and
`select_preferred_grasp` definitions with:

```python
def load_grasps_for_object(grasp_dir):
    from my_course_pkg.grasp.candidate_ranker import load_grasp_candidates

    return load_grasp_candidates(grasp_dir)


def select_preferred_grasp(T_world_obj, grasps):
    from my_course_pkg.grasp.candidate_ranker import rank_grasp_candidates

    ranked = rank_grasp_candidates(T_world_obj, grasps, top_k=1)
    selected = ranked[0]
    print(
        "Selected grasp tool +Z angle to global -Z: "
        f"{selected.approach_angle_deg:.2f} deg"
    )
    return selected.T_object_grasp
```

Change the `get_grasp_dir_from_selected_object` signature to use the new
configurable root:

```python
def get_grasp_dir_from_selected_object(json_path, grasp_root=GRASP_ROOT):
```

Add `GRASP_ROOT` to the imports from `my_course_pkg.grasp.config`.

- [ ] **Step 2: Run candidate tests again**

```powershell
$env:PYTHONPATH = "src/my_course_pkg"
python -m pytest src/my_course_pkg/test/test_candidate_ranker.py -v
```

Expected: 5 passed.

- [ ] **Step 3: Commit selector integration**

```powershell
git add src/my_course_pkg/my_course_pkg/grasp/config.py src/my_course_pkg/my_course_pkg/grasp/grasp_selector.py src/my_course_pkg/test/test_candidate_ranker.py
git commit -m "refactor: select grasps through ranked candidates"
```

## Task 4: Smoke-test against downloaded tomato-can data

**Files:**
- Test: `src/my_course_pkg/my_course_pkg/grasp/candidate_ranker.py`

- [ ] **Step 1: Run the local data smoke test**

```powershell
$env:PYTHONPATH = "src/my_course_pkg"
$env:GRASP_ROOT = "E:\IFL\ros2-docker-workspace-vscode-plmrs\Data\grasps"
@'
import os
from pathlib import Path
import numpy as np
from my_course_pkg.grasp.candidate_ranker import load_grasp_candidates, rank_grasp_candidates

root = Path(os.environ["GRASP_ROOT"]) / "005_tomato_soup_can"
poses = load_grasp_candidates(root)
ranked = rank_grasp_candidates(np.eye(4), poses, top_k=10)
print(f"loaded={poses.shape[0]}")
print(f"best_angle_deg={ranked[0].approach_angle_deg:.6f}")
print(f"top_k={len(ranked)}")
'@ | python -
```

Expected: `loaded=7391`, `top_k=10`, and a best approach angle between 0 and 180 degrees.

- [ ] **Step 2: Record the exact result in the commit message only if the smoke test passes**

```powershell
git status --short
```

Expected: no changes from the smoke test.

## Task 5: Hand off the ranked top-K interface to workstation motion verification

**Files:**
- No code changes in this plan.

- [ ] **Step 1: Run the ROS/MuJoCo stack on the workstation using the isolated remote worktree**

```bash
cd ~/ros2-docker-workspace-vscode-plmrs-grasp
git fetch origin
git rebase origin/wenyuan
```

Expected: the remote worktree incorporates the candidate-ranking commits once they have been pushed to the shared GitLab branch.

- [ ] **Step 2: Start a separate implementation plan for MoveIt candidate verification**

That plan must add a verifier that tests pre-grasp, grasp, and a world-Z
20 cm lift pose for each ranked candidate. It must also add attempt metrics
for planning, collision, candidate count, and physical lift success.
