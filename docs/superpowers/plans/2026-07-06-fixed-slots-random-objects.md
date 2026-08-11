# Fixed Slots Random Objects Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the initial scene use fixed table slots while always including `tomato_soup_can` and `banana` plus four random objects.

**Architecture:** Keep the existing object-selection logic. Add a small placement-slot helper that copies selected object configs, replaces `position_range` with fixed `position`, then pass `placement_slots` from Hydra config through `main.py`/`ros2_main.py` into `MuJoCoInterface`.

**Tech Stack:** Python, pytest, Hydra/OmegaConf YAML config, MuJoCo scene generation.

---

## File Structure

- Modify `src/ifl_air_mujoco_sim/env/utils/populate_scene.py`
  Add `assign_placement_slots()` helper.
- Modify `src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`
  Accept `placement_slots` and apply them after object selection.
- Modify `src/ifl_air_mujoco_sim/env/ros2_interface.py`
  Pass `placement_slots` into `MuJoCoInterface`.
- Modify `src/ifl_air_mujoco_sim/ros2_main.py`
  Read top-level `placement_slots` from Hydra config.
- Modify `src/ifl_air_mujoco_sim/main.py`
  Make the non-ROS entrypoint respect the same object-selection config.
- Modify `src/ifl_air_mujoco_sim/env/config/base_env.yaml`
  Add six fixed slots while keeping two fixed objects and random fill to six.
- Modify `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`
  Add helper and config tests.

### Task 1: Placement Slot Helper

**Files:**
- Modify: `src/ifl_air_mujoco_sim/env/utils/populate_scene.py`
- Test: `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`

- [ ] **Step 1: Write failing helper tests**

Add these tests after `test_zero_random_count_returns_only_fixed_objects_in_order()`:

```python
def test_assign_placement_slots_sets_fixed_positions_without_mutating_inputs():
    objects = [
        {
            "name": "tomato_soup_can",
            "type": "mesh",
            "position_range": {"x": [-1, 1], "y": [-1, 1], "z": [0, 0.02]},
        },
        {
            "name": "banana",
            "type": "mesh",
            "position_range": {"x": [-1, 1], "y": [-1, 1], "z": [0, 0.02]},
        },
    ]
    slots = [[-0.74, -0.08, 0.0], [-0.55, -0.08, 0.0]]

    assigned = populate_scene.assign_placement_slots(objects, slots)

    assert [obj["position"] for obj in assigned] == slots
    assert all("position_range" not in obj for obj in assigned)
    assert "position_range" in objects[0]
    assert "position" not in objects[0]


def test_assign_placement_slots_rejects_too_few_slots():
    objects = _objects("tomato_soup_can", "banana")

    with pytest.raises(ValueError, match="placement_slots"):
        populate_scene.assign_placement_slots(objects, [[-0.74, -0.08, 0.0]])
```

- [ ] **Step 2: Run helper tests to verify RED**

Run:

```powershell
$env:PYTHONPATH='src/ifl_air_mujoco_sim'; pytest src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py::test_assign_placement_slots_sets_fixed_positions_without_mutating_inputs src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py::test_assign_placement_slots_rejects_too_few_slots -v
```

Expected: FAIL with `AttributeError` because `assign_placement_slots` does not exist yet.

- [ ] **Step 3: Implement helper**

Add this function after `select_scene_objects()` in `populate_scene.py`:

```python
def assign_placement_slots(objects_config, placement_slots):
    """Return object configs copied onto fixed placement slots.

    Slots use the same [x, y, z] semantics as a mesh object's fixed position:
    target mesh centroid x/y plus table-relative z offset.
    """
    placement_slots = list(placement_slots or [])
    if not placement_slots:
        return list(objects_config or [])

    objects_config = list(objects_config or [])
    if len(placement_slots) < len(objects_config):
        raise ValueError(
            f"placement_slots has {len(placement_slots)} entries but "
            f"{len(objects_config)} objects were selected"
        )

    assigned = []
    for obj_config, slot in zip(objects_config, placement_slots):
        if len(slot) != 3:
            raise ValueError(f"placement_slots entries must be [x, y, z], got {slot!r}")
        copied = dict(obj_config)
        copied.pop("position_range", None)
        copied["position"] = [float(slot[0]), float(slot[1]), float(slot[2])]
        assigned.append(copied)
    return assigned
```

- [ ] **Step 4: Run helper tests to verify GREEN**

Run:

```powershell
$env:PYTHONPATH='src/ifl_air_mujoco_sim'; pytest src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py::test_assign_placement_slots_sets_fixed_positions_without_mutating_inputs src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py::test_assign_placement_slots_rejects_too_few_slots -v
```

Expected: both tests PASS.

### Task 2: Pass Placement Slots Through Runtime

**Files:**
- Modify: `src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`
- Modify: `src/ifl_air_mujoco_sim/env/ros2_interface.py`
- Modify: `src/ifl_air_mujoco_sim/ros2_main.py`
- Modify: `src/ifl_air_mujoco_sim/main.py`

- [ ] **Step 1: Update imports and constructor**

In `mjcontrol_interface.py`, change the import to:

```python
from env.utils.populate_scene import populate_scene, select_scene_objects, assign_placement_slots
```

Add a constructor parameter after `fixed_object_names=None`:

```python
                placement_slots=None,
```

Store it:

```python
        self.placement_slots = placement_slots or []
```

- [ ] **Step 2: Apply slots after object selection**

Immediately after the existing `select_scene_objects(...)` call in `MuJoCoInterface.__init__`, add:

```python
        if self.placement_slots:
            self.objects_config = assign_placement_slots(
                self.objects_config,
                self.placement_slots,
            )
```

- [ ] **Step 3: Pass slots from ROS interface**

In `ros2_interface.py`, add this argument to the `MuJoCoInterface(...)` call:

```python
            placement_slots=sim_cfg.get('placement_slots', []),
```

- [ ] **Step 4: Read slots in ROS entrypoint**

In `ros2_main.py`, add this key to `sim_cfg`:

```python
               'placement_slots': cfg.get('placement_slots', []),
```

- [ ] **Step 5: Make non-ROS entrypoint respect scene selection config**

In `main.py`, add these arguments to `MuJoCoInterface(...)`:

```python
        random_object_count=cfg.get('random_object_count', 0),
        fixed_object_names=cfg.get('fixed_object_names', []),
        placement_slots=cfg.get('placement_slots', []),
```

### Task 3: Configure Default Fixed Slots

**Files:**
- Modify: `src/ifl_air_mujoco_sim/env/config/base_env.yaml`
- Test: `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`

- [ ] **Step 1: Add fixed placement slots to YAML**

Insert after `random_object_count: 6`:

```yaml
# Fixed grasp-friendly initial table slots. Object identities may vary, but
# selected objects are placed in these slots in selection order.
placement_slots:
  - [-0.74, -0.08, 0.0]
  - [-0.55, -0.08, 0.0]
  - [-0.36, -0.08, 0.0]
  - [-0.74, 0.20, 0.0]
  - [-0.55, 0.20, 0.0]
  - [-0.36, 0.20, 0.0]
```

- [ ] **Step 2: Update default config test**

In `test_default_config_uses_fixed_priority_random_scene_layout()`, after `assert config["random_object_count"] == 6`, add:

```python
    assert config["placement_slots"] == [
        [-0.74, -0.08, 0.0],
        [-0.55, -0.08, 0.0],
        [-0.36, -0.08, 0.0],
        [-0.74, 0.20, 0.0],
        [-0.55, 0.20, 0.0],
        [-0.36, 0.20, 0.0],
    ]
```

Replace the final assertion:

```python
    assert all("position_range" in obj for obj in selected)
```

with:

```python
    assigned = populate_scene.assign_placement_slots(
        selected,
        config["placement_slots"],
    )
    assert [obj["position"] for obj in assigned] == config["placement_slots"]
    assert all("position_range" not in obj for obj in assigned)
```

### Task 4: Verification

**Files:**
- Test: `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`
- Inspect: `src/ifl_air_mujoco_sim/env/config/base_env.yaml`

- [ ] **Step 1: Run selection tests**

Run:

```powershell
$env:PYTHONPATH='src/ifl_air_mujoco_sim'; pytest src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py -v
```

Expected: all tests PASS.

- [ ] **Step 2: Print default selection and assigned slots**

Run:

```powershell
@'
import random
import sys
import yaml
from pathlib import Path

sys.path.insert(0, 'src/ifl_air_mujoco_sim')
from env.utils import populate_scene

cfg = yaml.safe_load(Path('src/ifl_air_mujoco_sim/env/config/base_env.yaml').read_text())
selected = populate_scene.select_scene_objects(
    cfg['objects'],
    random_object_count=cfg['random_object_count'],
    fixed_object_names=cfg['fixed_object_names'],
    rng=random.Random(17),
)
assigned = populate_scene.assign_placement_slots(selected, cfg['placement_slots'])
print(cfg['fixed_object_names'])
print(cfg['random_object_count'])
print([obj['name'] for obj in assigned])
print([obj['position'] for obj in assigned])
'@ | & 'E:\Anaconda\python.exe' -
```

Expected output starts with:

```text
['tomato_soup_can', 'banana']
6
```

The selected names should include `tomato_soup_can`, `banana`, and four random
objects. The printed positions should equal the six configured slots.

- [ ] **Step 3: Check diff**

Run:

```powershell
git diff --check -- src/ifl_air_mujoco_sim/env/utils/populate_scene.py src/ifl_air_mujoco_sim/env/mjcontrol_interface.py src/ifl_air_mujoco_sim/env/ros2_interface.py src/ifl_air_mujoco_sim/ros2_main.py src/ifl_air_mujoco_sim/main.py src/ifl_air_mujoco_sim/env/config/base_env.yaml src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py
```

Expected: no whitespace errors; Windows CRLF warnings are acceptable.

## Self-Review

- Spec coverage: covers fixed positions, random object identities, required fixed objects, no place/drop changes, slot semantics, and error handling.
- Placeholder scan: no TBD/TODO placeholders.
- Type consistency: uses `placement_slots`, `fixed_object_names`, `random_object_count`, `position`, and `position_range` consistently.
