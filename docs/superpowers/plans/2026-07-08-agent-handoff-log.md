# Agent Handoff Log Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Create a persistent Markdown handoff log and project-level startup rule so future Codex windows can resume this ROS/grasping project without rediscovering recent context.

**Architecture:** Add root `AGENTS.md` instructions that tell future Codex windows to read `docs/agent_handoff.md` and acknowledge it with a `Handoff loaded:` line. Add one stable handoff document at `docs/agent_handoff.md`; it is manually maintained, starts with the current project snapshot, and records reusable commands plus pitfalls discovered during scene-layout and tomato-can side-grasp work.

**Tech Stack:** Markdown, Git, PowerShell verification commands.

---

## File Map

- Create: `AGENTS.md`
  - Owns project-level Codex startup and handoff maintenance instructions.
  - Requires a visible `Handoff loaded:` acknowledgement when the handoff is read.
- Create: `docs/agent_handoff.md`
  - Owns the cross-window handoff state.
  - Includes current snapshot, recent work, pitfalls, and known-good commands.
- No code files are modified.

### Task 1: Create Project Startup Instructions

**Files:**
- Create: `AGENTS.md`

- [ ] **Step 1: Create the Codex project instructions**

Create `AGENTS.md` with this content:

```markdown
# Project Agent Instructions

## Startup Handoff

When starting work in this repository, read `docs/agent_handoff.md` before
inspecting code or rerunning experiments.

After reading it, the first user-facing response in the new window must include
a line in this exact format:

```text
Handoff loaded: <current goal>; next: <next recommended action>
```

If `docs/agent_handoff.md` is missing, say:

```text
Handoff missing: docs/agent_handoff.md
```

and continue with normal repository inspection.

## Handoff Maintenance

At the end of a meaningful work session, update `docs/agent_handoff.md` when
the work changes project state, command workflow, verification status, or
reusable debugging knowledge.

Keep updates short and factual:

- Update `Current Snapshot` when the current goal or next step changes.
- Add a `Recent Work` entry for important changes or investigations.
- Add repeatable mistakes to `Pitfalls To Avoid`.
- Add or correct reliable commands in `Useful Commands`.

Small conversational answers do not need a handoff update unless they capture a
reusable lesson.
```

- [ ] **Step 2: Verify the startup marker is present**

Run:

```powershell
Select-String -Path AGENTS.md -Pattern 'Handoff loaded:','docs/agent_handoff.md','Handoff missing:'
```

Expected: all three patterns are present.

### Task 2: Create The Handoff Log

**Files:**
- Create: `docs/agent_handoff.md`

- [ ] **Step 1: Create the Markdown document**

Create `docs/agent_handoff.md` with exactly these top-level sections and seed
content:

```markdown
# Agent Handoff Log

This file is the persistent handoff note for future Codex windows. Read it at
the start of a new session before inspecting code or rerunning experiments.

## Current Snapshot

- Branch: `xinzhe`
- Current goal: keep a lightweight cross-window record of project state,
  commands, verification results, and pitfalls.
- Scene setup: `tomato_soup_can` and `banana` are always included; the remaining
  objects are randomly selected; selected objects are assigned to fixed initial
  placement slots.
- Grasp focus: recent debugging focused on `tomato_soup_can` side grasp height
  and object dropping during lift.
- Next recommended action: test the lower tomato-can side-grasp environment
  variables in `grasp_demo`; if the object still drops, inspect side jaw-roll /
  closing-axis behavior instead of only lowering the grasp point.

## Recent Work

### 2026-07-08 - Handoff Log Workflow

- Done: designed this persistent handoff workflow so future windows can resume
  faster.
- Changed files:
  - `docs/superpowers/specs/2026-07-08-agent-handoff-log-design.md`
  - `docs/superpowers/plans/2026-07-08-agent-handoff-log.md`
  - `docs/agent_handoff.md`
- Verified: Markdown file contains the required handoff sections and seed
  project context.
- Known issues: this is a manual log, so it only stays useful if meaningful
  work sessions update it.
- Pitfalls learned: do not rely on chat history alone for cross-window context.
- Next: keep this file updated after meaningful scene, grasping, or command
  workflow changes.

### 2026-07-06 to 2026-07-08 - Scene And Tomato Can Grasp Context

- Done: configured deterministic placement slots while keeping object identities
  partly random: fixed priority objects are `tomato_soup_can` and `banana`, with
  four additional randomly selected objects for six total.
- Changed files:
  - `src/ifl_air_mujoco_sim/env/config/base_env.yaml`
  - `src/ifl_air_mujoco_sim/env/utils/populate_scene.py`
  - `src/ifl_air_mujoco_sim/env/mjcontrol_interface.py`
  - `src/ifl_air_mujoco_sim/env/ros2_interface.py`
  - `src/ifl_air_mujoco_sim/main.py`
  - `src/ifl_air_mujoco_sim/ros2_main.py`
  - `src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py`
- Verified: focused scene selection tests passed with
  `PYTHONPATH=src/ifl_air_mujoco_sim pytest src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py -v`.
- Known issues: current tomato-can side grasp can visually look high and may
  drop/displace the object during lift.
- Pitfalls learned: for `tomato_soup_can`, tune `SIDE_GRASP_Z_OFFSET`, not
  `GRASP_Z_OFFSET`, because the can uses the `side` grasp profile.
- Next: run a trial with the lower side-grasp parameters listed below.

## Pitfalls To Avoid

- After calling `/reset_sim`, rerun `ros2 run my_course_pkg pipeline` before
  `ros2 run my_course_pkg grasp_demo`; reset makes old perception output stale.
- `tomato_soup_can` uses the side profile, so `GRASP_Z_OFFSET` does not lower
  its final grasp. Use `SIDE_GRASP_Z_OFFSET`.
- In the current selector, `GRASP_SIDE_MIN_HEIGHT_M` and
  `GRASP_SIDE_MAX_HEIGHT_M` filter the raw grasp-library object-local height
  before `SIDE_GRASP_Z_OFFSET` is applied. Do not set the max to `0.075` with
  the current library, because that can remove all useful tomato-can candidates.
- If lowering the tomato-can grasp point does not fix dropping, investigate
  side jaw-roll / closing-axis behavior instead of continuing to lower only Z.
- The workspace may contain unrelated dirty or untracked files. Do not revert
  them unless the user explicitly asks.

## Useful Commands

### Terminal 1: Simulation And MoveIt

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py
```

### Reset Simulation

```bash
ros2 service call /reset_sim std_srvs/srv/Trigger {}
```

Expected successful response includes:

```text
success=True
message='Simulation reset to default configuration'
```

### Terminal 2: Perception Pipeline

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run my_course_pkg pipeline
```

### Terminal 3: Grasp Demo With Lower Tomato-Can Side Grasp

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

export SIDE_GRASP_Z_OFFSET=-0.011
export GRASP_SIDE_MIN_HEIGHT_M=0.075
export GRASP_SIDE_MAX_HEIGHT_M=0.085
export GRASP_SIDE_TARGET_HEIGHT_M=0.081

ros2 run my_course_pkg grasp_demo
```

### Optional Debug Stop After Close

Use this only when inspecting whether the gripper has actually closed around
the object before lift:

```bash
export GRASP_DEBUG_STOP_AFTER_CLOSE=1
ros2 run my_course_pkg grasp_demo
```

Disable it for normal runs:

```bash
unset GRASP_DEBUG_STOP_AFTER_CLOSE
```

## Maintenance Rule

At the end of a meaningful work session, update this file when the work changes
project state, command workflow, verification status, or reusable debugging
knowledge. Keep entries short and factual.
```

- [ ] **Step 2: Verify required sections exist**

Run:

```powershell
Select-String -Path docs\agent_handoff.md -Pattern '^## Current Snapshot','^## Recent Work','^## Pitfalls To Avoid','^## Useful Commands','^## Maintenance Rule'
```

Expected: one match for each required section.

- [ ] **Step 3: Verify seeded project facts exist**

Run:

```powershell
Select-String -Path docs\agent_handoff.md -Pattern 'tomato_soup_can','banana','SIDE_GRASP_Z_OFFSET','reset_sim','GRASP_SIDE_TARGET_HEIGHT_M'
```

Expected: all five patterns are present.

- [ ] **Step 4: Review git diff**

Run:

```powershell
git diff -- docs\agent_handoff.md
```

Expected: the diff only creates `docs/agent_handoff.md`.

- [ ] **Step 5: Commit the handoff log**

Run:

```powershell
git add -- AGENTS.md docs\agent_handoff.md docs\superpowers\plans\2026-07-08-agent-handoff-log.md
git commit -m "docs: add agent handoff log"
```

Expected: one commit containing the startup instructions, handoff log, and
implementation plan.
