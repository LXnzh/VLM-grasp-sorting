# Agent Handoff Log Design

## Goal

Create one persistent handoff document that lets a new Codex window quickly
understand the current project state, recent work, verified commands, and known
pitfalls without rediscovering the same context.

## Scope

This design adds a human-readable Markdown workflow document only. It does not
change ROS nodes, simulation behavior, grasp planning code, or scene
configuration.

The first implementation creates `docs/agent_handoff.md` and seeds it with the
current scene-layout and grasp-debugging context.

## Document Location

Use one stable file:

```text
docs/agent_handoff.md
```

The fixed path is intentional. A new Codex window should only need to read one
known file before continuing work.

## Document Structure

The document has four sections:

1. `Current Snapshot`
   The latest known state: active branch, current goal, important assumptions,
   and the next recommended action.

2. `Recent Work`
   Chronological notes for completed work sessions. Each entry includes date,
   completed changes, touched files, verification commands, and remaining
   questions.

3. `Pitfalls To Avoid`
   Durable lessons that should remain visible across sessions, such as which
   grasp offset applies to side grasps or when pipeline output becomes stale.

4. `Useful Commands`
   Known-good ROS, reset, perception, grasp, and debug commands. Commands should
   include required environment variables when those variables materially affect
   behavior.

## Entry Format

Each new work session appends or updates a concise entry:

```markdown
### YYYY-MM-DD - Short Topic

- Done:
- Changed files:
- Verified:
- Known issues:
- Pitfalls learned:
- Next:
```

Entries should be factual and short. The document is a handoff map, not a full
debug transcript.

## Update Rules

At the end of a meaningful work session, Codex should:

1. Update `Current Snapshot` if the project state or next step changed.
2. Add a `Recent Work` entry for important changes or investigations.
3. Promote repeatable mistakes into `Pitfalls To Avoid`.
4. Add or correct commands in `Useful Commands` when the user discovers a
   reliable run sequence.

Small conversational answers do not need an update unless they change project
state or capture a reusable lesson.

## Initial Seed Content

The first version should include:

- The current scene setup: always include `tomato_soup_can` and `banana`, fill
  remaining objects randomly, and assign selected objects to fixed initial slots.
- The current three-terminal ROS workflow: launch simulation, run pipeline, run
  `grasp_demo`.
- The reset rule: after `/reset_sim`, rerun `pipeline` before `grasp_demo`.
- The tomato can side-grasp lesson: `tomato_soup_can` uses
  `SIDE_GRASP_Z_OFFSET`, not `GRASP_Z_OFFSET`.
- The trial side-grasp parameters suggested for a lower tomato-can grasp point:
  `SIDE_GRASP_Z_OFFSET=-0.011`,
  `GRASP_SIDE_MIN_HEIGHT_M=0.075`,
  `GRASP_SIDE_MAX_HEIGHT_M=0.085`,
  `GRASP_SIDE_TARGET_HEIGHT_M=0.081`.

## Success Criteria

A new Codex window can read `docs/agent_handoff.md` and answer:

- What is the current working state?
- What changed recently?
- Which commands are known to work?
- Which mistakes should not be repeated?
- What should be tried next?

No code changes are required for this feature to be useful.

## Non-Goals

- No automated logging system.
- No generated database or JSON state file.
- No replacement for git history, test logs, or detailed specs.
- No attempt to capture every terminal line.
