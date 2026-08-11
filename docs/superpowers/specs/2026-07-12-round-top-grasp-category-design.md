# Round-Top Grasp Category Design

## Goal

Add a reusable vertical top-grasp policy for compact round fruit and balls
without adding object-specific apple logic or changing the validated tomato-can
side-grasp behavior.

The first development object is `apple`. Category-level generalization is
validated later with at least one additional fruit and one ball.

## Scope

This design adds explicit object categories and implements the `round_top`
category. It retains the current can algorithm through a compatibility mapping.

V1 does not add round-object side-grasp fallback, automatic geometry
classification, online grasp generation, semantic part detection, or new box,
banana, and hammer policies. `pear` is an extended validation object and does
not block V1 acceptance.

## Category And Profile Mapping

Category describes object geometry and intended behavior. Profile remains the
compatibility value consumed by existing selection and planning code.

```python
GRASP_CATEGORY_BY_OBJECT = {
    "tomato_soup_can": "cylindrical_can",
    "tuna_fish_can": "cylindrical_can",
    "apple": "round_top",
    "lemon": "round_top",
    "peach": "round_top",
    "pear": "round_top",
    "orange": "round_top",
    "plum": "round_top",
    "baseball": "round_top",
    "tennis_ball": "round_top",
    "racquetball": "round_top",
    "pudding_box": "box",
    "gelatin_box": "box",
    "sponge": "box",
    "foam_brick": "box",
    "rubiks_cube": "box",
    "banana": "banana",
    "hammer": "tool_top",
}

GRASP_PROFILE_BY_CATEGORY = {
    "cylindrical_can": "side",
    "round_top": "round_top",
    "box": "vertical",
    "banana": "centered",
    "tool_top": "top_down",
}
```

`cylindrical_can -> side` is a strict compatibility adapter. Existing checks
for `profile == "side"`, can symmetry expansion, geometry centers, scoring,
clearance, trajectory, and execution remain unchanged. An unmapped object
fails before arm motion instead of silently using a default profile.

## Mandatory Fact-Finding Gate

No round-top filter implementation or live arm motion begins until these facts
are measured:

1. **Q1 - Object geometry:** compute the `013_apple` mesh axis-aligned bounding
   box center, extents, and center offset from the model origin. Apply the same
   offline process to every round-top object and store static geometry data;
   runtime code does not load meshes.
2. **Q2 - Candidate coverage:** inspect real apple `.npz` grasps and report the
   number satisfying a provisional 20-degree vertical approach limit, plus the
   distribution of geometry-center offsets and normalized heights.
3. **Q3 - Fingertip geometry:** derive and cross-check the TCP-to-lowest-finger
   geometry from the active URDF and MuJoCo model. Do not assume `-0.045 m`.
4. **Q4 - Gripper mapping:** establish the direction and mapping between the
   commanded `0.0..0.79` position and physical finger separation. The current
   open/closed command constants do not establish a millimeter conversion.

Q2 and Q4 are go/no-go decisions. If the grasp library lacks usable vertical
candidates, report that and design an offline geometric candidate supplement;
do not hide the gap by relaxing filters. If the verified gripper opening cannot
contain apple, do not execute an apple grasp.

## Offline Geometry Data

One offline utility computes an axis-aligned mesh bounding-box center and
extents for all configured objects and emits reviewable static data such as:

```python
OBJECT_GEOMETRY_BY_NAME = {
    "apple": {
        "center": [...],
        "bbox_size": [...],
    },
}
```

The values are object data, not object-specific behavior. Candidate centering
and symmetry expansion use the geometry center rather than the YCB origin.

## Round-Top Candidate Pipeline

The selector processes candidates in this order:

1. Load `T_object_grasp` candidates from the existing object grasp library.
2. Retain candidates whose TCP `+Z` approach axis is within the configured
   angle of world down. The provisional investigation threshold is 20 degrees;
   the committed default is chosen from Q2 data.
3. Expand candidates around the object geometry center at configured yaw
   angles, initially considering 45-degree increments, and deduplicate them.
4. Transform the geometry center into TCP coordinates and reject candidates
   whose TCP-plane XY center offset exceeds the category limit.
5. Compute TCP height relative to geometry center, normalized by bounding-box
   height. Use Q2 statistics to select a shared range; do not add apple-specific
   limits.
6. Estimate required grasp width from the smaller horizontal bounding-box
   dimension and compare it with the verified effective gripper opening.
7. Reject candidates whose finger geometry lacks table clearance at the final
   grasp or along the vertical approach.
8. Rank remaining candidates and return multiple candidates to the existing
   planner, which tries them in order.

The V1 score is yaw-neutral and combines approach angle, geometry-center
offset, and normalized height error. Hard safety constraints are filters, not
soft penalties. Motion cost is left to the existing planner and is not added
to the selector score in V1.

The initial candidate limit is configurable, with eight as the starting value.

## Gripper Width Safety

Offline diagnostics always report geometry even when the width is marginal.
Live behavior is stricter:

- A width above the verified effective opening rejects the candidate.
- A verified margin of at least 5 mm permits the staged live procedure.
- A positive margin below 5 mm permits only human-in-the-loop pregrasp and
  final-grasp inspection at first; it does not permit an immediate full run.

Logs include estimated width, horizontal minimum and maximum extents, verified
maximum opening, and remaining margin. No formula converts command position to
millimeters until Q4 establishes it.

## Table And Approach Clearance

For vertical approach, a finger point has:

```text
finger_z = tcp_z + tcp_to_fingertip_z
```

where `tcp_to_fingertip_z` is measured and normally negative. The endpoint
clearance check is:

```text
lowest_finger_z = min(pregrasp_z, grasp_z) + tcp_to_fingertip_z
lowest_finger_z >= table_z + clearance
```

For the normal downward approach, the final grasp is the lowest endpoint.
V1 also samples the interpolated approach poses as a consistency check and to
avoid embedding an endpoint-only assumption in future nonvertical paths.

Round-top uses the existing non-side trajectory:
`pregrasp -> Cartesian approach -> grasp`. It does not use can
`direct_to_grasp` behavior.

## Failure Behavior And Diagnostics

Round-top tries multiple candidates within the same strategy. It never falls
back to a side grasp in V1. If filtering or planning exhausts the candidates,
the task fails closed: no blind approach, no gripper close, and no continuation
into lift or place.

Structured diagnostics include:

- object and category;
- raw and symmetry-expanded counts;
- remaining counts after orientation, center, height, width, and table checks;
- selected candidate angle, center offset, normalized height, width margin,
  and `finger_clearance_m`;
- MoveIt rejection count and final failure reason.

## Code Boundaries

Pure geometry, filtering, scoring, and round-top selection stay in the grasp
selector. Scene transforms, MoveIt reachability, and approach-corridor checks
remain in the pick-place planner. Execution and fail-closed command sequencing
remain in the executor and trajectory planner.

The existing planner already accepts a candidate list. Round-top changes the
selector to return several candidates rather than using the current non-side
single-candidate return. No planner interface redesign is required.

## Verification

### Automated

- Category and category-to-profile mapping tests.
- Unknown-object fail-closed test.
- Tomato/tuna `side` compatibility and existing can regression tests.
- Round-top orientation acceptance and side-candidate rejection.
- Symmetry rotation about geometry center and duplicate removal.
- Center, normalized-height, width, and table-clearance filters.
- Multiple ranked candidates and first-unreachable/second-reachable behavior.
- Exhaustion produces no close-gripper execution step.
- Real apple grasp-library offline statistics and at least one viable candidate
  before live testing.

### Staged Live Validation

A. Perception and selector only; visualize object pose, geometry center, final
candidate, pregrasp, and approach corridor.

B. Move to pregrasp and stop.

C. Descend to final grasp and stop without closing.

D. Close and stop without lifting.

E. Lift and actively hold the object.

F. Run the existing safe place and release sequence.

Each reset or camera movement requires fresh perception according to the
existing workflow.

## Acceptance

- Tomato-can selection and execution behavior remains unchanged and its focused
  regression suite passes.
- Apple is handled only by shared `round_top` code and geometry data.
- Multiple geometrically valid round-top candidates are attempted in order.
- Candidate exhaustion stops before approach and gripper close with structured
  reasons.
- Apple completes grasp and stable lift through the staged procedure.
- Without changing the scoring formula or adding object-specific branches, at
  least one additional fruit and one ball complete the category validation.
- Pear failure does not block V1, but its failure stage and cause are recorded.

## Known Risks

- Apple diameter may leave insufficient Robotiq 2F-85 opening margin.
- The existing grasp library may contain too few centered vertical candidates.
- FoundationPose translation or table-height error may consume fingertip
  clearance.
- Finger closing direction and local surface geometry may cause slip during
  lift even when the grasp is centered.
- Category refactoring could regress can behavior; the compatibility mapping
  and can regression suite guard against this.
