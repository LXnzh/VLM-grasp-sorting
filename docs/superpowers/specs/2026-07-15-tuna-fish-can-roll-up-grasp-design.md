# Tuna Fish Can Roll-Up Grasp Design

## Goal

Make `tuna_fish_can` graspable on the tabletop with the existing Robotiq
parallel-jaw gripper by splitting the manipulation into two physical phases:

1. establish a light top-and-side preclamp and roll the can far enough to open
   a gap below its lower face;
2. verify that the rolled can's oblique support width fits the calibrated pad
   gap, fully close, then perform a 30 mm test lift before the normal lift.

The first implementation is deliberately staged. Existing simulator AABB
feedback provides automatic fail-closed checks at every physical phase, while
operator observation remains an additional validation signal. This is not a
claim of camera-based autonomous closed-loop grasping.

## Safety And Isolation Principle

All other objects are already stable or are being handled by separate repairs.
This work therefore uses an exact-name opt-in boundary:

```text
normalized object name == "tuna_fish_can" -> Tuna roll-up candidate and plan
all other object names                     -> existing path unchanged
```

The Tuna change must not alter shared side-, vertical-, round-top-, centered-,
or top-down selector defaults; shared profile mappings; existing candidate
ordering; planner tolerances; Cartesian convergence; gripper effort; normal
lift height; or place behavior for any other object. `tomato_soup_can` must
continue through its existing side-grasp path even though both objects are
currently in the same broad cylindrical-can category.

`tomato_soup_can` is a negative-control object, not a claim that Tomato and Tuna
share mechanics. Its test proves only that the exact Tuna branch, calibration,
subscriptions, services, and step actions are never constructed for another
cylindrical can.

Pure geometry or plan-step helpers may be reusable, but only the exact Tuna
branch may call the new roll-up policy in this change. `pudding_box` and `pear`
remain out of scope.

## Exact-Name Planner Routing

Do not add a shared `roll_up` category or change
`GRASP_PROFILE_BY_OBJECT`. `tuna_fish_can` may remain in
`SIDE_GRASP_OBJECTS` for compatibility, but the generic side route becomes
unreachable for this exact selected name.

`plan_pick_place_candidates_from_perception()` performs the routing decision
immediately after it reads and normalizes the selected name and loads the raw
object pose. Those are the only operations permitted before the exact-name
decision. In particular, no generic pose canonicalization, profile lookup,
offset lookup, grasp-library load, candidate expansion, obstacle mutation, or
place-selection initialization may run first. The Tuna branch performs any
scene reads it needs through its dedicated path. Before the existing function
body has any side effect, it executes:

```text
selected name == "tuna_fish_can"
    -> load and validate live Tuna bounds
    -> generate Tuna contact-support candidates
    -> build Tuna-only plans
    -> return those plans
otherwise
    -> execute the existing function body without changed inputs or ordering
```

The dedicated plan may set `debug_info["grasp_profile"] = "tuna_roll_up"` as
an execution tag. That tag is not added to the shared category/profile maps.
Tests must prove the Tuna branch never calls
`select_grasp_pose_candidates_6d()` or loads `/home/ws/grasps/007_tuna_fish_can`.

## Tool-Frame Convention

The current robot model establishes these axes:

```text
TCP +Z = library and planner approach axis
TCP  X = physical parallel-jaw closing axis
TCP  Y = tangent axis for the Tuna radial/vertical roll plane
```

TCP `X` is not inferred from a generic Robotiq convention. In this workspace,
the gripper-local finger separation is along gripper `X`; the installed chain
rotates `tool0_ext -> ur_to_robotiq_link` by 90 degrees about Z and then
`ur_to_robotiq_link -> gripper_base` by another 90 degrees. The composed
180-degree rotation preserves the TCP X axis up to sign. The left/right finger
origins in the gripper model are at opposite local X positions. TCP `X` is
therefore the sign-symmetric physical closing direction. Every generated pose
and geometry test uses this repository-specific convention.

URDF reasoning alone is not the final axis proof. The physical MuJoCo
integration gate commands the empty gripper from `0.00` to `0.20 rad`, measures
both pad contact-frame displacement vectors in TCP, and requires pad separation
to decrease, each displacement to align sign-symmetrically with TCP X within
1 degree, and transverse displacement to remain below 0.5 mm. Failure blocks
all Tuna contact tests.

## Why The Existing Tuna Library Cannot Work On The Tabletop

The Tuna mesh has the following approximate dimensions:

```text
diameter: 85.54-85.56 mm
height:   33.54 mm
```

The existing `ROUND_TOP_MAX_GRIPPER_OPENING_M = 0.08516` is a conservative
selector bound, not the Robotiq nominal specification and not a complete
command-to-gap calibration. Forward kinematics of the current custom-160
finger collision meshes gives an approximately 85.517 mm pad gap at the fully
open `0.0 rad` command. The Tuna diameter is about 0.042 mm larger in that rigid
model. This difference is at the mesh/model tolerance scale and must not be
presented as a manufacturing-grade physical measurement. Diameter and maximum
opening are effectively equal within model uncertainty, while the nominal
rigid collision model still supplies zero positive insertion clearance. A
centered-diameter strategy therefore cannot satisfy the required explicit
opening margin. The Tuna design does not reuse the round-top constant as its
calibration source.

The library is not an alternative tabletop solution. Offline inspection of all
5,002 Tuna poses found:

- every tool approach axis lies within about 15 degrees of horizontal;
- every gripper closing axis lies at least about 74 degrees above horizontal,
  and most are almost vertical;
- all candidate TCP translations lie near the can's mid-height.

Those poses are free-space top/bottom thickness grasps: one finger approaches
the top face while the other must pass below the bottom face. On a tabletop the
lower finger is blocked by the table. Relaxing Tuna's current side-filter angle
or height thresholds would expose physically blocked poses rather than fix the
grasp.

## Alternatives Considered

### 1. Centered diameter pinch

Rejected as the primary strategy. The object diameter exceeds the effective
opening and leaves no insertion margin.

### 2. Off-center chord or fingertip-edge pinch

An off-center circular chord can be narrower than the full diameter, but both
contacts lie on the same half of the circle. This requires friction to prevent
the can from being squeezed out of the pads and provides little tolerance for
pose error. It may be useful as a diagnostic, but it does not meet the stable
grasp goal.

### 3. Pull-tab or rim-feature pinch

Rejected for the first repair. It depends on reliable pull-tab geometry and
object yaw. The latest Tuna pose estimate had unstable roll/pitch, and gripper
occlusion would make the required small-feature alignment fragile.

### 4. Top-and-side preclamp followed by roll-up

Selected. It initially uses accessible surfaces above the table, deliberately
creates bottom clearance, then closes across a geometry-verified oblique width.
It is more complex than a single grasp but does not require the gripper to
insert around the flat can's full diameter while the table blocks the lower
finger.

### 5. Vacuum suction

Mechanically plausible on the approximately 85 mm flat lid, but unavailable
with the installed Robotiq-only end effector. Adding a suction tool, vacuum
model, or tool-change workflow is outside this repair and would affect shared
hardware configuration.

## Required Scene Geometry

The Tuna plan must be anchored to the selected object's live scene marker
bounds, not to fixed world coordinates and not solely to the raw
FoundationPose translation. The latest failure showed a suspicious raw
FoundationPose height and an upside-down object-local Z axis. A circular,
flat-lying can does not require its texture yaw for this maneuver.

Before generating a Tuna candidate, load the exact selected target bounds and
validate all of the following:

- the center and half-extents are finite and positive;
- the bounds describe a low, approximately circular object whose horizontal
  dimensions and height are compatible with the stored Tuna geometry within a
  Tuna-only tolerance;
- the target bottom is consistent with the tabletop;
- the initial world-AABB height is compatible with a flat can; an already
  tilted or moving can is rejected;
- three consecutive fresh exact-name samples are stable, with center and size
  standard deviation no greater than
  `TUNA_INITIAL_BOUNDS_STABILITY_TOLERANCE_M` (nominally 0.002 m per
  component);
- the selected object name is exactly `tuna_fish_can`.

Missing, stale, inconsistent, or non-flat bounds fail closed before motion.
The raw pose may still be logged for diagnosis, but Tuna's contact locations
come from the validated live bounds center, radius, top, and bottom.

## Clear-Side Candidate Generation

Because Tuna is approximately circular in the tabletop plane, the maneuver
does not need a stable texture yaw. Generate a small Tuna-only set of radial
approach directions around the live center. `TUNA_RADIAL_DIRECTION_COUNT`,
exposed as `GRASP_TUNA_RADIAL_DIRECTION_COUNT`, accepts only `4` or `8` and
defaults to four world-horizontal directions (`+X`, `-X`, `+Y`, `-Y`). Eight
directions add the 45-degree diagonals without changing ranking or any
non-Tuna path.

For each radial direction:

1. Define a prospective lower support region on the sidewall immediately above
   the corresponding bottom-rim tangent.
2. Construct a gripper orientation in the radial/vertical plane.
3. Point tool `+Z` diagonally inward and downward, with an acute angle of
   approximately 35-45 degrees to the world-horizontal plane (equivalently
   45-55 degrees from world down). This defines both reference plane and sign;
   it is not an ambiguous "above horizontal" convention.
4. Orient TCP `X`, the Robotiq closing axis, in the same radial/vertical plane
   so one finger is lower and radially inward while the other is above the top
   face near the rim.
5. Keep TCP `Y` tangent to the can to reduce unintended lateral rolling.
6. Build a collision-aware pregrasp outside the can and a short final approach
   to the open contact-support pose.

Rank every generated direction by non-target obstacle clearance and
reachability. Use the highest-ranked candidate that passes all hard gates; if
it fails, try the next ranked radial direction. Tuna is not required to roll
toward a particular world direction. A candidate must have a clear pregrasp
corridor, a clear roll corridor, and enough open space for the 30 mm test lift.
The chosen direction and every rejected predecessor must be logged explicitly.

The Tuna pregrasp is explicit and does not call the generic
`build_pre_grasp_pose_6d()` implicitly:

```text
p_pregrasp = p_contact_support
           - TUNA_APPROACH_DIST_M * tool_z_world
R_pregrasp = R_contact_support
```

Tool `+Z` points inward and downward, so subtracting it retreats outward and
upward; it is not a downward retreat. MoveIt plans from the current arm state
to this pregrasp. The short pregrasp-to-contact path is a sampled Cartesian
line, and every sample must pass the full-state validity contract below before
the executor may servo it.

## Contact-Support Pose

The initial contact-support pose is not the final lifting grasp. With the
gripper fully open:

- place the lower pad contact center on the lower sidewall at exactly one of
  8, 9, or 10 mm above the live target bottom;
- place the upper pad above the top face near the same radial rim region;
- require every modeled gripper collision point to remain at least 4-6 mm
  above the tabletop;
- keep the wrist and knuckles outside the target footprint and table.

The discrete 8/9/10 mm sidewall contact heights and provisional 4-6 mm whole-
gripper table-clearance window are different constraints and must not be
conflated. The
window is not a different safety policy from Pudding's 5 mm value; Tuna's lower
sidewall contact requires calibration to select one exact
`TUNA_MIN_TABLE_CLEARANCE_M` inside that window. Acceptance uses that selected
value, not the range. If no pose satisfies both contact geometry and the
selected clearance, candidate generation fails before motion.

The full-state table check is not limited to finger meshes. At every sampled
pregrasp, contact, roll, close, and lift waypoint, request MoveIt IK seeded from
the previous accepted joint state, require the solution to stay on that
continuous branch, then run forward kinematics and MoveIt's full state-validity
check. The checked collision links cover the UR base, upper arm, forearm,
wrist links, flange, camera/adapter mounts, gripper base/knuckles, and both
complete finger chains. Self-collision, table, and non-target-object collision
remain forbidden. Intentional Tuna contact is allowed only for the two
finger-tip/pad regions named by the Tuna contact model; there is no broad
allowed-collision-matrix relaxation.

## Gripper Geometry Calibration

The action command is a knuckle-joint angle, not a pad gap. The adapter maps
`0.0-0.8 rad` linearly to its normalized `0-255` request, while the Robotiq
linkage makes pad separation nonlinear. Add the offline script
`src/my_course_pkg/tools/calibrate_tuna_gripper_geometry.py` with explicit
inputs:

- the expanded robot URDF used by MoveIt;
- the custom-160 left/right collision STL files;
- the command range and sample interval;
- the TCP and gripper mounting transforms.

The script performs URDF forward kinematics for both finger chains and emits a
machine-readable table containing command radians, inner pad gap, pad contact
frames including `T_tcp_lower_pad_contact(q_i)` at every command sample, a
conservative TCP-frame support hull covering every gripper collision mesh, and
the interpolation error bound between samples. A single nominal
`T_tcp_pivot_at_preclamp` is not authoritative because object contact can stop
the gripper at a different measured qpos. A scalar minimum-Z value in TCP is
also insufficient because the gripper rotates relative to world Z. The matrix
table and support hull must permit the contact frame and world-Z minimum to be
recomputed for the actual aperture.

The artifact records hashes of the expanded URDF, all contributing collision
meshes, mounting transforms, and calibration output schema using SHA-256.
Runtime must reproduce and verify those SHA-256 values before using the hull or
contact-frame matrix table; a mismatch fails before Tuna motion. Adaptive command subdivision
continues until the conservative support interpolation error is no more than
0.25 mm. A simulator check then commands the same samples and compares measured
finger-link transforms to the offline table.

The current model provides these preliminary values, which the committed
calibration must reproduce within tolerance:

| Command | Inner collision-pad gap |
|---:|---:|
| 0.00 rad | 85.517 mm |
| 0.20 rad | 66.783 mm |
| 0.40 rad | 45.833 mm |
| 0.50 rad | 34.785 mm |
| 0.79 rad | 1.819 mm |

`TUNA_PRECLAMP_POSITION`, exposed as `GRASP_TUNA_PRECLAMP_POSITION`, is added
only after that calibration selects and verifies a target. The design does not
guess its numeric default from the nominal 85 mm stroke or select it by matching
the can's 33.54 mm axial height. The pads contact an oblique top/side geometry,
so a command such as `0.50 rad -> 34.785 mm` is only a free-space reference and
is not evidence that 34.785 mm is the in-contact aperture. Runtime validates the
target strictly between the shared open and closed commands and validates the
measured contact-limited aperture separately.

## Contact-Height-To-Table-Clearance Validation Chain

The initial lower contact height and whole-gripper table clearance are coupled
and must be proved by one explicit chain. Candidate generation uses only these
Tuna-only discrete grids:

```text
contact height above live bottom = {0.008, 0.009, 0.010} m
tool +Z angle to world horizontal = {35, 40, 45} degrees
radial direction                 = configured 4 or 8 directions
```

For each grid candidate:

1. obtain `z_table` from the planning-scene table collision geometry and
   cross-check it against the validated flat Tuna bottom;
2. place the calibrated lower-pad contact frame at the requested sidewall
   height and solve the corresponding `T_world_tcp`;
3. before contact, conservatively sample the open-to-contact preclamp sweep and
   every calibration-table qpos that bounds the accepted contact band across
   the provisional roll and allowed-close sweeps; for each sample, use its own
   contact matrix and transform every vertex of its calibrated support hull into
   world;
4. compute the conservative lower bound

```text
clearance_lower_bound =
    min(world_z of all transformed support vertices)
  - z_table
  - calibration_interpolation_error
```

5. require `clearance_lower_bound >= TUNA_MIN_TABLE_CLEARANCE_M` at every
   sample, then independently require MoveIt full-state validity for adapter,
   wrist, arm, self-collision, table, and non-target geometry.

The pre-contact calculation is an aperture-band envelope, not a nominal-pivot
promise. Runtime repeats the support-hull calculation with the measured
preclamp qpos before the first roll and with measured qpos after every
close/observation boundary. A candidate is rejected if any provisional or
measured sample fails, if qpos lies outside the calibrated contact band, or if
the table-height cross-check disagrees. The planner may try the next discrete
height, pitch, or radial candidate; it may not interpolate an unverified contact
height, reduce the exact clearance threshold, or accept a candidate merely
because the lower-pad point itself is above table.

## Partial Preclamp

At the contact-support pose, command `TUNA_PRECLAMP_POSITION`. The calibrated
mapping supplies both its nominal free-space pad gap and expected contact range.

The preclamp should create two light contacts:

- upper finger on the top face near the rim;
- lower finger on the lower sidewall above the table.

Record target position, actual position, effort, `stalled`, and `reached_goal`.
The command fails closed when it is rejected, stalls implausibly early, closes
beyond the Tuna preclamp range, or provides no usable evidence of contact under
the calibrated simulator behavior. No roll motion follows an invalid
preclamp.

Contact evidence is operational, not visual-only. Compare the measured joint
position with the calibrated free-space result for the same command, convert
both through the monotonic command-to-gap table, and require
`measured_contact_gap - calibrated_free_space_gap >=
TUNA_MIN_CONTACT_DEFLECTION_M` (nominally 0.5 mm), together with a compatible
`stalled`/`reached_goal` result. MuJoCo contact-force data may be logged as
secondary evidence but is not required by the cross-backend contract.

The 0.5 mm contact-deflection threshold and the later 0.5 mm aperture-drift
limits have different physical sources and are not interchangeable. Contact
deflection compares the blocked result against a separately calibrated
free-space mapping for the same command. Pivot and retention drift compare a
single frozen in-contact aperture against later samples while the same position
target is held. The equal first-patch numbers are coincidental; each gate keeps
its own constant, reference sample, diagnostics, and test fixtures.

Immediately after preclamp, fresh AABB center and size must remain within a
separate `TUNA_PRECLAMP_BOUNDS_TOLERANCE_M` envelope, nominally 0.002 m per
component. An unexpected height increase is uncontrolled early rotation, not
credit toward the requested roll angle; it aborts the run rather than reducing
later segment angles automatically.

### Measured-Aperture Pivot Freeze Gate

Any qpos batch used by a Tuna gate consists of three consecutive measured
gripper samples with distinct, increasing timestamps strictly newer than its
relevant completed command boundary. For pivot freeze that boundary is the
preclamp command; for retention freeze it is full close. Stability means the
peak-to-peak range of the calibration-driving qpos across those three samples
is no greater than
`TUNA_QPOS_STABILITY_TOLERANCE_RAD = 0.002 rad`; an override may tighten but
not enlarge it. Convert their median through the hash-verified calibration
table and interpolate `T_tcp_lower_pad_contact(q_measured)`. The measured qpos
must lie inside the calibrated contact band, and matrix interpolation error must
remain within the artifact's 0.25 mm bound. Then compute and log:

```text
T_tcp_pivot_frozen = T_tcp_lower_pad_contact(q_measured)
T_world_pivot      = T_world_tcp_preclamp * T_tcp_pivot_frozen
pivot_contract     = {
    calibration_sha256,
    sample_timestamps,
    qpos_peak_to_peak,
    measured_qpos,
    measured_pad_gap,
    T_tcp_pivot_frozen,
    T_world_pivot
}
```

The initial Tuna plan contains executable steps only through
`observe_tuna_after_preclamp`, followed by a typed, non-executable deferred
suffix descriptor. It must not contain nominal-pivot roll poses. After freezing
the contract, the executor invokes the Tuna-only
`finalize_tuna_post_preclamp(plan, pivot_contract, measured_joint_state)` entry
point.

Before each finalization preparation attempt, consume two new stable exact-name
bounds samples whose timestamps are strictly later than the successful
`observe_tuna_after_preclamp` step, or later than the preceding failed attempt.
Compare them with the accepted post-preclamp center/size baseline using
`TUNA_PRECLAMP_BOUNDS_TOLERANCE_M`. Also collect three new qpos samples meeting
the quantitative stability rule above and require their median pad gap to remain
within `TUNA_PIVOT_APERTURE_DRIFT_TOLERANCE_M` of the already frozen contract.
These checks validate current state; they never freeze a replacement pivot.

The finalizer prepares the complete roll/close/lift suffix in an immutable local
value, reruns continuous-branch IK, full state validity, support-hull clearance,
and adjacent-motion limits from the measured current state, and does not mutate
the plan during preparation. A dependency timeout or service-unavailable result
that contains no semantic IK/state-validity answer and issued no robot command
may receive exactly one local retry after 500 ms. The executor holds the arm and
preclamp target during the wait, logs attempt number, dependency, elapsed time,
failure class, and fresh-state evidence, then repeats both fresh-bounds and qpos
gates while reusing the same frozen pivot. Explicit no-IK, invalid-state,
collision, malformed response, bounds change, qpos drift, and any robot-command
side effect are non-transient and receive no retry.

Only a fully validated local suffix may cross the atomic commit point and
replace the deferred descriptor. The commit records a unique
`suffix_generation_id`, attempt count, pivot-contract hash, and plan revision.
Single-use applies to this commit: at most one committed suffix can exist.
Failure of both preparation attempts leaves no executable suffix and raises a
structured non-retryable error. Failure while dispatching the first roll command
occurs after commit and must hold/abort; it cannot invoke finalization again.
The generation-only debug stage may print a conservative nominal envelope, but
no pose from that envelope is executable.

Before the first roll, rerun the complete support-hull table-clearance chain
with this measured qpos. Once roll begins, the contract is immutable. Split
each visible 10/20/30-degree stage into blocking Cartesian micro-segments whose
angular change is no greater than
`TUNA_ROLL_MICROSEGMENT_MAX_ANGLE_DEG = 2.5 degrees`. Before the first
micro-segment and after every micro-segment, convert current qpos to pad gap and
require drift from the frozen measured gap no greater than
`TUNA_PIVOT_APERTURE_DRIFT_TOLERANCE_M = 0.0005`. The next arm command cannot be
sent until this gate passes, so every 10-degree visible segment contains at
least three internal aperture checks in addition to its boundaries. A larger
drift invalidates the kinematic pivot: snapshot and hold the latest measured TCP
pose, preserve the gripper target, and raise a non-retryable error. Never
silently interpolate a new pivot or re-anchor `T_world_pivot` after intentional
object motion begins.

No parallel arm-plus-gripper executor primitive is required in the current
simulator. A gripper action writes the adapter's persistent
`internal_goal_position`; `ros2_interface.py` reads that target and reapplies it
to the MuJoCo actuator on every control cycle. During roll, the controller
therefore maintains the same preclamp position demand even if the measured
aperture changes with contact. The executor sends no second gripper command
until full close.

This persistent-setpoint behavior is a preflight contract, not an assumption:
before Tuna motion is enabled, command the calibrated preclamp without an
object at a full-state-valid calibration pose with at least 50 mm table
clearance. Execute exactly a `+10 mm world X -> return` move and a
`+10 mm world Z -> return` move, and verify that gripper qpos stays
within `TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M`, exposed as
`GRASP_TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M` and nominally 0.0005 m of
calibrated pad-gap equivalent, without command replay. No downward, inward-to-
object, rotational, or larger-amplitude motion belongs to this preflight. A
backend that does not pass this check fails closed.
`GRIPPER_EFFORT` is not split into a Tuna value in the first patch because the
current MuJoCo control loop is driven by the persistent position target; the
action's effort field does not provide a separately verified Tuna contact-force
controller. In particular, this design does not import Pudding force values or
claim a verified 50 N-to-140 N force transition for Tuna.

## Pivot-Preserving Roll Trajectory

The roll must be defined around the lower fingertip support point near the rim
rather than as an arbitrary simultaneous lift and wrist rotation.

Use only the post-preclamp `T_tcp_pivot_frozen` and `T_world_pivot` from the
measured-aperture gate; do not use the nominal command target or reconstruct the
pivot from nominal finger geometry. For a requested roll angle `theta`, generate
TCP waypoints from the rigid pivot relationship:

```text
T_world_tcp(theta) =
    T_world_pivot
  * R_tangent(theta)
  * inverse(T_tcp_pivot_frozen)
```

Here `T_world_pivot` is the calibrated lower-pad support point, not an assumed
point underneath the can, and `R_tangent` rotates in the radial/vertical plane
about the tangent axis. The commanded support-point origin is fixed in world;
it is not treated as a moving pivot. The waypoint sequence keeps the modeled
lower contact point approximately stationary while the TCP rises and the wrist
turns. Actual contact slip can move the physical instantaneous center, which is
why live center/height residuals remain mandatory rather than being folded into
the kinematic model.

The sign convention is fixed. Let world up be `u = [0, 0, 1]` and let `r_near`
point from the Tuna center toward the selected lower-pad contact side. Define
the tangent axis `t = normalize(cross(u, r_near))`. Positive `theta` is a
right-handed rotation about `t`; it raises the can center and far side while the
near support remains fixed. The 10/20/30-degree stages are positive roll-in
segments. Negative rollback/roll-out motion is not part of the first Tuna
implementation; on failure the executor holds instead of reversing.

The full target is 25-35 degrees, with 30 degrees as the nominal design value.
During staged validation, execute smaller increments first (for example 10,
20, then 30 degrees). The Tuna executor sends each at-most-2.5-degree Cartesian
micro-segment as a separate blocking arm command under the same indexed plan
step, runs the aperture gate after completion, and only then dispatches the next
micro-segment. Stop immediately on controller failure, pose non-convergence,
table-clearance violation, or an invalid gripper result. This Tuna-only
subdivision does not change the execution of ordinary non-Tuna `move` steps.

Before execution, sample the complete waypoint set and validate:

- finite rigid transforms and reachable IK on one continuous joint branch;
- full robot-state validity, including every arm, wrist, adapter, and gripper
  collision link described in the contact-support contract;
- no self-collision, table collision, or collision with non-target objects;
- bounded translation and rotation between adjacent samples;
- a monotonically increasing predicted clearance under the far side of the
  can.

The simulator already publishes fresh world-axis object AABBs on
`/scene_clearance_bounds` every joint-state publish cycle. After preclamp and
after every roll segment, wait for two fresh, stable samples whose marker text
matches exactly `tuna_fish_can`. For an ideal circular cylinder rolled by
`theta`, the predicted AABB height is:

```text
predicted_height(theta) = H * abs(cos(theta)) + D * abs(sin(theta))
```

For `H = 33.54 mm` and `D = 85.56 mm`, the expected heights are approximately
47.9, 60.8, and 71.8 mm at 10, 20, and 30 degrees. This full AABB extent depends
on cylinder orientation, not on the world location of the pivot. Moving the
pivot from the bottom edge to the calibrated 8/9/10 mm lower-pad contact changes
the predicted AABB *center trajectory*, but does not justify replacing the
height formula with a contact-height term.

The predicted center rotates the initial live AABB-center offset around the
same calibrated pivot used for the TCP. Because the real Tuna mesh is not a
perfectly symmetric analytic cylinder, the residual experiment uses a fixed
matrix rather than an informal yaw selection. For every enabled
radial direction and each 10/20/30-degree checkpoint, perform ten independent
fresh resets with object body yaw, measured relative to that radial direction,
in this order:

```text
0, 45, 90, 135, 180, 225, 270, 315, 0, 180 degrees
```

The first eight runs cover one complete 45-degree grid; the last two repeat
opposed seam/tab orientations to expose repeatability drift. Record per-cell
and aggregate mean, standard deviation, and maximum residual for radial,
tangential, and vertical components. The nominal
`TUNA_BOUNDS_TOLERANCE_M = 0.005` is a maximum acceptance envelope, not a value
that may be enlarged automatically to make data pass. If any cell exceeds it,
revise the pivot/contact model before enabling that radial-direction count.

This feedback distinguishes the main failure modes:

- **preclamp displacement:** center moves or height changes beyond the stricter
  calibrated preclamp envelope
  before a roll command;
- **slide without roll:** center translates while height remains near 33.54 mm;
- **roll with excessive slip:** height matches the commanded angle but radial
  or tangential center displacement leaves the predicted pivot envelope;
- **successful segment:** both height and center match the predicted envelope.

Any automatic mismatch stops before the next segment. Operator inspection
remains an additional staged check, not the sole evidence. This feedback uses
existing MuJoCo ground-truth bounds; it does not add camera re-segmentation or
FoundationPose during motion.

## Full Close And Test Lift

After the roll reaches the validated support pose:

1. stop arm motion while maintaining the persistent preclamp target;
2. validate the closing geometry below;
3. command the normal full-close target;
4. validate and log the gripper result;
5. hold briefly;
6. consume fresh exact-name AABB samples and require full close not to displace
   the validated rolled center or AABB size beyond the Tuna-only tolerance;
7. collect three consecutive stable post-close qpos samples, convert their
   median through the calibration table, and freeze the retention contract
   described below;
8. lift exactly 30 mm in world Z using separately dispatched at-most-10-mm
   micro-segments, checking retention aperture before the first and after every
   micro-segment, then stop;
9. consume fresh exact-name AABB samples, recheck retention aperture, and
   automatically require the Tuna
   center to rise
   `0.030 m +/- TUNA_LIFT_FOLLOW_TOLERANCE_M`, horizontal center drift and each
   AABB-size component change to remain within the same dedicated tolerance;
10. additionally require operator confirmation during staged validation that
   the can follows the gripper;
11. only after both checks pass enable the normal lift/transfer suffix.

The full-close command intentionally changes aperture, so lift monitoring must
not compare against the preclamp `pivot_contract`. After the close result and
post-close AABB check pass, use the same three-sample qpos stability rule to
freeze:

```text
retention_contract = {
    calibration_sha256,
    full_close_target,
    sample_timestamps,
    qpos_peak_to_peak,
    measured_post_close_qpos,
    measured_post_close_pad_gap
}
```

`TUNA_RETENTION_APERTURE_DRIFT_TOLERANCE_M` is at most 0.0005 m and compares
later lift samples only with `measured_post_close_pad_gap`. A larger drift raises
`RETENTION_APERTURE_DRIFT`, holds the latest measured TCP, preserves the current
full-close target, and stops before the next lift command. If aperture remains
stable but AABB follow fails, raise `RETENTION` instead; this separates gripper
position loss from object slip, obstruction, or detachment.

Do not assume the rolled jaws close across exactly the 33.54 mm axial
thickness. Let `delta` be the acute angle between TCP X and the predicted
cylinder axis after roll. A conservative cylinder support width along the
closing axis is:

```text
required_width(delta) = H * abs(cos(delta)) + D * abs(sin(delta))
```

At 30 degrees this is about 71.8 mm; at 45 degrees it is about 84.2 mm. The
alternative `H / cos(delta)` value describes only a tilted slab and omits the
cylinder's radial contribution, so it is not a safe containment calculation.
Do not compare this width with the free-space full-close gap. A full-close
command deliberately requests a smaller target than the contact-limited
aperture and is not expected to reach its 1.819 mm free-space gap while holding
the can.

Use three separate gates:

1. **Initial straddle clearance:** before first contact, the calibrated fully
   open pad gap must exceed the maximum projected support width at the open
   contact-support pose by `TUNA_MIN_STRADDLE_MARGIN_M`, nominally 1 mm. After
   contact, the measured aperture is compared with the predicted contact band;
   it is not required to retain free-space insertion margin.
2. **Raised-bottom kinematic access:** the complete sampled lower-finger
   collision-mesh sweep from preclamp through the close-command pose must fit
   inside the predicted raised-bottom region without table or non-pad contact.
3. **Contact-limited close and retention:** the full-close target must request
   additional travel beyond the measured preclose position, while the actual
   joint position remains in the calibrated object-contact band and the
   `stalled`/`reached_goal` result is compatible with contact. This proves a
   maintained position demand, not a numeric force closure. The subsequent
   retention-aperture and fresh-AABB 30 mm test-lift checks form the retention
   proof together.

The first patch does not claim a 50 N preclamp or a 140 N final clamp because
the current MuJoCo backend has no separately verified effort-to-contact-force
mapping for this action path.

For ideal pivoting at 30 degrees, the bottom-face center rises about
`R * sin(30 deg) = 21.4 mm`, while the opposite bottom rim rises as much as
`D * sin(30 deg) = 42.8 mm`. Neither scalar alone proves finger access. The
sampled collision-mesh sweep is the acceptance test.

Define the rolled, fully closed TCP as `rolled_close_pose`. The dedicated plan
derives:

```text
test_lift_pose   = rolled_close_pose + [0, 0, 0.030 m] in world translation
normal_lift_pose = rolled_close_pose + [0, 0, GRASP_LIFT_HEIGHT] in world translation
```

Both retain the rolled orientation. The normal-lift segment starts at
`test_lift_pose`. `TUNA_LIFT_FOLLOW_TOLERANCE_M`, exposed as
`GRASP_TUNA_LIFT_FOLLOW_TOLERANCE_M`, is at most 0.002 m for the first patch;
an override may tighten but never enlarge it.
The 5 mm roll-model residual envelope does not apply to pure lift translation;
2 mm is 6.7% of the 30 mm test lift rather than 16.7%. Divide the remaining
world-Z distance into Tuna-only observation segments no longer than
`TUNA_LIFT_OBSERVATION_SPACING_M`, nominally 50 mm. Further divide every test-
and normal-lift observation segment into separately dispatched Cartesian
micro-segments no longer than
`TUNA_LIFT_MICROSEGMENT_MAX_TRANSLATION_M = 0.010 m`. Check the retention
aperture before the first micro-segment, after every micro-segment, and again at
each blocking exact-name AABB observation. The next arm command cannot be sent
until the aperture gate passes. Each AABB observation requires the center to
follow the commanded world-Z increment, XY drift and size change to remain
within the same 2 mm tolerance, and samples to remain fresh and stable.
Transfer/place starts only after the final checkpoint at `normal_lift_pose`;
there is no mapping back to the original flat contact pose and no discontinuous
standard plan splice. Shared lift height and place helpers may be reused only to
compute suffix targets. All checkpointing remains Tuna-only.

Failure of full close, retention-contract freeze, any micro-segment aperture
check, Cartesian convergence, the test lift, or any normal-lift follow
checkpoint stops the plan without continuing to the next lift or place sequence.

## Execution-Time Feedback Boundary

The first patch may consume the existing live MuJoCo AABB stream at stage
boundaries because it is generated directly from the selected simulation body
and carries the exact object name. It may not add camera re-segmentation,
FoundationPose reruns, continuous visual servoing, or a new simulator topic.
The AABB supplies center and world-axis size but not full orientation, so the
analytical pivot model remains the source of the predicted cylinder axis. AABB
height and center only validate whether the observed motion is consistent with
that model.

Missing, stale, duplicate-name, or unstable post-stage bounds fail closed. A
future real-camera closed-loop version requires a separate design because
gripper occlusion and identity confidence are different failure modes.

## Tuna-Only Plan Shape

After the measured-aperture finalizer atomically installs the deferred suffix,
the dedicated Tuna plan has these semantically explicit steps. Before
finalization, only the prefix through `observe_tuna_after_preclamp` is
executable:

```text
open_gripper_before_approach
move_to_pre_grasp
approach_tuna_contact_support
preclamp_tuna
hold_after_tuna_preclamp
observe_tuna_after_preclamp
roll_tuna_segment_01_of_03
observe_tuna_segment_01_of_03
roll_tuna_segment_02_of_03
observe_tuna_segment_02_of_03
roll_tuna_segment_03_of_03
observe_tuna_segment_03_of_03
hold_after_tuna_roll
close_gripper_at_grasp
hold_after_close
observe_tuna_after_close
test_lift_tuna_30mm
observe_tuna_after_test_lift
hold_after_lift
normal_lift_tuna_segment_01_of_N
observe_tuna_normal_lift_segment_01_of_N
...
normal_lift_tuna_segment_N_of_N
observe_tuna_normal_lift_segment_N_of_N
...
```

Roll and normal-lift segment names encode stable indexes, not hard-coded angle
or distance values. The plan's debug metadata records segment counts,
start/end angles or heights, and every sampled waypoint, so changing configured
geometry does not rename the execution primitive. `N` is computed so every
post-test-lift segment is at most `TUNA_LIFT_OBSERVATION_SPACING_M`. Existing
non-Tuna plans retain their current step names and order.

Each `roll_tuna_segment_*` and lift remains an ordinary `action="move"` plan
step, but exact-Tuna debug metadata carries its validated micro-waypoint list.
Only the Tuna executor dispatches those micro-waypoints as individual blocking
arm commands with aperture checks between them; non-Tuna `move` semantics do
not change. Each `observe_tuna_*` entry is a Tuna-only blocking
`action="observe_tuna_bounds"` step. Its expected pre/post center, AABB size,
angle, freshness threshold, and tolerance are stored under the same step name
in `PickPlacePlan.debug_info["tuna_observations"]`; no shared `PlanStep` fields
or non-Tuna action semantics change. The executor accepts this action only when
the plan's normalized target name is exactly `tuna_fish_can`, waits for two
fresh stable exact-name samples, classifies the result, and either returns
success or aborts the plan. The `preclamp`, `roll_segment_*`, `close`,
`test_lift`, and `normal_lift` debug stops occur only after their corresponding
observation step succeeds; `normal_lift` means the final indexed lift
observation has passed.

## Staged Debug Workflow

Each stage is a fresh, explicit run. Advancing one stage must require success
at all earlier stages; no flag may silently skip a failed prerequisite.

Use one Tuna-only selector rather than adding several interacting booleans:

```text
TUNA_DEBUG_STOP_AFTER =
    generation | pregrasp | contact_support | preclamp |
    roll_segment_1 | roll_segment_2 | roll_segment_3 |
    close | test_lift | normal_lift | none

environment variable: GRASP_TUNA_DEBUG_STOP_AFTER
```

For an exact Tuna run, combining this selector with any legacy global
`GRASP_DEBUG_STOP_*` flag is a configuration error and fails before motion.
Non-Tuna runs never read the Tuna selector, and their existing flags retain
their current meaning.

1. **Offline generation only:** print live bounds, chosen clear side, contact
   points, calibration contact-matrix table, conservative nominal trajectory
   envelope, predicted clearances, and gripper gaps. Do not present the nominal
   envelope as an executable frozen-pivot suffix.
2. **Pregrasp stop:** move only to the collision-aware pregrasp.
3. **Open contact-support stop:** approach with the gripper open; verify lower
   and upper finger placement and table clearance.
4. **Preclamp stop:** partially close and verify top/side contact without
   sliding the can.
5. **Incremental roll stops:** first 10 degrees, then 20 degrees, then the
   nominal 30 degrees. Require fresh AABB height/center agreement and verify the
   lower contact does not scrape or translate excessively. `roll_segment_3`
   always means completion of all configured roll segments, even if the final
   configured angle is not exactly 30 degrees.
6. **After-close stop:** fully close at the rolled pose without lifting.
7. **30 mm test-lift stop:** automatically verify the expected 30 mm world-Z
   center rise, bounded XY drift, and stable rolled AABB size, then visually
   verify the can follows the gripper.
8. **Normal lift:** execute indexed at-most-50-mm Tuna lift/observation segments
   only after stage 7 is repeatable; `normal_lift` stops after the final bounds
   checkpoint, while `none` permits transfer/place to continue.

During the first implementation, stages 3-8 require both the automatic
stage-boundary checks and operator visual confirmation.

## Failure Behavior

All dedicated failures use one explicit contract from `grasp/tuna_errors.py`:

```text
TunaGraspError(
    code: TunaErrorCode,
    stage: str,
    diagnostics: dict,
    retryable: bool = False,
)
```

`TunaErrorCode` includes at least `CONFIGURATION`, `CALIBRATION_SHA`,
`DEPENDENCY_TRANSIENT`, `PIVOT_INVALIDATED`, `RETENTION_APERTURE_DRIFT`,
`BOUNDS`, `PLANNING`, `COLLISION`, `CONTACT`, `MOTION`, and `RETENTION`.
Diagnostics must be finite/serializable and include the exact Tuna name, plan
step, calibration SHA set, latest measured pose/qpos, relevant gate residual,
and finalization attempt/commit metadata when applicable. The executor never
interprets arbitrary message text to decide retryability.

The first transient finalization failure is an internal, logged attempt result,
not an externally retryable exception. If the one permitted local retry also
fails, the raised `DEPENDENCY_TRANSIENT` error has `retryable=False`; operator
recovery requires a fresh-scene trial rather than another in-place attempt.

The Tuna branch fails before or during motion when any of these conditions is
met:

- selected name is not exactly Tuna at a Tuna-only entry point;
- live target bounds are missing, stale, inconsistent, non-finite, or not a
  flat tabletop Tuna;
- no radial direction provides the required clearance;
- the contact-support pose or any roll waypoint lacks continuous-branch IK;
- any sampled full robot state is invalid or violates the table-clearance
  floor;
- calibration inputs or the per-command `T_tcp_lower_pad_contact(q_i)` matrix
  table do not match the current URDF/mesh/mount hashes;
- the backend fails the persistent gripper-setpoint preflight;
- the preclamp result is rejected or inconsistent with calibrated contact;
- measured preclamp qpos cannot produce a valid frozen pivot or aperture drift
  invalidates the frozen pivot contract;
- finalization fresh-state evidence fails, a non-transient preparation error
  occurs, or the one permitted transient retry is exhausted;
- preclamp moves the live bounds beyond tolerance;
- post-segment bounds indicate no roll or excessive slip;
- any Cartesian segment fails to converge;
- predicted oblique support width or lower-finger access fails the final
  closing geometry gate;
- full close is rejected;
- stable post-close qpos cannot produce a retention contract or its aperture
  drifts beyond tolerance during test/normal lift;
- the 30 mm test lift motion fails or its fresh AABB displacement/size check
  does not prove that the can followed;
- any indexed normal-lift observation is missing/stale or shows loss of Tuna
  follow.

After contact has begun, an execution-time observation failure snapshots the
latest measured TCP pose, issues an arm hold at that same pose, preserves the
current gripper position target, and raises a non-retryable Tuna error for
operator recovery. It must not command a return to the preceding checkpoint,
continue moving, wait indefinitely, or automatically open and drop the object.

There is no fallback to the old Tuna library, a full-diameter pinch, an
off-center chord pinch, relaxed shared thresholds, or a normal full lift after
an incomplete stage.

## Code Boundaries

The implementation is expected to stay within these boundaries:

- `grasp/config.py`: add only `TUNA_*` constants exposed through
  `GRASP_TUNA_*` environment variables. The initial set covers radial direction
  count, the fixed contact-height/pitch grids, approach distance, preclamp
  position, roll segment angles, initial and preclamp bounds stability, general
  bounds tolerance, minimum table clearance, calibration error bound, contact
  deflection, 0.002-rad qpos stability, setpoint hysteresis, 0.5 mm maximum
  frozen-pivot aperture drift, 0.5 mm maximum retention-aperture drift,
  2.5-degree maximum roll micro-segment angle, 10 mm maximum lift micro-segment
  translation, 2 mm maximum lift-follow tolerance, straddle margin, 30 mm test
  lift, 50 mm maximum lift-observation spacing, and the Tuna debug-stop stage.
  Shared defaults remain unchanged. Runtime validation enforces
  `0 < TUNA_LIFT_OBSERVATION_SPACING_M <= 0.050` and does not permit an
  environment override to enlarge the fixed 5 mm general bounds envelope, the
  2 mm lift-follow maximum, either 0.5 mm aperture-drift maximum, the
  0.002-rad qpos stability maximum, the 2.5-degree roll micro-segment maximum,
  or the 10 mm lift micro-segment maximum.
- `grasp/tuna_roll_grasp.py`: new isolated pure-geometry module for frame
  construction, contact/pivot geometry, pregrasp, roll waypoints, AABB
  predictions, oblique support width, and calibration-table interpolation.
  `grasp_selector.py` remains unchanged by this repair.
- `grasp/tuna_errors.py`: new isolated error enum and structured
  `TunaGraspError` contract. Non-Tuna exceptions and retry semantics remain
  unchanged.
- `grasp/tuna_finalizer.py`: new exact-Tuna-only post-preclamp preparation
  coordinator. It lazily owns the required MoveIt clients, validates the current
  frozen-pivot state, classifies dependency outcomes as transient or semantic,
  and returns an immutable prepared suffix; it never mutates a plan or sends a
  robot command. Non-Tuna execution never constructs it or its clients.
- `src/my_course_pkg/tools/calibrate_tuna_gripper_geometry.py`: offline URDF/STL
  calibration with a hashed machine-readable output, including
  the sampled `T_tcp_lower_pad_contact(q_i)` matrix table and conservative
  support hulls used by Tuna tests and measured-aperture pivot freezing. It
  requires explicit filesystem arguments for the expanded URDF, every named
  collision mesh, and mount-transform input; it does not use
  `ament_index_python`, `ROS_PACKAGE_PATH`, package discovery, or the current
  working directory to find them.
- `setup.py`: package only the generated Tuna calibration JSON as additive data;
  do not change console entry points or dependencies.
- `package.xml`: add only the `moveit_msgs` dependency needed by Tuna-lazy IK
  and state-validity clients.
- `grasp/pick_place_planner.py`: perform the exact-name short circuit before
  generic profile/library selection, load validated live Tuna bounds, rank
  radial sides, run sampled IK/state validity, and return only dedicated plans.
- `grasp/trajectory_planner.py`: add a separate Tuna plan builder for
  the preclamp prefix plus typed deferred suffix, pure prepared-suffix
  construction, and the single-use atomic commit for indexed roll/observation
  segments, full close, test lift, indexed normal-lift/observation segments, and
  the continuous transfer suffix. It owns no ROS client and performs no retry.
  Existing plan builders are unchanged.
- `grasp/executor.py`: add the single Tuna debug-stage selector, persistent
  setpoint preflight, Tuna-only separately dispatched roll/lift micro-segments,
  qpos/aperture gates, stage-boundary AABB validation, bounded finalization
  retry around `tuna_finalizer`, and fail-closed diagnostics. Do not add a
  parallel arm-plus-gripper primitive or alter non-Tuna `move` execution.
- focused Tuna geometry, planner, trajectory, executor, calibration, and
  non-Tuna isolation tests, plus a separately marked real-MuJoCo Tuna physical
  integration test that cannot select a hardware backend.

The existing MuJoCo gripper target persistence and live
`/scene_clearance_bounds` publisher are consumed as-is. Simulator source,
generic selector source, perception models, VLM/SAM selection, global scene
construction, shared object profiles, Pudding, Pear, Tomato, gripper effort,
and non-Tuna execution semantics are out of scope.

## Verification

### Automated Isolation Tests

1. Assert exact `tuna_fish_can` routing short-circuits before generic pose
   canonicalization, profile/offset lookup, selector, library loading, obstacle
   mutation, or place initialization and enters the dedicated plan builder.
2. Assert every other known object, especially `tomato_soup_can`, produces the
   same candidate matrices, step order, gripper targets, and debug behavior as
   before the patch for fixed inputs.
3. Assert Tuna never loads or falls back to the 5,002-pose library.
4. Assert invalid/missing target bounds, table-height disagreement, or
   three-sample initial instability fail before motion.
5. Assert the installed double-90-degree mount and finger origins establish TCP
   X as the sign-symmetric closing axis and TCP Z as approach.
6. Assert `pregrasp = contact - distance * tool_Z` moves outward and upward.
7. Assert four-direction default and eight-direction opt-in generate the
   expected radial axes without affecting ranking semantics.
8. Assert every discrete 8/9/10-mm contact-height and 35/40/45-degree pitch
   candidate derives TCP from the calibrated lower-pad frame and that the
   support-hull clearance lower bound rejects any unsafe open/preclamp/roll/close
   sample before the independent full-chain state-validity check.
9. Assert three distinct post-preclamp qpos samples pass only when their
   peak-to-peak range is at most 0.002 rad, select the hash-matched interpolated
   contact matrix, and freeze one `T_world_pivot`.
10. Assert every finalization attempt consumes two newer stable bounds samples
    and three newer stable qpos samples. Only timeout/unavailable-without-answer
    permits one logged retry after 500 ms; semantic failure never retries, and
    retry reuses rather than re-anchors the frozen pivot.
11. Assert the initial plan has no executable nominal-pivot roll pose and the
    immutable suffix remains invisible until one atomic commit. Both failed
    attempts, repeated commit, or first-roll dispatch failure leave no path to
    re-finalize or execute later motion.
12. Assert every visible roll segment is split into at-most-2.5-degree blocking
    micro-segments and aperture is checked before the first and after every
    micro-segment. Any greater-than-0.5-mm drift prevents the next arm command,
    preserves the pivot origin, and never re-interpolates or re-anchors it.
13. Assert the predicted AABB heights at 10/20/30 degrees and the slip/no-roll
    classifications from fresh marker samples.
14. Assert oblique support width includes both axial and radial terms and that
    initial-straddle, lower-finger-sweep, and contact-limited-close gates fail
    independently with distinct diagnostics.
15. Assert three stable post-close qpos samples freeze a separate retention
    contract. Test and normal lift use at-most-10-mm micro-segments and check
    aperture before/after each; they never compare against preclamp aperture.
16. Assert aperture drift raises `RETENTION_APERTURE_DRIFT`, while stable
    aperture plus failed AABB follow raises `RETENTION`, and both prevent the
    next lift command.
17. Assert the finalized Tuna plan contains preclamp observation, indexed
    roll/observe segments, full close and observation, 30 mm test lift and
    observation, and indexed at-most-50-mm normal-lift/observe segments in order.
18. Assert the test-lift and normal-lift poses share the rolled orientation and
    are translated from `rolled_close_pose`, not the flat contact pose.
19. Assert every Tuna debug-stop value stops after exactly its named stage and
    conflicts with legacy flags fail before motion.
20. Assert `observe_tuna_bounds` rejects a non-Tuna plan, missing/stale samples,
    wrong target identity, preclamp displacement, no-roll, slip, failed 30 mm
    follow motion, or lost follow at any normal-lift checkpoint.
21. Assert an observation failure after contact holds the latest measured arm
    pose, preserves the gripper target, raises a non-retryable error, and never
    continues or automatically opens.
22. Assert every dedicated failure is a structured `TunaGraspError` with a
    known `TunaErrorCode`, stage, finite serializable diagnostics, and explicit
    retryability; executor branching never parses exception message text.
23. Assert any failed preclamp/finalization/roll/close/test-lift/normal-lift or
    observation prevents later steps.

### Offline Model Checks

1. Run the calibration script with explicit absolute arguments for the expanded
   URDF, every named contributing gripper collision mesh, and mount-transform
   input. Require the documented command/gap values, per-sample TCP support
   hulls, no-more-than-0.25-mm interpolation error, full
   `T_tcp_lower_pad_contact(q_i)` matrices, and all source/schema hashes. Repeat
   a fixture invocation with ROS discovery variables removed and a different
   current directory; output must be identical.
2. Command representative simulator positions and compare finger transforms to
   the calibration output.
3. At a full-state-valid calibration pose with at least 50 mm table clearance,
   verify the persistent preclamp target across exactly `+10 mm world X ->
   return` and `+10 mm world Z -> return` arm-only motions without replaying
   the gripper command. Convert qpos through the calibration table and require
   pad-gap-equivalent hysteresis no greater than
   `TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M = 0.0005 m`.
4. Compute the lower contact point relative to TCP at the chosen preclamp.
5. Replay the full `{8,9,10} mm x {35,40,45} degree x 4-direction` candidate
   grid against the table and current scene; repeat with eight directions before
   enabling that opt-in mode. Require the clearance lower-bound equation at
   every open/preclamp/roll/allowed-close sample.
6. Sample every roll waypoint and verify full-chain clearance, state validity,
   and continuous IK.
7. For every enabled radial direction and each 10/20/30-degree checkpoint, run
   ten fresh-reset simulator trials with relative yaw exactly
   `[0,45,90,135,180,225,270,315,0,180]` degrees. Record per-cell and aggregate
   center/height residual mean, standard deviation, and maximum; require every
   cell to fit the fixed 5 mm envelope.
8. Verify predicted AABB center/height envelopes and no-roll/slide diagnostics.
9. Verify initial straddle clearance, swept lower-finger access, and
   contact-limited close separately rather than treating free-space full-close
   gap as object width.
10. Confirm the direct Tuna library remains physically blocked and is not used.

### Physical MuJoCo Integration Gate

This is an implementation-start gate for user contact testing, not an optional
post-hoc demo. Add a separately marked integration test that runs the actual
MuJoCo dynamics, the production ROS topics/actions, the installed robot model,
and the exact Tuna object. Mocks, fake backends, analytical object motion, and
manually edited AABB samples cannot satisfy this gate. The test must assert the
active backend is MuJoCo and refuse to run against hardware. It uses a dedicated
isolated Tuna scene, performs no transfer/place suffix, records the random seed,
and is started explicitly by the user after unit/offline qualification.

Run five nominal fresh-scene trials: seed `101` three consecutive times, then
seeds `202` and `303` once each. Each trial performs its own successful reset
and post-reset state/perception regeneration. Any failure, skip, interruption,
or invalid trial during the three seed-101 runs resets that consecutive streak
to zero. Across those runs the physical integration test must:

1. with no object between the pads, command the gripper from 0.00 to 0.20 rad
   and prove measured pad separation decreases, the two pad displacements are
   sign-symmetric along TCP X within 1 degree, and transverse displacement is
   below 0.5 mm;
2. create physical top/side preclamp contact, collect quantitatively stable
   measured qpos samples, freeze the interpolated pivot contract, exercise the
   fresh-bounds finalization precondition, and rerun the measured-aperture
   support-hull clearance check before roll;
3. execute only the positive 10/20/30-degree roll-in segments with real contact
   dynamics, an aperture check after every at-most-2.5-degree micro-segment, and
   fresh `/scene_clearance_bounds` evidence;
4. perform the three close gates, freeze the separate post-close retention
   contract, complete the 30 mm test lift and normal lift using at-most-10-mm
   aperture-checked micro-segments, and require object follow within configured
   `TUNA_LIFT_FOLLOW_TOLERANCE_M <= 0.002 m`; stop before transfer/place;
5. inject bounds dropout, contact/slip rejection, greater-than-0.5-mm pivot
   aperture drift, and greater-than-0.5-mm retention-aperture drift in separate
   runs and prove each causes hold-at-latest-measured-TCP, gripper-target
   preservation, the correct structured non-retryable Tuna error, and no later
   motion or pivot re-anchoring.

All five nominal runs, including the uninterrupted three-run seed-101 streak,
and all failure injections must pass before the first user-started live contact
stage. A skipped, xfailed, mocked, wrong-backend, or partially completed
integration test does not open the gate.

### Fresh-Scene Trial Definition

A `fresh-scene` Tuna trial is one independent trial satisfying all of the
following conditions:

- `/reset_sim` completed successfully after the preceding trial, and the trial
  log records a unique trial ID plus the reset-completion monotonic timestamp;
- the executor and Tuna observation caches were recreated or explicitly
  cleared, no prior frozen-pivot contract remains, the robot is at the required
  initialization pose, the gripper is open, and no action is active;
- selected-object identity, pose inputs, and three stable exact-name bounds
  samples were all regenerated after reset completion; no cached perception
  artifact or pre-reset message is accepted;
- no robot/object contact occurred after reset and before the trial's first
  planned contact, and the object is in its undisturbed flat tabletop state.

The deterministic scene layout may be identical between trials; `fresh` means
independent reset, state/cache invalidation, and post-reset perception rather
than randomized inventory. A failed or interrupted trial consumes its trial ID
and cannot be retried without another successful reset.

### Live Staged Checks

Run the eight debug stages in order. Do not combine first-time preclamp, roll,
full close, and lift into one unattended trial. Capture pose, gripper, and
stage-result diagnostics for every run.

After the physical MuJoCo integration gate and staged path succeed, complete
three consecutive Tuna trials satisfying the fresh-scene definition above for
30 mm test lifts with automatic AABB follow checks, followed by three
consecutive normal grasp-and-lift trials, before declaring the object stable.

## Acceptance

- Tuna uses only its exact-name roll-up branch.
- No Tuna motion requires a finger below the can while it is flat on the table.
- Every accepted contact height/pitch/radial candidate proves the conservative
  whole-gripper support-hull clearance lower bound across open, preclamp, roll,
  and allowed close samples. The bound maintains the exact Tuna table clearance
  selected inside the provisional 4-6 mm window after subtracting at most
  0.25 mm calibration interpolation error.
- The preclamp does not visibly slide or eject the can.
- Contact evidence exceeds the calibrated 0.5 mm minimum gap difference.
- The preclamp target persists without command replay within the 0.5 mm
  pad-gap-equivalent hysteresis tolerance, and live bounds remain inside the
  preclamp displacement tolerance.
- Three distinct measured preclamp qpos samples stay within the 0.002-rad
  peak-to-peak stability limit and select the hash-matched contact matrix. The
  resulting world pivot is frozen before roll, finalization rechecks two newer
  bounds plus three newer qpos samples, and at most one observable transient
  retry precedes a single atomic suffix commit.
- Measured-aperture support-hull clearance passes, every roll micro-segment is
  at most 2.5 degrees, and its post-command aperture check stays within the
  at-most-0.5-mm pivot drift limit without re-anchoring.
- The roll reaches approximately 25-35 degrees while fresh live AABB height
  and center remain inside the predicted pivot envelope for every cell in the
  explicit radial-direction/yaw/checkpoint qualification matrix.
- Full close independently passes initial-straddle, lower-finger-sweep, and
  contact-limited-close gates; no axial-thickness-only or free-space-full-close
  gap assumption remains.
- Three stable post-close qpos samples freeze a separate retention aperture.
  Every test/normal-lift micro-segment is at most 10 mm and stays within the
  at-most-0.5-mm retention drift limit; the preclamp aperture is never used as
  the post-close lift reference.
- Fresh AABB samples prove the can follows the 30 mm test lift within the
  at-most-2-mm Tuna lift-follow tolerance in three consecutive trials satisfying
  the explicit fresh-scene definition.
- Every at-most-50-mm normal-lift checkpoint proves Tuna follow, and the
  complete lift succeeds within the same 2 mm follow tolerance in three
  consecutive trials satisfying the explicit fresh-scene definition.
- The non-mocked physical MuJoCo integration gate passes three consecutive
  fresh-scene full-lift trials at seed 101, one at seed 202, one at seed 303,
  and all specified failure injections before live contact.
- All non-Tuna fixed-input regression tests are unchanged, with explicit
  coverage for `tomato_soup_can`, `pudding_box`, and `pear` routing.
- No shared threshold or non-Tuna executor behavior changes are included.
