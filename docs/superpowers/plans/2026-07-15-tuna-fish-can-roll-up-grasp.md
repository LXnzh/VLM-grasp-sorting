# Tuna Fish Can Roll-Up Grasp Implementation Plan

**Goal:** Implement the approved exact-name `tuna_fish_can` roll-up grasp while
proving that every non-Tuna candidate, threshold, plan, gripper command, debug
flag, and execution path remains unchanged.

**Design:**
`docs/superpowers/specs/2026-07-15-tuna-fish-can-roll-up-grasp-design.md`

## Implementation Rules

1. Only normalized exact name `tuna_fish_can` may construct or execute this
   policy. There is no shared `roll_up` profile and no fallback to the old Tuna
   library.
2. Work test-first. Each task starts with a focused failing test, adds the
   smallest implementation, and reruns the focused and non-Tuna isolation
   suites.
3. Do not modify `grasp_selector.py`, simulator source, shared profile maps,
   shared thresholds, global gripper effort, or non-Tuna plan semantics.
4. The current worktree contains unrelated changes in several overlapping
   files. Never stash, reset, overwrite, or broadly stage them. Record the
   pre-task diff and stage only exact Tuna hunks.
5. Do not begin runtime edits until `.git` is writable, the design revision is
   committed, and the index is confirmed free of unrelated staged files.
6. No simulator or robot motion is launched automatically. The user starts the
   physical MuJoCo integration gate after offline qualification and starts every
   live stage only after that gate passes.

## Task 1: Freeze The Dirty-Worktree And Non-Tuna Baseline

**Files:**

- Add: `src/my_course_pkg/test/test_tuna_non_target_isolation.py`
- Inspect only: all currently modified grasp and simulator files

1. Record branch, dirty paths, staged paths, and SHA-256 for every file that the
   Tuna implementation will touch. Stop if `.git/index.lock` exists or `.git`
   is not writable.
2. In a subprocess with all `GRASP_TUNA_*` variables removed, construct
   parametrized semantic fixtures for every non-Tuna object's normalized name,
   category/profile, candidate matrices, candidate order, step names/actions,
   gripper targets, debug behavior, and created service/subscription clients.
3. Include explicit fixtures for `tomato_soup_can`, `pudding_box`, and `pear`,
   plus every other configured object. `tomato_soup_can` is a routing negative
   control, not a claim of Tuna-equivalent mechanics.
4. Compare exact strings and sequence order, and use
   `numpy.testing.assert_allclose(rtol=0, atol=1e-12)` for numeric
   matrices/targets. Do not
   serialize private object dictionaries or exact floating-point text to a
   checked-in JSON snapshot.
5. Add spies proving every non-Tuna fixture constructs no Tuna geometry,
   client, subscription, error, or plan component.
6. Run the existing selector, planner, trajectory, and executor tests. Record
   failures already present before Tuna work; do not relabel them as Tuna
   regressions.

## Task 2: Add And Validate Tuna-Only Configuration

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/config.py`
- Modify: `src/my_course_pkg/test/test_tuna_non_target_isolation.py`
- Add: `src/my_course_pkg/test/test_tuna_config.py`

1. Add failing tests for every `TUNA_*` constant and corresponding
   `GRASP_TUNA_*` environment variable from the design.
2. Cover radial direction count, the exact `{8,9,10} mm` contact-height and
   `{35,40,45} degree` pitch grids, approach distance, initial/preclamp bounds
   stability, exact table clearance, 0.25 mm maximum calibration interpolation
   error, preclamp position, contact deflection, 0.002-rad qpos stability,
   setpoint hysteresis, 0.5 mm maximum pivot-aperture drift, separate 0.5 mm
   maximum retention-aperture drift, 2.5-degree roll micro-segments, 10 mm lift
   micro-segments, 2 mm maximum lift-follow tolerance, roll checkpoints, general
   bounds tolerance, straddle margin, test lift, lift-observation spacing, and
   debug stop stage.
3. Enforce `TUNA_RADIAL_DIRECTION_COUNT in {4, 8}`,
   `0 < TUNA_LIFT_OBSERVATION_SPACING_M <= 0.050`, the fixed 5 mm maximum bounds
   envelope, 2 mm lift-follow maximum, both 0.5 mm aperture-drift maxima,
   0.002-rad qpos-stability maximum, 2.5-degree roll-microsegment maximum, 10 mm
   lift-microsegment maximum, and strict enum validation. Environment variables
   may tighten but never loosen those safety limits.
4. Add the constants without changing any existing constant or object mapping.
5. Rerun the complete non-Tuna baseline test after the configuration import.

## Task 3: Generate And Package The Hashed Gripper Calibration Artifact

**Files:**

- Add: `src/my_course_pkg/tools/calibrate_tuna_gripper_geometry.py`
- Add: `src/my_course_pkg/my_course_pkg/grasp/data/tuna_gripper_geometry.json`
- Modify: `src/my_course_pkg/setup.py`
- Add: `src/my_course_pkg/test/test_tuna_gripper_calibration.py`

1. Write failing fixture tests for deterministic URDF joint traversal, every
   contributing gripper collision mesh, command-to-gap interpolation,
   per-command conservative TCP support hulls, and the full
   `T_tcp_lower_pad_contact(q_i)` homogeneous matrix at every command sample.
2. Require explicit filesystem CLI arguments such as `--urdf <absolute-path>`,
   repeated `--collision-mesh <link>=<absolute-path>`, and
   `--mount-transforms <absolute-path>`. Resolve and validate every path before
   reading; do not use `ament_index_python`, `ROS_PACKAGE_PATH`, package
   discovery, or current-working-directory-relative discovery.
3. Add fixture tests that invoke the tool with fixed known paths, ROS discovery
   variables removed, and two different current directories; require identical
   output. Implement deterministic URDF FK and STL vertex loading without a
   live ROS graph or MuJoCo process.
4. Emit schema version plus SHA-256 for the expanded URDF, all collision meshes,
   mount transforms, and canonical JSON content. Adaptively subdivide command
   intervals until support-hull interpolation error is at most 0.25 mm. Reject
   non-finite, non-rigid, non-monotonic, under-bounded, or incomplete output.
5. Reproduce the documented representative gaps, including approximately
   85.517 mm at `0.00 rad` and 1.819 mm at `0.79 rad`, within the test tolerance.
6. Package the JSON as additive package data and add an installed-module test
   that locates it without relying on the source-tree path.
7. Add a loader that fails closed on any schema or current-source SHA mismatch;
   do not silently regenerate calibration at runtime.

## Task 4: Implement Pure Tuna Geometry

**Files:**

- Add: `src/my_course_pkg/my_course_pkg/grasp/tuna_roll_grasp.py`
- Add: `src/my_course_pkg/my_course_pkg/grasp/tuna_errors.py`
- Add: `src/my_course_pkg/test/test_tuna_roll_grasp.py`
- Add: `src/my_course_pkg/test/test_tuna_errors.py`

1. Add pure tests for repository-specific TCP axes, four/eight radial
   directions, the unambiguous 35-45-degree angle to world horizontal, and
   sign-symmetric TCP-X closing geometry.
2. Implement typed immutable inputs/results for live bounds, table height,
   contact-support frame, calibrated support hull/error bound, frozen pivot
   contract, post-close retention contract, roll/lift micro-waypoint, expected
   AABB, finalization attempt/commit metadata, and gate result.
3. Define `TunaErrorCode` plus structured `TunaGraspError(code, stage,
   diagnostics, retryable=False)`. Reject non-finite/non-serializable
   diagnostics and test that control flow never parses error message text.
   Include distinct `DEPENDENCY_TRANSIENT`, `PIVOT_INVALIDATED`,
   `RETENTION_APERTURE_DRIFT`, and `RETENTION` codes.
4. Test and implement
   `pregrasp = contact - TUNA_APPROACH_DIST_M * tool_Z`; assert the result moves
   outward and upward for every allowed radial direction.
5. Implement the shared three-sample qpos validator: strictly increasing
   timestamps newer than the command boundary and peak-to-peak calibration-
   driving qpos no greater than 0.002 rad. Test stale, duplicate, non-finite,
   unstable, and boundary-equal samples.
6. Interpolate `T_tcp_lower_pad_contact(q_measured)` only from the verified
   artifact after that validator passes, freeze both that matrix and
   `T_world_pivot`, and rerun measured-aperture support-hull clearance. Generate
   only positive roll-in transforms split at no more than 2.5 degrees and prove
   the pivot origin is invariant at every sampled micro-waypoint.
7. Test that pivot aperture drift greater than 0.5 mm invalidates the frozen
   contract and that no API can re-interpolate or re-anchor it after roll begins.
8. Freeze a separate post-close retention contract from stable qpos. Test that
   test/normal-lift micro-waypoints are no more than 10 mm, compare only with
   post-close gap, and distinguish gripper drift from stable-gap AABB loss.
9. Implement the ideal-cylinder AABB height cross-check and predicted center
   trajectory. Keep pivot-dependent center prediction separate from the
   pivot-independent height extent.
10. Implement radial/tangential/vertical residual classification for preclamp
   displacement, slide-without-roll, excessive slip, and successful roll.
11. For every `{8,9,10} mm x {35,40,45} degree x radial-direction` candidate,
   derive TCP from the calibrated lower-pad frames and conservatively sample
   every qpos bounding the accepted contact band before motion. Compute
   `min(transformed support-hull world Z) - table Z - interpolation error` over
   the open, preclamp, provisional roll, and allowed-close samples. Reject any
   value below the exact Tuna table-clearance threshold; repeat with the actual
   measured qpos during finalization.
12. Implement `required_width(delta)` and three separate results for initial
   straddle, swept lower-finger access, and contact-limited close. Never compare
   object width with the free-space full-close gap.

## Task 5: Build The Isolated Tuna Planner Branch And Safety Gates

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
- Modify: `src/my_course_pkg/package.xml`
- Modify: `src/my_course_pkg/test/test_pick_place_planner.py`
- Modify: `src/my_course_pkg/test/test_tuna_non_target_isolation.py`
- Modify: `src/my_course_pkg/test/test_tuna_roll_grasp.py`

1. Add spy-based failing tests proving exact Tuna does not call generic pose
   canonicalization, profile/offset lookup, grasp selection/library loading,
   generic obstacle mutation, or place initialization.
2. Put the exact-name decision immediately after selected-name normalization and
   raw pose loading. Keep the existing function body byte-for-byte equivalent
   on the non-Tuna branch wherever practical.
3. Add Tuna-only loading of three fresh exact-name bounds samples. Reject
   duplicates, stale stamps, wrong frame/name, non-finite values, non-flat size,
   table inconsistency, or standard deviation beyond the 2 mm initial limit.
4. Generate the complete discrete height/pitch grid for all four/eight radial
   directions. Reject a candidate unless its support-hull clearance chain passes
   before ranking by remaining hard clearance and reachability; try the next
   ranked candidate without preferring a fixed world roll direction.
5. Assert Tuna never opens `/home/ws/grasps/007_tuna_fish_can` and non-Tuna
   planner call order remains unchanged.
6. Add mocked MoveIt service tests for continuous previous-state-seeded IK and
   full state validity at pregrasp, contact, every roll sample, close, test lift,
   and every normal-lift checkpoint.
7. Add only the required `moveit_msgs` dependency. Keep clients Tuna-lazy so a
   non-Tuna request creates no new service client or wait.
8. Validate all arm, wrist, flange, camera/adapter, gripper base, knuckle, and
   finger collision links. Allow Tuna contact only at the two named pad regions;
   do not apply a broad ACM relaxation.
9. Keep the explicit support-hull/table lower-bound check as a prerequisite to,
   not a replacement for, MoveIt full-state validity. Cross-check planning-scene
   table Z with the validated flat Tuna bottom before either check.
10. Reject branch jumps, missing services, timeouts, non-finite joints,
   self/table/non-target collision, and any sampled lower-finger mesh sweep that
   leaves the raised-bottom region.
11. Rerun non-Tuna planner tests to prove no new service dependency exists on
   their path.

## Task 6: Build The Dedicated Tuna Plan

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/trajectory_planner.py`
- Add: `src/my_course_pkg/my_course_pkg/grasp/tuna_finalizer.py`
- Modify: `src/my_course_pkg/test/test_trajectory_planner.py`
- Add: `src/my_course_pkg/test/test_tuna_finalizer.py`

1. Add failing tests that the initial plan contains executable open, pregrasp,
   contact-support, preclamp, and preclamp-observation steps followed only by a
   typed non-executable deferred suffix descriptor. Assert no nominal-pivot roll
   pose can execute.
2. Add an exact-Tuna-only finalizer that lazily owns MoveIt clients, accepts the
   already-frozen pivot plus current validated bounds/joint state, classifies
   dependency outcomes, and returns an immutable prepared suffix without plan
   mutation or robot commands. Non-Tuna execution never constructs it or waits
   for its services.
3. Keep suffix construction and atomic commit pure in `trajectory_planner`.
   Prepare the three indexed roll/observe pairs, close/observe, 30 mm test lift/
   observe, and indexed normal-lift/observe pairs in an immutable local value.
4. Test exactly one 500 ms executor retry only for timeout/service-unavailable
   without a semantic response or robot command. Log both attempts. Explicit
   no-IK, invalid state, collision, malformed data, bounds/qpos change, or side
   effect never retries. Only full success atomically records one suffix
   generation; exhausted, failed, repeated, or post-commit finalization cannot
   expose or replace a suffix.
5. Use ordinary `action="move"` for motion and Tuna-only
   `action="observe_tuna_bounds"` for blocking observations. Store expected
   values under the same step name in
   `PickPlacePlan.debug_info["tuna_observations"]`; store validated roll/lift
   micro-waypoint lists in Tuna-only debug metadata and do not add shared
   `PlanStep` fields.
6. Split each roll stage into at-most-2.5-degree micro-waypoints. Compute lift
   observation segments at most 50 mm and further split them into at-most-10-mm
   micro-waypoints. Retain rolled orientation and derive all heights from
   `rolled_close_pose`.
7. Add tests for configurable final roll angle: `roll_segment_3` means all roll
   segments complete, not exactly 30 degrees.
8. Prove every existing non-Tuna plan has identical steps and numeric targets.

## Task 7: Execute Tuna Observations And Debug Stops Fail-Closed

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_grasp_executor_gripper_failures.py`
- Add: `src/my_course_pkg/test/test_tuna_executor.py`

1. Add failing tests that `observe_tuna_bounds` is rejected for non-Tuna plans
   and that no non-Tuna execution creates a Tuna bounds subscription, finalizer
   retry, retention contract, or micro-waypoint dispatcher. Existing non-Tuna
   `move` steps must still issue exactly the same backend calls in the same order.
2. Lazily subscribe to the existing `/scene_clearance_bounds` stream on the
   first exact-Tuna observation. Require two new stable exact-name samples after
   each command boundary.
3. Implement the single `GRASP_TUNA_DEBUG_STOP_AFTER` enum, including
   `normal_lift`, and reject any simultaneous legacy `GRASP_DEBUG_STOP_*` flag
   before motion.
4. Implement preclamp contact evidence with
   `measured_contact_gap - calibrated_free_space_gap >= 0.5 mm` and the expected
   action result. Reject uncontrolled preclamp AABB motion beyond 2 mm. Document
   that contact deflection and later aperture drift have different references
   even though their first-patch maxima are both 0.5 mm.
5. Collect three distinct qpos samples within the 0.002-rad peak-to-peak limit,
   interpolate and freeze the hash-verified pivot, and rerun measured-aperture
   clearance. Before each finalization attempt require two newer stable bounds
   and three newer stable qpos samples. Implement the one observable 500 ms
   transient-only retry and atomic commit; no nominal or partial suffix executes.
6. Dispatch every Tuna roll micro-waypoint as a separate blocking arm command.
   Check pivot aperture before the first and after every at-most-2.5-degree
   micro-segment; drift prevents dispatch of the next command, raises
   `PIVOT_INVALIDATED`, holds latest measured TCP, preserves target, and never
   re-interpolates or re-anchors.
7. Implement the three close gates and fresh no-displacement bounds. Then
   collect three stable post-close qpos samples and freeze a separate retention
   contract. Do not add a Tuna effort override or reuse preclamp gap for lift.
8. Dispatch test/normal-lift micro-waypoints as separate blocking commands no
   longer than 10 mm. Check retention aperture before/after every micro-segment
   and at every AABB checkpoint. Aperture drift raises
   `RETENTION_APERTURE_DRIFT`; stable aperture plus failed at-most-2-mm AABB
   follow raises `RETENTION`. Both hold, preserve full-close target, and prevent
   the next move without auto-open or rollback.
9. Add publication-blackout, unstable-qpos, finalization freshness/retry/commit,
   per-microsegment pivot drift, post-close retention drift, stable-gap AABB
   loss, and error-code tests proving no later command executes after failure.

## Task 8: Verify Persistent Setpoint Hysteresis

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: `src/my_course_pkg/test/test_tuna_executor.py`

1. Add a fake-backend regression that commands the calibrated preclamp without
   an object at a full-state-valid pose with at least 50 mm table clearance,
   performs exactly `+10 mm world X -> return` and `+10 mm world Z -> return`,
   and never replays the gripper command. No downward, inward, rotational, or
   larger-amplitude motion belongs to this preflight.
2. Convert each measured qpos through the calibration table and require maximum
   pad-gap-equivalent hysteresis no greater than
   `TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M = 0.0005 m`.
3. Reject stale joint state, non-monotonic calibration, target drift, or a
   backend that requires command replay.
4. Keep this preflight Tuna-only and cache success only for the exact backend
   identity plus calibration SHA set; any SHA change invalidates it.

## Task 9: Wire The Exact Tuna Integration And Prove Isolation

**Files:**

- Modify: `src/my_course_pkg/my_course_pkg/grasp/pick_place_planner.py`
- Modify: `src/my_course_pkg/my_course_pkg/grasp/trajectory_planner.py`
- Modify: `src/my_course_pkg/my_course_pkg/grasp/executor.py`
- Modify: focused tests above

1. Add an end-to-end fake-ROS test from exact selected name through dedicated
   candidate, plan, preclamp, roll observations, close, test lift, normal lift,
   and suffix handoff.
2. Add failure injection at every physical/action boundary, including unstable
   qpos, finalization fresh-state/retry/commit, roll-microsegment pivot drift,
   post-close retention freeze, and lift-microsegment retention drift. Prove the
   structured error code is correct and no later arm command or step runs.
3. Rerun the frozen baseline for all non-Tuna objects and compare candidate
   matrices, step order, gripper targets, debug behavior, and service creation.
4. Run a source audit proving Tuna identifiers are absent from shared selector
   mappings and simulator source.
5. Stage only exact Tuna hunks and review `git diff --cached --check` plus the
   complete staged file list before each commit.

## Task 10: Run Offline Qualification

**Files:**

- Modify only if a failing qualification exposes a design violation
- Update: `docs/agent_handoff.md`

1. Run all focused Tuna tests, then the complete grasp test set.
2. Run Python compilation and fatal flake8 checks on every touched Python file.
3. Build `my_course_pkg` with `--symlink-install`; verify the installed module
   loads the packaged calibration JSON and matches source SHA-256.
4. Run the calibration tool using explicit absolute URDF, named collision-mesh,
   and mount-transform arguments. Repeat from a different current directory
   with ROS package-discovery variables removed; require identical output,
   support-hull interpolation error at most 0.25 mm, and matching sampled
   simulator finger transforms without an object.
5. Run the persistent setpoint `+10 mm world X -> return` and `+10 mm world Z
   -> return` preflight at the specified safe pose and require the 0.5 mm
   pad-gap-equivalent limit.
6. Exhaust the `{8,9,10} mm x {35,40,45} degree` grid for every enabled radial
   direction and require the clearance lower-bound equation at every
   open/preclamp/roll/allowed-close sample.
7. For every enabled radial direction and 10/20/30-degree checkpoint, run ten
   fresh resets at relative yaw exactly
   `[0,45,90,135,180,225,270,315,0,180]` degrees. Require every per-cell and
   aggregate center/height residual within the fixed 5 mm envelope; do not widen
   the envelope to pass.
8. Verify the initial plan has no executable nominal-pivot suffix; finalization
   requires fresh bounds/qpos, permits only one logged transient retry, and
   exposes exactly one atomic commit. Verify roll/lift microsegment aperture
   gates and the separate post-close retention reference.
9. Stop before the physical integration gate if any calibration, hash,
   isolation, IK, collision, bounds, pivot, or hysteresis gate fails.

## Task 11: Pass The Real MuJoCo Physical Integration Gate

**Files:**

- Add: `src/my_course_pkg/test/integration/test_tuna_mujoco_roll_up.py`
- Modify only if required to register a dedicated integration-test marker
- Update: `docs/agent_handoff.md`

1. Add a separately marked test that asserts the active backend is MuJoCo,
   refuses hardware, uses the production ROS topics/actions and installed model,
   and rejects mocks, fake bounds, or analytical object motion. The user starts
   the dedicated isolated Tuna scene; the test never runs transfer/place.
2. With no object between the pads, physically command 0.00 to 0.20 rad and
   prove pad separation decreases, both pads move sign-symmetrically along TCP X
   within 1 degree, and transverse displacement is below 0.5 mm.
3. Run seed `101` through three consecutive fresh-scene nominal trials; any
   failure, skip, interruption, or invalid trial resets its streak. Then run one
   fresh nominal trial each for seeds `202` and `303`. Every trial completes
   quantitative qpos/pivot freeze, fresh-state finalization, at-most-2.5-degree
   aperture-checked positive roll-in, the three close gates, post-close
   retention freeze, and aperture-checked test plus normal lift within the
   at-most-2-mm AABB tolerance; stop before transfer/place.
4. In separate physical runs inject bounds dropout, contact/slip rejection,
   greater-than-0.5-mm pivot aperture drift, and greater-than-0.5-mm retention-
   aperture drift. Require hold at latest measured TCP, preserved gripper
   target, expected structured non-retryable error, no later arm command, and no
   pivot re-anchoring.
5. Record backend identity, seed, trial/reset IDs, calibration SHA set, measured
   qpos/pad gaps, TCP/pad transforms, AABB evidence, error codes, and test result.
   A skip, xfail, wrong backend, mock, or partial completion is a gate failure.
6. Do not begin user-started live contact stages until all five nominal runs,
   including the uninterrupted three-run seed-101 streak, and every failure-
   injection run pass.

## Task 12: Run User-Started Live Stages

1. Before every trial, require a successful `/reset_sim` after the preceding
   trial; log a unique trial ID and reset-completion monotonic timestamp, clear
   executor/observation/pivot caches, return the robot to initialization with
   gripper open and no active action, and regenerate selection, pose inputs, and
   three stable exact-name bounds samples after reset. A failed/interrupted trial
   cannot resume without another reset. Identical deterministic layouts count
   as fresh; stale state or perception does not.
2. `generation`: inspect exact routing, ranked sides, TCP axes, the calibration
   contact-matrix table, conservative nominal envelope, mesh sweep, and expected
   AABB envelopes. No motion and no executable nominal-pivot suffix.
3. `pregrasp`: verify full-chain clearance and stop.
4. `contact_support`: verify both pads and log the complete coupling chain:
   chosen contact height/pitch/radial direction, table Z cross-check, support-
   hull minimum world Z, interpolation error, and final clearance lower bound.
5. `preclamp`: verify contact evidence, less than 2 mm AABB displacement, the
   three-sample 0.002-rad qpos stability, frozen pivot, measured-aperture
   clearance, two newer pre-finalization bounds samples, and successful atomic
   suffix commit. Any transient retry must be visibly logged.
6. `roll_segment_1`, `_2`, `_3`: run as separate fresh trials and require each
   positive roll-in observation, an aperture check after every at-most-2.5-
   degree micro-segment, and operator confirmation. There is no commanded roll-
   out recovery.
7. `close`: verify all three gates, post-close no-displacement bounds, and the
   separate stable-qpos retention-contract freeze.
8. `test_lift`: require an aperture check after every at-most-10-mm
   micro-segment and 30 mm AABB follow within the at-most-2-mm tolerance in three
   consecutive fresh-scene trials.
9. `normal_lift`: require every at-most-50-mm checkpoint within the same 2 mm
   tolerance, aperture checks after every at-most-10-mm micro-segment, and
   complete lift in three consecutive fresh-scene trials.
10. Only after all stages pass may `none` enable the existing transfer/place
   suffix. Reset the scene after every failed or interrupted contact trial.

## Final Acceptance And Stop Conditions

- All design acceptance criteria and non-Tuna baseline comparisons pass.
- The packaged calibration artifact, runtime URDF/meshes/mount transforms, and
  per-command contact-matrix/support-hull SHA-256 values agree.
- The real, non-mocked MuJoCo integration gate passes the three consecutive
  seed-101 nominal runs, seed-202/303 runs, and all failure injections before
  live contact.
- No shared threshold, selector mapping, simulator source, or non-Tuna executor
  behavior changed.
- Stop and return to design review rather than improvising if the 5 mm roll
  envelope, 2 mm lift-follow tolerance, 0.002-rad stability limit, any 0.5 mm
  setpoint/pivot/retention-aperture limit, measured fixed-world pivot, atomic
  finalization contract, roll/lift microsegment gate, real-MuJoCo gate, initial
  straddle, contact-height/table-clearance chain, lower-finger sweep, explicit
  yaw matrix, or contact-limited retention gate cannot be satisfied.
