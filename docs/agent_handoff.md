# Agent Handoff Log

This file is the persistent handoff note for future Codex windows. Read it at
the start of a new session before inspecting code or rerunning experiments.

## Current Snapshot

- Release checkpoint (2026-07-19): `grasp_stable` now treats the 16 remaining
  objects as the supported stable-grasp scope. By explicit user decision,
  `tuna_fish_can` and `pudding_box` are excluded from release success criteria
  and no longer block the branch. Tuna modules remain packaged only because the
  shared planner/executor modules import the fail-closed Tuna route; removing
  them would break imports for supported objects. Preparation commits are
  `d4886e4` (generated-artifact ignores), `242e62d` (grasp/session/simulator
  implementation and tests), and `c3e7a66` (validated VLM default). No-motion
  validation passed `632/632`, all 43 changed Python files passed compilation
  and fatal-flake8 checks, and isolated `my_course_pkg` plus
  `ifl_air_ur_launch` builds completed. No ROS graph, simulator, or robot motion
  was started. Next: use the pushed branch as the baseline for supported-object
  operation; do not reopen Tuna/Pudding work unless scope changes explicitly.

- Current VLM recovery: session `20260717_131535_756039`, trial 4, failed
  before SAM2 or motion because removed default `azure.gpt-4.1-mini` returned
  HTTP 400 `Model not found`. Although `GET /models` marked low-cost vision
  model `google.gemini-2.5-flash-lite` active, its real image request failed at
  the Google Vertex EU backend with HTTP 404 (publisher model unavailable in
  the configured project/region). The reviewed fallback `azure.gpt-5-mini`
  accepted the unchanged image request, including `temperature=0`, and returned
  `{"candidates": ["apple"]}` for the captured RGB image. It is now the
  unoverridden default in both host workspaces; the active container build path
  resolves to the updated source. Compilation and focused configuration/alias
  tests pass `12/12`; the successful direct check invoked no SAM2 or motion.
  Next: press Enter in the existing waiting supervisor to start a fresh trial.

- Current Apple regression diagnosis: session
  `20260716_220714_755218/trial_001` correctly resolved and segmented `apple`,
  FoundationPose succeeded, and no motion command ran, but grasp selection
  failed before MoveIt. The detected Apple object-local +Z axis was nearly
  world-down (`tilt_to_world_up_deg=174.85`); Apple is not in
  `TABLETOP_CANONICAL_OBJECTS`, so the canonical pose remained equally
  inverted. Offline replay of the real 5,013-pose Apple library gives
  `5013 raw -> 40104 symmetry -> 32 orientation -> 32 center -> 0 height`:
  all 32 remaining object-local normalized heights are `[-0.067,-0.038]`
  versus the shared round-top gate `[0.0,0.20]`. Holding translation/yaw fixed
  and making object-local Z upright yields 11,056 candidates through height,
  width, and table clearance, with `118.6-132.2 mm` finger clearance. Thus this
  is not width, table, MoveIt, VLM, SAM, or container-code drift; build/source
  selector hashes are identical and the build file resolves to source. The
  pear-only metrics were not invoked, but Apple and Pear share the generic
  `round_top` orientation/center/height/table branch. Existing non-Pear routing
  tests use synthetic upright identity poses and missed FoundationPose's valid
  180-degree orientation ambiguity for a visually symmetric Apple. Next:
  design an Apple/round-top orientation-ambiguity repair and a real-library
  inverted-pose regression without relaxing shared height or clearance gates;
  do not merely rerun until FoundationPose happens to choose the opposite sign.

- Current Docker recovery: there are now two live Dev Containers for the same
  workspace. `determined_germain` still contains the repaired simulator venv,
  but the active terminal/session is in newly created `interesting_lovelace`
  (created 2026-07-16 21:43 UTC). Its container-local `/home/ros2/.venvs`
  directory is absent, so the workspace `.venv` symlink is dangling. Session
  `20260716_214457_137086/trial_001` therefore stopped before MuJoCo started;
  `launch.log` records `.venv/bin/activate: No such file or directory`, and the
  later `/reset_sim` timeout is only the consequence of the missing simulator
  service. The active container's system `/usr/bin/python3` also still lacks
  `openai` and `pycocotools`. VS Code Dev Containers 0.466.0 logs prove this
  was not an intentional rebuild: reopen found `determined_germain` by its
  workspace-folder label and logged `Container already running`, but its exact
  lookup used a different config-file identity
  (`/mnt/wsl/docker-desktop-bind-mounts/.../devcontainer.json` instead of the
  old `/mnt/e/.../devcontainer.json`). The CLI then missed the old container
  and issued a new `docker run` for `interesting_lovelace`. Both containers use
  host networking/PID/IPC, so never run ROS in both simultaneously. Next: quit
  the failed supervisor, explicitly attach to the intended running container
  by container identity or keep only one container, then either reuse the
  repaired old environment or recreate the simulator venv inside
  `interesting_lovelace` and install its requirements,
  install `openai` and `pycocotools` for the `ros2` user's system Python, verify
  both import paths, and only then relaunch. A venv under `/home/ros2` does not
  survive creation of another Dev Container.

- Branch: `grasp_stable`
- Current Tuna goal: solve only exact selected name `tuna_fish_can`; all other
  object routes and shared thresholds remain unchanged. The approved design
  (`02e57de`) and plan (`6fe4c8d`) are now implemented through the offline
  qualification boundary in uncommitted Tuna-only runtime/test changes. The
  branch short-circuits before generic profile/library selection, uses a
  source-SHA-verified per-qpos contact matrix and real collision-mesh vertex
  cloud, freezes measured pivot/retention apertures, finalizes once with one
  observable 500 ms transient-only retry, and checks aperture after every
  at-most-2.5-degree roll and at-most-10-mm lift microcommand. The five final
  review observations are closed: post-reset VLM/SAM artifact freshness is
  required for physical stages, actual command dispatch is capped at 96,
  gripper-target/hold transitions are explicit, SHA fields have canonical
  source names, and post-close hold is exactly 0.5 s. Focused Tuna/trajectory
  tests pass `270/270`; the broader grasp set passes `381` with the same `26`
  known incomplete-`PoseStamped` test-stub failures. Compilation, fatal flake8,
  isolated `colcon` build, installed calibration loading, and deterministic
  calibration reproduction pass. Offline qualification triggers the approved
  hard stop: all `36/36` `{4 radial x 3 height x 3 pitch}` candidates fail the
  exact 5 mm whole-gripper table-clearance gate. The best 10-mm/45-degree cell
  reaches only `4.093146 mm` at contact and `2.022594 mm` across the full roll
  sweep; canonical calibration SHA is
  `e331f4e96dc92178d5eaf6b7d981119de1cba1da4c4e7ed01a5f894c1ad59dd5`.
  No ROS graph, simulator, or robot motion ran, and the MuJoCo integration gate
  must not start. Next: return only the Tuna contact-height/pitch/tool-envelope
  design to review; do not lower the 5 mm gate or touch any non-Tuna path.
- Current pear goal: stabilize only pear's bilateral contact and lift without
  changing any already-stable object path. The original failed grasp was
  top-down but its TCP-X
  closing axis was about 69 degrees from pear's short axis and required about
  97.4 mm mesh-projected width, beyond the 85.16 mm effective opening. Offline
  replay of all 7,466 raw poses plus existing yaw expansion found two safe
  library candidates (raw index 5675 at 0/180 degrees): 4.858-degree short-axis
  error, 74.81 mm conservative AABB width, and 10.35 mm opening margin. The
  approved exact-name `pear` design adds hard direction and candidate-specific
  width gates only; shared profile thresholds, planner, executor, perception,
  and gripper behavior remain unchanged. Design `90e802f` and plan `533300c`
  are committed. Implementation is synchronized to both dirty workspaces but
  remains uncommitted pending staged live validation. Only normalized name
  `pear` calls `_pear_short_axis_candidate_metrics`; it requires at most 5
  degrees short-axis error and at least 5 mm margin inside the existing 85.16
  mm opening. All other round-top objects retain the old scalar-minimum width
  branch, and routing tests cover every configured non-pear round-top name.
  The real 7,466-pose library test returns only index 5675 at yaw 0/180 with
  4.858-degree error, 74.81 mm projected width, and 10.35 mm margin. Container
  selector/planner tests pass `117/117`; the related grasp regression passes
  `303/303`; compilation, fatal flake8, three-file host/container hash equality,
  the `my_course_pkg` symlink build, and installed constants/helper import pass.
  Live trial 1 in session `20260715_181545_780230` confirms the direction fix:
  runtime retained exactly two candidates with the expected 4.858-degree error
  and 10.35 mm opening margin, scene clearance kept both, and the screenshot
  shows the open fingers straddling the pear's long sides. Candidate 1 then
  plateaued before close at `31.6 mm` total final-pose error: actual minus
  target was about `(+8.4, -6.3, +29.8) mm`, so XY error was about `10.5 mm`
  and the dominant residual was upward Z. Target was
  `[-0.2131,-0.9427,0.9028] m`; actual was
  `[-0.2047,-0.9490,0.9326] m`. No close command ran. This validates candidate
  direction but not grasp completion; the plateau and image suggest the pear
  final target is too deep for the tilted 16.16-degree pose. The user approved
  the exact-name pear-only depth correction documented by design `35c62b0` and
  plan `8c77fcb`: `GRASP_PEAR_Z_OFFSET=+0.010` replaces the shared `-0.020`
  offset only for normalized name `pear`, raising pear final/pregrasp world Z
  by exactly 30 mm. Planner, executor, global 18 mm tolerance, and all other
  object branches remain unchanged. Host/container files are synchronized;
  selector/planner tests pass `139/139`, the related grasp regression passes
  `325/325`, and compilation, fatal flake8, symlink build, and installed import
  pass. Real-library replay still retains only the same two candidates with
  4.858-degree error and 10.35 mm opening margin, while each prepared final Z
  moves by exactly +30 mm. Runtime/test changes remain uncommitted pending live
  validation. The next staged check was to restart the experiment launch and
  run `pick up the pear` with `GRASP_DEBUG_STOP_AT_GRASP=1`, require
  `z_offset=0.0100 m`, and visually verify final height before close-only and
  full-lift stages. Fresh session `20260715_191438_216925/trial_001` did not
  reach grasp planning: both VLM calls succeeded (`pear`, then mask candidate 1
  with high confidence), SAM2 outputs were fresh, but FoundationPose endpoint
  `172.22.222.220:5001` accepted TCP and returned no HTTP bytes; a separate
  five-second curl also timed out and no new `pose_result.json` was produced.
  The same service hang occurred in session `20260715_184623_816429`. This is
  not VLM API quota exhaustion. Restart/repair the external FoundationPose
  service before another pear trial; the client otherwise waits 300 seconds per
  attempt for up to three attempts. No motion ran in this trial. Session
  `20260716_183037_957297` then reached pear close: trial 1 used candidate 1 at
  `16.14 deg` from world-down, closed only from `0.000` to `0.038 rad` before a
  stalled contact was accepted, and completed the 200 mm lift-return while the
  user observed unstable wobble. Trial 3 reused the same candidate geometry at
  `16.10 deg` but stalled immediately at `0.000 rad`, so no lift ran. The image
  shows the fingers straddling the correct long sides but at unequal heights.
  Direct selected-matrix replay gives the two candidates closing-axis tilts of
  about `14.5/16.3 deg`, corresponding to about `21.4/23.9 mm` finger-height
  difference over the 85.16 mm opening; printed Euler angles are not a reliable
  way to recover this value because the runtime conversion convention differs.
  The current pear-only filter constrains only horizontal closing direction and
  projected width, not TCP-X horizontality. The user confirmed the high left
  finger catches only the pear's upper shoulder, causing the observed lift
  wobble. Full replay finds no existing library candidate satisfying the current
  direction/width gates plus even a 10-degree closing-axis leveling limit, so a
  filter alone would leave zero candidates. The approved pear-only level and
  bilateral-contact repair is documented by design `78d7a20`, plan `8e9dafe`,
  and 5 mm contact-depth revision `fce1b83`; runtime/test changes remain
  uncommitted. It synthesizes exact world-down poses from the two safe seeds,
  makes TCP-X the horizontal pear short axis, centers pear geometry in TCP XY,
  limits seed correction to 20 degrees and theoretical finger-height difference
  to 2 mm, and lowers only `GRASP_PEAR_Z_OFFSET` from +10 mm to +5 mm. It also
  derives an expected close position from final projected width and requires
  pear contact no more than 0.010 rad below it before lift; non-pear close
  behavior is unchanged. Live trial 11 proves the new geometry loaded:
  `finger_height_delta_m=0`, `center_offset_m=0`, final approach angle `0 deg`,
  width `66.61 mm`, margin `18.55 mm`, final Z `0.9291 m`, and the screenshot
  shows level fingers centered on the pear. The close reached `0.207 rad`, above
  expected/minimum `0.172/0.162 rad`, but an active-workspace compatibility bug
  initially passed `None` into the new validator because the older executor did
  not return `execute_step`'s gripper result. Returning the result fixes it;
  build-tree/source hashes point to the same symlinked file. Active-workspace
  selector/planner/executor tests pass `209/209`; including trajectory passes
  `307/307`; compilation, fatal flake8, symlink build, and installed defaults/
  helpers pass. Main-workspace isolated pear tests pass `46/46` and fatal
  flake8 passes; its broader isolated run passes 279 and shows the same 26 known
  incomplete-`PoseStamped` stub failures plus two unrelated fixture/baseline
  failures. Next: rerun pear in the existing supervisor; require close readiness
  to log actual about 0.207 above minimum 0.162 and visually inspect the lift.
  Do not change global effort or modify tuna, pudding, or stable object paths.
- Current gelatin-box/sponge diagnosis: session trials 3-5 for gelatin and
  trial 6 for sponge all stopped
  before `close_gripper_at_grasp`; neither is evidence of a weak close. VLM,
  mask, FoundationPose, top-down orientation, live-bounds centering, and the
  planned target were repeatable. Trial 3 reread the same cached intermediate
  pose three times about 0.6 seconds after the calibration command, called the
  zero-spread window stationary, then rejected a premature second correction
  above the 30 mm cap. Trial 4 correctly waited for motion, passed pregrasp and
  waypoints 1-18, then held at waypoint 19/20 with only about 1.6 mm world-XY
  error and 0.2 mm Z error versus the strict 1.5/3.0 mm gate. The screenshot is
  this safe hold: fingers appear to straddle the centered box, final waypoint
  and close were never sent. Trial 5 repeated the gelatin failure at waypoint
  20/21 with about 1.5 mm XY and 0.3 mm Z error. Sponge repeated it at waypoint
  18/20 with 1.6 mm XY and 0.2 mm Z error; its residual was again almost
  entirely world +Y and close was never sent. This proves a shared low-position
  vertical-tracking boundary, not two object-specific grasp failures. The
  gelatin mesh footprint is about 89.4 x 101.1 mm while
  the nominal Robotiq maximum opening used by the selector is 85.16 mm, so the
  user's earlier successes remain marginal and need an after-close bounded
  check before any full lift-return. Sponge's 79.1 mm short side leaves only
  about 3.0 mm per finger when aligned. The approved general boundary change is
  now implemented and synchronized to both dirty workspaces: strict pregrasp
  calibration remains at 1.5 mm, while lower-waypoint prechecks, post-command
  descent gates, and final close-readiness verification use the new validated
  `GRASP_VERTICAL_DESCENT_XY_TOLERANCE_M=0.0020` default. Z remains 3.0 mm;
  timeout, hold, and fail-closed behavior are unchanged. Logs and exceptions
  receive the stage tolerance explicitly. Focused tests pass `141/141`, related
  functional regression passes `321/321`, Python compilation, fatal flake8,
  file synchronization, the `my_course_pkg` symlink build, and installed
  defaults `0.0015/0.0020/0.0030` pass. Design `edd4c0e` and plan `3416ac6`
  are committed; implementation remains uncommitted. Next: restart with
  `GRASP_DEBUG_STOP_AFTER_CLOSE=1` and run separate fresh gelatin/sponge trials;
  do not increase close effort or proceed directly to full lift-return. The
  cached-feedback freshness issue from trial 3 remains a separate follow-up.
  Gelatin trial 2 in session `20260715_155845_030451` then proved the new XY
  split loaded and worked: pregrasp logged 1.5 mm and waypoints 1-20 passed with
  the 2.0 mm descent gate. The final waypoint alone stopped in Z: target
  `0.8980 m`, stable actual `0.9026 m`, XY error 1.3 mm, upward Z residual
  4.6 mm versus the symmetric 3.0 mm gate. Bounds were bottom `0.8900 m`,
  center `0.9053 m`, top about `0.9206 m`; the held TCP is inside the object's
  height and only 2.7 mm below its center, while the screenshot shows both
  fingers beside the box. Close was again never sent. The approved design is a
  bounded asymmetric final-only high-side Z fallback: keep ordinary Z at
  symmetric 3.0 mm, permit positive residuals up to 5.0 mm only at the final
  waypoint/close-readiness gate, require target and actual Z inside valid live
  bounds, and never relax the downward or intermediate-waypoint boundary. The
  final-waypoint fallback is evaluated only after the existing settle window
  and sends no additional lower command. This is now implemented and synced to
  both dirty workspaces with validated
  `GRASP_VERTICAL_FINAL_HIGH_Z_TOLERANCE_M=0.0050`. One live-bounds context is
  passed to the final waypoint and close-readiness verification; missing,
  invalid, non-vertical, out-of-bounds, downward-beyond-3-mm, and
  upward-beyond-5-mm cases retain hold-and-raise behavior. Diagnostics report
  signed Z, asymmetric limits, bounds, and acceptance mode. Focused config and
  executor tests pass `164/164`; related grasp regression passes `274/274`;
  compilation, fatal flake8, four-file host/container SHA synchronization,
  the `my_course_pkg` symlink build, and installed defaults
  `0.0015/0.0020/0.0030/0.0050` pass. Design `694b7b4` and plan `f1be782` are
  committed; implementation remains uncommitted. No simulator or robot motion
  ran automatically. Fresh gelatin trial 1 in session
  `20260715_164719_249422` proves this runtime loaded the new policy and valid
  bounds, but settled at actual Z `0.9032 m` versus target `0.8980 m`: a
  `+5.2 mm` residual, only `0.2 mm` above the configured cap. XY was `0.6 mm`,
  bounds were `0.8900/0.9053/0.9205 m`, and actual Z remained `2.1 mm` below
  the object center; the screenshot again shows the fingers straddling the
  box. The policy correctly rejected `high-side residual exceeds final
  tolerance`, held, and never sent close. No `GraspDebug`/after-close log is
  present, so the pasted run also does not prove the close-only debug flag was
  enabled. The user selected a live-center final boundary with no independent
  fixed positive-Z cap. Ordinary final success remains symmetric at 3.0 mm and
  bounds-independent; only a settled positive residual beyond 3.0 mm may use
  the fallback, requiring valid bounds, target inside the object, actual Z
  between bottom and center, and final XY within 2.0 mm. Stable ordinary-path
  `sponge`, `foam_brick`, and `rubiks_cube` behavior must remain unchanged.
  Design `85a6ae9` and plan `774a060` are committed and supersede the fixed
  5 mm design/plan. The replacement is implemented and synchronized to both
  dirty workspaces: the fixed high-Z environment setting is removed, ordinary
  final success remains bounds-independent at symmetric 3 mm, and only the
  settled final positive fallback uses the live center. Diagnostics report
  `controlled_center` and the per-run `center_z - target_z` limit. The latest
  gelatin values (`+5.2 mm` with a `+7.3 mm` center limit) pass in the installed
  runtime. Focused tests pass `160/160`, the related grasp regression passes
  `270/270`, Python compilation, fatal flake8, four-file host/container hash
  equality, the `my_course_pkg` symlink build, and installed defaults
  `0.0015/0.0020/0.0030` with no fixed high-Z constant all pass. Runtime/test
  implementation remains uncommitted pending live validation. No simulator or
  robot motion ran automatically. Next: restart the experiment launch, set
  `GRASP_DEBUG_STOP_AFTER_CLOSE=1`, run a fresh gelatin trial, and require
  `controlled_center`, successful close, and immediate after-close debug stop;
  do not proceed directly to lift-return.
- Current restart fix: Trial 3 of session
  `/tmp/my_course_experiment_sessions/20260715_124007_751132` proved that a
  stopped interface can leave a transient stale DDS action endpoint. The graph
  gate now starts a 30-second window on the first duplicate, keeps polling, and
  requires two consecutive complete/unique snapshots before passing. Missing,
  failed-query, or duplicate samples reset the clean streak; a duplicate still
  present at its deadline fails before reset or motion. The 180-second overall
  readiness deadline remains unchanged. Focused session tests pass `23/23`,
  core session/launch/grasp regression passes `73/73`, related suites pass
  `57/57` and `57/57`, compilation, fatal flake8, the symlink package build,
  installed-module discovery, and synchronization between both workspaces
  pass. The currently running supervisor loaded the old module. Next: press
  `q`, relaunch `experiment_session.launch.py`, complete one trial, and confirm
  the following restart either logs a bounded duplicate settle then passes or
  fails only after 30 seconds of persistent duplication.
- Current tomato-selection diagnosis: trial
  `/tmp/my_course_experiment_sessions/20260715_124007_751132/trial_001`
  received `pick up the tomato` correctly, but the generic VLM returned
  `{"candidates":["apple"]}`. The saved RGB frame visibly contains both a red
  apple that resembles a fresh tomato and the selected tomato-soup can.
  `tomato` has no deterministic target alias, the VLM receives all 18 YCB
  names rather than the six active scene identities, and the only downstream
  target check accepts any valid YCB name. FoundationPose and `grasp_demo`
  therefore ran for apple; the grasp log ends `Pick and place completed.`
  Next: design a scene-aware deterministic instruction resolver plus a
  fail-closed instruction/VLM identity-consistency gate before FoundationPose.
- Current pudding-box goal: solve only exact normalized name `pudding_box`
  without changing any other object's selector, planner, executor, gripper
  defaults, or thresholds. The box is about 138.6 x 129.4 x 39.6 mm while the
  effective gripper opening is about 85.16 mm. All 6,526 library poses close
  approximately vertically across the 39.6 mm thickness, which requires a
  finger below a tabletop object; the latest generic live-bounds centering also
  erased the required edge offset before MoveIt rejected the colliding
  pregrasp. The user approved a Pudding-only staged roll-up design: approach
  from planning -X with about +20 mm Y contact offset, lightly preclamp the near
  bottom-side corner, roll toward +X about the far +X bottom table edge in
  validated microsteps, close across the newly exposed thickness, and perform
  a 30 mm test lift before any full lift. Fixed-scene analysis leaves about
  94 mm conservative reserve after a worst-case 35-degree roll. The fixed
  rotation axis is the far bottom table edge; the lower finger is a moving
  support contact, not a world-fixed pivot. Initial design `8435570`, first
  review revision `4e85326`, and final approved review closure `8620d23` are in
  `docs/superpowers/specs/2026-07-15-pudding-box-roll-up-grasp-design.md`.
  The final design fixes the base-link/planning-frame bounds-axis derivation,
  restricts phase one to planning -X approach/+X roll with no side fallback,
  adds state-specific full gripper envelopes, a 0-3 mm support-gap gate,
  5 mm preclamp stability and model-residual gates, empirical tilt-height
  sampling, bounded reverse recovery with verified flat support before opening,
  an instance-private locked scene monitor, a non-Pudding shared-config
  baseline, and runtime table height derived from initial bounds. The 15-task
  initial implementation plan is committed as `db7bddb` in
  `docs/superpowers/plans/2026-07-15-pudding-box-roll-up-grasp.md`. Integrated
  spec/plan review closure `03e3560` separates controlled rollback into the
  `rollback_from_20` test action, distinguishes flat/preclamp baselines, records
  all seven 0–30-degree checkpoints, strengthens structured non-Pudding
  regression, and separates L1 offline tests from L2 online validation. It also
  preserves the verified planning `-X`/612.6 mm frame mapping and current
  fixed-layout maximum of six candidates. The plan defaults Pudding to
  `validate_only`; calibrated URDF envelope gates A/B and fixed-direction
  MoveIt hard gate C precede contact motion. Next: start Task 1 by snapshotting
  the current dirty worktree's non-Pudding configuration and behavior before
  adding Pudding constants. Do not run simulator/robot motion automatically;
  live stages require explicit user launch and observation.
- Current integration goal: replace the manual multi-terminal workflow with a
  persistent experiment session. The approved design/plan are in
  `docs/superpowers/specs/2026-07-15-persistent-experiment-session-design.md`
  and `docs/superpowers/plans/2026-07-15-persistent-experiment-session.md`.
  Implementation is synchronized to both dirty workspaces but uncommitted. Run
  `ros2 launch ifl_air_ur_launch experiment_session.launch.py`: startup reuses
  an exported API key or prompts once with hidden input, lists all 18 objects,
  accepts up to six, keeps MuJoCo open, and loops owned-interface restart ->
  reset -> fresh instruction/pipeline -> automatic `grasp_demo`. Failures stop
  with per-stage logs under `/tmp/my_course_experiment_sessions`; Enter retries
  the full safe boundary and `q` shuts down the complete launch. The integrated
  launch now redirects persistent MoveIt/MuJoCo/gripper/robot-state and
  owned-interface output to logs so prompts remain usable. The experiment
  include now explicitly passes both `launch_robot_rviz=false` and
  `launch_moveit_rviz=false`, so the next restarted session opens only the
  MuJoCo visualization; standalone full-launch RViz defaults remain unchanged.
  Core tests pass `69/69`, related simulator/scene and
  perception/grasp regressions pass
  `57/57` and `57/57`, focused flake8, compilation, both package builds,
  installed executable discovery, and launch argument inspection pass. The
  workspace-wide linter remains unusable because it recursively scans existing
  `build/`/`install/` files and reports 1866 unrelated baseline issues. A
  non-motion live smoke test reached a stable visible `Instruction:` without
  continuous runtime output; no instruction, pipeline, grasp, or motion ran.
  The first real user instruction then exposed Humble's no-op `ProcessStdin`
  handler; direct stdin-transport writing is implemented, tested, rebuilt, and
  requires a fresh launch for live validation. Next: restart, verify one real
  instruction creates `pipeline.log`, then validate failure retry before one
  real grasp.
- Current racquetball diagnosis: a live `random` scene contained racquetball in
  slot 3 and tennis ball in slot 6. GroundingDINO returned three alleged
  racquetball masks, but they covered hammer, tennis ball, and banana; the blue
  racquetball was absent. The VLM verifier selected the tennis-ball mask, so
  FoundationPose applied the racquetball model at the tennis-ball location.
  Live TF and bounds confirm the planned final XY near
  `[-0.6540, -1.0503] m` matches tennis-ball world XY near
  `[-0.6527, -1.0509] m`, while the actual racquetball is near
  `[-0.2347, -0.5704] m`. The exact-bounds gate therefore correctly rejected
  all eight candidates against `tennis_ball` before motion. Do not disable the
  clearance gate or use `FOUNDATIONPOSE_MASK_INDEX` on this output because no
  candidate contains the target. The user approved a fixed user alias
  `blue racquetball`: it maps to canonical identity `racquetball`, while
  GroundingDINO/SAM2 receives `blue ball`; FoundationPose and grasp planning
  retain the racquetball CAD/name. Design `749c93b` and implementation
  `50522d7` are committed, and runtime/test files are synchronized to the
  Docker-mounted workspace. Related tests pass `131/131`, Python compilation
  and the `my_course_pkg` symlink build pass. On the saved failure frame,
  `blue ball.` recalled both tennis ball (score `0.738`) and the actual blue
  racquetball (score `0.711`). The Docker exec environment lacked a VLM API
  key, so automatic verification correctly failed closed instead of taking the
  higher-score tennis ball. A bounded manual candidate-2 perception check then
  used a `4738 px` blue-ball mask and successfully ran FoundationPose with the
  canonical racquetball mesh; no planning or motion ran. Next: from the user's
  API-key-equipped terminal, run a fresh pipeline with
  `pick up the blue racquetball` and require the automatic verifier to select
  the blue candidate before any plan-only or motion trial.
- Current goal: validate the three startup-selectable scene modes live. `mix`
  and `random` retain their implemented behavior. New `assign` lists all 18
  configured objects and accepts up to six comma-separated required names;
  spaces are optional, matching is case-insensitive, input order maps to the
  first slots, and unfilled slots are sampled without duplicates. A blank line
  makes all six random. Unknown, duplicate, over-limit, or empty-token input
  retries the whole line before children start, and the simulator revalidates
  the final request against the YAML pool. Runtime and tests are synchronized
  to the Docker-mounted workspace. Combined simulator/launch tests pass
  `113/113`, grasp-selector compatibility passes `26/26`, Python compilation,
  Hydra propagation, actual-YAML 18-name loading, pure selector smoke, and the
  launch-package build pass. Installed `--show-args` reports all three modes
  and default `prompt`. Design is committed as `57f8068`; implementation is
  uncommitted. Next: user-run one `assign` launch and confirm the logged slot
  order, then complete the pending live `mix/random` checks. No simulator or
  robot motion was started automatically.
- Current banana diagnosis: the attached live run selected the correct object,
  reached pregrasp, and then failed before gripper close. The legacy `centered`
  profile chose one candidate whose raw TCP was 17.7 mm above the banana
  object origin, then the shared default `GRASP_Z_OFFSET=-0.020` lowered the
  final target to Z=0.8883 m, 2.3 mm below the estimated banana origin/bottom
  at Z=0.8906 m. Feedback stalled at Z=0.9043 m with residual
  `[-3.7,+7.5,+16.0] mm` (actual minus target), totaling just over the strict
  18 mm convergence gate. This is a physically over-low target, not an action
  server, MoveIt, perception, gripper-close, or lift failure. Do not loosen the
  18 mm gate: first validate a bounded banana close-only run without the
  legacy -20 mm lowering (raw target Z would be about 0.9083 m), then design a
  geometry/bounds-aware centered-profile height rule if confirmed. The user
  subsequently confirmed the bounded `GRASP_Z_OFFSET=0.0` run succeeded, then
  reported that a later plain `grasp_demo` also succeeded and chose not to
  change code. Treat the default behavior as marginal rather than proven
  deterministic: first check whether `GRASP_Z_OFFSET` is still exported in the
  shell. If it is unset, small perception, servo-state, feedback-timing, and
  contact differences around the prior rounded 18.0 mm gate can explain a
  pass/fail flip. No banana code change is approved.
- Current goal: a later foam-brick run reopened `vertical` approach reliability
  after the earlier user-confirmed six-object milestone. The approved generic
  feedback gate and per-run command-offset calibration are implemented in both
  workspaces. At safe pregrasp, it takes three feedback samples, uses component-
  median XYZ, accumulates at most three residual corrections with 0.4-second
  settling, caps total offset at 30 mm, and fails/holds on divergence or
  exhaustion. The accepted XYZ offset is frozen through feedback-confirmed
  5 mm descent commands, while actual motion is still compared to unmodified
  nominal waypoints at 1.5 mm X/Y and 3 mm Z. Orientation remains nominal and
  the offset exists only inside one `execute_plan` call. Focused tests pass
  `124/124`, related grasp regression passes `225/225`, and `my_course_pkg`
  compiles/builds with `--symlink-install`. Gate design is committed as
  `859a006`; calibration design is committed as `1045752`; implementation is
  synchronized to both dirty workspaces but uncommitted. The first live
  calibrated run learned `[-0.2,-8.9,+21.4] mm` in two commands but exposed
  premature median-only acceptance. The follow-up is now implemented: each
  window must have at most 0.75 mm pairwise XY spread and 1.5 mm Z spread, then
  both median and newest feedback must pass the unchanged physical gate. Moving
  windows wait without commanding for at most 1.5 s, and descent reuses the
  calibrated servo mode without a second state request. Stability design is
  committed as `0354c28`; implementation is synchronized but uncommitted.
  The first post-fix run proved the stability gate: it waited through a moving
  window, froze only after both median/latest passed, made no second mode
  request, and passed waypoint 1. Waypoint 2 then failed 44 ms after its command
  on a single 2.1 mm transient XY sample while Z was within tolerance. The user
  chose the minimal follow-up: post-command XY now receives the same existing
  1.5-second convergence period as Z. A waypoint passes only when one sample
  simultaneously satisfies the unchanged 1.5 mm XY and 3 mm Z gates; persistent
  error still times out and holds before any lower waypoint. Design `be6c8d3`
  is committed; implementation is synchronized but uncommitted. Related tests
  pass `227/227`, compilation and the symlink package build pass. The next live
  run confirmed the change: waypoints 1-15 passed, but waypoint 16 timed out
  only in Z with 0.3 mm XY error and 7.4 mm Z error. Feedback remained near
  Z=0.8987 m while the nominal final target was Z=0.8713 m; the screenshot shows
  both fingertips already at table height with the brick between them. The
  planned foam-brick grasp height is therefore about 25-27 mm too low. The user
  rejected an object-name-specific offset in favor of a general vertical rule.
  Approved design `0bd4fce` clamps every vertical final TCP to at least live
  world-AABB bottom plus 8 mm while preserving any higher library target; the
  recorded brick becomes Z=0.8980 m without a `foam_brick` branch. The rule is
  now implemented and synchronized to both dirty workspaces but uncommitted.
  Focused configuration/planner tests pass `167/167`, related regression passes
  `242/242`, Python compilation and the symlink package build pass, and all four
  runtime/test files match. The first live run verified that this floor clamp
  loaded: bounds bottom was 0.8900 m and minimum TCP Z was 0.8980 m. Fresh
  perception produced a higher valid library target Z=0.9393 m, so the clamp
  correctly preserved it. Descent waypoints 1-6 passed, but about 74 ms after
  waypoint 6 was accepted at the 1.5 mm XY boundary, feedback moved another
  1.4 mm in Y. The unchanged single-sample pre-command check for waypoint 7
  then saw 2.9 mm XY error and held before sending that waypoint. The approved
  bounded pre-command lateral wait is now implemented and synchronized to both
  workspaces. It samples at the existing 0.05-second period for at most the
  existing 1.5-second timeout, sends no lower command while XY is outside the
  unchanged 1.5 mm gate, sends exactly one waypoint command after recovery,
  and holds the latest pose on persistent drift. Executor tests pass `40/40`,
  related grasp regression passes `243/243`, Python compilation and the Docker-
  mounted symlink package build pass. Design `baf007e` is committed;
  implementation remains uncommitted. Next: user-run a bounded foam-brick
  close-only trial and confirm waypoint 7 either logs pre-command recovery
  before its command or safely times out without lowering; do not launch motion
  automatically.
- Current foam-brick diagnosis: the generic `vertical` bounds-centering change
  is active and worked as designed. The latest run shifted the requested TCP by
  `[-31.2,+0.4] mm` to exactly match the live bounds center. The physical
  failure is insufficient open-jaw clearance during descent. This pose spans
  the brick's roughly `80.4 mm` dimension with an 85 mm gripper, leaving only
  about `4.6 mm` total nominal gap, or `2.3 mm` per finger. The approach ended
  at `27.1 mm` 3D TCP error; the post-descent convergence command then accepted
  `12.6 mm` under the global `18 mm` tolerance. That correction happens after
  the fingers have already descended beside the object, so one finger can land
  on the top surface and push/rotate the brick before close. Close then stalled
  at `0.038 rad`, which maps to about `81.0 mm` opening and is consistent with
  the resulting near-limit contact. The current default close minimum is only
  `0.010`, so it is accepted. Post-run bounds kept the brick at tabletop height
  near its original slot, consistent with push/rotation rather than a reliable
  lift.
  Offline inspection found `384` library candidates within `10 degrees` of
  top-down, but all close approximately along the long local axis; the library
  has no ready short-side candidate. The strict pregrasp reanchor and
  feedback-gated 5 mm descent is now implemented and synchronized. A 90-degree
  short-side symmetry is optional extra robustness, not the primary explanation
  of this observed top-surface strike. Do not begin with a full return/release
  run; first use the bounded after-close validation, then bounded after-lift.
  The existing bounds-centering implementation remains synchronized in both
  dirty workspaces and its design is committed as `083714d`. The first live run
  of the new gate timed out at safe pregrasp with actual minus target near
  `[+0.1,+7.0,-16.1] mm`; it correctly issued a hold and never descended or
  closed. The next execution correction must compensate the command target for
  this measured bias at hover, then reuse that learned offset through descent.
- Current lemon diagnosis: the random slot-6 `lemon` produced 8 valid
  `round_top` candidates, but the exact-bounds approach filter rejected all 8
  before motion because their TCP-to-hammer-box XY distance was only
  `0.0841-0.0878 m` versus the required `0.1100 m`. Live bounds confirmed the
  hammer box at about `(-0.538, 0.246)`, size `0.183 x 0.333 m`, and lemon at
  about `(-0.361, 0.203)`. Even the `0.080 m` physical corridor radius leaves
  only about `5.5 mm` before the extra `0.030 m` margin, so this is a real
  slot-5/slot-6 clearance problem, not the old circumscribed-circle false
  rejection. The user approved changing only slot 6 from
  `[-0.36, 0.20, 0.0]` to `[-0.25, 0.20, 0.0]` plus its matching config test;
  preserve the validated hammer slot 5 and do not weaken or disable the
  clearance gate. Design spec is committed as `c590f32`. The config and its
  focused test were updated in both host workspaces, but live validation showed
  that `[-0.25, 0.20, 0.0]` projects the lemon center to pixel `(539,867)`,
  below the 720-pixel image. Grounding/SAM mislabeled banana and apple as lemon;
  the score fallback chose banana, so all eight alleged lemon candidates lay
  inside banana bounds and were correctly rejected. The user approved the
  revised far-table slot `[-0.85, 0.35, 0.0]`. It projects near `(742,110)` and
  has at least `0.143 m` exact-box distance to fixed obstacles. Revised design
  is committed as `67c1227`. The revised slot and matching test are now
  implemented in both workspaces; the Docker-mounted suite passes `7/7`,
  projection is `(741.5,109.9)`, and nearest-box distance is `0.1430 m`.
  Next: restart the full simulator launch, confirm live lemon bounds near the
  far slot, rerun fresh perception, visually verify the selected mask is lemon,
  then run `grasp_demo`.
- Current goal: replace the insufficient center-of-mass-only hammer pinch if
  strict parallel hold remains required. A full run proved the new candidate
  was selected (`1.1 mm` balance distance, `2.99 degrees` approach), closed,
  lifted, and held five seconds, but the user still observed large object tilt.
  The command omitted `GRASP_DEBUG_STOP_AFTER_LIFT=1`, so it then descended the
  long tilted hammer through an uncleared sweep, struck the banana, jammed, and
  failed both open attempts. Do not run full hammer return again. Next safe
  observation is a bounded after-lift stop; if tilt is still above `10 degrees`,
  design a mechanically constrained/caging grasp near the head-handle junction
  rather than further tuning the nominal mass-center score.
- The user has replaced remote placement as the default experiment goal with a
  global lift-return workflow: grasp, lift `0.20 m`, hold `5 s`, descend to the
  original grasp pose, release, and retreat vertically. The approved design is
  committed in the active workspace as `5cc2292`; implementation is committed
  there as `2859cf6` and synchronized here. `my_course_pkg` builds and all
  functional package tests pass. A clean plan-only tomato run with
  `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M=0.12` rejected four banana-intersecting
  candidates and accepted corridor-safe candidate 2 on its first MoveIt plan;
  no trajectory or gripper command executed and plan-only was restored false.
  The following real run safely avoided the banana, grasped, lifted `0.20 m`,
  held, and returned the can to the table, but aborted before release because
  `return_to_grasp` reused the initial final-approach convergence gate. Table
  contact left `0.0161 m` TCP error versus the `0.013 m` gate. The user chose
  the smallest teammate-friendly downstream fix: release `0.02 m` above the
  original grasp TCP instead of adding contact detection. Design `19edf18` is
  committed. The release-clearance setting and lift-return pose change are now
  implemented in both workspaces; the user confirmed the live tomato
  validation succeeded. The side-profile vertical margin now also defaults to
  the validated `0.12 m`, so normal trials no longer need any of the three
  runtime overrides. Related regression passes `128/128`, the
  package builds, and the installed planner reports a `0.020 m` return delta.
  Next: between formal trials, restart `moveit2_iface`, verify exactly one arm
  action server and one pose publisher, then reset, rerun fresh perception, and
  invoke `ros2 run my_course_pkg grasp_demo` without environment prefixes.
- Startup check: a new Codex window that loaded this file should say
  `Handoff loaded: <current goal>; next: <next recommended action>` in its
  first user-facing response.
- Scene setup: `tomato_soup_can`, `banana`, `apple`, `foam_brick`, and `hammer`
  are always included as the five grasp-category representatives; one remaining
  object is random; all six are assigned to fixed initial placement slots.
- Grasp focus: tomato-can grasp, lift, transfer, safe mid-table placement, and
  full release have succeeded live; the latest code now pauses actively at the
  drop pose before opening to reduce the can being thrown or tipped.
- Apple Phase A1 and A2 pass. The approved clearance-bounds and prior global
  three-run implementation is complete and synchronized from the active
  `-grasp-stable` workspace. Offline tests, build, live marker contracts, exact
  live AABB geometry, and AABB p95 performance pass. Single-camera 1280x720 at
  `render_fps: 20` published state at about `3.43 Hz`; this was accepted only
  for static A3 plan-only validation. A3 uses a functional gate: one fresh atomic six-
  object `/scene_clearance_bounds` message, finite positive bounds, fail-closed
  round-top filtering, and a `1.5 s` timeout. Project policy now requires two
  consecutive independent successes for every formal experiment; design
  `04fa14c` supersedes older three-run requirements. `18-20 Hz` remains a
  health gate before real arm motion. The
  dedicated non-executing `grasp_plan_only` entrypoint is implemented and
  committed as `c4cdac4`; 102 related tests, package build, and installed CLI
  discovery pass. The first live smoke stopped safely before any MoveIt goal
  because `camera_orbbec -> world` TF was absent, then restored
  `planonly=False`. Camera TF is now decoupled from disabled pointcloud in
  commit `4b4088e`; 67 simulator tests and live `tf2_echo` pass while the
  pointcloud topic remains absent. A post-fix non-qualifying smoke generated 8
  Apple candidates, but the current circumscribed-circle corridor rejected all
  because the banana center was about `0.175 m` away versus `0.215 m`
  required. Exact rectangle clearance is about `0.1207 m` versus a `0.1100 m`
  corridor plus margin, so this is a geometric false rejection. The user
  rejected moving Apple as a long-term workaround and approved the exact-OBB
  direction. Design `c3d3edb` is approved and implemented in both workspaces.
  The planner now preserves the published bounds box pose/half-extents, clips
  the approach to its expanded Z slab, and computes exact XY segment-to-
  rectangle distance without changing any safety threshold. Planner tests pass
  `56/56`, related grasp regression passes `124/124`, and the package builds.
  A non-qualifying live smoke kept all 8 Apple candidates, accepted MoveIt
  plan-only candidate 1, restored `planonly=False`, and produced no robot or
  object motion beyond micrometre-level simulator settling. Formal Apple Trial
  1 passed end to end: natural VLM candidate `apple`,
  mask area `7523 px`, successful FoundationPose, exact-bounds corridor kept
  all `8/8` candidates, MoveIt plan-only accepted candidate 1 on its first
  attempt, and cleanup restored `planonly=False`. No trajectory or gripper
  command executed. Formal Trial 2 also passed after an independently confirmed
  reset and fresh Apple pipeline: the exact-bounds corridor retained all `8/8`
  candidates, MoveIt plan-only accepted candidate 1 on its first attempt, and
  cleanup restored `planonly=False` with no trajectory or gripper command.
  Apple A3 is complete at `2/2` under the new project-wide policy. The generic
  evaluator now defaults to two trials and qualifies at two consecutive
  successes; focused tests pass `10/10`, related regression passes `124/124`,
  `my_course_pkg` builds, and runtime/test files are synchronized across both
  workspaces. The active config now uses `render_fps: 0.2`. Two independent
  live launches passed: Trial 1 state rates were `18.197/18.189/18.160 Hz` and
  Trial 2 rates were `18.301/18.292/18.282 Hz` for joint/scene/bounds. RGB-D
  delivered `7/7` and `8/8` matched 1280x720 frames; both bounds checks returned
  six finite positive markers with reliable/volatile QoS. Maximum observed
  state gaps were `1.262 s` and `1.988 s`, so average frequency alone is not
  sufficient evidence for real motion. The implemented automatic multi-mask
  policy has now passed two independent low-rate Apple A1 trials: Trial 1 used
  a valid verifier selection and Trial 2 exercised the highest-detection-score
  fallback. Both produced sub-millimetre live pose error and no motion, so A1
  is complete at `2/2`. Next: run two fresh-reset A3 plan-only trials. Do not
  run `grasp_demo` yet.

## Recent Work

### 2026-07-19 - Prepared Stable 16-Object Release

- Defined the release boundary as the 16 supported objects and removed Tuna and
  Pudding from success gating. Retained the dormant fail-closed Tuna modules to
  satisfy direct imports used by the shared supported-object path.
- Ignored generated Python/build/trial/graph artifacts and removed three cached
  `.pyc` files from version control without deleting local copies.
- Committed the grasp/session/simulator workflow and the validated
  `azure.gpt-5-mini` default separately for clear rollback history.
- Verification was non-actuating: `632/632` tests passed, 43 changed Python
  files compiled and passed fatal flake8, and both selected ROS packages built.

### 2026-07-16 - Implemented Tuna Through The Offline Hard Stop

- Added the exact-name planner/executor route, lazy source-hash calibration and
  MoveIt validation boundary, measured preclamp pivot, atomic suffix finalizer,
  per-microcommand pivot/retention monitoring, separate close gates, structured
  failures, command cap, post-reset perception provenance, and 0.5-second
  post-close hold. Non-Tuna semantic routing/matrix tests remain unchanged.
- Corrected the calibration support representation from the eight corners of a
  TCP-frame global AABB to the deterministically ordered actual collision-mesh
  vertices. Rotating global-AABB corner combinations had introduced phantom
  points tens of millimetres below any real mesh vertex; real vertices are exact
  for the linear world-Z support minimum and remain pointwise interpolable.
- Regenerated and source-verified the nine-sample artifact with observed maximum
  interpolation error `0.085234 mm` and canonical content SHA
  `e331f4e96dc92178d5eaf6b7d981119de1cba1da4c4e7ed01a5f894c1ad59dd5`.
- The corrected model still fails the required 5 mm gate physically: the best
  of 36 cells is 10 mm contact height/45-degree pitch, with `4.093146 mm` at
  contact and `2.022594 mm` across the complete roll sweep. Candidate acceptance
  is `0/36`, so qualification stops before MuJoCo as the design requires.
- Verification: Tuna/trajectory `270/270`; broader scoped grasp suite `381`
  passed plus the same 26 incomplete-ROS-message-stub failures; Python compile,
  fatal flake8, isolated `colcon` build, installed calibration load, and
  cwd/ROS-discovery-independent artifact reproduction passed. A package-wide
  collection also requires unavailable container dependencies (`ament_*`,
  `pycocotools`, `requests`, and full `rclpy`). No motion was launched.

### 2026-07-15 - Closed Second Tuna Motion-Monitoring Review

- Roll execution is now split into separately dispatched at-most-2.5-degree
  micro-segments; qpos-to-gap drift is checked before the first and after every
  micro-segment, so a full visible roll stage cannot hide cumulative jaw creep.
- Added one bounded finalization retry only for timeout/unavailable responses
  without a semantic result or robot side effect. Each attempt consumes newer
  bounds/qpos and reuses the same pivot; only one atomic suffix commit is legal.
  A new exact-Tuna-only `tuna_finalizer.py` boundary owns lazy MoveIt clients,
  while trajectory construction remains pure and executor owns retry/commit.
- Full close now freezes a separate retention contract. Test and normal lift
  use at-most-10-mm micro-segments with retention-aperture checks; post-close
  monitoring never compares against the preclamp aperture. Stable-gap AABB loss
  and gripper aperture drift have distinct structured errors.
- Calibration requires explicit absolute URDF/mesh/mount paths without ROS
  package discovery. Qpos stability is three increasing-timestamp samples with
  at most 0.002 rad peak-to-peak range. The equal 0.5 mm contact, pivot-drift,
  and retention-drift values are documented as physically distinct gates.
- The real-MuJoCo gate now requires five nominal full-lift runs: seed 101 three
  consecutive times plus seeds 202 and 303 once each, before all failure
  injections. Design `02e57de` and plan `6fe4c8d` are committed; no runtime
  code or simulator source changed.

### 2026-07-15 - Hardened Tuna Implementation Gates After Final Review

- Replaced the nominal preclamp pivot with a hash-verified per-command contact-
  matrix table. Runtime must freeze the pivot from three stable measured qpos
  samples, recheck support-hull clearance, and reject greater-than-0.5-mm
  aperture drift without re-interpolation or re-anchoring.
- Made the initial Tuna plan executable only through the preclamp observation.
  A typed deferred suffix is atomically and exactly once finalized from the
  frozen pivot and measured joint state; nominal-pivot roll poses cannot run.
- Added a mandatory real-MuJoCo physical integration gate before live contact,
  a maximum 2 mm pure-lift follow tolerance, and an operational fresh-scene
  reset/cache/perception definition.
- Replaced brittle JSON baselines with semantic non-Tuna fixtures, merged the
  overlapping planner tasks, defined structured Tuna error codes, narrowed the
  setpoint preflight to safe +10 mm world-X/world-Z return motions, fixed the
  positive roll-in sign, and documented Tomato as a routing negative control.
- Design revision `c2602a3` and plan revision `178cafb` are committed. No
  runtime code or simulator source changed; final user review is still required
  before Task 1.

### 2026-07-15 - Diagnosed Stale Dev Container Restart Failure

- Docker Desktop's generic `HTTP 400` for stopped container
  `friendly_visvesvaraya` reproduces at the CLI as an invalid bind mount: the
  saved Dev Container config source under
  `/run/desktop/mnt/host/wsl/docker-desktop-bind-mounts/Ubuntu-24.04/0228...`
  no longer exists. The container previously exited cleanly with code 0; the
  Docker 29.6.1 client/server, image, and host workspace are present.
- The old container must be recreated rather than restarted. Before rebuilding,
  note that `.devcontainer/devcontainer.json` also requires `/dev/dri`, which
  is currently absent in Ubuntu-24.04 WSL and may cause the next bind-mount
  failure. No container was removed and no configuration was changed.
- The first rebuild attempt did not reach Docker at all: Docker Desktop
  `settings-store.json` has `EnableIntegrationWithDefaultWslDistro=false`, and
  Ubuntu-24.04's `/usr/bin/docker` symlink target under
  `/mnt/wsl/docker-desktop/cli-tools` is missing. Enable Docker Desktop WSL
  integration for Ubuntu-24.04, apply/restart, verify `/usr/bin/docker version`,
  and only then rerun Dev Containers rebuild.

### 2026-07-15 - Resolved Tuna Roll-Up Design Review

- Revised the Tuna-only design against all 15 review findings without changing
  runtime code. The exact-name branch now short-circuits before shared profile,
  selector, and grasp-library paths; non-Tuna behavior stays out of scope.
- Repository inspection corrected two review assumptions: the installed double
  90-degree mount makes TCP X the physical closing axis, and MuJoCo reapplies
  the adapter's persistent gripper position target every control cycle.
- Added explicit URDF/STL gap calibration, continuous-branch IK and full-chain
  state validity, live AABB preclamp/roll/close/test-lift checks, true oblique
  cylinder support-width gating, a continuous rolled lift suffix, one Tuna-only
  debug-stage selector, and indexed roll-step names. Next: user re-review before
  any implementation plan or code.
- The second review was approved after separating initial straddle,
  lower-finger sweep, and contact-limited close; adding three-sample initial
  stability, stricter preclamp displacement, residual calibration, and
  at-most-50-mm normal-lift follow checks; and clarifying that pivot location
  changes center trajectory but not the orientation-only AABB height extent.
- The final requested addition defines
  `TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M=0.0005` in calibrated pad-gap units and
  requires the calibration artifact to contain `T_tcp_pivot_at_preclamp` plus
  matching source/schema SHA-256 values. The final design and detailed
  implementation plan are committed as `0f0b281` and `9f6ba29`; no runtime code
  changed.
- Pudding/Tuna architecture intentionally diverges within their exact-name
  boundaries. Pudding keeps dedicated planner/executor/scene-monitor modules
  around a fixed far-table-edge box pivot; Tuna uses one pure cylindrical
  geometry module plus Tuna-only branches in the existing planner, trajectory
  builder, and executor while consuming the existing bounds stream around a
  calibrated lower-pad pivot. Do not introduce a shared roll-up abstraction
  until both strategies are independently implemented and live-stable.
- The final review closed the remaining two ambiguities: Tuna candidates now
  exhaust `{8,9,10} mm x {35,40,45} degree x radial direction` and prove a
  conservative whole-gripper support-hull table-clearance lower bound across
  open/preclamp/roll/allowed-close samples; AABB qualification uses the exact
  ten-yaw schedule above instead of "representative yaw". No runtime code
  changed.

### 2026-07-15 - Added Pear-Only Short-Axis Candidate Gates

- Kept all non-pear branches unchanged and routed only exact normalized
  `pear` through sign-symmetric TCP-X/short-axis alignment and conservative
  projected-AABB opening checks.
- Added pear-namespaced 5-degree alignment and 5 mm opening-margin defaults,
  fail-closed diagnostics, all non-pear round-top routing regression, and a
  real 7,466-pose library test.
- Real-library replay retains only raw index 5675 at yaw 0/180 with 4.858-degree
  alignment error, 74.81 mm projected width, and 10.35 mm opening margin.
  Related container tests pass `303/303`; build and installed import pass. No
  simulator or motion ran.
- Subsequent live trial `20260715_181545_780230/trial_001` visually validated
  the new straddling direction, but candidate 1 plateaued before close with
  about 10.5 mm XY and +29.8 mm Z residual. Close was never sent. Treat final
  depth/contact as the next pear-only problem; do not weaken global tolerance.
- Added exact-name pear-only `GRASP_PEAR_Z_OFFSET=+0.010`, which raises prepared
  pear final/pregrasp world Z exactly 30 mm relative to the shared `-0.020`
  offset without changing XY, rotation, planner, executor, final tolerance, or
  any non-pear branch. Design `35c62b0` and plan `8c77fcb` are committed;
  runtime/test changes remain uncommitted for staged live validation.
- Container selector/planner tests pass `139/139`, related grasp tests pass
  `325/325`, and compilation, fatal flake8, symlink build, installed import,
  host/container hash checks, and real pear library replay pass. The replay
  retains the same two direction-safe candidates and proves an exact +30 mm
  final-Z delta. No new simulator motion ran after this depth change.

### 2026-07-15 - Replaced Fixed Final Z Cap With Live Center Boundary

- The latest gelatin trial settled `+5.2 mm` above target while remaining
  `2.1 mm` below the live object center; the experimental fixed 5 mm gate alone
  prevented close.
- Replaced the fixed cap with a final-only `controlled_center` fallback. The
  ordinary symmetric 3 mm path is evaluated first and remains independent of
  bounds; the fallback requires a positive residual, valid live bounds, target
  inside bounds, actual Z between bottom and center, and XY within 2 mm.
- Removed `GRASP_VERTICAL_FINAL_HIGH_Z_TOLERANCE_M`. Installed verification
  reports defaults `0.0015/0.0020/0.0030`, no fixed-high constant, and accepts
  the recorded gelatin values with a live center limit of about 7.3 mm.
- Focused tests pass `160/160`, related grasp tests pass `270/270`, compilation,
  fatal flake8, four synchronized file hashes, and the Docker symlink build all
  pass. No motion was launched; implementation stays uncommitted until a fresh
  close-only gelatin validation.

### 2026-07-15 - Gelatin Box And Sponge Stopped Before Close

- Two fresh attempts selected nearly identical gelatin-box masks and poses and
  planned the same centered 0.34-degree top-down grasp.
- Trial 3 falsely treated three reads of one cached intermediate pose as a
  stationary feedback window and exceeded the calibration offset cap. Trial 4
  passed through waypoint 18 and stopped at waypoint 19 with approximately
  1.6 mm XY and 0.2 mm Z error; the close command was never executed.
- Direct inspection of the live container logs found gelatin trial 5 stopping
  at waypoint 20/21 with about 1.5 mm XY / 0.3 mm Z error and sponge trial 6
  stopping at waypoint 18/20 with 1.6 mm XY / 0.2 mm Z error. Both retained an
  almost pure world +Y residual after the hover-calibrated offset was frozen,
  and both stopped before close. The common vertical executor is the fault
  boundary; the screenshot is consistent with its deliberate hold.
- Implemented the approved split tolerance: pregrasp stays at 1.5 mm and all
  descent/final-close XY checks use a separately validated 2.0 mm default.
  Error objects and success/error logs now report the actual stage tolerance.
  Z, timeout, hold, command-offset, waypoint, gripper, and non-vertical behavior
  remain unchanged; no object-specific branch was added.
- Focused tests pass `141/141`, related functional regression passes `321/321`,
  compilation, fatal flake8, both-workspace SHA synchronization, the symlink
  package build, and installed defaults `0.0015/0.0020/0.0030` pass. No
  simulator or robot motion ran automatically. Next: after-close-only live
  trials for gelatin and sponge before any lift-return attempt.
- The first post-change gelatin live trial confirmed all 20 intermediate
  waypoints now pass at the 2.0 mm XY gate. Waypoint 21/21 then timed out only
  because actual Z stayed 4.6 mm above the too-low target, versus the unchanged
  3.0 mm absolute Z gate. Actual Z remained inside the live object bounds and
  close was never commanded. Treat this as a final close-readiness Z-policy
  issue, not an XY regression, perception error, or weak gripper.
- Approved and documented a final-only asymmetric Z policy: ordinary and
  downward Z remain limited to 3.0 mm; a positive residual may reach 5.0 mm
  only after final settling when target/actual Z are inside valid live bounds.
  Missing or invalid bounds fail closed, and no extra lower command is sent.
  Design spec commit: `694b7b4`.
- Implemented the policy with a validated 5.0 mm high-side configuration,
  explicit live-bounds/decision records, final-waypoint timeout fallback, and
  the same predicate at close readiness. Logs/errors now expose signed Z,
  asymmetric limits, bounds, acceptance mode, and rejection reason. Focused
  tests pass `164/164`, related grasp regression passes `274/274`, compilation,
  fatal flake8, file hashes, symlink build, and installed defaults pass. Plan
  commit: `f1be782`; runtime/test changes remain uncommitted pending close-only
  live validation, and no motion was launched automatically.
- Fresh gelatin trial 1 in session `20260715_164719_249422` reached final
  waypoint 20/20 with XY `0.6 mm`, valid live bounds
  `0.8900/0.9053/0.9205 m`, and actual Z `0.9032 m`. It missed the configured
  positive limit by only `0.2 mm` (`+5.2 mm` versus `+5.0 mm`) while remaining
  `2.1 mm` below object center and visibly between the fingers. The new policy
  was loaded and rejected exactly as designed; close was never sent. This is
  now evidence that the fixed 5.0 mm edge is too brittle for the observed
  tracking distribution, not a stale build, invalid bounds, XY problem, or
  gripper-close failure.

### 2026-07-15 - Restart Duplicate Was A Stale DDS Endpoint

- Trial 2's owned `moveit2_iface` received SIGINT and exited; Trial 3's exact
  `pgrep -a -x moveit2_iface` pre-start gate had no output, so the supervisor
  did not leave a second live operating-system process behind.
- After the replacement registered, the action graph temporarily reported two
  same-name `/moveit2_iface` servers. At inspection time there was exactly one
  live process and a fresh `ros2 action info /arm/move_to_pose` reported one
  server, proving that the second entry had aged out of DDS discovery.
- `RosGraphProbe.wait_until_ready` retries missing endpoints but returns
  immediately for any duplicate. In a controlled restart this can be a false
  stop while the prior endpoint's discovery record expires. A proper follow-up
  should retry duplicates for a bounded settle interval and require consecutive
  clean samples; a persistent duplicate must still stop the trial.
- Implemented that policy with internal defaults of 30 seconds and two clean
  samples. All graph queries and settle progress remain in `graph_ready.log`;
  the terminal stays quiet. Deterministic tests cover transient recovery,
  clean-streak reset, persistent duplicate failure, missing-endpoint timeout,
  and per-command query timeouts.
- Verification passes: focused `23/23`, core `73/73`, related `57/57` and
  `57/57`, Python compilation, fatal flake8, `my_course_pkg` symlink build, and
  installed defaults `30.0`/`2`. The active old supervisor requires a full
  quit/relaunch before live validation.

### 2026-07-15 - Tomato Instruction Selected And Grasped Apple

- The pipeline log proves stdin was fixed and preserved the exact instruction
  `pick up the tomato`; target corruption did not occur in the session wrapper,
  SAM2, or FoundationPose. Step 3 itself returned raw VLM JSON selecting apple.
- The captured RGB image contains a prominent red apple and a tomato-soup can.
  Only `blue racquetball` currently has a deterministic instruction alias, so
  plain `tomato` used the generic multimodal prompt. That prompt exposes all 18
  YCB names and asks the model to combine image and text; the red apple was a
  plausible visual match for a fresh tomato even though the canonical can name
  was available.
- Postprocessing merely filters candidates to `YCB_OBJECTS`. Because `apple`
  is legal, the pipeline accepted it as candidate zero with no comparison to
  an instruction-derived expected identity or the six active scene objects.
  The downstream apple pick/place completed, so this is a motion-safety gap,
  not just a labeling/logging issue.
- Recommended next design: pass the fixed scene inventory into the pipeline,
  resolve unique instruction tokens deterministically (`tomato` ->
  `tomato soup can`), stop/clarify ambiguous terms such as `can` or `ball`, and
  reject any VLM identity that conflicts with the resolved target before SAM2,
  FoundationPose, or grasp execution.

### 2026-07-15 - Supervisor Stdin Forwarding Corrected

- The first real instruction was echoed by the terminal but no `pipeline.log`
  appeared; the supervisor remained in `pipe_read`. The accompanying
  `ExecuteProcess.__on_process_stdin_event()` warning identified the delivery
  failure rather than a pipeline failure.
- Inspection of the installed Humble `launch` source confirmed its
  `ProcessStdin` handler only logs that warning and never writes the event bytes.
  The session forwarder now schedules a direct write to pipe 0 of the exact
  supervisor action's `_subprocess_transport`, checks for a missing/closing
  pipe, and retains EOF-to-`q` behavior. This private API is intentionally
  isolated in one helper and covered with fake-transport tests.
- RED produced two expected failures; focused launch tests pass `11/11`, core
  tests pass `69/69`, fatal flake8 and compilation pass, and
  `ifl_air_ur_launch` was rebuilt. The user's already-running launch still has
  the old module loaded and must be restarted before live input validation.
- `howto.md` now documents second-terminal `tail -F` commands for the ROS
  `launch.log` and current trial's interface/pipeline/grasp logs.

### 2026-07-15 - Experiment Session RViz Disabled

- Changed only the `experiment_session.launch.py` include boundary to pass
  `launch_robot_rviz=false` and `launch_moveit_rviz=false`. The MuJoCo include,
  startup ordering, and all standalone launch defaults remain unchanged.
- Added exact session-argument regression coverage plus assertions that the full
  launch still defaults robot RViz off and MoveIt RViz on. RED failed on the
  missing session argument; the final launch suite passes `40/40`.
- Python compilation, fatal flake8, the `ifl_air_ur_launch` symlink build, and
  source/container/installed launch hash equality pass. Design `bb70519` and
  plan `f17146e` are committed; runtime/test files remain uncommitted with the
  surrounding session work. No new simulator was launched automatically. The
  currently running session loaded the old module and must be restarted once
  before RViz stays closed.

### 2026-07-15 - Experiment Session Runtime Output Quieted

- A restarted live session passed the ROS graph gate and blocked in
  `pipe_read`, proving it was waiting for the instruction, but MoveIt warnings
  continued to overwrite the one-time prompt and made terminal input unusable.
- Added `quiet_runtime_output` with default `false` through the full, MoveIt,
  and robot/gripper launch boundaries; only `experiment_session.launch.py`
  enables it. MuJoCo, MoveIt, RViz, gripper, and robot-state stdout/stderr are
  redirected to ROS launch logs, while the owned interface drains into its
  existing per-trial log without terminal mirroring. Standalone output remains
  unchanged.
- ROS 2 Humble's string `output="log"` alias still mirrors stderr to screen, so
  quiet mode uses an explicit `{"both": "log"}` destination map constructed
  after resolving the launch argument. The first live attempt exposed this;
  the corrected non-motion smoke test showed only finite startup/reset/session
  lines and a stable `Instruction:` with no pipeline or grasp execution.
- Core tests pass `68/68`; the related suites pass `57/57` and `57/57`;
  focused fatal flake8, compilation, both package builds, installed executable
  discovery, and launch argument inspection pass.

### 2026-07-15 - Session ROS-Graph Timeout Fixed

- The first live integrated launch opened MuJoCo but never displayed the
  instruction prompt. Its graph log showed every `ros2 node list` was cut off
  by the hard-coded 5-second subprocess timeout. A direct live query required
  about 6.2 seconds; both action queries and the pose-topic query also required
  about 6 seconds and confirmed exactly one expected endpoint each. The
  repeated missing-knuckle warning was visible but was not the supervisor's
  blocking condition.
- Increased each ROS CLI graph-query timeout from 5 to 20 seconds and the
  overall readiness window from 45 to 180 seconds. Instruction and Enter/`q`
  prompts are now printed as newline-terminated, explicitly flushed lines
  before reading stdin so launch output cannot hide the paused state.
- Added regression coverage for configurable query timeout and prompt
  visibility. Core tests pass `64/64`, focused flake8 and compilation pass, and
  `my_course_pkg` was rebuilt. The already-running launch still has the old
  module loaded and must be restarted for the fix to take effect.

### 2026-07-15 - Pudding-Box Pregrasp Failure Diagnosed

- Correlated the attached `grasp_demo` run with live ROS logs. All configured
  planners rejected the same invalid pregrasp; the decisive diagnostics were
  missing goal IK, right-gripper-knuckle/table collision, and no valid OMPL goal
  state. The grasp library contains `6526` poses with a best reconstructed
  tool-Z/down angle of about `75.6 degrees`, so the issue is upstream candidate
  suitability/selection rather than the action server, controller switch,
  perception, Cartesian descent, closing, or lift stages.

### 2026-07-15 - Persistent Experiment Session Designed

- Approved a top-level launch plus dedicated Python trial supervisor. The base
  MuJoCo/MoveIt stack remains alive, while the supervisor exclusively owns the
  standalone interface and restarts it through its recorded process group.
- Startup reuses an exported `VLM_API_KEY` or prompts once with hidden input,
  then directly displays the configured 18 objects and accepts up to six. The
  key stays in the process environment for the session and is never logged or
  written; the plaintext key in `howto.md` must be removed and rotated during
  implementation.
- Every trial uses the same safe boundary: interface uniqueness, reset, fresh
  instruction and pipeline, then automatic grasp only after pipeline success.
  Any failure pauses with timestamped stage logs; Enter restarts from the safe
  boundary and `q` closes MuJoCo and the complete session.
- The implementation plan now identifies exact launch/package/test files,
  preserves the dirty assign-mode work, adds RED/GREEN state-machine and
  process-ownership tests, removes stale manual perception overrides, and
  reserves live simulator/motion validation for the user.
- Implemented the top-level launch, explicit assign handoff, conditional legacy
  interface include, API-key/environment handling, Humble-compatible stdin
  forwarding, supervisor console entry point, owned process-group lifecycle,
  fail-closed `pgrep`/ROS graph gates, reset/pipeline/grasp state machine, and
  streamed per-stage logs. The plaintext key was removed from `howto.md`; it
  still requires server-side rotation because Git history retains it.
- Synchronized 11 scoped files to the Docker-mounted workspace. Core regression
  passes `62/62`; related suites pass `57/57` and `57/57`; focused flake8,
  compilation, package builds, executable discovery, and `--show-args` pass.
  No MuJoCo launch or robot motion ran.

### 2026-07-15 - Racquetball Failure Is A Target-Mask Mismatch

- Inspected the failed run, current `/scene_clearance_bounds`, live
  `world <- base_link` TF, saved candidate metadata, and candidate-card image.
  The perception pipeline selected tennis ball as racquetball; the actual blue
  racquetball was not among the detector candidates.
- The planner generated a valid racquetball grasp at the perceived tennis-ball
  location, then the exact-bounds safety gate stopped all candidates before
  motion because the real tennis-ball marker occupied that corridor. No grasp
  or safety-threshold code was changed.
- The user approved the fixed alias `blue racquetball`, with canonical identity
  `racquetball` and visual grounding prompt `blue ball`. Design `749c93b`
  preserves normal-object behavior, makes the alias mask path fail closed, and
  requires saved-frame non-motion validation before plan-only or motion.
- Implementation `50522d7` adds a deterministic alias resolver, preserves
  `VLM_CANDIDATE_OVERRIDE` precedence, writes auditable identity/prompt/class
  metadata, extends `_sam2_class_matches_target()` with dynamic accepted class
  names, and prohibits highest-score fallback for an uncertain alias mask.
  Focused and related tests pass `131/131`; compilation and package build pass.
- Saved-frame SAM2 produced two `blue ball` candidates. Without a VLM key the
  strict alias path stopped safely rather than selecting the higher-score
  tennis ball. Manually selecting the visually confirmed blue candidate for a
  non-motion check produced a `4738 px` mask and successful racquetball
  FoundationPose result. Automatic verifier selection still needs one user-run
  API-key-equipped pipeline validation.

### 2026-07-15 - Assign Scene Mode Implemented

- Added `assign` as an independent third selector while leaving `mix/random`
  selection branches unchanged. Assigned names retain input/slot order and a
  full-pool sample without replacement fills the scene to exactly six objects.
- The launch reads the 18 canonical names from `base_env.yaml`, accepts optional
  spaces and case-insensitive input, and retries the entire line on unknown,
  duplicate, over-six, or empty-token input. Empty input requests six random
  objects. The simulator repeats the name, uniqueness, and pool-size checks.
- The launch passes `scene_mode=assign` and `assigned_object_names=[...]`
  through Hydra before any child starts. `python3-yaml` is declared as a launch
  runtime dependency. Design `57f8068` is committed; implementation remains
  uncommitted and is synchronized to the Docker-mounted workspace.
- Simulator/launch regression passes `113/113`; grasp-selector compatibility
  passes `26/26`; compilation, Hydra/list propagation, actual-YAML loading,
  pure selection, package build, and installed `--show-args` pass. No simulator
  or arm motion was started.

### 2026-07-15 - Interactive Mix/Random Scene Modes Implemented

- Kept the existing `select_scene_objects()` code as the exact `mix` path and
  added a separate validated category-aware selector for `random`. Slot order
  is can, banana, round-top, box, hammer, then a non-duplicating full-pool
  random object. Existing placement coordinates and `/reset_sim` are unchanged.
- The full launch now prompts before children start. Explicit
  `scene_mode:=mix` or `scene_mode:=random` bypasses the prompt; invalid values
  fail before startup. Hydra override inspection confirmed `random` propagation.
- Runtime and tests are synchronized to the active Docker-mounted workspace.
  Combined simulator/launch tests pass `84/84`, grasp-selector compatibility
  passes `26/26`, compilation and the `ifl_air_ur_launch` symlink build pass,
  and installed `--show-args` reports default `prompt`. No simulator or arm
  motion was started.
- Existing feasible vertical-grasp work was checkpointed as `b755713`; related
  regression passed `233/233`, `my_course_pkg` built, and `grasp_stable` was
  pushed before this scene-mode design work began.

### 2026-07-15 - Banana Change Cancelled After Plain Run Succeeded

- The close-only run with `GRASP_Z_OFFSET=0.0` succeeded. The user then ran
  plain `grasp_demo`, also observed success, and cancelled the proposed default
  change.
- Before classifying this as stochastic, check the live shell with
  `printenv GRASP_Z_OFFSET`: an exported `0.0` still affects plain commands,
  while an inline `NAME=value command` assignment does not persist. If unset,
  the earlier default run was only just outside the convergence threshold and
  the mixed result indicates a low-margin, state/contact-sensitive path rather
  than a uniformly random software branch.

### 2026-07-14 - Banana Final-Approach Failure Diagnosed

- The attached `grasp_demo` log reached pregrasp but never executed gripper
  close. Its sole `centered` banana candidate was lowered from raw Z=0.9083 m
  to Z=0.8883 m by the shared `GRASP_Z_OFFSET=-0.020`, below the estimated
  banana bottom/origin at Z=0.8906 m.
- Final feedback stalled at Z=0.9043 m and 18.0 mm rounded 3D error after 14
  convergence commands. The dominant 16.0 mm vertical residual is consistent
  with contact from an over-low target. Raising the tolerance would merely
  accept the wrong pose; no code or live motion was started.

### 2026-07-14 - Vertical Waypoint Pre-command Settling Implemented

- Replaced the immediate pre-command XY abort with a bounded wait that reuses
  the existing 1.5-second timeout and 0.05-second sampling period. It sends no
  descent or compensation command while XY exceeds 1.5 mm, then sends the
  lower waypoint once if feedback returns inside the unchanged gate.
- Persistent pre-command lateral drift now times out and holds the latest
  observed 6D pose with command count zero. Post-command XY/Z convergence,
  waypoint spacing, frozen offset, tolerances, and non-vertical behavior are
  unchanged.
- Tests first reproduced the 2.9 mm waypoint-7 transient and persistent-drift
  cases, then passed after implementation. Executor tests pass `40/40`, related
  grasp regression passes `243/243`, compilation and symlink build pass, and
  runtime/test files match across both workspaces. Design `baf007e` is
  committed; implementation remains uncommitted. No live motion was started.

### 2026-07-14 - Floor Clamp Verified; Waypoint 7 Hit Pre-command XY Transient

- The live run reported bounds bottom Z=0.8900 m and minimum TCP Z=0.8980 m.
  Fresh upright perception selected library Z=0.9393 m, so the generic clamp
  correctly preserved the higher target and was not the cause of this abort.
- Safe-pregrasp calibration converged and descent waypoints 1-6 passed.
  Waypoint 6 was accepted at approximately 1.5 mm XY / 1.8 mm Z error; roughly
  74 ms later the waypoint-7 precheck sampled 2.9 mm XY / 2.6 mm Z error and
  held without sending waypoint 7.
- The remaining asymmetry is the unchanged single-sample pre-command XY gate.
  A minimal general follow-up is to wait up to the existing 1.5 s for lateral
  feedback to return inside the unchanged gate, without sending the lower
  waypoint; persistent drift must still time out and hold. No code changed.

### 2026-07-14 - Generic Vertical Bounds Floor Clamp Implemented

- Added validated `GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M`, default 8 mm;
  non-finite and negative overrides fail during configuration.
- Extended the existing vertical live-bounds XY alignment to apply
  `final_z = max(library_z, bounds_bottom_z + 0.008)` and return/log the full XYZ
  correction. It has no object-name branch and preserves higher library Z.
- Preserved legacy XY debug fields while adding original/target/correction XYZ,
  bounds-bottom Z, and minimum-TCP Z. Corrected raw-library-Z diagnostics to
  remove both profile offset and bounds-floor correction.
- Test-first coverage includes the recorded foam-brick values, pudding-box
  generality, higher/equal targets, invalid bounds/config, immutable inputs,
  transform consistency, and downstream pregrasp/lift/return poses. Focused
  config/planner tests pass `167/167`, related regression passes `242/242`,
  compilation and symlink build pass, and four changed files match across both
  workspaces. Implementation remains uncommitted; no live motion was started.

### 2026-07-14 - Generic Vertical Bounds Floor Clamp Designed

- The user prioritized generality over a foam-brick-only +25 mm offset and
  approved a one-sided live-bounds rule for every `vertical` grasp:
  `final_z = max(library_z, bounds_bottom_z + 0.008)`.
- Because `/scene_clearance_bounds` publishes world-axis AABBs, bounds bottom is
  exactly `center_z - half_height`. The recorded brick target changes from
  0.8713 m to 0.8980 m, close to the reached 0.8987 m pose.
- Design `0bd4fce` specifies validated configuration, fail-closed geometry,
  diagnostics, tests, and bounded live validation. No runtime code changed.

### 2026-07-14 - XY Settling Passed; Waypoint 16 Reached Table Height

- The post-change live run passed safe-pregrasp calibration and descent
  waypoints 1-15. This proves the previous 2.1 mm one-frame XY failure is fixed.
- Waypoint 16 targeted nominal Z=0.8913 m but timed out at actual Z=0.8987 m;
  XY error was only 0.3 mm while Z error remained 7.4 mm. No close command was
  issued.
- The screenshot shows the foam brick centered between open fingers and both
  fingertips already at table height. The planned final TCP Z=0.8713 m would
  require another 27.4 mm descent and is physically too low. A targeted roughly
  +25 mm foam-brick final-grasp Z correction is the next candidate; no code was
  changed in this diagnosis.

### 2026-07-14 - Vertical Waypoint XY Settling Implemented

- Replaced the immediate post-command lateral-abort branch with a joint XY/Z
  convergence condition. A transient sample such as 2.1 mm XY now waits without
  another command; the waypoint advances only after one later sample is within
  both the unchanged 1.5 mm XY and 3 mm Z gates.
- The pre-command lateral gate, 5 mm step size, frozen command offset, 1.5-second
  timeout, hold-on-failure behavior, and final nominal verification are
  unchanged. Persistent lateral error now follows the same timeout path as
  persistent Z error.
- Test-first verification covered transient recovery, persistent lateral
  timeout, and same-sample XY/Z acceptance. The executor file passes `39/39`,
  related grasp regression passes `227/227`, Python compilation and
  `colcon build --packages-select my_course_pkg --symlink-install` pass, and the
  runtime/test files match across both host workspaces. Design is committed as
  `be6c8d3`; implementation remains uncommitted. No live motion was started.

### 2026-07-14 - Stability Fix Passed; Waypoint 2 Hit Transient XY Gate

- The post-fix run behaved as designed at pregrasp: initial offset
  `[-0.1,-7.1,+15.9] mm`, one moving window waited with no command, then a
  stationary window passed at 0.3 mm XY / 0.8 mm Z for median and newest
  feedback. No redundant servo-mode request occurred after freeze.
- Descent waypoint 1 passed at 0.7 mm XY / 2.0 mm Z. Waypoint 2 was commanded
  at timestamp `1784059997.4586` and aborted at `1784059997.5022`, about 44 ms
  later, because its first observed lateral error was 2.1 mm versus the 1.5 mm
  gate; Z error was already acceptable at 2.6 mm.
- The descent loop currently fails on any single lateral sample immediately
  after a command, while it permits Z to settle. This treats controller
  transient motion as settled lateral drift. The next design should wait for a
  stationary/confirmed waypoint observation before judging ordinary lateral
  convergence, without issuing the next lower waypoint; retain a separate hard
  emergency bound if needed. No code was changed in this diagnosis.

### 2026-07-14 - Vertical Pregrasp Stability Gate Implemented

- Added direction-independent stationarity checks over each feedback window:
  maximum pairwise XY spread must be at most 0.75 mm and Z peak-to-peak spread
  at most 1.5 mm with current safety tolerances.
- A moving window neither updates the learned offset nor consumes a compensated
  command. It waits and resamples for at most 1.5 seconds, then holds the newest
  raw pose and fails. A stationary window freezes only when both its median and
  newest sample pass the unchanged 1.5 mm XY / 3 mm Z physical gate.
- Removed the redundant `SERVO_POS_CTL` request between calibration and first
  descent. Focused tests pass `124/124`, related regression passes `225/225`,
  Python compilation and the symlink package build pass, and the two changed
  runtime/test files match across workspaces. Design is committed as `0354c28`;
  implementation remains uncommitted. No post-fix live trial has run yet.

### 2026-07-14 - Live Calibration Exposed Premature Stability Acceptance

- The first calibrated foam-brick run reduced the original 17.5 mm residual
  and learned offset `[-0.2,-8.9,+21.4] mm` in two compensated commands.
- The three-sample median passed at 1.0 mm XY and 1.7 mm Z, but the newest raw
  sample was already at about 1.7 mm XY and 3.7 mm Z and still moving. After a
  redundant second `SERVO_POS_CTL` request, the first descent precheck measured
  2.2 mm XY and 9.9 mm Z relative to the next waypoint and held before motion.
- The screenshot looks centered, but the brick's prior estimated per-finger
  nominal clearance is only about 2.3 mm, so widening the descent gate to 3 mm
  would consume the physical margin. Correct the transition by requiring the
  newest sample/stability to pass before freezing and by avoiding the redundant
  mode request. No code was changed in this diagnosis.

### 2026-07-14 - Per-Run Vertical Command-Offset Calibration Implemented

- Replaced repeated nominal safe-hover commands with three-sample component-
  median XYZ feedback and cumulative residual compensation. Orientation stays
  nominal; the learned XYZ offset exists only inside one `execute_plan` call.
- Added a 30 mm cumulative cap, three-command limit, 0.4-second settle,
  divergence detection, and latest-observed-pose hold on every calibration
  failure path. The accepted offset is frozen and explicitly passed into each
  5 mm descent command, while all feedback checks remain against nominal
  physical waypoints and the strict final verification remains nominal.
- Added validated configuration readers and tests for defaults/invalid values,
  recorded bias, zero bias, cumulative correction, median outlier rejection,
  cap/divergence/exhaustion, orientation preservation, and nominal descent
  gating. Focused tests pass `118/118`; related regression passes `219/219`;
  Python compilation and `colcon build --packages-select my_course_pkg
  --symlink-install` pass. Four runtime/test files match between workspaces.
  No live arm trial has been run after this implementation.

### 2026-07-14 - Vertical Command-Offset Calibration Design Committed

- The user approved per-run calibration only: learn XYZ command bias at safe
  pregrasp, freeze it through descent, and discard it after the plan.
- Design `1045752` specifies three-sample median XYZ feedback, at most three
  0.4-second correction iterations, a 30 mm total-offset cap, explicit
  divergence detection, and nominal-waypoint feedback gates. It contains no
  hard-coded foam-brick offset and makes no runtime-code change.
- The specification was self-reviewed and amended to keep the newest raw full
  pose for hold commands rather than synthesizing a median orientation. Next:
  user review, implementation plan, then test-first implementation.

### 2026-07-14 - Vertical Gate Safely Exposed Pregrasp Servo Bias

- A fresh foam-brick run used the centered target and entered the new pregrasp
  gate. It sent 15 reanchor commands, but feedback settled at actual minus
  target approximately `[+0.1,+7.0,-16.1] mm`.
- The gate timed out with X/Y error `0.0070 m` and Z error `0.0161 m`, commanded
  a hold at the observed pose, raised `VerticalApproachConvergenceError`, and
  sent no descent or gripper-close command. The safety behavior passed.
- Repeating the nominal target cannot cancel the controller's steady-state
  mapping error. The next design should learn a bounded command offset at safe
  hover, add that same offset to each commanded descent waypoint, and continue
  comparing feedback with the original nominal waypoint. Do not weaken the
  strict gate. No code was changed during this diagnosis.

### 2026-07-14 - Vertical Approach Feedback Gate Implemented

- Added validated defaults for 1.5 mm world-XY tracking, 3 mm Z tracking, and
  5 mm maximum vertical-approach waypoint spacing.
- `vertical` plans now actively converge at the safe pregrasp, send one descent
  waypoint at a time, wait for feedback before the next waypoint, and perform a
  strict final verification before close. Lateral drift or Z timeout overwrites
  the outstanding target with a hold at the observed pose and raises a
  non-retryable execution error.
- Non-vertical plans retain their generic Cartesian path and 18 mm final gate.
  Focused tests pass `91/91`, related regression passes `192/192`, the package
  builds, installed defaults are `0.0015/0.003/0.005`, and all four changed
  runtime/test files match between workspaces apart from line endings. No live
  arm trial has been run yet.

### 2026-07-14 - Vertical Approach Feedback-Gate Design Committed

- The user approved preserving the current vertical grasp orientation and
  bounds-centered target while correcting the execution-time descent miss.
- Design `859a006` adds safe-height pregrasp convergence, 5 mm feedback-gated
  descent stages, 1.5 mm X/Y and 3 mm Z limits, and hold-and-fail behavior that
  blocks later descent and gripper close on tracking error.
- The specification was self-reviewed and contains no runtime-code changes.
  Next: user review, implementation plan, then test-first implementation in
  both workspaces.

### 2026-07-14 - Six-Object Stable-Grasp Milestone Confirmed

- The user confirmed stable grasping of the five fixed representatives plus a
  randomly refreshed `tennis_ball` in slot 6.
- Report wording must distinguish the fixed five-category scene contract from
  the successful random sixth-object instance. The tennis ball uses the shared
  `round_top` category and is evidence of category-level reuse rather than an
  object-specific branch.
- Before committing, the 19 affected implementation/test files were confirmed
  identical to the Docker-mounted runtime workspace apart from line endings.
  Grasp regression passed `168/168`, simulator scene/bounds regression passed
  `65/65`, and `my_course_pkg` built successfully. No additional live-motion
  run was performed while recording this milestone.

### 2026-07-14 - Foam Brick Descent Clearance Failure Identified

- The bounds-centering log proved the planned final X/Y exactly matched the
  live foam-brick center after a `[-31.2,+0.4] mm` correction. The prior nominal
  off-center planning bug is fixed.
- The open gripper provides only about `4.6 mm` total clearance around the
  `0.0804 m` brick span in this orientation. The approach first ended with
  `0.0271 m` error, then post-descent convergence accepted `0.0126 m` under the
  global `0.018 m` tolerance. The user's direct observation that one finger hit
  the top surface is consistent with those numbers: nominal per-side clearance
  is only about `2.3 mm`, far below the residual motion error.
- The actual close subsequently stalled at `0.038 rad`, equivalent to about
  `0.0810 m` opening. This describes the near-limit contact after the descent
  strike; it does not by itself prove that choosing the long-axis orientation
  was the sole failure. The current close minimum is `0.010`, so the result
  passed and the unverified lift-return sequence continued.
- Offline real-library inspection found 10,085 foam-brick poses and 384 within
  10 degrees of top-down, but their closing axes all project to at least about
  `0.0786 m` across the brick. A short-side grasp must therefore be generated by
  90-degree box symmetry rather than selected unchanged from this library.
- No code or configuration was changed during this diagnosis. The primary
  execution fix is to converge/re-anchor lateral position while still hovering
  above the object, before final descent. A short-side orientation provides more
  clearance but is optional; a higher close threshold is secondary detection.

### 2026-07-14 - Generic Vertical Bounds Centering Implemented

- Added fail-closed target parsing for `/scene_clearance_bounds`: vertical
  planning requires exactly one normalized name match, positive finite bounds,
  and a valid transform into `world`. It does not fall back to the uncentered
  perception/library target.
- Each selected `vertical` candidate is copied, then its final world X/Y is set
  to the target-bounds center in both the 6D grasp pose and `T_world_grasp`.
  Z, orientation, and the downstream executor sequence are unchanged. The log
  records original X/Y, target X/Y, correction, and centered X/Y.
- Added the recorded foam-brick regression (`-31.2 mm` world-X and `+0.5 mm`
  world-Y correction), pregrasp translation, non-vertical isolation, name/TF,
  missing/duplicate/invalid marker, timeout, and immutability coverage.
  Docker-mounted planner tests pass `65/65`; selector/planner/executor/
  trajectory/evaluator regression passes `168/168`; `my_course_pkg` builds.
- The implementation is synchronized in both dirty workspaces but is not
  separately committed because the same files contain unrelated existing
  edits. No simulator reset, perception run, or robot motion was performed.

### 2026-07-14 - Generic Vertical Bounds-Centering Design Approved

- The user confirmed that the required fix is the actual down-grasp alignment,
  not merely a stricter failure criterion, and rejected a foam-brick-only
  special case.
- The approved design applies to the entire `vertical` profile. It preserves
  the library-selected Z and orientation while translating the final candidate
  X/Y to the selected object's live world-frame bounds center; downstream
  pregrasp construction inherits the same translation delta.
- Missing, duplicate, invalid, or untransformable target bounds fail closed
  before motion. Other grasp profiles, gripper thresholds, lift verification,
  and the executor sequence stay unchanged. The design spec is committed as
  `083714d` and awaits written user review before implementation planning.

### 2026-07-14 - Foam Brick Edge Grasp Falsely Passed Close Gate

- The perception overlay contained three foam-brick detections, and the VLM
  verifier correctly selected the real brick as candidate 1 with high
  confidence. This was not a wrong-object or wrong-mask failure.
- The single library `vertical` candidate placed the requested TCP roughly
  `31 mm` off the pre-run live brick-bounds center in world X. Because the final
  approach gate accepted `17 mm` position error, the physical jaw center could
  reach or cross the brick edge. After the run, live bounds showed about
  `6 mm` XY displacement, consistent with nudging rather than lifting.
- Close returned `position=0.050`, `effort=139.7`, `stalled=True`. The generic
  minimum is only `0.040`, so this near-empty/edge contact passed, whereas the
  previously successful brick capture stopped near `0.333-0.336`. No object
  pose/lift verification exists after close, so the executor ran lift, hold,
  return, release, and reported completion despite the brick remaining down.
- No code or configuration was changed during this diagnosis. Prefer centering
  the box-profile grasp from reliable geometry and adding a stronger
  false-positive check rather than merely repeating the same single candidate.

### 2026-07-14 - Balanced Pinch Held But Return Swept Banana

- Live code selected the intended balanced candidate with
  `selected_angle_deg=2.993`, balance distance `0.0011 m`, translation
  `[-0.0311,-0.0106,0.0164]`, and Z offset `0.0`. Final approach converged at
  `0.0109 m`; gripper contact was accepted at position `0.502`, effort `139.7`,
  and the robot completed the `0.20 m` lift plus five-second hold.
- The user ran plain `ros2 run my_course_pkg grasp_demo`, not the requested
  bounded `GRASP_DEBUG_STOP_AFTER_LIFT=1` validation. Execution therefore
  continued into `return_to_grasp`; the visibly tilted long hammer struck the
  adjacent banana. At release, two open commands stalled at positions `0.590`
  and `0.593` and raised `GripperCommandError`, consistent with the held tool
  being jammed under contact load.
- Centering the TCP removes nominal gravity torque but does not make a two-
  finger pinch on a round wooden handle torsionally rigid. Small effective-COM,
  asymmetric-contact, and friction errors can still rotate the hammer about the
  jaw contact line. Strict `<=10 degree` parallel hold is therefore unreliable
  with balance scoring alone. If the bounded hold confirms the same tilt, the
  next solution must use a caging/anti-rotation grasp near the head-handle
  junction or change the physical contact model; do not keep shifting the
  balance point blindly.

### 2026-07-14 - Balanced Hammer Candidate Selection Implemented

- Added validated hammer balance-point configuration using collision-mesh mass
  center `[-0.030227,-0.009931,0.015676]`, independent score weight `250`, and
  maximum top-down angle `10 degrees`. Added profile-specific top-down Z offset
  default `0.0`; other profile offsets, global `0.018 m` convergence tolerance,
  `0.20 m` lift, and five-second hold are unchanged.
- Hammer selection now filters the real library by the angle limit, ranks by
  `angle_deg + weight * XY_balance_distance_m`, fails closed when no pose
  survives, and logs the selected angle, distance, and object-frame position.
  No executor or motion-path branch was added.
- User review correctly flagged potential candidate-index ambiguity. Direct
  inspection confirmed `10074` poses and zero-based index `3716` at
  `[-0.0311105,-0.0106208,0.0164314]`; adjacent index `3715` shares the
  translation, so implementation ranks geometry rather than hard-coding an
  index. The unrelated unused `CENTERED_GRASP_OBJECTS` set was intentionally
  left untouched to keep the merge focused.
- Config/selector tests pass `76/76`; selector/planner/executor regression
  passes `150/150`; `my_course_pkg` rebuilds with `--symlink-install`. Real-
  library validation for the recorded hammer pose retained `134/10074` poses
  and selected zero-based index `3716`, balance distance `0.0011209 m`, angle
  `3.0495 degrees`, and top-down Z offset `0.0000 m`. Implementation is
  synchronized in both dirty workspaces but not committed with unrelated edits.

### 2026-07-14 - Lemon Blocked By Hammer At Random Slot 6

- The lemon run completed perception and generated 8 ranked `round_top`
  candidates, but sent no MoveIt or gripper command. The exact 2.5D bounds
  filter rejected every candidate against `hammer`, with box distances
  `0.0841-0.0878 m` below the `0.1100 m` corridor-plus-margin requirement.
- A fresh `/scene_clearance_bounds` sample matched the failure: hammer center
  and size were approximately `(-0.538, 0.246)` and `0.183 x 0.333 m`; lemon
  center was approximately `(-0.361, 0.203)`. The lemon TCP is therefore only
  about `85.5 mm` from the hammer AABB short edge.
- Do not bypass `GRASP_ROUND_TOP_CLEARANCE_*` for this scene. Unlike the former
  Apple/banana circle rejection, this failure remains after exact rectangle
  distance and has only about `5.5 mm` clearance beyond the `0.080 m` physical
  corridor before the safety margin. Preferred design scope is moving only
  random placement slot 6 while preserving hammer's validated slot-5 Y value.
- The user approved changing only slot 6 to `[-0.25, 0.20, 0.0]` and updating
  the corresponding configuration test. The self-reviewed design is committed
  as `c590f32`.
- Implemented the approved two-line logical change in both the current repo and
  the separate Docker-mounted `-grasp-stable` repo without overwriting their
  existing unrelated edits. Test-first verification failed only at the old
  slot-6 value, then passed `7/7` after the YAML change. `git diff --check`
  passes in both workspaces. An offline translation of the eight observed
  lemon distances gives `0.1941-0.1978 m` against the unchanged `0.1100 m`
  requirement. No simulator restart, reset, perception run, or arm motion was
  performed; the live scene must be relaunched to consume the new YAML.
- After relaunch, the new slot was active at about `(-0.251,0.203)` in live
  bounds, but the fixed camera projected it below the image at approximately
  `(539,867)`. The lemon was not visible. The two generated lemon masks were
  visibly banana and apple; low-confidence verification fell back to the higher
  detection score and selected banana. FoundationPose consequently returned a
  pose inside the banana box, and the exact corridor gate rejected all eight
  candidates with `box_xy_distance=0.0000 m` before motion.
- The user approved moving slot 6 to the far table corner
  `[-0.85,0.35,0.0]`. Live-TF projection predicts pixel `(742,110)`. Using the
  live exact bounds, the nearest fixed object is foam brick at `0.1430 m`,
  leaving `0.0330 m` beyond the unchanged `0.1100 m` gate; hammer distance is
  `0.2216 m`. The revised, self-reviewed design is committed as `67c1227`.
- Implemented `[-0.85,0.35,0.0]` plus the matching configuration-test
  expectation in both host workspaces, preserving their unrelated dirty
  changes. Test-first verification failed only on the old slot value, then the
  Docker-mounted focused suite passed `7/7`; `git diff --check` passes in both
  workspaces. Recomputed live-TF projection is `(741.5,109.9)` and the minimum
  fixed-box distance remains `0.1430 m`. No launch restart or new motion was
  performed, so the currently running scene still uses its startup-time slot.

### 2026-07-14 - Hammer Software Pass Still Dragged Physically

- Retrying hammer with `GRASP_Z_OFFSET=0.0` passed the software convergence and
  grasp path, but the screenshot showed the captured hammer rotating about the
  handle-end grasp while the metal head remained on the table. Current success
  checks cover TCP motion and gripper result, not object ground clearance.
- Hammer mesh AABB is approximately `0.182 x 0.333 x 0.033 m`. Mapping the live
  grasp into the nearly 90-degree-yawed object frame places it near local long-
  axis coordinate `-0.108 m`; the AABB center is about `-0.023 m`. The grasp is
  therefore roughly `85 mm` toward the handle end from geometric center, which
  creates a large gravity moment and leaves about `0.252 m` from grasp to the
  far head end. A universal `0.20 m` lift cannot guarantee head clearance.
- For a roughly horizontal lift, prefer shifting the hammer-only top-down grasp
  `8-9 cm` toward the metal head/center of mass while staying on the wooden
  handle. Pure AABB center is a useful first candidate, but the true mass center
  lies farther toward the heavy head and may require a small follow-up shift.
  Raising hammer-only lift height to about `0.30 m` is the simpler alternative
  if horizontal orientation is not required.

### 2026-07-14 - Hammer Stopped 1.4 mm Outside Unified Gate

- The first hammer run after the global-tolerance change selected the intended
  `hammer` and profile `top_down`; the log confirmed
  `position_tolerance_m=0.0180`, so the rebuilt code was active.
- Final target was `[-0.4596,-0.6929,0.8866]`; feedback stabilized at
  `[-0.4597,-0.6859,0.9047]`, a `0.0194 m` norm dominated by `18.1 mm` high Z
  plus `7.0 mm` Y. It exceeded the gate by only `1.4 mm` and raised
  `FinalApproachConvergenceError` before `close_gripper_at_grasp`; the open
  fingers in the screenshot are therefore not a gripper-command failure.
- The target TCP Z is about `4.5 mm` below the estimated hammer-object origin,
  while the actual TCP is about `13.6 mm` above it. The generic
  `GRASP_Z_OFFSET=-0.020` accounts for the mismatch: without that offset, target
  Z becomes `0.9066`, only `1.9 mm` above observed actual Z, and the same XY
  residual gives about `7.3 mm` total error. Prefer a temporary
  `GRASP_Z_OFFSET=0.0` hammer test over weakening the unified tolerance again.

### 2026-07-14 - Final-Approach Tolerance Unified At 18 mm

- A normal banana run selected profile `centered`; final approach settled at
  `0.0137 m` versus the former `0.013 m` gate and raised
  `FinalApproachConvergenceError` before `close_gripper_at_grasp`. Target minus
  actual was dominated by about `12.1 mm` in Z; the visible open fingers were a
  consequence of the fail-closed gate, not a gripper failure.
- The user approved one unified setting. Removed the vertical-only tolerance
  reader, constant, environment variable, and executor branch. Every profile
  now uses `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M`, default `0.018`, with
  finite and non-negative validation. Vertical Z-offset isolation is unchanged.
- Design commit is `c2a5899`; implementation is synchronized in both dirty
  workspaces but intentionally not committed with unrelated existing edits.
  Direct tests pass `50/50`, related grasp regression passes `130/130`, and
  `my_course_pkg` rebuilt successfully. Installed runtime introspection reports
  `0.018` for `vertical`, `centered`, and `side`; the removed vertical-only
  environment variable no longer affects the result.

### 2026-07-14 - Vertical Defaults Implemented And Foam Brick Full Run Passed

- Added isolated vertical-profile defaults:
  `GRASP_VERTICAL_Z_OFFSET=0.0` and
  `GRASP_VERTICAL_FINAL_APPROACH_POSITION_TOLERANCE_M=0.018`. Other profiles
  retain their existing offsets and the global `0.013 m` tolerance. The plan
  now records `grasp_profile`, and executor logs the selected profile/gate.
- Design is committed in the active workspace as `337e558`; implementation is
  synchronized in both workspaces. Focused selector/planner/trajectory/executor
  regression passes `130/130`. Functional package tests pass `174`, with `1`
  skipped; the two whole-workspace lint harnesses still fail on pre-existing
  build/install and unrelated source issues. The active Docker workspace
  rebuilt `my_course_pkg` successfully with `--symlink-install`.
- A direct `ros2 run my_course_pkg grasp_demo` proved the new code was active:
  it logged `z_offset=0.0000 m` and
  `profile=vertical, position_tolerance_m=0.0180`, accepted the `0.0180 m`
  sample, closed on the brick at position `0.333`, lifted `0.20 m`, held for
  `5 s`, returned to the elevated release pose, released after the built-in
  open retry, retreated, returned home, and printed `Pick and place completed.`

### 2026-07-14 - Foam Brick Initial Approach Stopped At Table Contact

- Fresh `foam_brick` perception selected the single vertical-profile candidate.
  The plan correctly included the new elevated release pose, but execution
  never reached close, lift, hold, return, or release; it stopped during the
  initial `approach_grasp` convergence gate.
- Target TCP was `[-0.4684,-0.9396,0.9193]`; final feedback was
  `[-0.4685,-0.9310,0.9040]`. The `0.0176 m` norm consists mainly of about
  `15.3 mm` low Z and `8.6 mm` Y error versus the `0.013 m` gate.
- The screenshot initially suggested table contact, and the contemporaneous
  `moveit2_iface` log accepted every repeated target with collision-free IK.
  A later `GRASP_Z_OFFSET=0.0` trial disproved contact as the dominant cause:
  raising the target by `20 mm` raised actual Z by `19.9 mm`, while the final
  error stayed `0.0175 m` with essentially the same `+8.4 mm` Y and `-15.4 mm`
  Z components. This is a repeatable controller/feedback tracking bias, not IK
  rejection, approach speed, or a fixed table-height obstruction.
- Next safe diagnostic: reset, rerun fresh brick perception, and retry once with
  only `GRASP_INTERPOLATE_AVG_SPEED=0.05`, keeping the strict `0.013 m`
  convergence gate. Do not widen tolerance before distinguishing overshoot
  from a repeatable geometry/contact limit.
- That low-speed retry reproduced the same target, final actual pose, and
  `0.0176 m` error almost exactly, ruling out approach speed/overshoot. Next:
  keep `0.05 m/s`, temporarily set `GRASP_Z_OFFSET=0.0` to raise the vertical
  target by `20 mm`, and use `GRASP_DEBUG_STOP_AFTER_CLOSE=1` so the trial only
  verifies convergence and brick capture before any lift. Preserve the
  `0.013 m` gate.
- Historical log `/tmp/foam_brick_full_trial1_grasp_retry.log` proves that the
  same perceived object pose, `GRASP_Z_OFFSET=-0.020`, and final TCP target
  `[-0.4684,-0.9396,0.9193]` previously converged to `0.0103 m`, closed on the
  brick at gripper position `0.336`, and lifted it. The current regression is
  therefore not a changed grasp target, lift-return planning, or the new
  release clearance; it is a run-time motion/contact/controller-state change
  before close. The exact repeated `0.0176 m` plateau makes the current state
  deterministic, even though the earlier identical target succeeded.
- The historical "success" did not demonstrate stable convergence: feedback
  swept from `0.6052 m` through `0.0158 m` to a single `0.0103 m` sample, and
  the current executor closes immediately on the first sample under the
  `0.013 m` gate. It likely crossed the gate transiently while feedback/control
  was still catching up. The next safe diagnostic is a `5 s` settle timeout at
  `GRASP_Z_OFFSET=0.0`, `0.05 m/s`, and the unchanged `0.013 m` tolerance.
- That `5 s` diagnostic sent `43` repeated final-pose commands. Error settled
  at `0.0175 m` by command 7 and remained exactly there through timeout;
  target/actual were `[-0.4684,-0.9396,0.9393]` and
  `[-0.4685,-0.9312,0.9239]`. Extra settling time cannot solve it. The next
  bounded live check is close-only with the raised vertical target and a
  temporary `0.018 m` tolerance; if capture is good, implement those settings
  only for the vertical/box profile rather than weakening the global gate.
- The bounded close-only check with `GRASP_Z_OFFSET=0.0`, `0.05 m/s`, and a
  temporary `0.018 m` gate reached close and completed the three-second hold.
  Gripper contact was accepted at position `0.333`, effort `139.7`,
  `stalled=True`, essentially matching the earlier lifted-brick contact at
  `0.336`. Lift was skipped exactly as requested by
  `GRASP_DEBUG_STOP_AFTER_CLOSE=1`. This run passed on one transient
  `0.0113 m` sample with `commands=0`; the immediately logged debug pose was
  about `0.0153 m` from target, so a permanent solution should not interpret a
  single under-threshold sample as stable convergence.

### 2026-07-14 - Apple Attempt Had No MoveIt2 Interface

- A direct Apple `grasp_demo` attempt reported all four arm Action Servers and
  all six services unavailable, then failed because
  `/arm/state/current_pose` had no publisher. This happened before any
  object-specific grasp logic or motion.
- Live process inspection found the full launch, MoveIt, MuJoCo, gripper,
  robot-state publisher, and watchdog alive, but no `moveit2_iface` process.
  Its launch log shows receipt of `SIGINT/SIGTERM` at timestamp `1784031044`,
  followed by exit code `-6`; the parent launch did not respawn the child.
- The simplified `ros2 run ... grasp_demo` command removes only the three
  environment prefixes. It does not replace the current no-code cross-trial
  procedure: after `pkill -INT -x moveit2_iface`, immediately relaunch the
  standalone interface and verify one server/publisher before reset/pipeline.
- The user confirmed the exact sequence: after `pkill`, a pre-relaunch audit
  misleadingly listed one `/moveit2_iface` node from ROS daemon cache, but
  correctly reported `arm_action_servers: 0` and an unknown current-pose topic.
  Starting the standalone interface then made `grasp_demo` succeed. Treat node
  count as supporting evidence only; server and publisher gates are decisive.

### 2026-07-14 - Side Vertical Margin Default Raised To 0.12 m

- The user confirmed the release-clearance live validation succeeded and asked
  to remove all repeated runtime prefixes from normal `grasp_demo` runs.
- Changed the shared side-profile default
  `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M` from `0.05` to `0.12 m` for every
  cylindrical-can side grasp. Explicit environment override remains available;
  other grasp profiles and all horizontal clearance settings are unchanged.
- Design commit is `ff1000b`. The change is synchronized to both workspaces.
  Focused tests pass `38/38`, related regression passes `128/128`, the package
  builds, and installed code with all three variables unset reports
  `lift_return`, release clearance `0.02`, and side margin `0.12`.

### 2026-07-14 - Lift-Return Release Clearance Implemented

- The user clarified that their evaluation ends with grasp plus the five-second
  lifted hold; a teammate owns the later workflow and will merge it afterward.
- Chose the smallest compatible fix: add a default `0.02 m` world-Z release
  clearance in `lift_return` only. Return, release holds, and retreat use the
  elevated pose, while the initial `0.013 m` grasp-approach gate and
  `safe_place` behavior stay unchanged.
- The approved, self-reviewed design is committed as `19edf18`. Added the
  validated `GRASP_RETURN_RELEASE_CLEARANCE_M` configuration with default
  `0.02`, and elevated the `lift_return` return/release pose by that world-Z
  amount. Initial grasp convergence and `safe_place` are unchanged.
- Synchronized the feature patch to the Docker-mounted `-grasp-stable`
  workspace without overwriting its intentional `0.01` gripper-close threshold.
  Focused tests pass `36/36`, related regression passes `126/126`,
  `my_course_pkg` builds with `--symlink-install`, and the installed planner
  reports configured and planned return clearance `0.020 m`.

### 2026-07-14 - Tomato Lift-Return Reached Table; Return Gate Aborted Release

- With `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M=0.12`, the real tomato run
  rejected four banana-intersecting candidates and executed safe candidate 2.
- Initial approach converged at `0.0124 m`; gripper stalled on the can at
  position `0.195`; the `0.20 m` lift, five-second hold, and descent all ran.
- On `return_to_grasp`, the upright can reached the table, but TCP feedback
  stabilized at `0.0161 m` from the original grasp target, mostly in Y. The
  configured convergence tolerance is `0.013 m`, so the executor raised
  `FinalApproachConvergenceError` before `hold_before_release`, gripper open,
  or retreat.
- Root cause in `executor.py`: `_step_reaches_final_grasp()` matches both the
  initial `approach_grasp` and later `return_to_grasp`, and `execute_plan()`
  calls `converge_at_final_grasp()` for every matching move. Reusing the strict
  free-space initial-approach gate after the held object contacts the table is
  not appropriate. Do not simply widen the global gate because it also weakens
  the initial grasp approach.

### 2026-07-14 - Tomato 0.12 m Side-Envelope Plan-Only Passed

- Ran `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M=0.12 ros2 run my_course_pkg
  grasp_plan_only` on a fresh tomato perception result.
- Exact 2.5D bounds clearance rejected candidates 1, 7, 10, and 14 for banana
  intersection (`required=0.1100 m`) and retained 12 of 16 candidates.
- MoveIt accepted corridor-safe candidate 2 on the first plan-only attempt.
  The guard executed no trajectory or gripper command and restored
  `planonly=False`. The trailing rclpy `Destroyable` teardown message occurred
  after the explicit PASS and does not invalidate this result.

### 2026-07-14 - Tomato Run Stopped On Stale Final-Pose Feedback

- The user confirmed that `tomato_soup_can` was the intended target; the
  earlier wrong-target interpretation was incorrect.
- Candidate 1 reached its MoveIt pregrasp, but the Cartesian final approach
  stalled at about `0.048 m` position error versus the configured `0.013 m`
  tolerance and failed closed before gripper close or object motion. The first
  gripper-open result was rejected, but the built-in retry succeeded and was
  not the terminal failure.
- The user confirmed a default, no-override tomato run succeeded the previous
  day, so missing horizontal-grasp environment variables are not the root
  cause. The same default pose can succeed when control feedback is healthy.
- In the failed run, convergence readings plateaued at `0.0481 m`. A subsequent
  live measurement observed a `2.226 s` maximum gap on
  `/arm/state/current_pose`, longer than the `1.5 s` convergence timeout. This
  can make the executor repeatedly read a stale pose and fail even while the
  controller is still settling.
- A later retry after restarting the full launch again selected the same
  candidate and failed at `0.0474 m`; target-to-actual delta was approximately
  `[-18.6, -19.0, +39.3] mm`, nearly identical to the preceding high stop. This
  disproves restart-as-fix and makes a repeatable default oblique-approach
  contact/tracking limit more likely than a one-off stale sample.
- The user then visually identified the physical cause: the lower gripper
  contacted the adjacent banana during the tomato approach. Candidate 1
  approaches from the banana side and the physical contact explains the
  repeatable roughly `+40 mm` Z stop.
- The planner incorrectly logged `side approach clearance ... rejected=0`.
  Side clearance currently uses only `0.05 m` vertical margin, while the same
  gripper's measured TCP-to-lowest-finger reach is about `0.104 m` and the
  round-top profile already uses `0.12 m`. The side 2.5D TCP-corridor model
  therefore under-models the below-TCP gripper envelope and can miss a banana
  collision. Next diagnostic: use
  `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M=0.12`, verify candidate 1 is rejected,
  and allow only a remaining candidate approaching from the clear side.
- After the user restarted the launch, ROS showed two `moveit2_iface` nodes and
  two publishers on `/arm/state/current_pose`: one standalone interface plus
  one from the full simulation launch. Do not retry motion until the standalone
  duplicate is stopped and the topic reports exactly one publisher.
- Process ancestry confirmed the standalone
  `ros2 launch arm_api2 moveit2_iface.launch.py` started first, followed two
  minutes later by `cell_small_full_mujoco_moveit.launch.py`; the latter already
  includes the same `arm_api2` launch. The previously prescribed per-trial
  `pkill` plus standalone relaunch workflow was therefore ownership-ambiguous
  and is retired.
- Canonical ownership is now the full launch only. Do not separately launch
  `arm_api2 moveit2_iface.launch.py`, and do not restart only the interface.
  Before each independent formal motion trial, stop and restart the entire
  `cell_small_full_mujoco_moveit.launch.py`, then verify one current-pose
  publisher and one move action server before motion.
- `/reset_sim` only resets MuJoCo `qpos/qvel/ctrl`, its simulation-side target,
  active simulation trajectory, and gripper adapter. `arm_api2` has no
  `/reset_sim` integration, so the service does not clear the live
  `moveit2_iface` servo/interpolation state. A reset alone is therefore not the
  canonical boundary between independent formal motion trials, even though it
  cannot create a second MoveIt node.
- The later retry log warned of multiple gripper action servers. Full graph
  inspection found one arm interface but two each of
  `robotiq_2f_urcap_adapter`, `robot_state_publisher`, and
  `servo_watchdog_node`. The older copies survived the prior launch shutdown
  as processes parented directly by container PID 1. A launch-level restart is
  therefore not a clean boundary in this container state; restart the Docker
  container before the next diagnostic and verify all duplicates are gone.
- Then reset and run a fresh tomato pipeline before retrying plain
  `ros2 run my_course_pkg grasp_demo`; the failed approach may have contacted
  or disturbed the can.

### 2026-07-14 - Global Lift-Return Mode Implemented

- Default `GRASP_EXECUTION_MODE=lift_return` now skips safe-place selection and
  horizontal transfer for every category. It grasps, lifts `0.20 m`, actively
  holds `5 s`, returns to the original grasp pose, releases, and retreats to
  pregrasp.
- Legacy remote placement remains available only through
  `GRASP_EXECUTION_MODE=safe_place`; invalid mode values fail at configuration
  load. Existing approach-clearance and executor/gripper failure checks remain.
- TDD and regression verification passed: `71/71` focused, `128/128` related,
  and `149 passed, 1 skipped` across functional package tests. The two whole-
  repository lint harness failures are pre-existing. `my_course_pkg` rebuilt,
  and installed-runtime inspection confirmed a `0.200 m` lift, `5.0 s` hold,
  exact return, no transfer step, and preserved legacy transfer mode.
- Active-workspace implementation commit is `2859cf6`; the same source and
  tests are synchronized here. Next live qualification is foam-brick
  lift-return Trial 1 after an independent reset and fresh perception pipeline.

### 2026-07-14 - Global Lift-Return Design Approved

- The user requested that every object category stop searching for a remote
  placement location and instead evaluate stable grasp plus a `0.20 m` vertical
  lift and `5 s` hold before returning the object to its original location.
- The design adds default `GRASP_EXECUTION_MODE=lift_return`, keeps legacy
  `safe_place` as opt-in, removes safe-place search and horizontal transfer from
  the default path, and preserves approach clearance and existing failure
  checks.
- The active-workspace design is committed as `5cc2292`; implementation later
  completed in `2859cf6`.

### 2026-07-14 - Foam Brick Attempt Stopped At Placement Search

- Fresh perception selected `foam_brick`, FoundationPose succeeded, and the
  vertical profile prepared its grasp candidate normally.
- Before any robot motion, safe-placement planning rejected all `36/36` points
  in the default world-frame region `x=[-0.55,-0.20]`,
  `y=[-0.85,-0.50]`; five non-target obstacles left at most about `0.067 m`
  clearance versus the required `0.080 m`. This attempt does not count.
- Recommended minimal experiment is to expand only the upper Y bound to
  `GRASP_PLACE_Y_MAX=-0.30`. Current scene geometry then offers a point near
  `(-0.39,-0.34)` with about `0.206 m` obstacle clearance; do not lower the
  clearance threshold unless this wider-region attempt fails.
- The user approved this runtime-only wider-region experiment. Rerun from an
  independent reset and fresh foam-brick pipeline; count it as Trial 1 only if
  the complete pick-transfer-place sequence succeeds.
- The first retry still logged `y=[-0.850,-0.500]`, proving
  `GRASP_PLACE_Y_MAX=-0.30` was not present in the `grasp_demo` process. It
  again stopped before motion and does not count. Prefix the variable directly
  on the `ros2 run` command so shell/session scope cannot drop it; because no
  motion occurred, the same fresh perception result can be reused for this
  Trial 1 execution retry.
- A later run was not a foam-brick retry: it loaded `apple`, used an effective
  Y maximum of `-0.400`, and successfully selected `[-0.29,-0.44]` with
  `0.2009 m` clearance. It then moved to Apple and stopped at close validation
  (`actual=0.03765`, minimum `0.040`) before lift. This proves placement search
  worked; the run does not count for foam brick. Reset is now required, followed
  by a fresh pipeline explicitly confirmed as `foam_brick` and a grasp command
  prefixed with `GRASP_PLACE_Y_MAX=-0.30`.
- The corrected foam-brick run did use `Y_MAX=-0.30` and selected exactly
  `[-0.39,-0.34]` with `0.2061 m` obstacle clearance. Grasp, close, and the
  `0.20 m` lift succeeded. MoveIt then timed out/faulted while planning
  `transfer_to_drop_high` at the same vertical grasp orientation, before any
  descent or release. The issue is transfer-pose reachability, not safe-place
  detection. Reset is required because the process exited while holding the
  lifted brick. This run does not count. A practical next runtime experiment is
  to restrict the upper Y bound to `-0.43`, selecting the farther point near
  `[-0.39,-0.49]` with about `0.0916 m` clearance; preflight reachability would
  be the more robust code-level follow-up if that also fails.
- Cross-target environment contamination was then observed: an Apple run
  inherited a nondefault placement Y maximum (an earlier log already showed
  persistent `-0.40`) and failed at `transfer_to_drop_high`. Apple previously
  succeeded with the default `GRASP_PLACE_Y_MAX=-0.50`, selecting near
  `[-0.29,-0.54]`; the expanded region can instead select near
  `[-0.29,-0.44]`, which is not reliably reachable for the held orientation.
  Before returning to Apple, unset all temporary `GRASP_PLACE_*` overrides and
  verify the log shows `y=[-0.850,-0.500]`. Keep foam-only overrides inline on
  the foam command rather than exporting them across experiments.
- The user clarified that the later `Y_MAX=-0.50` run was intentionally another
  foam-brick attempt, not an Apple restore. It correctly loaded `foam_brick`,
  but the brick was displaced and tilted about `91.74 deg`; the default narrow
  placement region again had no safe point and planning exited before motion.
  Reset and rerun a fresh brick pipeline, then use the proposed foam-only
  `GRASP_PLACE_Y_MAX=-0.43` experiment to select near `[-0.39,-0.49]` while
  keeping the `0.080 m` clearance gate.

### 2026-07-14 - Apple Complete At 2/2; Foam Brick Is Next

- The user confirmed complete Apple Trial 2 succeeded again. Apple therefore
  completes the fixed-position `round_top` representative at `2/2`, including
  stable grasp, lift, transfer, placement, release, and return.
- The proven close minimum is now the repository default `0.04` in both
  workspaces via one-line commit `6979c00`; no environment override is needed.
  The unchanged gripper regression passed `14/14` and `my_course_pkg` rebuilt.
- Follow the representative order already defined in the roadmap:
  `foam_brick` (`box -> vertical`), then `banana` (`centered`), then `hammer`
  (`tool_top -> top_down`). Foam brick is next because its regular box geometry
  isolates vertical-profile behavior before the elongated representatives.
- `pipeline` reads an interactive instruction. For this target enter exactly
  `pick up the foam brick`; keep candidate and mask overrides unset.

### 2026-07-14 - Complete Apple Trial 1 Passed

- After independent reset and fresh perception, the complete Apple experiment
  was rerun with runtime-only `GRASP_GRIPPER_CLOSE_MIN_POSITION=0.04` and all
  debug stops cleared.
- The user confirmed the full run formally passed and described the grasp as
  very stable and ideal. The adjusted close threshold therefore removed the
  false rejection without degrading the observed grasp.
- Complete Apple qualification is `1/2`. Next: another independent reset and
  fresh full Trial 2 using the same runtime override.

### 2026-07-14 - Apple Close False Rejection Diagnosed

- The first full Apple run selected candidate 1, reached pregrasp, converged at
  final grasp to `11.4 mm`, and visibly wrapped the fingers around the Apple.
- The close action then reported `actual_position=0.0470588`, `effort=139.7`,
  and `stalled=True`. The generic `0.050` minimum rejected this useful contact
  by about `0.003`, so execution correctly stopped before hold/lift even though
  the visual grasp was good. The run does not qualify as a complete success.
- The user approved the minimal runtime-only override
  `GRASP_GRIPPER_CLOSE_MIN_POSITION=0.04` and skipped further written review.
  Design commit is `66f859d`; the global code default remains `0.050` and all
  other gripper/release checks remain active. Reset and rerun a fresh pipeline;
  do not resume from the held-object state.

### 2026-07-14 - User Authorized Direct Full Apple Experiment

- The user explicitly chose not to treat the observed `1.262-1.988 s` maximum
  render gaps as a blocker for this simulation student project and requested a
  complete experiment first, with failures handled if they occur.
- The earlier staged Phase B pregrasp-only sequence is no longer required
  before this run. Proceed directly to the full Apple pick-transfer-place path
  after an independent reset and fresh successful pipeline, with every
  `GRASP_DEBUG_STOP_*` variable cleared.
- Preserve the project-wide two-run policy: the complete experiment qualifies
  after two independent successes. Fresh-data chaining and normal geometric,
  planning, gripper-result, and convergence failures remain active; no code or
  safety threshold is being changed for this workflow decision.

### 2026-07-14 - Guarded Phase B Entry Approved

- The documented numerical readiness gate is satisfied by two independent
  low-rate cadence trials: `/joint_states` averaged `18.197 Hz` and
  `18.301 Hz`, both at or above the required `18 Hz`. The threshold does not
  need to be removed.
- Observed maximum message gaps of `1.262 s` and `1.988 s` remain a known
  software-render risk. Phase B therefore starts only with the existing staged
  pregrasp debug stop and visual inspection in simulation; no Cartesian final
  approach, gripper, lift, transfer, placement, or release is authorized.
- Each of the two formal Phase B trials still requires independent reset,
  visibly open reset gripper, fresh Apple pipeline, exact-corridor plan-only
  pass, then `GRASP_DEBUG_STOP_AT_PREGRASP=1`. Inspect the held pose, press
  Ctrl-C, and reset before the next trial because the camera has moved.

### 2026-07-14 - Low-Rate Apple A3 Completed At 2/2

- Formal Trial 2 began with another successful independent `/reset_sim` and a
  fresh 1280x720 pipeline that naturally selected Apple, produced a `7517 px`
  mask, and completed FoundationPose.
- Plan-only loaded five obstacles, used `method=exact_bounds_box_2p5d`, retained
  all `8/8` corridor-safe candidates, and accepted candidate 1 on the first
  MoveIt planning attempt.
- Cleanup restored `planonly=False`; no trajectory or gripper command executed.
  Together with Trial 1, low-rate Apple A3 is complete at `2/2`. Phase A is
  complete; next is the explicit Phase B control-health/motion-entry decision.

### 2026-07-14 - Low-Rate Apple A3 Trial 1 Passed

- A successful independent `/reset_sim` was followed by a fresh 1280x720 Apple
  pipeline: natural VLM target `apple`, `7517 px` mask, and successful
  FoundationPose output.
- The dedicated non-executing A3 command loaded five obstacles, used
  `method=exact_bounds_box_2p5d`, retained all `8/8` corridor-safe candidates,
  and MoveIt plan-only accepted candidate 1 on its first attempt.
- Cleanup restored `planonly=False`; no trajectory or gripper command executed.
  The final `Destroyable` message was an asynchronous shutdown warning after
  the explicit pass and does not invalidate the run. A3 is now `1/2`.

### 2026-07-14 - First Low-Rate A3 Retry Did Not Qualify

- The independent `/reset_sim` succeeded, but the required fresh pipeline
  stopped at LLM/SAM2 because neither `VLM_API_KEY` nor `OPENAI_API_KEY` was
  present in that shell.
- Because the commands were separated instead of success-chained, the shell
  still launched `grasp_plan_only`. Its no-motion behavior passed in isolation:
  `method=exact_bounds_box_2p5d`, `8/8` candidates retained, candidate 1 accepted
  on the first MoveIt plan-only attempt, cleanup restored `planonly=False`, and
  no trajectory or gripper command executed.
- This run does not count toward A3 because planning consumed the pre-existing
  pose result rather than a fresh successful post-reset pipeline result. A3
  remains `0/2`; rerun with a privately set key and success-chain pipeline to
  plan-only with `&&`.

### 2026-07-14 - Low-Rate Apple A1 Completed At 2/2

- Formal Trial 2 started with an independently logged successful `/reset_sim`,
  followed by a fresh 1280x720 RGB-D capture and natural VLM target `apple`.
- The verifier did not make a usable selection, so the implemented global
  policy automatically selected candidate 1 by highest detection score. The
  selected Apple mask was `7517 px`; no manual override was present.
- FoundationPose succeeded. Live scene/TF comparison measured `0.252 mm`
  mesh-center translation error and `0.312 deg` rotation delta, below the
  `10 mm` A1 gate. No planner, trajectory, arm, or gripper command ran.
- Together with Trial 1, low-rate Apple A1 is complete at `2/2`. Next: rerun
  A3 twice through the dedicated non-executing `grasp_plan_only` entrypoint.

### 2026-07-14 - Low-Rate Apple A1 Trial 1 Passed

- An independently logged `/reset_sim` succeeded, followed by a fresh 1280x720
  RGB-D capture and natural VLM target `apple`.
- SAM2 returned two Apple-labeled candidates. The enabled VLM verifier selected
  candidate 1 automatically with `high` confidence; no manual override or
  score fallback was needed. The correct Apple mask was `7515 px` with
  grounding score `0.828052`.
- FoundationPose succeeded. Live scene/TF comparison measured `0.226 mm`
  mesh-center translation error and `0.429 deg` rotation delta, below the
  `10 mm` A1 gate. No planner, trajectory, arm, or gripper command ran.
- Formal low-rate A1 count is `1/2`; next run must start with another independent
  reset and fresh pipeline.

### 2026-07-14 - Highest-Detection-Score Fallback Implemented

- User-reviewed clarifications were committed as design `e4ee1b3`; manual plan
  `98a15e6` replaced the unavailable `writing-plans` skill. Implementation is
  committed as `26347b7` and synchronized semantically to active `/home/ws`.
- Multi-mask selection preserves a valid high-confidence verifier result. With
  verification enabled but unusable, it ranks finite `grounding_score`, then
  finite `score`, then lower candidate index and records
  `highest_detection_score_fallback`. Explicit
  `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0` remains fail-closed.
- TDD red was 7 new failures with 8 existing passes; green was `15/15` focused.
  Full package functional tests produced `145 passed, 1 skipped`; two repository
  lint harnesses still fail on 1861 existing whole-workspace/build-tree issues.
  The changed files add no new flake8 issue. `my_course_pkg` builds successfully.
- A read-only live-format tomato smoke used a low verifier result and selected
  candidate 1 by grounding score `0.733585`, recording the fallback source and
  a `10574 px` mask. No API request, FoundationPose request, planner, trajectory,
  arm, or gripper command ran in this smoke. Next: restart Apple A1 at `0/2`.

### 2026-07-14 - User Chose Highest-Detection-Score Automatic Fallback

- The user explicitly rejected a stricter second-stage verifier for this
  student project. Multi-mask handling must remain fully automatic: use a valid
  verifier selection when available; when it is uncertain, select the matching
  candidate with the highest detection/grounding score.
- Do not add a second VLM call, an Apple/lemon-specific rule, a manual mask
  requirement, or an extra confidence-margin gate. Record the fallback source
  and selected candidate so the result remains inspectable after the pipeline.
- Design is committed as `0263f2c` in
  `2026-07-14-highest-score-mask-fallback-design.md`; implementation waits for
  the user's written-spec review. A3 remains gated behind fallback
  implementation plus A1 `2/2`.

### 2026-07-14 - Formal Low-Rate A1 Reproduced Verifier Uncertainty

- The next formal pipeline again returned two Apple-labeled masks: candidate 1
  was the red Apple (`7517 px`, grounding `0.8210`) and candidate 2 was the
  yellow lemon (`3914 px`, grounding `0.6593`).
- The existing single-call verifier returned `selected_index: null` and
  `confidence: low`; FoundationPose and all arm motion failed closed as
  intended. A1 remains `0/2`.
- Across equivalent captures the verifier has now produced low, high, then low.
  Pause live retries and resume design approval for a target-generic structured
  second-stage verifier; do not use score-margin or fixed-index selection.

### 2026-07-14 - Existing Verifier Passed Automatic Non-Qualifying Smoke

- A later fresh pipeline, without `FOUNDATIONPOSE_MASK_INDEX`, again produced
  two Apple-labeled masks. The existing VLM verifier automatically selected
  candidate 1 with `confidence: high`; metadata records
  `selected_by: vlm_candidate_verifier` and the red Apple mask area was
  `7523 px`.
- FoundationPose succeeded; live truth comparison measured `0.239 mm`
  mesh-center translation error and `0.239 deg` rotation delta.
- This run did not show an independent `/reset_sim`, so it is a non-qualifying
  smoke rather than A1 Trial 1. Defer verifier changes and run two independent
  reset + fresh-capture trials; resume the generic verifier design only if an
  automatic trial fails closed again.

### 2026-07-14 - Low-Rate Apple A1 Attempt Failed Formal Mask Gate

- After a successful `/reset_sim`, the low-rate pipeline received a fresh
  1280x720 RGB-D pair, selected `apple`, and stopped fail-closed before
  FoundationPose because SAM2 returned two target-labeled masks.
- `mask_candidates.json` and the overlay show candidate 1 is the red apple
  (`7516 px`, grounding score `0.8202`); candidate 2 is a yellow non-target
  object (`3918 px`, grounding score `0.6602`). The automatic verification
  correctly returned `selected_index: null`, `confidence: low`.
- A diagnostic rerun with `FOUNDATIONPOSE_MASK_INDEX=1` completed FoundationPose.
  The Apple mesh-center translation error was `0.232 mm` and rotation delta was
  `0.312 deg`, so the pose sub-gate passed comfortably below `10 mm`.
- Formal A1 still failed because its contract requires the unique correct mask
  without a manual index. A1 remains `0/2`; fix automatic safe selection before
  starting another qualifying attempt.
- User clarified that multiple raw SAM2 candidates are acceptable when the VLM
  verifier automatically selects the correct candidate with `high` confidence;
  this counts as A1 and does not require `candidate_count == 1`.

### 2026-07-14 - Low-Rate RGB-D Cadence Passed Twice

- Changed only the active runtime config and focused assertion from
  `render_fps: 20` to `0.2`. Focused tests passed `7/7`; simulator/scene-
  clearance regression passed `65/65`; `git diff --check` passed.
- Two independent simulator launches each sustained all three state topics at
  `>=18 Hz` average. Trial 1 ended at `18.197/18.189/18.160 Hz`; Trial 2 ended
  at `18.301/18.292/18.282 Hz` for joint/scene/bounds.
- Trial 1 received 7 matched RGB-D frames in 36.141 s; Trial 2 received 8 in
  36.131 s. All were 1280x720 `bgr8`/`32FC1` with unique matching timestamps.
- Both trials published six finite positive bounds markers with reliable/
  volatile QoS, no forbidden camera/pointcloud topics, and no bounds errors.
- Maximum state-message gaps were about `1.262 s` and `1.988 s`; retain this
  evidence for functional control-health review before Phase B.
- No pipeline, MoveIt goal, arm, Cartesian, or gripper command ran. Trial 2 is
  left running for the user-driven A1 sequence.

### 2026-07-14 - Low-Rate RGB-D Experiment Approved

- Confirmed the active workspace already uses one 1280x720 `camera_orbbec`
  RGB-D stream with pointcloud disabled; the only production config change for
  the experiment will be `render_fps: 20 -> 0.2`.
- Confirmed ROS camera scheduling consumes `render_fps` as a float; the separate
  integer-clamped MuJoCo fields are unused by `_sim_loop` camera scheduling.
- Updated and committed design `88469db`. Two independent live cadence trials,
  two A1 trials, and two A3 trials are required; no motion is authorized.
- If fixed `>=18 Hz` still fails, restore the config and separately design a
  functional control-health replacement. Do not proceed with no health gate.

### 2026-07-14 - Project-Wide Two-Run Policy Implemented

- Changed the generic evaluator default and qualification streak from three to
  two while preserving failure reset and explicit longer diagnostic batches.
- Updated focused tests for one-success failure, two-success qualification,
  failure reset, later recovery, and summary output. Focused tests passed
  `10/10`; related grasp/evaluator regression passed `124/124`.
- `colcon build --packages-select my_course_pkg --symlink-install` passed in
  `friendly_visvesvaraya`. Runtime and test files match across both workspaces.
- Updated active normative documentation; older three-run design/plan text is
  explicitly superseded, while factual historical three-run results remain.
- No simulator, perception, MoveIt goal, arm, Cartesian, or gripper command ran.
  Phase B remains locked pending the separately reviewed `>=18 Hz` recovery.

### 2026-07-14 - Project-Wide Two-Run Policy Approved

- The user changed all current and future repeated experiment qualifications
  from three consecutive successes to two. Failures still reset the streak;
  every geometry, collision, convergence, gripper, fresh-data, and fail-closed
  safety gate remains unchanged.
- Added and committed design `04fa14c`. It supersedes older normative three-run
  requirements without rewriting historical results.
- The two completed Apple A3 trials now satisfy `2/2`, so A3 is complete.
- Phase B still cannot start: real motion retains the `>=18 Hz` control-loop
  and `/joint_states` health gate, while the last accepted live rate is about
  `3.43 Hz`. Implement/verify the policy first, then design cadence recovery.

### 2026-07-14 - Apple A3 Formal Trial 2 Passed

- The user confirmed the Trial 2 plan-only run was immediately preceded by the
  required independent simulator reset and fresh Apple pipeline.
- Together with the recorded plan-only pass, Trial 2 counts as the second of
  three required successes. Next: perform the complete independent Trial 3.

### 2026-07-14 - Trial 2 Plan-Only Stage Passed; Preconditions Unconfirmed

- The Trial 2-labeled plan-only output selected Apple, loaded five obstacles,
  used `method=exact_bounds_box_2p5d`, retained all eight candidates, and
  accepted candidate 1 on the first MoveIt plan-only attempt.
- Cleanup restored `planonly=False`, and the run explicitly reported no
  trajectory or gripper command.
- The supplied excerpt begins at the plan-only command and does not show the
  required independent `/reset_sim` plus fresh Apple `pipeline`. Confirm those
  occurred immediately beforehand before counting this as Formal Trial 2.

### 2026-07-14 - Apple A3 Formal Trial 1 Passed

- The matching plan-only run loaded five non-target obstacles and reported
  `method=exact_bounds_box_2p5d`; all eight Apple candidates remained
  corridor-safe.
- MoveIt plan-only accepted pre-grasp candidate 1 after one attempt and emitted
  `A3 PLAN-ONLY PASS`. The guard was restored to `planonly=False`.
- The run explicitly reported that no trajectory or gripper command executed.
  Trial 1 counts as the first of three required independent successes.
- Next: reset the simulator and repeat fresh perception plus plan-only for
  formal Trial 2.

### 2026-07-14 - Apple A3 Trial 1 Perception Ready

- After the repeated Trial 1 reset, the fresh pipeline naturally selected
  `apple` without a candidate override. SAM2 mask area was `7523 px`, and
  FoundationPose succeeded and saved a fresh `pose_result.json`.
- Do not reset before the matching formal `grasp_plan_only` run. This trial is
  still incomplete until plan-only passes and reports cleanup to
  `planonly=False`.
- The same exposed VLM credential was pasted again; it must be revoked and must
  not be copied into the repository or handoff. Plan-only does not need it.

### 2026-07-14 - Apple A3 Trial 1 Pipeline Must Be Repeated

- The post-reset pipeline completed RGB-D, mask, and FoundationPose stages, but
  `VLM_CANDIDATE_OVERRIDE="tomato soup can"` forced the target to the can.
- This run does not count toward the three Apple A3 plan-only trials; no
  `grasp_plan_only` command was run.
- The VLM credential was pasted into chat. Revoke/rotate it, do not record it in
  the repository, then reset and rerun with the candidate override unset.

### 2026-07-14 - Exact Bounds-Box Corridor Implemented And Smoked

- Extended each scene obstacle with the full published bounds transform and
  half extents. Marker pose and TF are composed and checked as rigid transforms;
  invalid scale/quaternion/TF or non-world-aligned local Z fails closed.
- Replaced shared round-top/side circle clearance with Z-slab clipping plus
  exact XY segment-to-rectangle distance. The fixed numeric epsilon is `1e-9 m`;
  safe placement still uses the conservative circumscribed circle.
- Added marker/TF, point/segment/rectangle, decisive Z-correlation, banana false
  rejection, true face/corner, yaw, epsilon, missing-geometry, and profile
  regression tests. Planner passed `56/56`; related grasp tests passed
  `124/124`; `my_course_pkg` build passed.
- Synchronized planner and test files byte-for-byte between both workspaces.
- A live non-qualifying smoke loaded five obstacles, retained all eight Apple
  candidates, and MoveIt plan-only accepted candidate 1. Cleanup restored
  `planonly=False`. TCP delta was about `1e-9 m`; maximum bounds-center drift
  was about `15.8 um`, consistent with passive simulator settling. No trajectory
  or gripper command executed.
- Formal A3 is still pending three independent reset + fresh Apple pipeline +
  plan-only trials; the user must provide the VLM credential in their terminal,
  and it must not be stored in the repository or handoff.
- Formal Trial 1 `/reset_sim` returned success after the smoke. Do not reuse the
  pre-reset perception output; wait for the user to run a fresh Apple pipeline.

### 2026-07-14 - Exact OBB Corridor Design Ready For Review

- Confirmed the Apple A3 rejection is caused by reducing banana's long AABB to
  a circumscribed XY circle, not by FoundationPose, selector, or MoveIt.
- The captured geometry has about `0.1207 m` point-to-rectangle clearance
  versus `0.1100 m` corridor plus margin, while the circle method requires
  about `0.2148 m` center distance and rejects all eight candidates.
- The user chose the long-term random-position-compatible fix: retain the full
  bounds transform and half extents, clip the approach by the box Z slab, then
  compute exact XY segment-to-rectangle distance. Keep safe placement's circle,
  all safety thresholds, fail-closed handling, and both profile paths.
- Added and committed the design as `c3d3edb`. Per the design gate, no runtime
  code, slot, threshold, simulator, planning goal, arm, or gripper change was
  made in this step.

### 2026-07-14 - Camera TF Fixed; Apple Slot Is Next Safety Gate

- Added two tests proving camera TF publishes with both pointcloud flags false
  and preserves the existing camera-frame adjustment. Simulator/scene tests
  passed `67/67`.
- Moved TF publication from the optional pointcloud worker to the rendered-
  camera path and committed only those focused hunks plus tests as `4b4088e`.
- Restarted simulation. Live `world -> camera_orbbec` TF is continuously
  available; RGB and depth topics remain available; the pointcloud topic
  remains absent.
- A post-fix plan-only smoke loaded all five non-target AABBs and produced 8
  Apple candidates. All were rejected before any MoveIt goal because the
  banana corridor distance was about `0.175 m`, below the conservative
  `0.215 m` requirement. `planonly=False` cleanup succeeded.
- Proposed next safe slot for Apple is `[-0.30, 0.00, 0]`; it balances estimated
  conservative clearance to banana, hammer, and the random sixth slot without
  weakening any collision threshold. Simulation is running; no motion occurred.

### 2026-07-14 - A3 Entrypoint Implemented; Camera TF Gate Found

- Added and committed `grasp_plan_only` as `c4cdac4`. It enables server
  plan-only before perception/planning, verifies corridor-safe pregrasp
  candidates through MoveIt with fallback, and always restores plan-only false.
- Added 10 focused tests including cleanup failures and static prohibition of
  executor, Cartesian, gripper, and return-home calls. Related grasp regression
  passed `102` tests; `my_course_pkg` build and ROS executable discovery pass.
- The first fresh pipeline attempt captured RGB-D but lacked the external VLM
  credential; the user is rerunning it separately. No credential was stored.
- A non-qualifying smoke run used prior Apple output and failed closed before
  any MoveIt goal because camera TF was missing. `planonly=False` cleanup passed.
- Confirmed `_publish_camera_tf()` is called only by the pointcloud worker.
  Disabling pointcloud for camera cadence therefore unintentionally removes
  `camera_orbbec` TF even though RGB-D still publishes. Simulation is currently
  running; no arm, Cartesian, gripper, or object motion was commanded.

### 2026-07-14 - Apple A3 Plan-Only Design Committed

- Relaxed the `>=18 Hz` requirement for static A3 planning only; retained it as
  a control-loop and `/joint_states` health check before real motion.
- Verified existing `grasp_demo` and `GRASP_DEBUG_STOP_AT_PREGRASP=1` are unsafe
  A3 entrypoints because they can execute motion.
- Added committed design `3026d3d` for a dedicated `grasp_plan_only` command
  that performs perception, round-top/corridor filtering, and MoveIt pregrasp
  planning with server plan-only enabled, while excluding executor, Cartesian,
  gripper, and return-home paths.
- No runtime code, perception pipeline, MoveIt goal, arm motion, or gripper
  command was run in this design step. Simulator remains stopped.

### 2026-07-13 - Single-Camera/Resolution Experiments Completed

- Active workspace Stage 1 retained only `camera_orbbec` RGB+depth at 1280x720
  and disabled pointcloud. Live state topics improved from about `1.48 Hz` to
  `3.43 Hz`; camera render remained about `212-293 ms` per frame.
- Stage 2 used single-camera 640x480. State topics reached only about `3.77 Hz`
  and render remained about `195-251 ms`; the configuration was restored to
  1280x720 after measurement.
- Discovered perception intrinsics are hard-coded for 1280x720, so 640x480 is
  not a safe config-only optimization. The pipeline waits for only one RGB-D
  frame with a 30-second timeout, while `render_fps` already controls a separate
  camera cadence in `_sim_loop`.
- Added and committed low-rate RGB-D design `1a961e7`, recommending single
  1280x720 camera at 0.2 Hz before any thread/snapshot implementation.
- Simulator was stopped after restoring the active config. No pipeline, A3,
  MoveIt goal, arm, or gripper command was run.

### 2026-07-13 - Camera Cadence Code Path Verified And Designed

- Verified pipeline subscribes only to `/camera_orbbec/color/image_raw` and
  `/camera_orbbec/depth/image_raw`; `camera_third_person` is not consumed.
- Verified both configured cameras render RGB+depth in the same `_sim_loop`
  that performs `mj_step()` and publishes joint/scene/bounds state.
- Added and committed design `c460666`: first retain only `camera_orbbec` at
  1280x720, then try 640x480 only if needed, and require a separately reviewed
  immutable-snapshot design if both configuration stages miss `18 Hz`.
- No simulator configuration, runtime code, perception, MoveIt goal, arm, or
  gripper behavior was changed in this design-only step.

### 2026-07-13 - Clearance Bounds Implemented; Cadence Gate Still Blocked

- Added cached visual+collision geometry parsing, live MuJoCo world-axis AABB
  computation, and atomic reliable/volatile `/scene_clearance_bounds`
  publishing while preserving `/scene_description` pose semantics.
- Migrated round-top, side, and safe-placement clearance consumers to the new
  bounds topic with strict scale validation, conservative XY radii, and
  fail-closed round-top behavior; changed evaluator qualification to three
  consecutive full successes.
- Verified 65 simulator tests and 92 grasp/evaluator tests (`157` total), plus
  `colcon build --packages-select my_course_pkg --symlink-install`.
- Live six-object validation matched independently transformed OBJ vertices
  for Apple, banana, and hammer to numerical precision. The bounds profiler
  reported median `2.175 ms`, p95 `4.756 ms`, max `136.554 ms` over 100 samples.
- `/scene_clearance_bounds`, `/scene_description`, and `/joint_states` all ran
  at the same approximately `1.48 Hz`; camera/software-render stages dominate
  the loop. Because the approved gate requires `>=18 Hz`, Apple A3 remains
  blocked. No perception pipeline, MoveIt goal, arm motion, or gripper command
  was run.

### 2026-07-13 - Scene Clearance Plan Review Passed

- External code-level review approved the implementation plan for execution.
- Clarified the `summarize_results()` streak integration and output, preserved
  placement orientation semantics after sharing the raw OBJ loader, added an
  old-topic residual scan, and documented near-limit performance handling.
- No implementation code, simulator restart, MoveIt planning, arm motion, or
  gripper command was run during this review update.

### 2026-07-13 - Scene Clearance Implementation Plan Ready

- User approved the formal scene-clearance/three-run design.
- Added a task-by-task TDD implementation plan covering geometry cache parsing,
  live MuJoCo AABBs, atomic ROS publishing, all three planner consumers, the
  three-success evaluator rule, topic-only performance gates, and controlled
  cross-workspace synchronization.
- The plan keeps Apple A3 and all motion blocked until offline tests, build,
  live bounds-topic validation, and synchronization all pass.
- No implementation code, simulator restart, MoveIt planning, arm motion, or
  gripper command was run while writing the plan.

### 2026-07-13 - Scene Clearance Bounds Design Committed

- User approved a separate `/scene_clearance_bounds` MarkerArray while keeping
  `/scene_description` body-origin/pose semantics unchanged.
- The approved design computes atomic world-axis AABBs from cached OBJ vertices
  and live MuJoCo body poses, migrates round-top/side/safe-placement consumers,
  and uses `0.5 * hypot(size_x, size_y)` for the conservative XY radius.
- Stability qualification is now designed as three consecutive full successes
  for representatives, remaining category objects, and randomized positions;
  the five-second lift hold remains unchanged.
- Added and committed the formal design spec as `d794b7c`; no implementation,
  MoveIt planning, trajectory, arm motion, or gripper command was run.

### 2026-07-13 - Apple Phase A1/A2 Passed

- Ran three independent simulation resets followed by fresh `pick up the
  apple` perception pipelines. All three selected one Apple mask automatically
  without `FOUNDATIONPOSE_MASK_INDEX`; mask areas were 7518, 7517, and 7517 px.
- Compared the FoundationPose Apple mesh geometry center against live
  `/scene_description` truth using the configured camera convention and TF.
  Three-dimensional errors were 0.266, 0.263, and 0.245 mm, well below the
  10 mm A1 gate; roll/pitch and object-local `+Z` were repeatable.
- The Apple mask touches the lower image boundary (`bbox y_max=719.4/720`), so
  the object is partially cropped. This is recorded as a visibility risk, but
  it did not cause misselection or pose instability in the fixed scene.
- Ran the pure A2 selector against `/home/ws/grasps/013_apple`: 5013 raw
  grasps expanded to 40104 round-top symmetry variants; 11046 passed all hard
  filters and the expected top eight were returned.
- All eight candidates passed: world-down angle 0.092-0.773 deg,
  geometry-center plane offset 0.459-2.191 mm, normalized height
  0.0497-0.0822, estimated width 74.871 mm, opening margin 10.289 mm, and
  finger clearance 123.5-125.8 mm.
- Archived A1 captures and the A2 JSON/overlay under the active workspace's
  ignored `outputs_local/apple_a1/` directory. No MoveIt planning, trajectory,
  arm motion, or gripper command was run.

### 2026-07-13 - Shared Round-Top Approach Corridor Implemented

- Replaced the side-only approach filter with a profile-independent corridor
  implementation using explicit per-profile configuration.
- Added independent round-top enable/require-scene/radius/clearance/vertical
  margin settings. Round-top now loads `/scene_description` even when
  scene-aware placement is disabled and fails closed when the scene is missing
  or a non-target marker cannot be converted into a usable obstacle.
- The round-top vertical envelope defaults to `0.12 m`, covering the measured
  `0.104 m` TCP-to-lowest-finger reach; a regression test enforces this lower
  bound so tabletop obstacles are not skipped by an undersized Z margin.
- Preserved side-grasp missing-scene behavior and existing near/far obstacle
  behavior; a successfully loaded empty non-target list remains distinct from
  scene-load failure.
- Confirmed release `GripperCommandError` is not swallowed: the existing
  executor test stops before retreat, `execute_first_reachable_plan()` only
  catches pregrasp `MoveItMotionError`, and return-to-initial is called only
  after successful plan execution. No release code change was needed.
- Verified in `friendly_visvesvaraya`: 73 focused grasp tests and seven scene
  tests pass; `colcon build --packages-select my_course_pkg --symlink-install`
  passes. Synchronized the three implementation/test files semantically with
  `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`.
- No live perception, MoveIt planning, trajectory, arm, or gripper command was
  run.

### 2026-07-13 - Apple Roadmap Review Corrections

- Revised the Apple stable-grasp roadmap after code-level external review.
- Made the shared/profile-independent round-top approach-corridor gate a
  mandatory prerequisite before Phase A3/B, with fail-closed scene handling
  and round-top/side regression requirements.
- Clarified the 10 mm Apple final gate versus the 13 mm shared default,
  FoundationPose round-object pose risk, release TCP Z/offset metrics, the
  actual gripper release validation chain, semantic cross-repo diffing, and
  the focused test command.
- No implementation code, simulation, perception, planning, or robot motion
  was run during this documentation revision.

### 2026-07-13 - Hammer Safe Slot Implemented

- Changed only placement slot 5 from `[-0.55, 0.20, 0.0]` to
  `[-0.55, 0.30, 0.0]`; all other slots, fixed-object order, orientations, and
  `random_object_count: 6` remain unchanged.
- Updated the focused configuration test and verified all seven scene-selection
  tests pass in container `friendly_visvesvaraya`.
- A formal 500-step headless MuJoCo replay found no non-table hammer contacts,
  only `0.0418 mm` XY drift, and no lift/ejection.
- Synchronized the YAML and test byte-for-byte with
  `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`.
- No live simulation, perception pipeline, or robot motion was run.

### 2026-07-13 - Hammer Startup Ejection Diagnosed

- Reproduced the screenshot behavior in an isolated headless MuJoCo run.
- Found an initial `banana_collision_5` / `hammer_collision_0` penetration of
  about `4.56 mm`; the long hammer footprint overlaps banana across slots 5
  and 2 despite both slot centers being valid table positions.
- The first physics step gave hammer linear velocity about
  `[2.88, 0.73, 6.41] m/s` plus large angular velocity, ejecting it from the
  table. This is not a fixed-selection failure, mesh-origin display offset, or
  table-boundary error.
- No scene code was changed during diagnosis.

### 2026-07-13 - Fixed One Representative Per Grasp Category

- Changed `fixed_object_names` to `tomato_soup_can`, `banana`, `apple`,
  `foam_brick`, and `hammer`; kept `random_object_count: 6`, so slot 6 remains
  random while the total stays six.
- Added scene-selection assertions for the fixed prefix and non-fixed sixth
  object, plus an integration test proving the five names cover five distinct
  configured grasp categories.
- Verified in container `friendly_visvesvaraya`: seven scene-selection tests
  and all 23 grasp-selector tests passed; all five mapped grasp-library
  directories exist under `/home/ws/grasps`.
- Synchronized the three affected implementation files byte-for-byte with
  `E:\IFL\ros2-docker-workspace-vscode-plmrs-grasp-stable`.
- No simulation, perception pipeline, or robot motion was run.

### 2026-07-12 - Baseball Stage C Final Pose Reached

- Fixed round-top table clearance to include the later world-Z grasp offset;
  the fresh baseball run retained 1,288 candidates and reported selected
  conservative finger clearance `0.11841-0.12649 m` after `-0.020 m` offset.
- Updated final-grasp debug mode to skip the pre-approach gripper command.
- Live candidate 1 reached final target `[-0.2221,-0.5759,0.9102]`; actual TCP
  was `[-0.2258,-0.5714,0.9170]`, about 9 mm position error.
- Robot is holding at final grasp; no gripper, lift, transfer, or release
  command was executed.

### 2026-07-12 - Baseball Stage B Pregrasp Reached

- Added and tested `GRASP_DEBUG_STOP_AT_PREGRASP`; when enabled it skips the
  pre-approach gripper command, stops immediately after a successful
  `move_to_pre_grasp`, and skips approach/gripper/lift/drop.
- Verified 14 focused executor tests and rebuilt the active container.
- Live: candidate 1 reached pregrasp TCP
  `[-0.2229, -0.5782, 1.0102]`, 0.10 m above the final target, and is holding.
- Found before stage C: round-top filtering evaluates the raw library pose,
  but `select_grasp_pose_candidates_6d` later applies the shared
  `GRASP_Z_OFFSET=-0.020 m`. Clearance must account for this lowering before
  permitting final descent.

### 2026-07-12 - Peach And Baseball Stage A Passed

- Found live perception and ROS were running in `friendly_visvesvaraya`, while
  the first implementation build was in `objective_einstein`; synchronized
  the round-top source changes into the active container and rebuilt there.
- Verified 21 selector tests pass in the active container.
- Peach: fresh FoundationPose plus read-only selector returned eight candidates
  from 3,191 fully filtered candidates; approach angles were about
  `0.33-1.25 deg`, opening margin `23.04 mm`.
- Baseball: fresh FoundationPose plus read-only selector returned eight
  candidates from 1,286 fully filtered candidates; approach angles were about
  `0.98-2.29 deg`, opening margin `12.08 mm`.
- No MoveIt planning, trajectory execution, gripper command, or arm motion was
  performed during either stage-A selector check.

### 2026-07-12 - Generic Round-Top Offline Implementation Completed

- Added object category and category-to-profile mappings while preserving
  `cylindrical_can -> side`; unknown objects now fail closed.
- Added shared static geometry for nine round-top fruits and balls, plus
  configurable orientation, center, normalized-height, opening, fingertip,
  table-clearance, and candidate-count limits.
- Added 45-degree geometry-center symmetry expansion, deduplication, all five
  hard filters, yaw-neutral ranking, diagnostics, and up to eight candidates.
- Verified the real `/home/ws/grasps/013_apple` library yields eight selected
  candidates after filtering 5,013 raw / 40,104 expanded poses.
- Verified 62 focused selector, planner, trajectory, and executor tests pass.
- Verified `colcon build --packages-select my_course_pkg --symlink-install`.
- No simulation or arm motion was run.

### 2026-07-12 - Round-Top Implementation Approved And Started

- User confirmed implementation after the fourth review found no substantive
  issues in `2026-07-12-round-top-grasp-category-design.md`.
- Located the real apple grasp library at `/home/ws/grasps/013_apple` and the
  apple mesh at `src/my_course_pkg/YCB_Dataset/ycb/apple/textured.obj`.
- Confirmed current unknown-object handling silently falls back to `top_down`;
  the approved implementation must replace this with fail-closed mapping.
- Next: complete and record mandatory Q1-Q4 measurements before adding the
  round-top selector or initiating live arm motion.

### 2026-07-11 - Gentle Release Pre-Open Hold Added

- Changed: added `GRASP_RELEASE_PRE_OPEN_HOLD_SEC`, defaulting to `1.0 s`.
- Changed: the pick-place trajectory now inserts active `hold_before_release`
  immediately after `descend_to_drop` and before `open_gripper_to_release` for
  both normal and `direct_to_grasp` plans.
- Verified: in container `objective_einstein`,
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/my_course_pkg/test/test_trajectory_planner.py -q`
  passed with 11 tests.
- Verified: in container `objective_einstein`, the focused grasp suite
  `test_grasp_selector.py`, `test_gripper_control.py`,
  `test_pick_place_planner.py`, `test_grasp_executor_gripper_failures.py`, and
  `test_trajectory_planner.py` passed with 65 tests.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Next: live-test full release and watch whether the can remains upright after
  the pre-open hold.

### 2026-07-11 - Final Approach Tolerance Default Raised

- Changed: `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M` now defaults to
  `0.013 m`, matching repeated successful live tomato-can runs. The environment
  variable still overrides the default for future tuning.
- Verified: in container `objective_einstein`,
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/my_course_pkg/test/test_grasp_executor_gripper_failures.py -q`
  passed with 13 tests.
- Verified: in container `objective_einstein`, the focused grasp suite
  `test_grasp_selector.py`, `test_gripper_control.py`,
  `test_pick_place_planner.py`, `test_grasp_executor_gripper_failures.py`, and
  `test_trajectory_planner.py` passed with 63 tests.
- Next: for live trials, do not export
  `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M` unless intentionally testing a
  different value.

### 2026-07-11 - Automatic Mid-Table Placement Defaults Implemented

- Changed: default safe-placement bounds now use the live-validated mid-table
  region: `x=[-0.55, -0.20]`, `y=[-0.85, -0.50]`,
  preferred XY `[-0.38, -0.65]`, and object clearance `0.08 m`.
- Changed: `GRASP_PLACE_PREFERRED_X/Y` no longer fall back to `DROP_X/Y`; they
  have independent defaults so old fixed-drop environment variables do not
  pull the automatic safe placement toward the base side.
- Changed: added a configurable rectangular robot-base exclusion zone enabled
  by default: `x=[-0.05, 0.35]`, `y=[-0.45, 0.10]`.
- Changed: safe-placement logs now print `[PlaceDebug]` bounds, base-exclusion
  state, rejection counts, selected preferred-distance, selected XY,
  clearance, and score while preserving the old `Safe placement selected` line.
- Verified: in container `objective_einstein`,
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/my_course_pkg/test/test_pick_place_planner.py -q`
  passed with 17 tests.
- Verified: in container `objective_einstein`, the focused grasp suite
  `test_grasp_selector.py`, `test_gripper_control.py`,
  `test_pick_place_planner.py`, `test_grasp_executor_gripper_failures.py`, and
  `test_trajectory_planner.py` passed with 63 tests.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Next: live-run `grasp_demo` using default placement settings and inspect the
  `[PlaceDebug]` output plus the pre-release pose before opening the gripper.

### 2026-07-11 - Manual Mid-Table Safe Placement Validated

- Verified live: the full tomato-can grasp pipeline reached the pre-release
  debug stop with a good mid-table placement when using
  `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M=0.013`,
  `GRASP_DEBUG_STOP_BEFORE_RELEASE=1`,
  `GRASP_PLACE_X_MIN=-0.55`, `GRASP_PLACE_X_MAX=-0.20`,
  `GRASP_PLACE_Y_MIN=-0.85`, `GRASP_PLACE_Y_MAX=-0.50`,
  `GRASP_PLACE_PREFERRED_X=-0.38`, `GRASP_PLACE_PREFERRED_Y=-0.65`, and
  `GRASP_PLACE_OBJECT_CLEARANCE_M=0.08`.
- Found: the original unrestricted safe-placement policy can choose points too
  close to the robot base because it only considers non-target scene objects,
  not the base footprint or base-side no-place zone.
- Found: the earlier default clearance `0.12 m` can reject every point in the
  useful mid-table region for this scene; `0.08 m` worked visually and should
  become part of the automatic policy or an object-aware clearance rule.
- Next: add a base exclusion zone/default mid-table preference so the robot
  finds this kind of safe placement without manual environment overrides.

### 2026-07-11 - Safe Lift Hold, Place, And Release Gate Implemented

- Changed: gripper commands now return a structured `GripperCommandResult`
  carrying target position, actual position, effort, stalled, reached-goal, and
  action-status fields instead of collapsing everything to a bool.
- Changed: close-on-object can still accept a useful stalled contact, but
  open/release commands must actually reach the configured open position.
  Stalled release results such as `actual_position=0.740` are now rejected and
  stop execution before retreat/home motion.
- Changed: post-grasp lift hold defaults to 5 seconds and uses active pose
  holding instead of passive sleep. Added `GRASP_DEBUG_STOP_AFTER_LIFT` and
  `GRASP_DEBUG_STOP_BEFORE_RELEASE` debug stops.
- Changed: drop/release height is derived from the successful grasp TCP height;
  retreat/drop-high height uses `GRASP_LIFT_HEIGHT` instead of
  `APPROACH_DIST`. The old fixed `DROP_Z=0.35` is no longer used as the
  release height.
- Changed: safe placement samples a configurable table rectangle and rejects
  candidate XY locations inside conservative clearance disks around every
  non-target `/scene_description` object. Missing scene data fails closed by
  default for placement.
- Verified: in container `objective_einstein`, the focused suite
  `test_grasp_selector.py`, `test_gripper_control.py`,
  `test_pick_place_planner.py`, `test_grasp_executor_gripper_failures.py`, and
  `test_trajectory_planner.py` passed with 59 tests.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Verified live: with `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M=0.013` and
  `GRASP_DEBUG_STOP_AFTER_LIFT=1`, final approach converged at about
  `0.0111 m`, the gripper closed on the tomato can, the arm lifted the can
  about 20 cm, and active lift hold kept it suspended. The previous `0.010 m`
  tolerance was too tight for the current servo residual.
- Next: live-test with `GRASP_DEBUG_STOP_AFTER_LIFT=1`, then
  `GRASP_DEBUG_STOP_BEFORE_RELEASE=1`, then a full run if the selected drop
  pose is safe and release opens cleanly.

### 2026-07-11 - Generic Side-Grasp Approach Clearance Implemented

- Changed: `pick_place_planner.py` now loads `/scene_description` markers for
  side grasps, transforms marker centers into the `world` planning frame, and
  converts every non-target marker into a conservative clearance obstacle.
- Changed: side-grasp candidates are rejected when the straight
  `pregrasp -> final grasp` corridor overlaps any non-target obstacle in XY and
  Z. The selected target marker is ignored by normalized name, including YCB
  numeric prefixes such as `005_tomato_soup_can`.
- Changed: added environment knobs
  `GRASP_SIDE_CLEARANCE_ENABLED`,
  `GRASP_SIDE_APPROACH_CORRIDOR_RADIUS_M`,
  `GRASP_SIDE_APPROACH_CLEARANCE_MARGIN_M`,
  `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M`, and
  `GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC`.
- Verified: in container `objective_einstein`, the focused planner test
  command
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/my_course_pkg/test/test_pick_place_planner.py -v`
  passed with 10 tests.
- Verified: in container `objective_einstein`, the broader focused grasp suite
  `test_grasp_selector.py`, `test_pick_place_planner.py`,
  `test_grasp_executor_gripper_failures.py`, and `test_trajectory_planner.py`
  passed with 44 tests.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Next: live-reset, rerun perception, and run a guarded tomato side-grasp to
  inspect which candidates the generic clearance gate rejects before any
  gripper close.

### 2026-07-11 - Tomato Can Lift Succeeded, Place/Release Unsafe

- Verified live: tomato-can perception returned the known-good upright pose
  (`tilt_to_world_up_deg` near zero), side approach clearance rejected banana
  sweep candidates, final approach converged with a 10 mm temporary tolerance,
  close stalled on the can around gripper position `0.195`, and the can lifted.
- Found: the later place sequence is still the old fixed-drop routine. It
  transfers to `[-0.55, -0.45, high]`, descends to the hard-coded
  `DROP_Z=0.35`, then opens and retreats. This is not table/object aware and
  can look like uncontrolled/freeform motion after a successful lift.
- Found: release was unhealthy: `open_gripper_to_release` requested open
  position `0.0` but returned stalled at position about `0.740` with
  `reached_goal=False`; current gripper code accepts any stalled result, so it
  continued to retreat after a failed release.
- Next: before trusting full pick-place, add a safe place/release stage: compute
  release height from the tabletop/object grasp height instead of `DROP_Z=0.35`,
  add a debug stop before release, and reject failed open commands instead of
  accepting every stalled gripper result.

### 2026-07-11 - Devcontainer GPU Settings Added

- Changed: `.devcontainer/devcontainer.json` now requests Docker GPU access
  with `--gpus=all`, passes `/dev/dxg`, mounts `/usr/lib/wsl/lib` read-only,
  and sets EGL/D3D12/NVIDIA environment variables for MuJoCo/WSLg rendering.
- Verified: a one-shot `docker run --rm` using the same image plus the new GPU
  arguments can see `/dev/dxg`, WSL `libd3d12.so`/`libdxcore.so`, and the RTX
  4060 through `/usr/lib/wsl/lib/nvidia-smi`.
- Note: the already-running `mystifying_lamarr` container still has its old
  launch configuration until VS Code rebuilds or reopens the devcontainer.
- Next: rebuild/reopen the devcontainer, start the simulation, and check
  `ps -L -p <ros2_main_pid> -o pid,tid,pcpu,stat,comm` for absence of
  `llvmpipe-*`.

### 2026-07-10 - WSL/Docker MuJoCo GPU Diagnostic

- Found: Windows and the Ubuntu-24.04 WSL2 distro both see the RTX 4060.
  WSL has `/dev/dxg`, and `/usr/lib/wsl/lib/nvidia-smi` works.
- Found: the active devcontainer was not launched with Docker GPU requests:
  `DeviceRequests=null`, `Runtime=runc`, and no `NVIDIA_VISIBLE_DEVICES`.
- Found: current MuJoCo processes are using Mesa CPU software rendering:
  `ps -L` shows many `llvmpipe-*` threads under each `python3 ros2_main.py`.
- Found: three `python3 ros2_main.py` processes were running for 5-10 hours,
  and `docker stats` showed about `1660%` CPU and `85%` memory use. Multiple
  stale simulations alone can make MuJoCo/RViz stutter badly.
- Found: the container has `/dev/dxg` and Mesa `d3d12_dri.so`, but lacks
  `/usr/lib/wsl/lib` and `/mnt/wslg` mounts, so WSLg D3D12 OpenGL falls back to
  `llvmpipe`.
- Next: stop stale simulations, add container GPU/WSLg mounts and environment,
  rebuild/reopen the container, and verify the renderer is not `llvmpipe`.

### 2026-07-10 - Perception Mask Safety Gate Spec

- Changed: wrote and committed `79f7268` (`docs: specify perception mask
  safety gate`) at
  `docs/superpowers/specs/2026-07-10-perception-mask-safety-gate-design.md`.
- Design: FoundationPose mask loading must fail closed on zero or multiple
  matching target masks, support an explicit `FOUNDATIONPOSE_MASK_INDEX`
  override, write `mask_candidates.json` and selected metadata, generate a
  numbered candidate overlay when bbox data exists, and remove or invalidate
  stale `pose_result.json` before mask selection.
- Next: user review of the written spec, then implement the Phase 1 safety
  gate and focused mask-selection tests.

### 2026-07-10 - Perception Mask Safety Gate Implemented

- Changed: `FoundationPoseEstimationNode._load_mask()` now writes
  `mask_candidates.json`, generates `mask_candidates_overlay.jpg` when RGB and
  bbox data are available, accepts exactly one target mask by default, refuses
  multiple matching masks, and supports explicit
  `FOUNDATIONPOSE_MASK_INDEX=<n>` selection.
- Changed: `FoundationPoseEstimationNode.run()` removes stale
  `pose_result.json`, `pose_error.json`, and `selected_mask_metadata.json`
  before mask selection. `pose_result.json` is now written only on
  FoundationPose success; failures write `pose_error.json`, so an ambiguous or
  failed perception run does not leave a usable old pose behind.
- Changed: focused mask tests cover alias matching, zero matches, multiple
  ambiguous matches, explicit override, invalid override, selected metadata,
  and stale pose-result invalidation.
- Verified: in container `mystifying_lamarr`,
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/my_course_pkg/test/test_foundationpose_mask_matching.py -v`
  passed with 7 tests.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Next: run `pipeline` only, confirm the safety gate behavior on the live
  tomato scene, and do not run `grasp_demo` until the selected mask and
  FoundationPose pose are verified.

### 2026-07-10 - Live Safety Gate Triggered Correctly

- Verified live: running `ros2 run my_course_pkg pipeline` for
  `pick up the tomato` stopped before FoundationPose with
  `SAM2 returned 2 masks matching target 'tomato soup can'`.
- Candidate overlay shows candidate `#1` is the tomato soup can and candidate
  `#2` is the apple. Candidate `#1` bbox is roughly
  `[157, 158, 301, 278]`; candidate `#2` bbox is roughly
  `[519, 186, 608, 281]`.
- Verified: after the ambiguity stop, `pose_result.json`,
  `pose_error.json`, and `selected_mask_metadata.json` were absent, so no stale
  pose result should be available for `grasp_demo`.
- Next: rerun pipeline with `FOUNDATIONPOSE_MASK_INDEX=1`; inspect
  `selected_mask_metadata.json` and `pose_result.json` before any arm motion.

### 2026-07-10 - Can Candidate Verifier Spec

- Changed: wrote and committed `b86fe8a` (`docs: specify can candidate
  verifier`) at
  `docs/superpowers/specs/2026-07-10-can-candidate-verifier-design.md`.
- Design: when multiple target-labeled masks exist, build numbered candidate
  crops, ask the VLM to return a high-confidence candidate index, select only a
  valid high-confidence answer, and otherwise keep the existing fail-closed
  ambiguity behavior. Manual `FOUNDATIONPOSE_MASK_INDEX` remains a debug
  override.
- Next: implement verifier helpers, fake-verifier tests, and live
  perception-only validation on the current tomato-can/apple ambiguity.

### 2026-07-10 - Can Candidate Verifier Implemented

- Changed: multi-mask target selection now generates
  `mask_verification_candidates.jpg`, calls a VLM candidate verifier by
  default, accepts only a valid high-confidence candidate index, and otherwise
  preserves fail-closed ambiguity behavior.
- Changed: manual `FOUNDATIONPOSE_MASK_INDEX` still has priority over the VLM
  verifier; `FOUNDATIONPOSE_AUTO_MASK_VERIFY=0` disables auto verification for
  debugging.
- Changed: verifier attempts write `mask_verification_result.json`; successful
  verifier selections write `selected_mask_metadata.json` with
  `selected_by: "vlm_candidate_verifier"`.
- Verified: in container `mystifying_lamarr`,
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest src/my_course_pkg/test/test_foundationpose_mask_matching.py -v`
  passed with 10 tests.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Next: rerun the live tomato pipeline without manual mask index and check that
  the verifier selects candidate `#1` (can) over candidate `#2` (apple).

### 2026-07-10 - Can Verifier Low-Confidence Live Case

- Found live: a fresh `pick up the tomato` pipeline produced four
  `tomato soup` candidates. Candidate `#3` was the actual can; the other
  candidates were fruit-like objects.
- Found: the first verifier attempt returned
  `{"selected_index": null, "confidence": "low"}`. The generated crop card
  showed only the silver can top/bottom for candidate `#3`, with no red side
  label, so the VLM was too conservative for `tomato soup can`.
- Changed: the verifier card now combines the full-scene numbered overlay with
  the candidate crops, and the prompt includes a can-specific visual hint that
  a top-down tomato/tuna can may appear mainly as a gray/silver circular metal
  lid or bottom with the label hidden.
- Verified after this prompt/card revision: focused mask tests still passed
  with 10 tests, and `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Next: rerun pipeline without manual mask index and inspect
  `mask_verification_result.json`; expected current-scene selection is
  candidate `#3` if the scene has not changed.

### 2026-07-10 - Can Candidate Verifier Live Success

- Verified live: rerunning `ros2 run my_course_pkg pipeline` without
  `FOUNDATIONPOSE_MASK_INDEX` selected the tomato can automatically via
  `selected_by="vlm_candidate_verifier"`.
- Evidence: `mask_verification_result.json` returned
  `selected_index=1`, `confidence="high"`, `decision="selected"`, and
  candidate bbox roughly `[157, 156, 301, 277]`.
- Verified: FoundationPose succeeded and wrote the known-good tomato camera
  pose translation `[-0.4302, -0.0985, 0.6926]`, matching prior correct tomato
  runs near `[-0.4307, -0.1005, 0.6929]`.
- Next: do not rerun `pipeline` unless the scene/camera changes. Use this fresh
  pose for a guarded grasp debug run and evaluate final TCP convergence,
  gripper close, and lift separately.

### 2026-07-10 - Guarded Tomato Grasp Stopped At Final Approach

- Verified live: with the fresh auto-selected tomato pose, `grasp_demo` reached
  candidate 1 pregrasp and executed the side final approach, but failed before
  close with `FinalApproachConvergenceError`.
- Evidence: final target was `[-0.2214, -0.9322, 0.9584]`; actual TCP was
  `[-0.2234, -0.9193, 0.9524]`, leaving `0.0144 m` error against the configured
  `0.005 m` tolerance. No `close_gripper_at_grasp` command was sent.
- Good: perception and grasp selection were correct. The selected final TCP was
  near the tomato geometry center:
  `final_tcp_relative_to_geometry_center=[+10.4, +1.4, +17.6] mm`.
- Still suspicious but secondary for this run: the initial open command returned
  `status=6`, `stalled=True`, `reached_goal=False`, position `0.333`, and was
  accepted by current gripper logic. This must be fixed before trusting a full
  pick-place, but it did not cause the current stop because close was never
  reached.
- Next: debug the final approach servo/convergence path or run a clearly marked
  diagnostic-only close trial with a temporarily looser tolerance to inspect
  gripper contact.

### 2026-07-10 - Repeated Wrong Tomato Mask Confirmed

- Found: the 18:00 pipeline output labeled four unrelated round objects as
  `tomato soup`. The first annotation was the lower orange fruit at bbox
  `[495, 651, 593, 719]` with GroundingDINO score `0.56`; the real tomato-can
  top was the third annotation at bbox `[139, 623, 268, 719]` with score `0.42`.
- Found: `FoundationPoseEstimationNode._load_mask()` still selects `anns[0]`,
  so this run sent the orange-fruit mask to FoundationPose.
- Evidence: the resulting camera translation
  `[-0.1570, +0.2566, 0.5520]` is about `0.47 m` from the repeatedly verified
  tomato-can camera translation near `[-0.4307, -0.1005, 0.6929]`.
- Conclusion: the folded arm pose in the latest screenshot is consistent with
  moving toward the wrong fruit pose. Do not run `grasp_demo` again until mask
  selection is guarded or explicitly overridden.

### 2026-07-10 - Close Command Sent But Unhealthy Result Was Accepted

- Found: the latest run used the correct tomato-can FoundationPose result and
  did send `close_gripper_at_grasp` with position `0.79`; closing was not
  skipped.
- Found: final approach was allowed to close at `24.2 mm` TCP position error,
  indicating the diagnostic `25 mm` tolerance was still active rather than the
  new `5 mm` default.
- Found: the close action returned status `6`, position `0.182`, effort
  `139.7`, `stalled=True`, and `reached_goal=False`. Current
  `gripper_control.py` accepts any stalled result, so execution continued
  through lift and transfer without proving that the can rose.
- Found: the later release command explicitly requested open position `0.0`
  and also returned an unhealthy stalled result at position `0.339`. A
  screenshot after the completed cycle therefore shows the release/home state,
  not whether a close command was sent at the grasp pose.
- Next: run a fresh close-only inspection with the 5 mm final tolerance and
  `GRASP_DEBUG_STOP_AFTER_CLOSE=1`; distinguish valid close-on-object stall from
  premature/one-sided contact and reject failed open commands instead of
  treating every stall as success.

### 2026-07-10 - Tomato Can Perception Mask Misselection

- Found: a later `pipeline` run did not detect the actual tomato soup can as
  the target. `grounded_sam2_annotated_image_with_mask.jpg` labeled the red
  fruit at lower left as `tomato soup 0.74` and another round object as
  `tomato soup 0.50`; the real can top was unlabeled.
- Found: `FoundationPoseEstimationNode._load_mask()` currently takes the first
  matching SAM2 annotation (`anns[0]`). With multiple false `tomato soup`
  annotations, FoundationPose estimates a pose for the wrong mask, so the robot
  moves far from the visible can. This is distinct from the prior side-grasp
  centerline and final-approach tracking issues.
- Evidence: latest `pose_result.json` translation
  `[-0.3653, +0.1749, 0.5879]` differs sharply from the earlier valid tomato
  can camera pose around `[-0.4306, -0.1005, 0.6929]`.
- Next: add a mask-selection guard/override for ambiguous SAM2 detections and
  require reviewing the annotated mask image before running `grasp_demo`.

### 2026-07-10 - Live Run After Correct Tomato Mask

- Found: a later fresh pipeline correctly segmented the tomato can again
  (`tomato soup 0.78`) and FoundationPose returned the known-good camera pose
  around `[-0.4306, -0.1005, 0.6929]`.
- Found: the grasp candidate was centered on the tomato geometry center:
  `final_tcp_relative_to_geometry_center=[+10.4, +1.4, +17.6] mm`.
- Found: execution still did not close because the final approach convergence
  guard timed out at `20.3 mm` with tolerance `15 mm`. Final target
  `[-0.2208, -0.9309, 0.9586]`; actual
  `[-0.2290, -0.9155, 0.9689]`, so actual-target was about
  `[-8.2, +15.4, +10.3] mm`.
- Next: for a diagnostic close-only trial, rerun from a fresh pipeline with
  `GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M=0.025` and
  `GRASP_DEBUG_STOP_AFTER_CLOSE=1`; for a robust fix, improve final approach
  tracking/logging rather than lowering the side-grasp target blindly.
- Found: a follow-up run executed only `ros2 run my_course_pkg grasp_demo` in a
  shell without the horizontal-grasp environment variables. It fell back to
  default side-grasp selection (`selected_height_m=0.0810-0.0878`,
  `tool +Z angle to -Z=47.06 deg`) and default approach speed
  `0.200 m/s`. Final approach failed before close with `41.5 mm` error,
  dominated by actual Z being about `+40 mm` above target. This run does not
  validate the previous 25 mm tolerance trial.

### 2026-07-10 - Tomato Can Geometry-Center Side-Grasp Fix

- Changed: added per-object side-grasp geometry centers for `tomato_soup_can`
  and `tuna_fish_can`. The tomato center is the mesh bbox center, about
  `[-0.009169, 0.084018, 0.051006]` m in object coordinates.
- Changed: `expand_side_symmetric_grasps()` now rotates side-grasp candidates
  about that geometry center rather than YCB object-local `[0,0,0]`.
- Changed: side-grasp centerline score, hard filter, and selected-candidate
  logs now measure the geometry center in TCP coordinates. The old origin-based
  helper remains only for diagnostics/tests.
- Changed: planner/executor debug info now logs
  `side_grasp_geometry_center_object`, `side_grasp_geometry_center_world_base`,
  `final_tcp_relative_to_geometry_center`, and
  `final_tcp_actual_relative_to_geometry_center`.
- Verified: 38 focused tests passed:
  `test_grasp_selector.py`, `test_pick_place_planner.py`,
  `test_grasp_executor_gripper_failures.py`, and `test_trajectory_planner.py`
  with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  passed.
- Offline sanity check: real tomato grasp library now selects candidates with
  geometry-centerline offset about `0.0154 m` while the same candidates still
  show object-origin XY offsets of about `0.033-0.111 m`. This confirms the
  visible miss was an origin-vs-geometry-center bug, not just a Z-only error.
- Live verification: after reset, fresh pipeline, and geometry-center fix,
  candidate 1 planned `final_tcp_relative_to_geometry_center` at about
  `[+9.5, -2.1, +30.0] mm`, so XY is centered and the target is high-mid can.
  The final approach did not converge: target
  `[-0.2173, -0.9320, 0.9710]`, actual
  `[-0.2260, -0.9429, 1.0106]`, error `41.9 mm`, dominated by `+39.6 mm` Z.
  Approximate actual relative to geometry center is about
  `[-1.5, +6.5, +69.6] mm`; the screenshot is a failed high final-approach
  pose, not a closed grasp.
- Live verification with more horizontal side grasp:
  `GRASP_SIDE_PREFERRED_TILT_DEG=25`,
  `GRASP_SIDE_MIN_TOOL_Z_DOWN_ANGLE_DEG=60`,
  `GRASP_SIDE_MAX_TOOL_Z_DOWN_ANGLE_DEG=80`,
  `GRASP_SIDE_TARGET_HEIGHT_M=0.069` selected candidates with
  `selected_geometry_centerline_offset_m` mean about `0.0115 m` and candidate 1
  target relative to geometry center about `[+10.4, +1.4, +17.6] mm`. A
  debug-stopped run reached `12.9 mm` final TCP error, close enough to try a
  controlled close with a 15 mm tolerance. A later full run without reset and a
  fresh pipeline was invalid: the can had already been disturbed/tilted, the
  planner still used the old upright pose, and final approach failed high again
  (`48.9 mm` error) before gripper close.
- Next tuning direction: avoid compensating with a blind negative
  `SIDE_GRASP_Z_OFFSET` first. The selected approach is still oblique downward
  (`tool +Z` angle to world down about `47 deg`) and may collide with the can
  rim/top before reaching target. The real tomato library has centered
  candidates around `65-68 deg` to world down with object-local heights near
  `0.069-0.079 m`; verify those via environment variables before changing
  defaults.
- Note: `python3 -m pytest src/my_course_pkg/test/test_flake8.py -v` timed out
  in the current dirty workspace. A touched-file flake8 run still reports many
  pre-existing long lines; the new blank-line warning was fixed.
- Next: perform one fresh live run with `GRASP_DEBUG_STOP_AT_GRASP=1` and check
  whether the final/actual TCP is centered relative to the can geometry center.

### 2026-07-10 - Live FoundationPose/Servo Debug

- Found: `ros2 topic echo /scene_description --once` fails if
  `ROS_DOMAIN_ID` is set to an empty string. Use `unset ROS_DOMAIN_ID` or
  `export ROS_DOMAIN_ID=0`.
- Found: `/scene_description` publishes object poses in `base_link`; current
  `world -> base_link` is translated and yaw-rotated, so object truth must be
  transformed before comparing to `canonical object pose world/base`.
- Verified: when the robot/camera is at the same pose used by the old
  FoundationPose result, tomato-can canonical world pose is close to MuJoCo
  truth (about 1 mm XY/Z error), so the object position estimate itself is not
  the active miss in that condition.
- Found: a later run used the same FoundationPose camera pose but a different
  current `T_world_cam`, producing a wrong world object pose. This indicates
  capture-time camera TF must be saved or queried by RGBD timestamp; do not
  multiply stale FoundationPose output by the live camera TF after the robot
  moves.
- Found: a `pipeline` run stopped at LLM/SAM2 because no `VLM_API_KEY` or
  `OPENAI_API_KEY` was set; FoundationPose was not refreshed in that run.
- Found: with `GRASP_DEBUG_STOP_AT_GRASP=1`, candidate 3 reached pregrasp, but
  after the servo approach the actual TCP was about 3.7 cm from the final
  target and about 3.6 cm short along the approach direction
  (`target=[-0.0540,-0.9248,0.9799]`,
  `actual=[-0.0242,-0.9180,1.0007]`). This points to final Cartesian servo
  tracking/settling, not FoundationPose position.
- Next: rerun with a valid API key and a slower final approach
  (`GRASP_INTERPOLATE_AVG_SPEED=0.05`, optionally smaller
  `GRASP_WAYPOINT_MAX_DIST`) to see whether target-vs-actual TCP error shrinks.
- Found: `GRASP_DEBUG_STOP_AT_GRASP=1` intentionally keeps `grasp_demo` alive at
  the final grasp pose until Ctrl-C. When output is piped through `tee`, Python
  `print` diagnostics may remain buffered; set `PYTHONUNBUFFERED=1` for live
  `[GraspDebug]` output.
- Verified live: after stopping at a final grasp and launching `grasp_demo`
  again without a fresh pipeline/camera capture, unchanged FoundationPose
  camera coordinates `[-0.4307,-0.1005,0.6929]` converted to the wrong world
  can pose `[-0.3813,-0.5245,0.3567]` rather than the capture-pose result
  `[-0.1355,-0.9330,0.8900]` (about `0.72 m` apart). Every pregrasp was then
  correctly rejected by MoveIt. Do not reuse a FoundationPose result with a
  moved camera; rerun `pipeline` before each stopped-debug retry until the
  capture-time camera TF is persisted in code.
- Verified live after reset plus a fresh pipeline: FoundationPose again
  produced the correct tomato world pose `[-0.1355,-0.9330,0.8900]`, and
  candidate 3 reached its MoveIt pregrasp. At `0.05 m/s` with `0.02 m`
  waypoints, final TCP target `[-0.0539,-0.9248,0.9799]` still ended at
  `[-0.0320,-0.9181,0.9910]`: 25.5 mm total error, about 23.5 mm short along
  approach. This is final servo tracking, not FoundationPose or selection.
- Changed: wrote and committed `d9df4f7` (`docs: specify final approach
  convergence`) with the approved feedback-loop design at
  `docs/superpowers/specs/2026-07-10-final-approach-convergence-design.md`.
- Implementation review accepted: convergence must use `send_pose_cmd` rather
  than `move_to_pose`; use a separate `FinalApproachConvergenceError`; test
  debug-stop with a converged fake arm; and limit v1 to Euclidean position
  convergence. On a 1.5 s timeout, fail safely without closing the gripper.
- Changed: added final TCP convergence defaults (5 mm tolerance, 1.5 s
  timeout, 50 ms command period), a `FinalApproachConvergenceError`, and a
  direct `send_pose_cmd` feedback loop before the gripper can close.
- Verified: 38 focused selector, planner, trajectory, and executor tests
  passed with `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1`; `colcon build
  --packages-select my_course_pkg --symlink-install` passed.
- Live verification: the new final TCP feedback guard ran correctly and safely
  refused to close. It reduced a 30.5 mm initial error to 10.0 mm, then drifted
  to a 16.0 mm timeout error after 11 direct commands. This is not a perception
  or frame error; the target was correct and the guard was active.
- Found lower-level cause: `src/arm_api2/src/moveit2_iface.cpp`
  `move_to_pose_servo` sets `active_start_positions_` from the previous
  commanded `active_target_positions_` whenever a target is already active,
  not from current joint state or the smoother output. Repeated final-pose
  commands therefore retain an open-loop tracking assumption and can chase or
  drift near the target. Investigate updating this start state from `q_out_` or
  current joints, and avoid resetting identical targets unnecessarily.
- Changed: wrote and committed `f6c4ba8` (`docs: specify servo target
  reanchor`) at `docs/superpowers/specs/2026-07-10-servo-target-reanchor-design.md`.
  It uses real current joints for changed targets, refreshes but does not
  restart identical targets, preserves active interpolation if current state is
  unavailable, and removes the early active-target reset.
- Changed: implemented the target-reanchor fix in
  `src/arm_api2/src/moveit2_iface.cpp`. Changed targets now start from measured
  current joints; identical targets only refresh the watchdog and emit a
  throttled maximum joint-delta log; missing current state preserves the active
  interpolation; the `t >= 1.4` early reset is removed.
- Verified: `colcon build --packages-select arm_api2 --symlink-install` passed.
  A running launch still uses the old binary until it is restarted. Live logs
  should show `Servo target max joint delta=... same_target=true` for repeated
  final-pose commands and `Final approach converged` at 5 mm or less.
- Found after the first restart attempt: `pipeline` can still capture RGB-D and
  finish FoundationPose while every arm action/service client is unavailable.
  The live ROS graph had `/mujoco_ur10e_interface` but no `/moveit2_iface`, and
  `/arm/state/current_pose` had zero publishers. Do not run `grasp_demo` until
  `ros2 node list` includes `/moveit2_iface` and `ros2 topic info
  /arm/state/current_pose --verbose` reports one publisher.
- Found: the full launch failed before starting MoveIt because
  `servo_watchdog.py` was installed as a symlink to a non-executable source file
  on the Windows-mounted workspace. `chmod +x` on the source did not take
  effect. Fixed `src/arm_api2/CMakeLists.txt` so CMake creates an executable
  build-directory copy and installs that entry instead.
- Verified: `colcon build --packages-select arm_api2 --symlink-install` passed,
  and `ros2 pkg executables arm_api2` now lists `servo_watchdog.py`.
- Found: the visible tomato-can miss is not explained by MoveIt-vs-MuJoCo TCP
  frame mismatch. Runtime-expanded URDF uses `tool0_to_tcp xyz="0 0 0.18"`,
  and offline FK at the same joint state puts URDF `tcp` within about 4 mm of
  MuJoCo `robotiq_2f85_pinch`.
- Found: the larger miss comes from the YCB tomato model origin being far from
  the visible can center. `textured.obj` bbox center is roughly
  `[-0.009, 0.084, 0.051]` m in object coordinates. The failed debug pose was
  only about 16 mm from its commanded TCP target, but relative to this geometry
  center the TCP was about 164 mm off in XY.
- Found: `expand_side_symmetric_grasps()` rotates side-grasp candidates about
  object-local `[0,0,0]`, and the side centerline filter also measures the model
  origin in TCP coordinates. For tomato cans this rotates otherwise plausible
  side grasps around the wrong point. Use a per-object geometry/grasp center
  offset for tomato (and likely tuna) in symmetry expansion, centerline scoring,
  and debug logs.

### 2026-07-09 - Side-Grasp Centerline Selector Fix

- Changed: added `GRASP_SIDE_MAX_CENTERLINE_OFFSET_M` and
  `GRASP_SIDE_CENTERLINE_WEIGHT`, plus a side-profile TCP centerline score and
  hard filter in `grasp_selector.py`.
- Changed: side selector logs now include `centerline_filter` and selected
  `centerline_offset` ranges.
- Changed: `test_grasp_selector.py` now constructs side grasps by object origin
  in TCP coordinates and covers centerline offset, preference, and rejection.
- Verified: focused selector tests passed:
  `python3 -m pytest src/my_course_pkg/test/test_grasp_selector.py -v`.
- Verified: planner/trajectory/executor-focused tests passed; the separate
  untracked `test_gripper_control.py` suite still fails against missing gripper
  acceptance/diagnostics helpers and is unrelated to this centerline change.
- Verified: `colcon build --packages-select my_course_pkg --symlink-install`
  finished successfully.
- Found: with the real tomato grasp library and default height range, 128
  candidates pass the `0.025 m` centerline limit; top selected offset is about
  `0.0058 m`.
- Found: the old lower height window `0.075-0.085 m` excludes all candidates
  under the `0.025 m` centerline limit. Use the updated command window below.
- Next: run a live stopped-after-close grasp and check whether the can is
  visually centered between the Robotiq fingers.

### 2026-07-08 - Tomato Can Still Misses In Screenshot

- Done: inspected the latest screenshot plus current `outputs_local`
  perception results. VLM selected `tomato_soup_can`; FoundationPose produced
  a tilted pose, but current code canonicalizes tomato cans upright before
  grasp selection.
- Found: the latest MuJoCo `ros2_main.log` only contains simulation loop
  output, not `grasp_demo` stdout; no fresh `GraspDebug` log was available for
  the screenshot.
- Found: offline selector geometry with current `src` chooses candidates whose
  TCP X closing axis is nearly horizontal (`x_vert_abs` about `0.025`), so the
  remaining visible miss is not explained by the old TCP-X-vertical jaw-roll
  alone.
- Found: current top candidates put the object origin far from the gripper
  centerline in TCP coordinates (`obj_origin_in_tcp` local X/Y norm about
  `0.108 m`), while Robotiq fingertip mesh thickness in TCP local Y is only
  about `0.028 m`.
- Found: better side candidates already exist in the library after current
  side filters, with TCP-local object-origin X/Y norm about `0.003-0.006 m`,
  but the selector does not score or filter for this centerline condition.
- Next: implement a tested side-profile centerline constraint/penalty before
  tuning height or jaw-roll further.

### 2026-07-08 - Tomato Can Side-Grasp Code Scan

- Done: traced current `src/my_course_pkg/my_course_pkg/grasp` side-grasp
  selection, planning, and execution code against recent tomato-can failure
  logs.
- Found: current `src` selector maps `tomato_soup_can` to the `side` profile,
  uses `SIDE_GRASP_Z_OFFSET`, filters raw object-local height before applying
  that offset, and has no implemented closing-axis gate.
- Found: with current `src` plus
  `SIDE_GRASP_Z_OFFSET=-0.011`,
  `GRASP_SIDE_MIN_HEIGHT_M=0.075`,
  `GRASP_SIDE_MAX_HEIGHT_M=0.085`,
  `GRASP_SIDE_TARGET_HEIGHT_M=0.081`, offline tomato-can selector output had
  zero filtered candidates with TCP X vertical component above `0.25`; selected
  candidates had TCP X nearly horizontal.
- Found: old trial logs still show an `object_lift_failed` case and a historical
  candidate whose TCP X axis was vertical (`x_vert_abs` about `0.73`), matching
  the documented jaw-roll failure mode.
- Known issue: repository state is inconsistent with old logs; current `src`
  lacks the older `grasp_eval` source and verify-step/metrics code, while stale
  build/egg-info artifacts still reference `grasp_eval`.
- Next: inspect a live stopped-after-close pose before changing selector
  geometry; if TCP X is vertical in the live run, finish the planned
  closing-axis normalization/gate.

### 2026-07-08 - Handoff Log Workflow

- Done: designed this persistent handoff workflow so future windows can resume
  faster and added a visible `Handoff loaded:` startup acknowledgement rule.
- Changed files:
  - `AGENTS.md`
  - `docs/superpowers/specs/2026-07-08-agent-handoff-log-design.md`
  - `docs/superpowers/plans/2026-07-08-agent-handoff-log.md`
  - `docs/agent_handoff.md`
- Verified: Markdown files contain the required startup marker, handoff
  sections, and seed project context.
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

- Do not use the eight corner combinations of one TCP-frame global AABB as a
  rotated gripper support hull. Those corners can combine extrema from different
  meshes into nonexistent points and create false table collisions. For the
  world-Z clearance minimum, transform the deterministically ordered real
  collision-mesh vertices (or a proven convex hull of those vertices).
- Do not launch the Tuna MuJoCo/contact gate while the static 5 mm envelope has
  zero accepted candidates. The approved stop condition requires design review;
  lowering the clearance, expanding the grid, or changing shared/non-Tuna
  behavior is not an implementation-time adjustment.
- Never derive Tuna's executable roll pivot from the requested preclamp command
  or a nominal free-space aperture. Object contact changes measured qpos; freeze
  the hash-verified interpolated matrix after contact, and do not build an
  executable roll suffix before that gate passes.
- Never compare lift aperture with Tuna's preclamp pivot aperture. Full close
  intentionally changes jaw position; freeze a separate stable post-close
  retention reference and diagnose its drift independently from AABB object
  follow.

- Docker Desktop's `HTTP 400: Bad Request` hides the daemon detail when a
  stopped VS Code Dev Container retains an expired WSL bind source. Reproduce
  with `docker start <container>` and inspect the exact missing path; do not
  treat a clean `Exited (0)` state as an application crash. Recreate the Dev
  Container, and verify every configured WSL device/bind source (notably
  `/dev/dri`) exists first.
- Dev Containers opened through a WSL folder requires Docker Desktop WSL
  integration for that distro; Windows-side `docker version` succeeding is not
  sufficient. If the log repeatedly says `docker could not be found in this
  WSL 2 distro`, check the integration toggle and the `/usr/bin/docker` symlink
  target before investigating Dockerfiles or build failures.

- Do not reintroduce an arbitrary fixed positive-Z cap for the final vertical
  grasp. Ordinary `+/-3 mm` acceptance must run first and without bounds; only
  a settled positive residual beyond that may use `controlled_center`, with
  actual Z no higher than the live object center. Intermediate and downward
  limits remain 3 mm.

- The Windows-only ROS test stubs in `src/my_course_pkg/test/conftest.py` do not
  define `geometry_msgs.msg.PoseStamped` or `Quaternion`. Pure config/decision
  tests can run under local Anaconda, but executor tests that construct pose
  commands fail at the stub boundary. Run those tests in the ROS Humble
  container; do not misdiagnose that import failure as a grasp regression.

- Repeated identical XYZ values from `get_current_ee_pose()` are not proof of
  stationarity: it returns the latest cached message, so a fast sample window
  can read one ROS feedback message multiple times. Require distinct/fresh
  message stamps (or an equivalent freshness condition) before using spread to
  authorize another vertical pregrasp calibration command.

- A duplicate action endpoint immediately after an owned interface restart is
  not by itself proof of two live processes. First compare the owned-process
  exit log, exact `pgrep`, and a repeated graph query. DDS can briefly retain
  the old endpoint after the process exits; blindly retrying the whole boundary
  creates another stop/start overlap and can reproduce the same false failure.
- A syntactically valid VLM candidate is not proof that it matches an explicit
  instruction. Do not allow membership in the global `YCB_OBJECTS` list alone
  to authorize FoundationPose or motion; explicit/uniquely resolved target
  identity must agree, and ambiguity/mismatch must stop before grasp.
- The installed ROS 2 Humble `ProcessStdin` event handler is a no-op that only
  emits `in ExecuteProcess(...).__on_process_stdin_event()`. Do not use it for
  this interactive supervisor. The scoped workaround writes bytes through pipe
  0 of the exact `ExecuteProcess._subprocess_transport`; keep that private-API
  dependency isolated and regression-tested.
- In ROS 2 Humble, launch `output="log"` is not screen-silent: its alias sends
  stderr to both the launch log and the screen. For an interactive terminal,
  use an explicit output destination map such as `{"both": "log"}` after the
  quiet launch configuration is resolved.
- For `foam_brick`, the 85 mm open gripper has only about `2.3 mm` clearance per
  finger around the observed `80.4 mm` span. A post-descent convergence gate is
  too late if the initial approach is off by centimetres: one finger can hit the
  top surface before feedback correction begins. The implemented path reanchors
  X/Y at safe hover and gates every 5 mm descent segment; do not bypass it by
  increasing its tolerances. Position near `0.038 rad` is not sufficient
  evidence of a good capture, and lowering the close threshold does not fix the
  top strike.
- Reissuing an identical Cartesian target does not remove a repeatable servo
  steady-state bias. The first strict pregrasp run remained about `7 mm` off in
  Y and `16.1 mm` low in Z after 15 commands. Correct the command target at safe
  hover and carry the learned offset through descent; do not reinterpret this
  as a reason to loosen the physical feedback gate.

- `vertical` grasp profiles now require a fresh, uniquely matched target marker
  from `/scene_clearance_bounds`. A missing, duplicate, invalid, or
  untransformable target is an intentional pre-motion failure. Do not bypass it
  or reuse the old uncentered candidate; repair the bounds publisher, target
  name, or TF instead.

- Do not run the hammer's full `lift_return` while its held orientation and
  return swept volume are unqualified. Plain `grasp_demo` descends after the
  five-second hold and can sweep the long tilted hammer into the banana. Use
  `GRASP_DEBUG_STOP_AFTER_LIFT=1` until a mechanically constrained grasp and a
  collision-safe return are separately validated.
- `pkill -INT -x moveit2_iface` kills the interface child inside the full
  launch, and that launch does not respawn it. Never proceed from `pkill`
  directly to reset/pipeline/demo; relaunch `arm_api2 moveit2_iface.launch.py`,
  keep that terminal alive, and verify one arm Action Server plus one
  `/arm/state/current_pose` publisher first.
- Immediately after killing a node, `ros2 node list` can retain its name in the
  ROS daemon cache. A displayed `/moveit2_iface` count of one does not override
  `Action servers: 0`, `Publisher count: 0`, an unknown pose topic, or an empty
  `pgrep`; all decisive gates must pass after relaunch.

- Round-top approach clearance requires a usable current
  `/scene_clearance_bounds` by default. Do not disable
  `GRASP_ROUND_TOP_CLEARANCE_REQUIRE_SCENE` to bypass a missing/invalid scene;
  repair the publisher/TF/marker data instead.

- `/scene_description` intentionally remains body-origin/body-orientation
  truth and is not a clearance-size contract. All planner clearance consumers
  must use `/scene_clearance_bounds`; do not reintroduce generic `0.1/0.05 m`
  fallback dimensions.

- Do not activate the MuJoCo simulator venv for ROS/grasp tests: its NumPy
  `1.26` breaks Humble `transforms3d` through removed `np.float`. Use system
  Python/NumPy for grasp tests and add the simulator venv site-packages only
  for MuJoCo-dependent simulator tests.

- A fast AABB profiler does not imply a fast bounds topic when publication is
  inside the camera/render loop. Under `llvmpipe`, all same-loop topics were
  about `1.48 Hz` despite AABB p95 `4.756 ms`; check both the component profile
  and end-to-end topic rate before unlocking A3.

- `ros2 topic hz` average can hide long render stalls. At `render_fps: 0.2`,
  averages exceeded `18 Hz` in two runs, but observed maximum state-message
  gaps reached about `1.262 s` and `1.988 s`. Evaluate maximum gap and state
  freshness before real motion; do not rely on average rate alone.

- Do not use 640x480 camera output with the current perception code as a pure
  YAML change: `RGBDPerceptionNode._init_camera_intrinsic()` is hard-coded for
  1280x720, so the image and intrinsic matrix would disagree.

- Fixed slot centers alone do not guarantee collision-free placement for long
  objects. Hammer at the old slot 5 `[-0.55, 0.20, 0.0]` overlaps banana at
  slot 2 and is ejected; keep the validated slot 5 Y coordinate at `0.30`.

- `random_object_count` is the target total selected-object count, not the
  number of random objects. Keep it at `6` for five fixed representatives plus
  one random object.

- After calling `/reset_sim`, rerun `ros2 run my_course_pkg pipeline` before
  `ros2 run my_course_pkg grasp_demo`; reset makes old perception output stale.
- `cell_small_full_mujoco_moveit.launch.py` already includes
  `arm_api2/moveit2_iface.launch.py`. Never start a standalone
  `ros2 launch arm_api2 moveit2_iface.launch.py` alongside it. The old
  per-trial `pkill -x moveit2_iface` plus standalone-relaunch procedure is
  retired; it kills a child executable without establishing durable ownership
  and can create duplicates after the full launch is restarted.
- `/reset_sim` does not reset `moveit2_iface` internal servo/interpolation
  state. Between independent formal motion trials, restart the entire full
  launch; do not rely on reset alone and do not substitute a separately owned
  interface.
- Do not assume Ctrl-C/restarting the full launch removed all children. An old
  gripper adapter, robot-state publisher, and servo watchdog were observed
  reparented to container PID 1 and remained live beside the new launch. If any
  exact node name or action server is duplicated after a launch restart,
  restart the Docker container before motion rather than stacking more launch
  processes.
- Side approach clearance currently defaults to only `0.05 m` vertical margin,
  despite the measured TCP-to-lowest-finger reach being about `0.104 m`. This
  allowed a tomato candidate approaching over the banana to pass `rejected=0`
  and then physically contact the banana. Until the code default and regression
  coverage are fixed, use `GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M=0.12` and
  require the unsafe banana-side candidate to be rejected before motion.
- `tomato_soup_can` uses the side profile, so `GRASP_Z_OFFSET` does not lower
  its final grasp. Use `SIDE_GRASP_Z_OFFSET`.
- In the current selector, `GRASP_SIDE_MIN_HEIGHT_M` and
  `GRASP_SIDE_MAX_HEIGHT_M` filter the raw grasp-library object-local height
  before `SIDE_GRASP_Z_OFFSET` is applied. Do not set the max to `0.075` with
  the current library, because that can remove all useful tomato-can candidates.
- Do not carry forward the old tomato height-window workaround blindly. After
  the geometry-center fix, the real tomato library has hundreds of candidates
  under `GRASP_SIDE_MAX_CENTERLINE_OFFSET_M=0.025` with the default side height
  range, and the top verified candidate was around `0.081 m` object-local Z.
  Start with defaults unless debugging a separate height-only issue.
- If lowering the tomato-can grasp point does not fix dropping, investigate
  side jaw-roll / closing-axis behavior instead of continuing to lower only Z.
- Do not over-trust stale `build/lib` or old `grasp_eval` artifacts; standard
  `install/setup.bash` currently puts `/home/ws/build/my_course_pkg` on
  `PYTHONPATH`, and that directory symlinks back to current `src`.
- Current `src` does not contain the old verify-step/metrics implementation
  shown in some trial logs. Confirm which code path is running before comparing
  new `grasp_demo` behavior to those logs.
- Do not treat a stationary robot under `GRASP_DEBUG_STOP_AT_GRASP=1` as a
  hang: the demo deliberately waits for Ctrl-C at the final pose. Use
  `PYTHONUNBUFFERED=1` when piping output through `tee` so the wait message and
  target-vs-actual TCP diagnostics appear immediately.
- Do not assume YCB object-local `[0,0,0]` is the visible object center. For
  `tomato_soup_can`, the mesh bbox center is about
  `[-0.009, 0.084, 0.051]` m, so side-grasp symmetry and centerline checks must
  use an object-specific center offset.
- After a debug stop, do not launch `grasp_demo` again against the same
  `pose_result.json`: the camera has moved, but the current code applies the
  current camera TF to the old FoundationPose camera pose. Rerun `pipeline`
  first (or restore the capture camera pose) until capture-time TF is saved.
- For formal A3 trials, do not place `pipeline` and `grasp_plan_only` as
  independent shell commands: a failed pipeline leaves the previous
  `pose_result.json` available and the plan-only command can still pass on that
  stale file. Join them with `&&` so planning runs only after fresh perception
  succeeds.
- In this container, global pytest plugin discovery can fail because `anyio`
  expects a newer pytest API. Run project tests with
  `PYTEST_DISABLE_PLUGIN_AUTOLOAD=1` unless that environment mismatch is
  repaired.
- A successful `pipeline` does not prove the arm controller is available: it
  only needs the simulation camera and can finish with all arm clients false.
  Check `/moveit2_iface` and the `/arm/state/current_pose` publisher before a
  grasp run after restarting the launch.
- In this Windows-mounted workspace, Python script executable bits under
  `/home/ws/src` may not change with `chmod`. If launch says an installed Python
  executable is missing even though the symlink exists, check whether the symlink
  target is actually executable.
- WSL can expose `/dev/dxg` and run `nvidia-smi` while Docker/MuJoCo still uses
  CPU OpenGL. Check for `llvmpipe-*` threads under `ros2_main.py`; if present,
  the renderer is software.
- Do not judge MuJoCo performance while multiple stale `ros2_main.py` processes
  are still running. Old orphaned simulations can consume more than 1000% CPU.
- `/scene_description` and `/scene_clearance_bounds` markers are published in
  their marker header frame (currently `base_link`), while grasp plans are
  commanded in `world`. Always transform marker centers into the planning
  frame before comparing them with pregrasp/final TCP poses.
- The workspace may contain unrelated dirty or untracked files. Do not revert
  them unless the user explicitly asks.

## Useful Commands

### Tuna Offline Qualification (No Motion)

These commands do not start ROS or MuJoCo. The real-calibration test currently
passes by proving the approved 5 mm gate rejects all 36 cells.

```bash
cd /home/ws
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
export PYTHONPATH=/home/ws/src/my_course_pkg:/home/ws/src/sim_pick_place
python3 -m pytest -q \
  src/my_course_pkg/test/test_tuna_*.py \
  src/my_course_pkg/test/test_trajectory_planner.py
python3 -m pytest -q \
  src/my_course_pkg/test/test_tuna_pick_place_planner.py::test_real_calibration_fails_closed_below_exact_five_mm_gate
```

Do not proceed to a MuJoCo Tuna gate until the second command is replaced by an
approved positive-envelope qualification and at least one candidate exceeds the
unchanged 5 mm minimum across the full open/preclamp/roll/allowed-close sweep.

### Default Lift-Return Trial

All validated grasp values are now defaults, so do not repeat their environment
prefixes. Between formal trials, first restart the interface. After the `pkill`
command, run the launch command in a dedicated terminal and keep it open:

```bash
pkill -INT -x moveit2_iface
sleep 2
pgrep -a -x moveit2_iface

cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
unset ROS_DOMAIN_ID
ros2 launch arm_api2 moveit2_iface.launch.py \
  robot_name:=ur \
  launch_joy:=false \
  launch_servo_watchdog:=false
```

In another terminal, require one server and one publisher before motion:

```bash
ros2 action info /arm/move_to_pose
ros2 topic info /arm/state/current_pose --verbose
```

Then run the trial without environment prefixes:

```bash
ros2 service call /reset_sim std_srvs/srv/Trigger "{}"
sleep 3
ros2 run my_course_pkg pipeline
ros2 run my_course_pkg grasp_demo
```

### Scene Clearance Verification Environments

Use system ROS Python for planner/evaluator tests. Use the MuJoCo venv only for
simulator tests that import MuJoCo; never combine its NumPy with ROS Humble
`transforms3d`.

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest \
  src/my_course_pkg/test/test_grasp_selector.py \
  src/my_course_pkg/test/test_pick_place_planner.py \
  src/my_course_pkg/test/test_grasp_executor_gripper_failures.py \
  src/my_course_pkg/test/test_trajectory_planner.py \
  src/my_course_pkg/test/test_grasp_eval.py -q

PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
PYTHONPATH=/home/ws/src/ifl_air_mujoco_sim:/home/ws/src/ifl_air_mujoco_sim/.venv/lib/python3.10/site-packages:$PYTHONPATH \
/usr/bin/python3 -m pytest \
  src/ifl_air_mujoco_sim/test/test_populate_scene_selection.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_bounds.py \
  src/ifl_air_mujoco_sim/test/test_scene_clearance_publisher.py -q
```

### Terminal 1: Simulation And MoveIt

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py
```

The normal command now prompts `Select scene mode [mix/random/assign]:`. For scripts
or non-interactive startup, pass exactly one explicit override:

```bash
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py scene_mode:=random
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py scene_mode:=mix
ros2 launch ifl_air_ur_launch cell_small_full_mujoco_moveit.launch.py scene_mode:=assign
```

`assign` then prints all 18 valid names and prompts for up to six required
objects in one comma-separated line. Spaces after commas are optional. For
example, `banana, hammer` assigns those objects to slots 1 and 2 and randomly
fills slots 3-6 without duplicates; a blank line makes all six random.

This full launch owns the single `moveit2_iface`. Do not start the standalone
`arm_api2 moveit2_iface.launch.py` in another terminal. Restart this entire
full launch between independent formal motion trials. Before motion,
`ros2 topic info /arm/state/current_pose --verbose` must report exactly one
publisher and `ros2 action info /arm/move_to_pose` exactly one action server.

### ROS Graph Uniqueness Gate

Before motion, exact node-name counts for `/moveit2_iface`,
`/robotiq_2f_urcap_adapter`, `/robot_state_publisher`, and
`/servo_watchdog_node` must each be `1`. Both `/arm/move_to_pose` and
`/robotiq_2f_urcap_adapter/gripper_command` must report one action server, and
`/arm/state/current_pose` must report one publisher. Action-client counts may
exceed one, and `/joint_states` legitimately has separate arm and gripper
publishers, so those are not uniqueness failures.

```bash
unset ROS_DOMAIN_ID
ros2 node list 2>/dev/null | sort | uniq -cd
ros2 node list 2>/dev/null | grep -E \
  '^/(moveit2_iface|robotiq_2f_urcap_adapter|robot_state_publisher|servo_watchdog_node)$' \
  | sort | uniq -c
ros2 action info /arm/move_to_pose
ros2 action info /robotiq_2f_urcap_adapter/gripper_command
ros2 topic info /arm/state/current_pose --verbose
ps -eo pid,ppid,args --forest | grep -E \
  '[r]os2 launch|[m]oveit2_iface|[r]obotiq_2f_adapter_node.py|[r]obot_state_publisher|[s]ervo_watchdog.py'
```

The first command must print nothing. The focused node list must show `1` for
all four names. The process tree must contain one full launch, one instance of
each child, and no standalone `arm_api2 moveit2_iface.launch.py`.

### MuJoCo WSL/Docker GPU Checks

From Windows PowerShell:

```powershell
wsl.exe -d Ubuntu-24.04 --exec /bin/bash -lc "ls -l /dev/dxg; /usr/lib/wsl/lib/nvidia-smi"
docker inspect mystifying_lamarr --format "DeviceRequests={{json .HostConfig.DeviceRequests}} Runtime={{.HostConfig.Runtime}}"
docker stats --no-stream mystifying_lamarr
docker exec mystifying_lamarr bash -lc "ps -eo pid,ppid,lstart,etime,pcpu,pmem,comm,args | grep '[r]os2_main.py'"
docker exec mystifying_lamarr bash -lc "ps -L -p <PID> -o pid,tid,pcpu,comm | sort -k3 -nr | head -20"
```

If the thread list contains `llvmpipe-*`, MuJoCo is using Mesa CPU software
rendering rather than WSLg/NVIDIA hardware OpenGL.

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

### Hammer Balanced-Lift Validation

After rebuilding, reset and run fresh perception for `pick up the hammer`.
Top-down Z offset and balance selection are defaults; do not add tuning
prefixes. The one debug flag allows the full five-second hold, then stops before
return/release so the balanced lift can be inspected safely:

```bash
ros2 service call /reset_sim std_srvs/srv/Trigger "{}"
sleep 3
ros2 run my_course_pkg pipeline
GRASP_DEBUG_STOP_AFTER_LIFT=1 ros2 run my_course_pkg grasp_demo
```

Require `Hammer balanced top-down selection`, selected balance distance at most
`0.005 m`, selected angle at most `10 degrees`, a fully clear hammer head, and
hammer long-axis tilt at most `10 degrees` throughout the five-second hold.

### Foam-Brick Lift-Return Trial

Run after the package has been rebuilt. Enter `pick up the foam brick` when the
pipeline prompts for the instruction. First stop immediately after close to
confirm that both fingers passed beside the brick. If that passes, run two
independent after-lift validations so a missed grasp cannot continue into return
and release. Restart the complete simulator launch between lift trials and
recheck unique arm/gripper servers before motion.

```bash
unset GRASP_EXECUTION_MODE
unset GRASP_PLACE_Y_MAX
unset GRASP_DEBUG_STOP_AT_GRASP
export GRASP_DEBUG_STOP_AFTER_CLOSE=1
unset GRASP_DEBUG_STOP_BEFORE_RELEASE
unset GRASP_DEBUG_STOP_AFTER_LIFT
unset GRASP_VERTICAL_REANCHOR_MAX_OFFSET_M
unset GRASP_VERTICAL_REANCHOR_MAX_ITERATIONS
unset GRASP_VERTICAL_REANCHOR_SETTLE_SEC
unset GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT

ros2 service call /reset_sim std_srvs/srv/Trigger "{}"
sleep 3
ros2 run my_course_pkg pipeline
PYTHONUNBUFFERED=1 ros2 run my_course_pkg grasp_demo 2>&1 \
  | tee /tmp/foam_brick_vertical_close_check.log
```

A qualifying bounded run logs `Vertical grasp target bounds`, `Vertical grasp
centering`, one or more bounded `Vertical pregrasp calibration command` lines,
a stationary `Vertical pregrasp feedback window`, a convergence check with
median/latest both passing, `calibration frozen for descent`, and nominal
waypoint gates passing. There must be no second `SERVO_POS_CTL` request between
freeze and the first waypoint. The logged `centered_xy` equals
`target_bounds_xy`; for the recorded scene, the centering correction should be
near `[-0.0312, +0.0005] m`, but fresh bounds are authoritative. The learned
command offset must stay within 30 mm. The gripper must descend through the
brick center and visibly capture it between the fingers. After that close-only
check, switch to the bounded lift stop and require two consecutive independent
runs in which the brick rises with the arm and avoids neighboring objects:

```bash
unset GRASP_DEBUG_STOP_AFTER_CLOSE
export GRASP_DEBUG_STOP_AFTER_LIFT=1
```

After both bounded lift runs pass, clear `GRASP_DEBUG_STOP_AFTER_LIFT` before a
normal full lift-return run.

### Apple A3 Plan-Only Trial

Each formal trial must start from a reset and fresh successful Apple pipeline.
Do not use `grasp_demo` or any debug-stop motion variable for A3.

```bash
ros2 service call /reset_sim std_srvs/srv/Trigger {}
ros2 run my_course_pkg pipeline

export GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC=1.5
ros2 run my_course_pkg grasp_plan_only 2>&1 | tee /tmp/apple_a3_plan_only.log
```

The pipeline must naturally select `apple`; keep `VLM_CANDIDATE_OVERRIDE` and
`FOUNDATIONPOSE_MASK_INDEX` unset. A qualifying plan-only run must report
`method=exact_bounds_box_2p5d`, at least one remaining corridor-safe candidate,
a MoveIt plan-only pass, and successful restoration of `planonly=False`.

### Terminal 3: Geometry-Center Tomato Debug Stop

```bash
cd /home/ws
source /opt/ros/humble/setup.bash
source install/setup.bash

unset ROS_DOMAIN_ID
export PYTHONUNBUFFERED=1
export GRASP_DEBUG_STOP_AT_GRASP=1
export GRASP_SIDE_MAX_CENTERLINE_OFFSET_M=0.025
export GRASP_SIDE_CENTERLINE_WEIGHT=300.0
export GRASP_INTERPOLATE_AVG_SPEED=0.05
export GRASP_WAYPOINT_MAX_DIST=0.02

ros2 run my_course_pkg grasp_demo 2>&1 | tee /tmp/grasp_geometry_center.log
```

Useful log check:

```bash
grep -E "Side-grasp approach clearance|selected_geometry_centerline|relative_to_geometry_center|Final approach" \
  /tmp/grasp_geometry_center.log
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

### Safe Place/Release Debug Stops

After a successful fresh `pipeline`, first verify the lifted hold:

```bash
unset GRASP_DEBUG_STOP_AT_GRASP
unset GRASP_DEBUG_STOP_AFTER_CLOSE
unset GRASP_DEBUG_STOP_BEFORE_RELEASE
export GRASP_DEBUG_STOP_AFTER_LIFT=1
ros2 run my_course_pkg grasp_demo 2>&1 | tee /tmp/grasp_after_lift.log
```

If the can stays lifted, inspect the planned drop pose before opening:

```bash
unset GRASP_DEBUG_STOP_AFTER_LIFT
export GRASP_DEBUG_STOP_BEFORE_RELEASE=1
ros2 run my_course_pkg grasp_demo 2>&1 | tee /tmp/grasp_before_release.log
```

For a full run:

```bash
unset GRASP_DEBUG_STOP_AFTER_LIFT
unset GRASP_DEBUG_STOP_BEFORE_RELEASE
ros2 run my_course_pkg grasp_demo 2>&1 | tee /tmp/grasp_full_place.log
```

## Maintenance Rule

At the end of a meaningful work session, update this file when the work changes
project state, command workflow, verification status, or reusable debugging
knowledge. Keep entries short and factual.
