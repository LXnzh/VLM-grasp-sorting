import argparse
import csv
from dataclasses import dataclass
from datetime import datetime
import json
import os
from pathlib import Path
import subprocess
import time

import numpy as np

from my_course_pkg.grasp.config import DROP_POSITION
from my_course_pkg.paths import OUTPUT_DIR


RESET_COMMAND = ["ros2", "service", "call", "/reset_sim", "std_srvs/srv/Trigger", "{}"]
PIPELINE_COMMAND = ["ros2", "run", "my_course_pkg", "pipeline"]
GRASP_COMMAND = ["ros2", "run", "my_course_pkg", "grasp_demo"]
REQUIRED_CONSECUTIVE_SUCCESSES = 2


@dataclass(frozen=True)
class CommandResult:
    returncode: int
    timed_out: bool = False
    message: str = ""

    @property
    def ok(self):
        return self.returncode == 0 and not self.timed_out


@dataclass(frozen=True)
class AutoVerdict:
    success: bool
    reason: str
    drop_xy_distance_m: float | None = None


@dataclass(frozen=True)
class HumanConfirmation:
    human_success: bool | None
    final_success: bool
    final_reason: str
    stop_requested: bool = False


@dataclass(frozen=True)
class EvalConfig:
    trials: int
    object_name: str
    instruction: str
    output_dir: Path
    drop_xyz: np.ndarray
    drop_xy_tolerance_m: float
    min_object_z_m: float
    reset_timeout_sec: float
    pipeline_timeout_sec: float
    grasp_timeout_sec: float
    settle_sec: float


@dataclass(frozen=True)
class TrialResult:
    trial: int
    object_name: str
    auto_success: bool
    human_success: bool | None
    final_success: bool
    auto_reason: str
    final_reason: str
    reset_ok: bool
    pipeline_ok: bool
    grasp_ok: bool
    object_start_xyz: np.ndarray | None
    object_end_xyz: np.ndarray | None
    drop_xyz: np.ndarray
    duration_sec: float
    stop_requested: bool


def canonical_object_name(name):
    return " ".join(str(name).strip().lower().replace("_", " ").split())


def estimate_auto_success(
    *,
    pipeline_ok,
    grasp_ok,
    object_end_xyz,
    drop_xyz,
    drop_xy_tolerance_m,
    min_object_z_m,
):
    if not pipeline_ok:
        return AutoVerdict(False, "pipeline_failed")
    if not grasp_ok:
        return AutoVerdict(False, "grasp_failed")
    if object_end_xyz is None:
        return AutoVerdict(False, "object_pose_unavailable")

    object_end_xyz = np.asarray(object_end_xyz, dtype=float)
    drop_xyz = np.asarray(drop_xyz, dtype=float)
    if object_end_xyz[2] < min_object_z_m:
        return AutoVerdict(False, "object_below_min_z")

    drop_xy_distance = float(np.linalg.norm(object_end_xyz[:2] - drop_xyz[:2]))
    if drop_xy_distance <= float(drop_xy_tolerance_m):
        return AutoVerdict(True, "object_near_drop", drop_xy_distance)
    return AutoVerdict(False, "object_not_near_drop", drop_xy_distance)


def apply_human_confirmation(choice, *, auto_success):
    normalized = str(choice).strip().lower()
    if normalized == "a":
        return HumanConfirmation(None, bool(auto_success), "accepted_auto")
    if normalized == "y":
        return HumanConfirmation(True, True, "human_marked_success")
    if normalized == "n":
        return HumanConfirmation(False, False, "human_marked_failure")
    if normalized == "q":
        return HumanConfirmation(
            None,
            bool(auto_success),
            "stop_requested_accepted_auto",
            stop_requested=True,
        )
    raise ValueError("Expected one of: y, n, a, q")


def bool_text(value):
    if value is None:
        return ""
    return "true" if bool(value) else "false"


def xyz_text(value):
    if value is None:
        return ""
    array = np.asarray(value, dtype=float).reshape(3)
    return json.dumps([float(item) for item in array])


def trial_result_row(result):
    return {
        "trial": result.trial,
        "object_name": result.object_name,
        "auto_success": bool_text(result.auto_success),
        "human_success": bool_text(result.human_success),
        "final_success": bool_text(result.final_success),
        "auto_reason": result.auto_reason,
        "final_reason": result.final_reason,
        "reset_ok": bool_text(result.reset_ok),
        "pipeline_ok": bool_text(result.pipeline_ok),
        "grasp_ok": bool_text(result.grasp_ok),
        "object_start_xyz": xyz_text(result.object_start_xyz),
        "object_end_xyz": xyz_text(result.object_end_xyz),
        "drop_xyz": xyz_text(result.drop_xyz),
        "duration_sec": f"{result.duration_sec:.3f}",
        "stop_requested": bool_text(result.stop_requested),
    }


def append_trial_result(csv_path, result):
    csv_path = Path(csv_path)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    row = trial_result_row(result)
    write_header = not csv_path.exists()
    with csv_path.open("a", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(row.keys()))
        if write_header:
            writer.writeheader()
        writer.writerow(row)


def run_command(command, *, timeout_sec, env=None, input_text=None):
    try:
        completed = subprocess.run(
            list(command),
            input=input_text,
            text=True,
            env=env,
            timeout=float(timeout_sec),
            check=False,
        )
        return CommandResult(returncode=int(completed.returncode))
    except subprocess.TimeoutExpired:
        return CommandResult(
            returncode=124,
            timed_out=True,
            message=f"Timed out after {timeout_sec:.1f}s",
        )


def build_trial_env(object_name):
    env = dict(os.environ)
    env["VLM_CANDIDATE_OVERRIDE"] = object_name
    env.pop("GRASP_DEBUG_STOP_AT_GRASP", None)
    env.pop("GRASP_DEBUG_STOP_AFTER_CLOSE", None)
    return env


def _scene_key_for_object(objects_info, object_name):
    wanted = canonical_object_name(object_name)
    for key in objects_info:
        if canonical_object_name(key) == wanted:
            return key
    return None


def read_scene_object_pose(object_name, timeout_sec=5.0):
    import rclpy
    from rclpy.executors import MultiThreadedExecutor
    from sim_pick_place.sim_client_node import SimClientNode

    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init(args=None)

    node = SimClientNode("grasp_eval_scene_probe")
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        deadline = time.monotonic() + float(timeout_sec)
        while rclpy.ok() and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
            scene_key = _scene_key_for_object(node._get_objects_info(), object_name)
            if scene_key is None:
                continue
            return np.asarray(node._world_pose_for(scene_key)[:3], dtype=float)
        raise RuntimeError(f"Object {object_name!r} was not found in /scene_description.")
    finally:
        executor.shutdown()
        executor.remove_node(node)
        node.destroy_node()
        if initialized_here and rclpy.ok():
            rclpy.shutdown()


def read_pose_safely(pose_reader, object_name, timeout_sec):
    try:
        return pose_reader(object_name, timeout_sec)
    except Exception as exc:
        print(f"[grasp_eval] Could not read object pose for {object_name!r}: {exc}")
        return None


def prompt_human_confirmation(prompt):
    while True:
        choice = input(prompt)
        try:
            apply_human_confirmation(choice, auto_success=False)
            return choice
        except ValueError:
            print("Please enter y, n, a, or q.")


def run_trial(
    *,
    trial,
    config,
    command_runner=run_command,
    pose_reader=read_scene_object_pose,
    prompt_fn=prompt_human_confirmation,
    sleep_fn=time.sleep,
    monotonic_fn=time.monotonic,
):
    start_time = monotonic_fn()
    env = build_trial_env(config.object_name)

    print(f"\n[grasp_eval] Trial {trial}/{config.trials}: reset simulation")
    reset_result = command_runner(
        RESET_COMMAND,
        timeout_sec=config.reset_timeout_sec,
        env=env,
    )
    sleep_fn(config.settle_sec)
    object_start_xyz = read_pose_safely(pose_reader, config.object_name, 5.0)

    print(f"[grasp_eval] Trial {trial}/{config.trials}: run perception pipeline")
    pipeline_result = command_runner(
        PIPELINE_COMMAND,
        timeout_sec=config.pipeline_timeout_sec,
        env=env,
        input_text=f"{config.instruction}\n",
    )

    if pipeline_result.ok:
        print(f"[grasp_eval] Trial {trial}/{config.trials}: run grasp_demo")
        grasp_result = command_runner(
            GRASP_COMMAND,
            timeout_sec=config.grasp_timeout_sec,
            env=env,
        )
    else:
        grasp_result = CommandResult(returncode=1, message="Skipped after pipeline failure")

    sleep_fn(config.settle_sec)
    object_end_xyz = read_pose_safely(pose_reader, config.object_name, 5.0)
    auto_verdict = estimate_auto_success(
        pipeline_ok=pipeline_result.ok,
        grasp_ok=grasp_result.ok,
        object_end_xyz=object_end_xyz,
        drop_xyz=config.drop_xyz,
        drop_xy_tolerance_m=config.drop_xy_tolerance_m,
        min_object_z_m=config.min_object_z_m,
    )

    print(
        "[grasp_eval] Auto verdict: "
        f"{'success' if auto_verdict.success else 'failure'} "
        f"({auto_verdict.reason})"
    )
    if auto_verdict.drop_xy_distance_m is not None:
        print(
            "[grasp_eval] Object-drop XY distance: "
            f"{auto_verdict.drop_xy_distance_m:.3f} m"
        )

    choice = prompt_fn(
        "Watch the animation. Mark result "
        "[y=success, n=failure, a=accept auto, q=quit]: "
    )
    confirmation = apply_human_confirmation(choice, auto_success=auto_verdict.success)

    return TrialResult(
        trial=int(trial),
        object_name=config.object_name,
        auto_success=auto_verdict.success,
        human_success=confirmation.human_success,
        final_success=confirmation.final_success,
        auto_reason=auto_verdict.reason,
        final_reason=confirmation.final_reason,
        reset_ok=reset_result.ok,
        pipeline_ok=pipeline_result.ok,
        grasp_ok=grasp_result.ok,
        object_start_xyz=object_start_xyz,
        object_end_xyz=object_end_xyz,
        drop_xyz=np.asarray(config.drop_xyz, dtype=float),
        duration_sec=float(monotonic_fn() - start_time),
        stop_requested=confirmation.stop_requested,
    )


def make_output_csv_path(output_dir):
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    return Path(output_dir) / f"grasp_eval_{timestamp}.csv"


def compute_consecutive_streaks(results):
    """Return current streak, maximum streak, and qualification state."""
    current_streak = 0
    max_streak = 0
    for result in results:
        if bool(result.final_success):
            current_streak += 1
            max_streak = max(max_streak, current_streak)
        else:
            current_streak = 0
    return (
        current_streak,
        max_streak,
        max_streak >= REQUIRED_CONSECUTIVE_SUCCESSES,
    )


def summarize_results(results):
    if not results:
        return "No trials were recorded."
    final_successes = sum(1 for result in results if result.final_success)
    auto_successes = sum(1 for result in results if result.auto_success)
    total = len(results)
    _, max_streak, qualification_passed = compute_consecutive_streaks(results)
    qualification = "PASS" if qualification_passed else "NOT PASS"
    return (
        f"Trials: {total}\n"
        f"Human-confirmed success: {final_successes}/{total} = "
        f"{final_successes / total * 100.0:.1f}%\n"
        f"Auto estimate success: {auto_successes}/{total} = "
        f"{auto_successes / total * 100.0:.1f}%\n"
        f"Qualification: {qualification} (max streak {max_streak}/"
        f"{REQUIRED_CONSECUTIVE_SUCCESSES})"
    )


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description="Run repeated grasp trials.")
    default_object = os.environ.get("VLM_CANDIDATE_OVERRIDE", "tomato soup can")
    parser.add_argument(
        "--trials",
        type=int,
        default=REQUIRED_CONSECUTIVE_SUCCESSES,
    )
    parser.add_argument("--object", dest="object_name", default=default_object)
    parser.add_argument("--instruction", default=None)
    parser.add_argument(
        "--output-dir",
        type=Path,
        default=OUTPUT_DIR / "grasp_eval",
    )
    parser.add_argument("--drop-xy-tolerance", type=float, default=0.15)
    parser.add_argument("--min-object-z", type=float, default=-0.05)
    parser.add_argument("--reset-timeout", type=float, default=10.0)
    parser.add_argument("--pipeline-timeout", type=float, default=420.0)
    parser.add_argument("--grasp-timeout", type=float, default=240.0)
    parser.add_argument("--settle-sec", type=float, default=1.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    object_name = args.object_name
    instruction = args.instruction or f"pick up the {object_name}"
    config = EvalConfig(
        trials=max(1, int(args.trials)),
        object_name=object_name,
        instruction=instruction,
        output_dir=args.output_dir,
        drop_xyz=np.asarray(DROP_POSITION, dtype=float),
        drop_xy_tolerance_m=float(args.drop_xy_tolerance),
        min_object_z_m=float(args.min_object_z),
        reset_timeout_sec=float(args.reset_timeout),
        pipeline_timeout_sec=float(args.pipeline_timeout),
        grasp_timeout_sec=float(args.grasp_timeout),
        settle_sec=float(args.settle_sec),
    )
    csv_path = make_output_csv_path(config.output_dir)
    print(f"[grasp_eval] Writing results to {csv_path}")

    results = []
    for trial in range(1, config.trials + 1):
        result = run_trial(trial=trial, config=config)
        append_trial_result(csv_path, result)
        results.append(result)
        print(f"[grasp_eval] Recorded trial {trial}: final_success={result.final_success}")
        if result.stop_requested:
            print("[grasp_eval] Stop requested; ending evaluation.")
            break

    print("\n" + summarize_results(results))
    print(f"CSV: {csv_path}")


if __name__ == "__main__":
    main()
