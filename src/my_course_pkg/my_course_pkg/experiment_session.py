#!/usr/bin/env python3
"""Persistent reset-perception-grasp experiment session supervisor."""

import argparse
from collections import Counter
from dataclasses import dataclass
from datetime import datetime
import os
from pathlib import Path
import queue
import re
import shlex
import signal
import subprocess
import sys
import threading
import time


RESET_COMMAND = [
    "ros2",
    "service",
    "call",
    "/reset_sim",
    "std_srvs/srv/Trigger",
    "{}",
]
PIPELINE_COMMAND = ["ros2", "run", "my_course_pkg", "pipeline"]
GRASP_COMMAND = ["ros2", "run", "my_course_pkg", "grasp_demo"]
INTERFACE_COMMAND = [
    "ros2",
    "launch",
    "arm_api2",
    "moveit2_iface.launch.py",
    "robot_name:=ur",
    "launch_joy:=false",
    "launch_servo_watchdog:=false",
]
PGREP_COMMAND = ["pgrep", "-a", "-x", "moveit2_iface"]

REQUIRED_NODE_NAMES = (
    "/moveit2_iface",
    "/robotiq_2f_urcap_adapter",
    "/robot_state_publisher",
    "/move_group",
)
OPTIONAL_UNIQUE_NODE_NAMES = ("/servo_watchdog_node",)
MANUAL_OVERRIDE_ENVIRONMENT_KEYS = (
    "ROS_DOMAIN_ID",
    "VLM_CANDIDATE_OVERRIDE",
    "FOUNDATIONPOSE_MASK_INDEX",
)


@dataclass(frozen=True)
class StageResult:
    stage: str
    returncode: int
    output: str = ""
    timed_out: bool = False
    interrupted: bool = False
    message: str = ""
    log_path: Path | None = None

    @property
    def ok(self):
        return self.returncode == 0 and not self.timed_out and not self.interrupted


@dataclass(frozen=True)
class GraphEvaluation:
    ready: bool
    duplicate: bool
    message: str


@dataclass(frozen=True)
class SessionConfig:
    log_root: Path = Path("/tmp/my_course_experiment_sessions")
    settle_sec: float = 3.0
    interface_ready_timeout_sec: float = 180.0
    interface_stop_timeout_sec: float = 8.0
    graph_query_timeout_sec: float = 20.0
    graph_duplicate_settle_sec: float = 30.0
    graph_required_clean_samples: int = 2
    reset_timeout_sec: float = 20.0
    pipeline_timeout_sec: float = 600.0
    grasp_timeout_sec: float = 600.0
    graph_poll_sec: float = 0.5

    def __post_init__(self):
        duplicate_settle_sec = float(self.graph_duplicate_settle_sec)
        required_clean_samples = int(self.graph_required_clean_samples)
        if duplicate_settle_sec < 0.0:
            raise ValueError("graph_duplicate_settle_sec must be non-negative")
        if required_clean_samples < 1:
            raise ValueError("graph_required_clean_samples must be at least 1")
        object.__setattr__(
            self,
            "graph_duplicate_settle_sec",
            duplicate_settle_sec,
        )
        object.__setattr__(
            self,
            "graph_required_clean_samples",
            required_clean_samples,
        )


class SessionLogManager:
    def __init__(self, root):
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        self.session_dir = (Path(root) / stamp).resolve()
        self.session_dir.mkdir(parents=True, exist_ok=False)

    def stage_path(self, trial, stage):
        safe_stage = re.sub(r"[^a-zA-Z0-9_.-]+", "_", str(stage)).strip("_")
        trial_dir = self.session_dir / f"trial_{int(trial):03d}"
        trial_dir.mkdir(parents=True, exist_ok=True)
        return (trial_dir / f"{safe_stage}.log").resolve()


def build_trial_environment(environ):
    child_env = dict(environ)
    for key in MANUAL_OVERRIDE_ENVIRONMENT_KEYS:
        child_env.pop(key, None)
    return child_env


def trigger_response_succeeded(output):
    return bool(
        re.search(
            r"\bsuccess\s*(?:=|:)\s*(?:True|true)\b",
            str(output),
        )
    )


def parse_action_server_count(output):
    match = re.search(r"Action servers:\s*(\d+)", str(output))
    return int(match.group(1)) if match else 0


def parse_topic_publisher_count(output):
    match = re.search(r"Publisher count:\s*(\d+)", str(output))
    return int(match.group(1)) if match else 0


def parse_node_names(output):
    return [
        line.strip()
        for line in str(output).splitlines()
        if line.strip().startswith("/")
    ]


def evaluate_graph_snapshot(
    *,
    node_names,
    arm_action_servers,
    gripper_action_servers,
    pose_publishers,
):
    counts = Counter(node_names)
    duplicate_items = []
    missing_items = []

    for name in REQUIRED_NODE_NAMES:
        if counts[name] > 1:
            duplicate_items.append(f"node {name}={counts[name]}")
        elif counts[name] < 1:
            missing_items.append(f"node {name}=0")
    for name in OPTIONAL_UNIQUE_NODE_NAMES:
        if counts[name] > 1:
            duplicate_items.append(f"node {name}={counts[name]}")

    endpoint_counts = {
        "arm action servers": int(arm_action_servers),
        "gripper action servers": int(gripper_action_servers),
        "pose publishers": int(pose_publishers),
    }
    for label, count in endpoint_counts.items():
        if count > 1:
            duplicate_items.append(f"{label}={count}")
        elif count < 1:
            missing_items.append(f"{label}=0")

    if duplicate_items:
        return GraphEvaluation(
            ready=False,
            duplicate=True,
            message="duplicate ROS graph endpoints: " + ", ".join(duplicate_items),
        )
    if missing_items:
        return GraphEvaluation(
            ready=False,
            duplicate=False,
            message="waiting for ROS graph endpoints: " + ", ".join(missing_items),
        )
    return GraphEvaluation(
        ready=True,
        duplicate=False,
        message="ROS graph uniqueness/readiness gate passed",
    )


def _screen_writer(text):
    print(text, end="", flush=True)


def _append_text(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(str(text))


def _command_text(command):
    return shlex.join(str(item) for item in command)


def _wait_or_timeout(process, timeout):
    try:
        process.wait(timeout=max(0.0, float(timeout)))
        return True
    except subprocess.TimeoutExpired:
        return False


def _terminate_owned_group(
    process,
    *,
    pgid,
    killpg_fn=os.killpg,
    timeout_sec=2.0,
):
    if process.poll() is not None:
        return
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGKILL):
        try:
            killpg_fn(pgid, sig)
        except ProcessLookupError:
            return
        if _wait_or_timeout(process, timeout_sec):
            return


class TeeCommandRunner:
    def __init__(
        self,
        *,
        popen_factory=subprocess.Popen,
        killpg_fn=os.killpg,
        output_fn=None,
        monotonic_fn=time.monotonic,
    ):
        self._popen_factory = popen_factory
        self._killpg_fn = killpg_fn
        self._output_fn = _screen_writer if output_fn is None else output_fn
        self._monotonic_fn = monotonic_fn

    def run(
        self,
        stage,
        command,
        *,
        timeout_sec,
        env,
        input_text=None,
        log_path,
    ):
        log_path = Path(log_path)
        _append_text(log_path, f"$ {_command_text(command)}\n")
        try:
            process = self._popen_factory(
                list(command),
                stdin=subprocess.PIPE if input_text is not None else subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=dict(env),
                start_new_session=True,
            )
        except OSError as exc:
            message = f"could not start command: {exc}"
            _append_text(log_path, message + "\n")
            return StageResult(
                stage=stage,
                returncode=127,
                message=message,
                log_path=log_path,
            )

        if input_text is not None:
            try:
                process.stdin.write(str(input_text))
                process.stdin.flush()
            finally:
                process.stdin.close()

        output_queue = queue.Queue()
        output_parts = []

        def _read_output():
            try:
                for chunk in iter(process.stdout.readline, ""):
                    output_queue.put(chunk)
            finally:
                output_queue.put(None)

        reader = threading.Thread(
            target=_read_output,
            name=f"experiment-session-{stage}-output",
            daemon=True,
        )
        reader.start()

        deadline = self._monotonic_fn() + float(timeout_sec)
        saw_eof = False
        timed_out = False
        interrupted = False
        try:
            while True:
                try:
                    chunk = output_queue.get(timeout=0.05)
                except queue.Empty:
                    chunk = ""
                if chunk is None:
                    saw_eof = True
                elif chunk:
                    output_parts.append(chunk)
                    _append_text(log_path, chunk)
                    self._output_fn(chunk)

                if process.poll() is not None and saw_eof:
                    break
                if self._monotonic_fn() >= deadline and process.poll() is None:
                    timed_out = True
                    _terminate_owned_group(
                        process,
                        pgid=process.pid,
                        killpg_fn=self._killpg_fn,
                    )
                    break
        except KeyboardInterrupt:
            interrupted = True
            _terminate_owned_group(
                process,
                pgid=process.pid,
                killpg_fn=self._killpg_fn,
            )
        finally:
            if process.poll() is None:
                _terminate_owned_group(
                    process,
                    pgid=process.pid,
                    killpg_fn=self._killpg_fn,
                )
            reader.join(timeout=2.0)

        output = "".join(output_parts)
        returncode = 124 if timed_out else int(process.returncode or 0)
        if interrupted:
            returncode = 130
        message = ""
        if timed_out:
            message = f"timed out after {float(timeout_sec):.1f}s"
        elif interrupted:
            message = "interrupted"
        elif returncode != 0:
            message = f"command exited with code {returncode}"
        return StageResult(
            stage=stage,
            returncode=returncode,
            output=output,
            timed_out=timed_out,
            interrupted=interrupted,
            message=message,
            log_path=log_path,
        )


class OwnedInterfaceProcess:
    def __init__(
        self,
        *,
        log_manager,
        env=None,
        command=INTERFACE_COMMAND,
        popen_factory=subprocess.Popen,
        killpg_fn=os.killpg,
        getpgid_fn=os.getpgid,
        output_fn=None,
        mirror_output=True,
        stop_timeout_sec=8.0,
    ):
        self._logs = log_manager
        self._env = dict(os.environ if env is None else env)
        self._command = list(command)
        self._popen_factory = popen_factory
        self._killpg_fn = killpg_fn
        self._getpgid_fn = getpgid_fn
        self._output_fn = _screen_writer if output_fn is None else output_fn
        self._mirror_output = bool(mirror_output)
        self._stop_timeout_sec = float(stop_timeout_sec)
        self._process = None
        self._pgid = None
        self._reader = None
        self._log_handle = None
        self._log_path = None

    def has_owned_process(self):
        return self._process is not None

    def _pump_output(self):
        try:
            for line in self._process.stdout:
                self._log_handle.write(line)
                self._log_handle.flush()
                if self._mirror_output:
                    self._output_fn(line)
        finally:
            self._log_handle.flush()

    def start(self, trial):
        stage = "start_interface"
        if self._process is not None:
            return StageResult(
                stage=stage,
                returncode=1,
                message="an owned interface process already exists",
                log_path=self._log_path,
            )

        self._log_path = self._logs.stage_path(trial, "moveit2_iface")
        self._log_handle = self._log_path.open("a", encoding="utf-8")
        self._log_handle.write(f"$ {_command_text(self._command)}\n")
        self._log_handle.flush()
        try:
            self._process = self._popen_factory(
                self._command,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=self._env,
                start_new_session=True,
            )
            self._pgid = self._getpgid_fn(self._process.pid)
        except OSError as exc:
            if self._process is not None:
                _terminate_owned_group(
                    self._process,
                    pgid=self._process.pid,
                    killpg_fn=self._killpg_fn,
                    timeout_sec=self._stop_timeout_sec,
                )
            self._log_handle.write(f"could not start interface: {exc}\n")
            self._log_handle.close()
            self._log_handle = None
            self._process = None
            self._pgid = None
            return StageResult(
                stage=stage,
                returncode=127,
                message=f"could not start interface: {exc}",
                log_path=self._log_path,
            )

        self._reader = threading.Thread(
            target=self._pump_output,
            name="experiment-session-interface-output",
            daemon=True,
        )
        self._reader.start()
        return StageResult(stage=stage, returncode=0, log_path=self._log_path)

    def stop(self, trial):
        del trial
        stage = "stop_interface"
        if self._process is None:
            return StageResult(stage=stage, returncode=0)

        process = self._process
        pgid = self._pgid
        _terminate_owned_group(
            process,
            pgid=pgid,
            killpg_fn=self._killpg_fn,
            timeout_sec=self._stop_timeout_sec,
        )
        if process.poll() is None:
            result = StageResult(
                stage=stage,
                returncode=1,
                message="owned interface process group did not exit",
                log_path=self._log_path,
            )
        else:
            result = StageResult(
                stage=stage,
                returncode=0,
                log_path=self._log_path,
            )

        if self._reader is not None:
            self._reader.join(timeout=2.0)
        if self._log_handle is not None:
            self._log_handle.close()
        self._process = None
        self._pgid = None
        self._reader = None
        self._log_handle = None
        return result

    def cleanup(self):
        return self.stop(0)


class ExactProcessProbe:
    def __init__(self, *, run_fn=subprocess.run):
        self._run_fn = run_fn

    def verify_empty(self, trial, log_path):
        del trial
        log_path = Path(log_path)
        try:
            completed = self._run_fn(
                PGREP_COMMAND,
                capture_output=True,
                text=True,
                check=False,
                timeout=5.0,
            )
        except (OSError, subprocess.TimeoutExpired) as exc:
            message = f"could not verify interface processes: {exc}"
            _append_text(log_path, message + "\n")
            return StageResult(
                stage="verify_empty",
                returncode=1,
                message=message,
                log_path=log_path,
            )

        output = (completed.stdout or "") + (completed.stderr or "")
        _append_text(log_path, f"$ {_command_text(PGREP_COMMAND)}\n{output}")
        if completed.returncode == 1 and not (completed.stdout or "").strip():
            return StageResult(
                stage="verify_empty",
                returncode=0,
                output=output,
                log_path=log_path,
            )
        if completed.returncode == 0:
            stale = (completed.stdout or "").strip() or "unknown moveit2_iface"
            return StageResult(
                stage="verify_empty",
                returncode=1,
                output=output,
                message=f"foreign/stale interface process remains: {stale}",
                log_path=log_path,
            )
        return StageResult(
            stage="verify_empty",
            returncode=int(completed.returncode),
            output=output,
            message=f"pgrep failed with code {completed.returncode}",
            log_path=log_path,
        )


class RosGraphProbe:
    def __init__(
        self,
        *,
        run_fn=subprocess.run,
        sleep_fn=time.sleep,
        monotonic_fn=time.monotonic,
        poll_sec=0.5,
        query_timeout_sec=20.0,
        duplicate_settle_sec=30.0,
        required_clean_samples=2,
    ):
        self._run_fn = run_fn
        self._sleep_fn = sleep_fn
        self._monotonic_fn = monotonic_fn
        self._poll_sec = float(poll_sec)
        self._query_timeout_sec = float(query_timeout_sec)
        self._duplicate_settle_sec = max(0.0, float(duplicate_settle_sec))
        self._required_clean_samples = max(1, int(required_clean_samples))

    def _command_output(self, command):
        completed = self._run_fn(
            command,
            capture_output=True,
            text=True,
            check=False,
            timeout=self._query_timeout_sec,
        )
        return completed.returncode, (completed.stdout or "") + (
            completed.stderr or ""
        )

    def wait_until_ready(self, trial, timeout_sec, log_path):
        del trial
        log_path = Path(log_path)
        deadline = self._monotonic_fn() + float(timeout_sec)
        duplicate_deadline = None
        clean_samples = 0
        last_message = "ROS graph has not been checked"
        while True:
            try:
                node_rc, node_output = self._command_output(
                    ["ros2", "node", "list"]
                )
                arm_rc, arm_output = self._command_output(
                    ["ros2", "action", "info", "/arm/move_to_pose"]
                )
                gripper_rc, gripper_output = self._command_output(
                    [
                        "ros2",
                        "action",
                        "info",
                        "/robotiq_2f_urcap_adapter/gripper_command",
                    ]
                )
                pose_rc, pose_output = self._command_output(
                    [
                        "ros2",
                        "topic",
                        "info",
                        "/arm/state/current_pose",
                        "--verbose",
                    ]
                )
            except (OSError, subprocess.TimeoutExpired) as exc:
                last_message = f"ROS graph probe failed: {exc}"
                evaluation = GraphEvaluation(False, False, last_message)
            else:
                _append_text(
                    log_path,
                    (
                        "\n$ ros2 node list\n"
                        + node_output
                        + "\n$ ros2 action info /arm/move_to_pose\n"
                        + arm_output
                        + "\n$ ros2 action info "
                        "/robotiq_2f_urcap_adapter/gripper_command\n"
                        + gripper_output
                        + "\n$ ros2 topic info /arm/state/current_pose --verbose\n"
                        + pose_output
                    ),
                )
                if any(rc != 0 for rc in (node_rc, arm_rc, gripper_rc, pose_rc)):
                    evaluation = GraphEvaluation(
                        ready=False,
                        duplicate=False,
                        message="one or more ROS graph queries are not ready",
                    )
                else:
                    evaluation = evaluate_graph_snapshot(
                        node_names=parse_node_names(node_output),
                        arm_action_servers=parse_action_server_count(arm_output),
                        gripper_action_servers=parse_action_server_count(
                            gripper_output
                        ),
                        pose_publishers=parse_topic_publisher_count(pose_output),
                    )
                last_message = evaluation.message

            if evaluation.ready:
                clean_samples += 1
                clean_message = (
                    "clean ROS graph sample "
                    f"{clean_samples}/{self._required_clean_samples}"
                )
                _append_text(log_path, clean_message + "\n")
                if clean_samples >= self._required_clean_samples:
                    message = (
                        evaluation.message
                        + "; consecutive clean samples="
                        + str(clean_samples)
                    )
                    _append_text(log_path, message + "\n")
                    return StageResult(
                        stage="graph_ready",
                        returncode=0,
                        message=message,
                        log_path=log_path,
                    )
            else:
                clean_samples = 0

            now = self._monotonic_fn()
            if evaluation.duplicate:
                if duplicate_deadline is None:
                    duplicate_deadline = now + self._duplicate_settle_sec
                    _append_text(
                        log_path,
                        (
                            "duplicate settle window started: "
                            f"{self._duplicate_settle_sec:.1f}s; "
                            f"{evaluation.message}\n"
                        ),
                    )
                else:
                    remaining = max(0.0, duplicate_deadline - now)
                    _append_text(
                        log_path,
                        (
                            "duplicate ROS graph endpoints still present; "
                            f"settle remaining={remaining:.1f}s; "
                            f"{evaluation.message}\n"
                        ),
                    )
                if now >= duplicate_deadline:
                    message = (
                        evaluation.message
                        + "; duplicate settle window expired after "
                        + f"{self._duplicate_settle_sec:.1f}s"
                    )
                    _append_text(log_path, message + "\n")
                    return StageResult(
                        stage="graph_ready",
                        returncode=1,
                        message=message,
                        log_path=log_path,
                    )
            if now >= deadline:
                message = f"ROS graph readiness timed out: {last_message}"
                _append_text(log_path, message + "\n")
                return StageResult(
                    stage="graph_ready",
                    returncode=1,
                    timed_out=True,
                    message=message,
                    log_path=log_path,
                )
            self._sleep_fn(self._poll_sec)


class ExperimentSession:
    def __init__(
        self,
        *,
        config=None,
        interface_manager=None,
        process_probe=None,
        graph_probe=None,
        command_runner=None,
        input_fn=None,
        output_fn=None,
        sleep_fn=time.sleep,
        environ=None,
    ):
        self.config = SessionConfig() if config is None else config
        self.logs = SessionLogManager(self.config.log_root)
        self.env = build_trial_environment(
            os.environ if environ is None else environ
        )
        self.output_fn = (
            (lambda message: print(message, flush=True))
            if output_fn is None
            else output_fn
        )
        self.input_fn = input if input_fn is None else input_fn
        self.sleep_fn = sleep_fn
        self.command_runner = (
            TeeCommandRunner() if command_runner is None else command_runner
        )
        self.interface_manager = interface_manager or OwnedInterfaceProcess(
            log_manager=self.logs,
            env=self.env,
            mirror_output=False,
            stop_timeout_sec=self.config.interface_stop_timeout_sec,
        )
        self.process_probe = process_probe or ExactProcessProbe()
        self.graph_probe = graph_probe or RosGraphProbe(
            poll_sec=self.config.graph_poll_sec,
            query_timeout_sec=self.config.graph_query_timeout_sec,
            duplicate_settle_sec=self.config.graph_duplicate_settle_sec,
            required_clean_samples=self.config.graph_required_clean_samples,
        )

    def _require(self, result):
        if not result.ok:
            return result
        return None

    def _prompt_instruction(self):
        while True:
            instruction = str(self._read_input("Instruction:")).strip()
            if instruction:
                return instruction
            self.output_fn("Instruction cannot be empty.")

    def _read_input(self, prompt):
        self.output_fn(prompt)
        return self.input_fn("")

    def _run_trial(self, trial):
        if self.interface_manager.has_owned_process():
            failure = self._require(self.interface_manager.stop(trial))
            if failure:
                return failure

        failure = self._require(
            self.process_probe.verify_empty(
                trial,
                self.logs.stage_path(trial, "verify_empty"),
            )
        )
        if failure:
            return failure

        failure = self._require(self.interface_manager.start(trial))
        if failure:
            return failure

        failure = self._require(
            self.graph_probe.wait_until_ready(
                trial,
                self.config.interface_ready_timeout_sec,
                self.logs.stage_path(trial, "graph_ready"),
            )
        )
        if failure:
            return failure

        reset_result = self.command_runner.run(
            "reset_sim",
            RESET_COMMAND,
            timeout_sec=self.config.reset_timeout_sec,
            env=self.env,
            log_path=self.logs.stage_path(trial, "reset_sim"),
        )
        failure = self._require(reset_result)
        if failure:
            return failure
        if not trigger_response_succeeded(reset_result.output):
            return StageResult(
                stage="reset_sim",
                returncode=1,
                output=reset_result.output,
                message="reset service did not return success=True",
                log_path=reset_result.log_path,
            )

        self.sleep_fn(self.config.settle_sec)
        instruction = self._prompt_instruction()
        pipeline_result = self.command_runner.run(
            "pipeline",
            PIPELINE_COMMAND,
            timeout_sec=self.config.pipeline_timeout_sec,
            env=self.env,
            input_text=instruction + "\n",
            log_path=self.logs.stage_path(trial, "pipeline"),
        )
        failure = self._require(pipeline_result)
        if failure:
            return failure

        grasp_result = self.command_runner.run(
            "grasp_demo",
            GRASP_COMMAND,
            timeout_sec=self.config.grasp_timeout_sec,
            env=self.env,
            log_path=self.logs.stage_path(trial, "grasp_demo"),
        )
        failure = self._require(grasp_result)
        if failure:
            return failure
        return None

    def _prompt_next(self, failed):
        if failed:
            prompt = (
                "Trial stopped. Inspect/fix the problem, then press Enter to "
                "restart from the safe interface boundary, or q to quit: "
            )
        else:
            prompt = "Press Enter for the next trial, or q to quit:"
        while True:
            response = str(self._read_input(prompt)).strip().casefold()
            if response == "q":
                return "q"
            if response == "":
                return "next"
            self.output_fn("Enter only Enter or q.")

    def _report_failure(self, trial, failure):
        self.output_fn(
            f"[experiment_session] Trial {trial} stopped at {failure.stage}: "
            f"{failure.message or f'exit code {failure.returncode}'}"
        )
        if failure.log_path is not None:
            self.output_fn(f"[experiment_session] Log: {failure.log_path}")

    def run(self):
        trial = 1
        exit_code = 0
        try:
            while True:
                self.output_fn(f"\n[experiment_session] Trial {trial}")
                failure = self._run_trial(trial)
                if failure is not None:
                    self._report_failure(trial, failure)
                    decision = self._prompt_next(failed=True)
                    if decision == "q":
                        exit_code = 1
                        break
                else:
                    decision = self._prompt_next(failed=False)
                    if decision == "q":
                        exit_code = 0
                        break
                trial += 1
        except EOFError:
            self.output_fn("[experiment_session] Input stream closed; shutting down.")
            exit_code = 1
        except KeyboardInterrupt:
            self.output_fn("[experiment_session] Interrupted; shutting down.")
            exit_code = 130

        cleanup_result = self.interface_manager.cleanup()
        if not cleanup_result.ok:
            self._report_failure(trial, cleanup_result)
            exit_code = 1
        self.output_fn(f"[experiment_session] Session logs: {self.logs.session_dir}")
        return exit_code


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="Run a persistent MuJoCo grasp experiment session."
    )
    parser.add_argument(
        "--log-root",
        type=Path,
        default=Path("/tmp/my_course_experiment_sessions"),
    )
    parser.add_argument("--settle-sec", type=float, default=3.0)
    parser.add_argument("--interface-ready-timeout", type=float, default=180.0)
    parser.add_argument("--interface-stop-timeout", type=float, default=8.0)
    parser.add_argument("--graph-query-timeout", type=float, default=20.0)
    parser.add_argument("--reset-timeout", type=float, default=20.0)
    parser.add_argument("--pipeline-timeout", type=float, default=600.0)
    parser.add_argument("--grasp-timeout", type=float, default=600.0)
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    if not str(os.environ.get("VLM_API_KEY", "")).strip():
        print(
            "[experiment_session] VLM_API_KEY is missing. Start through "
            "experiment_session.launch.py or export it first."
        )
        return 2
    config = SessionConfig(
        log_root=args.log_root,
        settle_sec=max(0.0, float(args.settle_sec)),
        interface_ready_timeout_sec=max(
            0.1, float(args.interface_ready_timeout)
        ),
        interface_stop_timeout_sec=max(0.1, float(args.interface_stop_timeout)),
        graph_query_timeout_sec=max(0.1, float(args.graph_query_timeout)),
        reset_timeout_sec=max(0.1, float(args.reset_timeout)),
        pipeline_timeout_sec=max(0.1, float(args.pipeline_timeout)),
        grasp_timeout_sec=max(0.1, float(args.grasp_timeout)),
    )
    return ExperimentSession(config=config).run()


if __name__ == "__main__":
    sys.exit(main())
