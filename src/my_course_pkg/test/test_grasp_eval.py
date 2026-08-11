import csv
from pathlib import Path
from types import SimpleNamespace

import numpy as np

from my_course_pkg import grasp_eval


def _summary_result(final_success, auto_success=None):
    if auto_success is None:
        auto_success = final_success
    return SimpleNamespace(
        final_success=bool(final_success),
        auto_success=bool(auto_success),
    )


def test_parse_args_defaults_to_two_trials():
    args = grasp_eval.parse_args([])

    assert args.trials == 2


def test_parse_args_preserves_explicit_longer_trial_count():
    args = grasp_eval.parse_args(["--trials", "8"])

    assert args.trials == 8


def test_qualification_streak_requires_two_consecutive_successes():
    results = [
        _summary_result(True),
        _summary_result(False),
        _summary_result(True),
    ]

    current_streak, max_streak, passed = grasp_eval.compute_consecutive_streaks(
        results
    )

    assert current_streak == 1
    assert max_streak == 1
    assert passed is False


def test_qualification_streak_resets_after_failure_and_can_reach_two():
    results = [
        _summary_result(False),
        _summary_result(True),
        _summary_result(True),
    ]

    current_streak, max_streak, passed = grasp_eval.compute_consecutive_streaks(
        results
    )

    assert current_streak == 2
    assert max_streak == 2
    assert passed is True


def test_summarize_results_includes_qualification_line():
    summary = grasp_eval.summarize_results(
        [_summary_result(True), _summary_result(False), _summary_result(True)]
    )

    assert "Qualification: NOT PASS (max streak 1/2)" in summary


def test_summarize_results_reports_pass_after_two_consecutive():
    summary = grasp_eval.summarize_results(
        [
            _summary_result(False),
            _summary_result(True),
            _summary_result(True),
        ]
    )

    assert "Qualification: PASS (max streak 2/2)" in summary


def test_estimate_auto_success_uses_drop_xy_and_minimum_height():
    drop_xyz = np.array([-0.55, -0.45, 0.35], dtype=float)

    success = grasp_eval.estimate_auto_success(
        pipeline_ok=True,
        grasp_ok=True,
        object_end_xyz=np.array([-0.50, -0.40, 0.06], dtype=float),
        drop_xyz=drop_xyz,
        drop_xy_tolerance_m=0.10,
        min_object_z_m=-0.02,
    )

    assert success.success is True
    assert success.reason == "object_near_drop"

    too_far = grasp_eval.estimate_auto_success(
        pipeline_ok=True,
        grasp_ok=True,
        object_end_xyz=np.array([0.0, 0.0, 0.06], dtype=float),
        drop_xyz=drop_xyz,
        drop_xy_tolerance_m=0.10,
        min_object_z_m=-0.02,
    )
    assert too_far.success is False
    assert too_far.reason == "object_not_near_drop"

    below_table = grasp_eval.estimate_auto_success(
        pipeline_ok=True,
        grasp_ok=True,
        object_end_xyz=np.array([-0.50, -0.40, -0.10], dtype=float),
        drop_xyz=drop_xyz,
        drop_xy_tolerance_m=0.10,
        min_object_z_m=-0.02,
    )
    assert below_table.success is False
    assert below_table.reason == "object_below_min_z"


def test_apply_human_confirmation_accepts_auto_and_overrides():
    accepted = grasp_eval.apply_human_confirmation("a", auto_success=True)
    assert accepted.final_success is True
    assert accepted.human_success is None
    assert accepted.stop_requested is False

    marked_failure = grasp_eval.apply_human_confirmation("n", auto_success=True)
    assert marked_failure.final_success is False
    assert marked_failure.human_success is False

    marked_success = grasp_eval.apply_human_confirmation("y", auto_success=False)
    assert marked_success.final_success is True
    assert marked_success.human_success is True

    stopped = grasp_eval.apply_human_confirmation("q", auto_success=True)
    assert stopped.stop_requested is True


def test_append_trial_result_writes_csv_rows(tmp_path):
    csv_path = tmp_path / "eval.csv"
    result = grasp_eval.TrialResult(
        trial=1,
        object_name="tomato soup can",
        auto_success=True,
        human_success=None,
        final_success=True,
        auto_reason="object_near_drop",
        final_reason="accepted_auto",
        reset_ok=True,
        pipeline_ok=True,
        grasp_ok=True,
        object_start_xyz=np.array([0.1, 0.2, 0.3], dtype=float),
        object_end_xyz=np.array([-0.5, -0.4, 0.1], dtype=float),
        drop_xyz=np.array([-0.55, -0.45, 0.35], dtype=float),
        duration_sec=12.3,
        stop_requested=False,
    )

    grasp_eval.append_trial_result(csv_path, result)

    rows = list(csv.DictReader(csv_path.open(newline="", encoding="utf-8")))
    assert len(rows) == 1
    assert rows[0]["trial"] == "1"
    assert rows[0]["object_name"] == "tomato soup can"
    assert rows[0]["auto_success"] == "true"
    assert rows[0]["human_success"] == ""
    assert rows[0]["final_success"] == "true"
    assert rows[0]["object_end_xyz"] == "[-0.5, -0.4, 0.1]"


def test_run_trial_executes_reset_pipeline_grasp_and_records_human_override():
    commands = []
    poses = [
        np.array([0.2, 0.1, 0.05], dtype=float),
        np.array([0.2, 0.1, 0.05], dtype=float),
    ]
    config = grasp_eval.EvalConfig(
        trials=1,
        object_name="tomato soup can",
        instruction="pick up the tomato soup can",
        output_dir=Path("."),
        drop_xyz=np.array([-0.55, -0.45, 0.35], dtype=float),
        drop_xy_tolerance_m=0.10,
        min_object_z_m=-0.02,
        reset_timeout_sec=1.0,
        pipeline_timeout_sec=1.0,
        grasp_timeout_sec=1.0,
        settle_sec=0.0,
    )

    def command_runner(command, *, timeout_sec, env=None, input_text=None):
        commands.append((command, timeout_sec, input_text, env["VLM_CANDIDATE_OVERRIDE"]))
        return grasp_eval.CommandResult(returncode=0)

    def pose_reader(object_name, timeout_sec):
        assert object_name == "tomato soup can"
        return poses.pop(0)

    result = grasp_eval.run_trial(
        trial=1,
        config=config,
        command_runner=command_runner,
        pose_reader=pose_reader,
        prompt_fn=lambda prompt: "n",
        sleep_fn=lambda seconds: None,
        monotonic_fn=iter([100.0, 112.0]).__next__,
    )

    assert [entry[0][:3] for entry in commands] == [
        ["ros2", "service", "call"],
        ["ros2", "run", "my_course_pkg"],
        ["ros2", "run", "my_course_pkg"],
    ]
    assert commands[1][2] == "pick up the tomato soup can\n"
    assert result.auto_success is False
    assert result.auto_reason == "object_not_near_drop"
    assert result.human_success is False
    assert result.final_success is False
    assert result.final_reason == "human_marked_failure"
    np.testing.assert_allclose(result.object_start_xyz, [0.2, 0.1, 0.05])
    np.testing.assert_allclose(result.object_end_xyz, [0.2, 0.1, 0.05])
    assert result.duration_sec == 12.0
