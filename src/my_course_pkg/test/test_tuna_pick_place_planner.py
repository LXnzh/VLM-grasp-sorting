from types import SimpleNamespace
import os
import time

import numpy as np
import pytest

from my_course_pkg.grasp import pick_place_planner as planner
from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.tuna_roll_grasp import (
    CalibrationPoint,
    CalibrationTable,
    TunaBoundsSample,
    StableTunaBounds,
    interpolate_gripper_geometry,
    validate_tuna_bounds_samples,
)


def _calibration():
    transform = np.eye(4)
    hull = np.array([[0.0, 0.0, 0.0], [0.001, 0.0, 0.0]])
    table = CalibrationTable(
        source_sha256="a" * 64,
        max_interpolation_error_m=0.0001,
        points=(
            CalibrationPoint(0.0, 0.200, transform, hull),
            CalibrationPoint(1.0, 0.010, transform, hull),
        ),
    )
    return table, {"canonical_content_sha256": "a" * 64}


def _bounds_samples(reception_boundary, *, name="tuna_fish_can", frame="world"):
    return tuple(
        TunaBoundsSample(
            object_name=name,
            frame_id=frame,
            source_timestamp=float(index + 1),
            received_monotonic=float(reception_boundary + index + 1),
            center=np.array([0.4, -0.2, 0.015 + 0.0001 * index]),
            size=np.array([0.085, 0.084, 0.030]),
        )
        for index in range(3)
    )


def _node():
    node = SimpleNamespace(tuna_table_z_m=0.0)
    node.collect_tuna_bounds_samples = (
        lambda *, required_count, reception_boundary: _bounds_samples(
            reception_boundary
        )
    )
    node.load_tuna_calibration = _calibration
    return node


def _fail_generic(*_args, **_kwargs):
    pytest.fail("generic grasp path must not run for exact-name Tuna")


def test_exact_name_routes_immediately_after_raw_pose(monkeypatch):
    raw = np.eye(4)
    sentinel = [object()]
    monkeypatch.setattr(planner, "get_selected_object_info", lambda _path: "tuna_fish_can")
    monkeypatch.setattr(planner, "estimate_object_world_pose", lambda *_args: raw)
    monkeypatch.setattr(planner, "canonicalize_tabletop_object_pose", _fail_generic)
    monkeypatch.setattr(planner, "get_grasp_profile", _fail_generic)
    monkeypatch.setattr(planner, "select_grasp_pose_candidates_6d", _fail_generic)

    def route(**kwargs):
        assert kwargs["T_world_obj_raw"] is raw
        return sentinel

    monkeypatch.setattr(planner, "_plan_tuna_fish_can_candidates", route)
    actual = planner.plan_pick_place_candidates_from_perception(
        object(), np.zeros(6), "/pose.json", "/selected.json"
    )
    assert actual is sentinel


def test_dataset_prefixed_name_stays_on_generic_path(monkeypatch):
    monkeypatch.setattr(
        planner, "get_selected_object_info", lambda _path: "007_tuna_fish_can"
    )
    monkeypatch.setattr(planner, "estimate_object_world_pose", lambda *_args: np.eye(4))
    monkeypatch.setattr(planner, "_plan_tuna_fish_can_candidates", _fail_generic)

    class GenericReached(RuntimeError):
        pass

    monkeypatch.setattr(
        planner,
        "canonicalize_tabletop_object_pose",
        lambda *_args: (_ for _ in ()).throw(GenericReached()),
    )
    with pytest.raises(GenericReached):
        planner.plan_pick_place_candidates_from_perception(
            object(), np.zeros(6), "/pose.json", "/selected.json"
        )


def test_generation_enumerates_grid_without_library_or_commands(monkeypatch):
    monkeypatch.delenv("GRASP_TUNA_PRECLAMP_POSITION", raising=False)
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "generation")
    monkeypatch.setattr(planner, "select_grasp_pose_candidates_6d", _fail_generic)
    results = planner._plan_tuna_fish_can_candidates(
        node=_node(),
        start_pose_6d=np.zeros(6),
        T_world_obj_raw=np.eye(4),
        selected_object_path="/missing-selected.json",
        object_cam_pose_path="/missing-pose.json",
    )
    assert len(results) == 36
    assert all(result.candidate_count == 36 for result in results)
    assert all(result.plan.steps == [] for result in results)
    assert all(result.plan.debug_info["tuna_non_executable"] for result in results)
    assert results[0].plan.debug_info["tuna_candidate_grid_count"] == 36
    assert not results[0].plan.debug_info[
        "tuna_perception_cache_invalidation_verified"
    ]


def test_real_calibration_fails_closed_below_exact_five_mm_gate(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "generation")
    config = planner.read_tuna_config()
    calibration, _artifact = planner._load_tuna_calibration_for_planning(
        SimpleNamespace()
    )
    geometry = interpolate_gripper_geometry(
        calibration,
        0.5 * (calibration.points[0].qpos + calibration.points[-1].qpos),
    )
    bounds = StableTunaBounds(
        source_timestamps=(1.0, 2.0, 3.0),
        received_monotonic=(1.0, 2.0, 3.0),
        center=np.array([0.4, -0.2, 0.015]),
        size=np.array([0.085, 0.084, 0.030]),
        bottom_z_m=0.0,
        top_z_m=0.030,
        radius_m=0.04225,
    )
    candidates, rejected, grid_count = planner._enumerate_tuna_candidates(
        node=SimpleNamespace(),
        config=config,
        bounds=bounds,
        table_z_m=0.0,
        calibration=calibration,
        geometry=geometry,
        require_moveit=False,
    )
    assert grid_count == 36
    assert candidates == ()
    best = max(rejected, key=lambda item: item["clearance_m"])
    assert best["reason"] == "support_hull_table_clearance"
    assert 0.0020 < best["clearance_m"] < config.min_table_clearance_m
    assert 0.0040 < best["contact_clearance_m"] < config.min_table_clearance_m
    assert best["roll_minimum_clearance_m"] == pytest.approx(
        best["clearance_m"]
    )


def test_motion_stage_without_qualified_preclamp_fails_before_motion(monkeypatch):
    monkeypatch.delenv("GRASP_TUNA_PRECLAMP_POSITION", raising=False)
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "pregrasp")
    with pytest.raises(TunaGraspError) as exc_info:
        planner._plan_tuna_fish_can_candidates(
            node=_node(),
            start_pose_6d=np.zeros(6),
            T_world_obj_raw=np.eye(4),
            selected_object_path="/selected.json",
            object_cam_pose_path="/pose.json",
        )
    assert exc_info.value.code is TunaErrorCode.CONFIGURATION
    assert exc_info.value.stage == "preclamp"


def test_physical_stage_requires_post_reset_cache_invalidation_outputs(tmp_path):
    selected = tmp_path / "selected.json"
    pose = tmp_path / "pose.json"
    invalidated = time.time() - 2.0
    node = SimpleNamespace(
        tuna_scene_reset_wall_time=invalidated - 1.0,
        tuna_perception_cache_invalidated_wall_time=invalidated,
    )
    selected.write_text('{"name":"tuna_fish_can"}', encoding="utf-8")
    pose.write_text("{}", encoding="utf-8")
    os.utime(selected, (invalidated + 1.0, invalidated + 1.0))
    os.utime(pose, (invalidated + 1.0, invalidated + 1.0))

    provenance = planner._validate_tuna_perception_cache(
        node,
        selected_object_path=selected,
        object_cam_pose_path=pose,
        require_fresh_scene=True,
    )
    assert provenance.cache_invalidation_verified
    assert all(item["sha256"] for item in provenance.files)


def test_stale_perception_output_fails_physical_stage(tmp_path):
    selected = tmp_path / "selected.json"
    pose = tmp_path / "pose.json"
    invalidated = time.time() - 2.0
    selected.write_text('{"name":"tuna_fish_can"}', encoding="utf-8")
    pose.write_text("{}", encoding="utf-8")
    os.utime(selected, (invalidated - 1.0, invalidated - 1.0))
    os.utime(pose, (invalidated + 1.0, invalidated + 1.0))
    node = SimpleNamespace(
        tuna_scene_reset_wall_time=invalidated - 1.0,
        tuna_perception_cache_invalidated_wall_time=invalidated,
    )
    with pytest.raises(TunaGraspError) as exc_info:
        planner._validate_tuna_perception_cache(
            node,
            selected_object_path=selected,
            object_cam_pose_path=pose,
            require_fresh_scene=True,
        )
    assert exc_info.value.code is TunaErrorCode.BOUNDS
    assert exc_info.value.stage == "perception_cache"


def test_tuna_and_legacy_debug_selectors_cannot_be_combined(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", "generation")
    monkeypatch.setattr(planner, "GRASP_DEBUG_STOP_AT_PREGRASP", True)
    with pytest.raises(TunaGraspError) as exc_info:
        planner._plan_tuna_fish_can_candidates(
            node=_node(),
            start_pose_6d=np.zeros(6),
            T_world_obj_raw=np.eye(4),
            selected_object_path="/selected.json",
            object_cam_pose_path="/pose.json",
        )
    assert exc_info.value.code is TunaErrorCode.CONFIGURATION
    assert exc_info.value.stage == "debug_selector"


@pytest.mark.parametrize(
    ("mutation", "match"),
    [
        (lambda samples: samples[:2], "exactly 3"),
        (
            lambda samples: (
                samples[0],
                samples[0],
                samples[2],
            ),
            "distinct and increasing",
        ),
        (
            lambda samples: tuple(
                TunaBoundsSample(
                    object_name="007_tuna_fish_can",
                    frame_id=sample.frame_id,
                    source_timestamp=sample.source_timestamp,
                    received_monotonic=sample.received_monotonic,
                    center=sample.center,
                    size=sample.size,
                )
                for sample in samples
            ),
            "exact tuna_fish_can",
        ),
        (
            lambda samples: tuple(
                TunaBoundsSample(
                    object_name=sample.object_name,
                    frame_id="base_link",
                    source_timestamp=sample.source_timestamp,
                    received_monotonic=sample.received_monotonic,
                    center=sample.center,
                    size=sample.size,
                )
                for sample in samples
            ),
            "world frame",
        ),
    ],
)
def test_bounds_fail_closed_for_count_freshness_identity_and_frame(mutation, match):
    samples = _bounds_samples(10.0)
    with pytest.raises(ValueError, match=match):
        validate_tuna_bounds_samples(
            mutation(samples),
            required_count=3,
            source_boundary=0.0,
            reception_boundary=10.0,
            stability_tolerance_m=0.002,
            table_z_m=0.0,
            table_consistency_tolerance_m=0.002,
        )


def test_three_fresh_stable_flat_bounds_pass():
    bounds = validate_tuna_bounds_samples(
        _bounds_samples(10.0),
        required_count=3,
        source_boundary=0.0,
        reception_boundary=10.0,
        stability_tolerance_m=0.002,
        table_z_m=0.0,
        table_consistency_tolerance_m=0.002,
    )
    assert bounds.radius_m == pytest.approx(0.04225)
    assert bounds.bottom_z_m == pytest.approx(0.0001)
