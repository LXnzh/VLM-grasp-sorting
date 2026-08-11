"""Product-boundary tests for VLM tracking and sorting placement."""

from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from my_course_pkg.grasp import pick_place_planner as planner
from my_course_pkg.grasp.config import YCB_GRASP_NAME_MAP
from my_course_pkg.perception import llm_sam2
from my_course_pkg.ycb_models import YCB_DIRECTORY_BY_OBJECT
from my_course_pkg.tasks.sorting import drop_target as drop_target_module
from my_course_pkg.tasks.sorting.drop_target import (
    DropTarget,
    normalize_sorting_category,
    resolve_drop_target,
)
from my_course_pkg.tasks.sorting.vlm_classifier import classify_food
from my_course_pkg.tasks.tracking import node as tracking_node


def make_drop_target(category="food"):
    return DropTarget(
        position=(-0.60, 0.42, 0.14),
        category=category,
        bin_name="food_bin" if category == "food" else "default_non_food",
        observation_stamp=12.5,
        source_frame="camera_orbbec",
        rgb_path="overview/rgb.png",
        depth_path="overview/depth.npy",
        detection_method="random_color_rgbd",
    )


@pytest.mark.parametrize("value", ["unknown", "maybe_food", "", None])
def test_sorting_category_rejects_every_non_product_value(value):
    with pytest.raises(RuntimeError, match="exactly 'food' or 'non_food'"):
        normalize_sorting_category(value)


def test_vlm_classifier_fails_closed_on_unknown_category():
    with pytest.raises(RuntimeError, match="exactly 'food' or 'non_food'"):
        classify_food(
            "unused.png",
            "apple",
            lambda *_args: {
                "category": "unknown",
                "confidence": 0.2,
                "reason": "uncertain",
            },
        )


def test_non_food_drop_target_uses_configured_world_position(tmp_path):
    rgb_path = tmp_path / "rgb.png"
    depth_path = tmp_path / "depth.npy"
    rgb_path.write_bytes(b"rgb")
    depth_path.write_bytes(b"depth")

    target = resolve_drop_target(
        "non-food",
        observation_stamp=3.0,
        rgb_path=rgb_path,
        depth_path=depth_path,
        default_position=(-0.40, -0.60, 0.30),
    )

    assert target.category == "non_food"
    assert target.position == (-0.40, -0.60, 0.30)
    assert target.detection_method == "configured_non_food_target"


def test_food_drop_target_keeps_overview_detection_provenance(
    tmp_path,
    monkeypatch,
):
    rgb_path = tmp_path / "rgb.png"
    depth_path = tmp_path / "depth.npy"
    rgb_path.write_bytes(b"rgb")
    depth_path.write_bytes(b"depth")
    monkeypatch.setattr(
        drop_target_module,
        "locate_sorting_bins_from_rgbd",
        lambda *_args, **_kwargs: {
            "single_bin": {
                "name": "food_bin",
                "drop_position": np.array([-0.61, 0.43, 0.14]),
                "detection_method": "random_color_rgbd",
            }
        },
    )

    target = resolve_drop_target(
        "food",
        observation_stamp=8.25,
        rgb_path=rgb_path,
        depth_path=depth_path,
        node=object(),
    )

    assert target.position == (-0.61, 0.43, 0.14)
    assert target.observation_stamp == 8.25
    assert target.source_frame == "camera_orbbec"
    assert target.detection_method == "random_color_rgbd"


def test_explicit_drop_target_uses_pure_grasp_then_safe_place(monkeypatch):
    target = make_drop_target()
    object_pose = np.eye(4)
    object_pose[:3, 3] = [-0.65, -0.10, 0.06]
    grasp_transform = np.eye(4)
    grasp_transform[:3, 3] = [-0.65, -0.10, 0.15]
    grasp_pose = np.array([-0.65, -0.10, 0.15, 0.0, 0.0, 0.0])
    canonicalize_calls = []

    monkeypatch.setattr(planner, "get_selected_object_info", lambda *_: "apple")
    monkeypatch.setattr(
        planner,
        "estimate_object_world_pose",
        lambda *_args, **_kwargs: object_pose.copy(),
    )
    monkeypatch.setattr(
        planner,
        "canonicalize_tabletop_object_pose",
        lambda pose, name: canonicalize_calls.append(name) or pose.copy(),
    )
    monkeypatch.setattr(planner, "_log_object_pose_canonicalization", lambda *_: None)
    monkeypatch.setattr(planner, "get_grasp_profile", lambda *_: "side")
    monkeypatch.setattr(planner, "get_grasp_z_offset", lambda *_: 0.0)
    monkeypatch.setattr(
        planner,
        "select_grasp_pose_candidates_6d",
        lambda *_: [(grasp_pose.copy(), grasp_transform.copy())],
    )
    monkeypatch.setattr(
        planner,
        "_load_obstacles_for_place_and_clearance",
        lambda *_: [],
    )
    monkeypatch.setattr(
        planner,
        "_select_safe_place_xy",
        lambda *_: pytest.fail("typed target must bypass safe-place search"),
    )
    monkeypatch.setattr(
        planner,
        "get_side_grasp_geometry_center",
        lambda *_: np.zeros(3),
    )
    monkeypatch.setattr(
        planner,
        "_passes_side_world_z_filter",
        lambda *_: True,
    )
    monkeypatch.setattr(
        planner,
        "_filter_results_by_approach_clearance",
        lambda results, *_: results,
    )
    monkeypatch.setattr(planner, "_log_selected_world_z_ranges", lambda *_: None)

    result = planner.plan_pick_place_candidates_from_perception(
        SimpleNamespace(),
        np.array([-0.50, -0.10, 0.40, 0.0, 0.0, 0.0]),
        drop_target=target,
        execution_mode="safe_place",
    )[0]

    assert canonicalize_calls == ["apple"]
    assert result.plan.debug_info["drop_target"] == target.as_dict()
    np.testing.assert_allclose(result.plan.drop_pose_6d[:3], target.position)
    assert [step.name for step in result.plan.steps[:7]] == [
        "open_gripper_before_approach",
        "move_to_pre_grasp",
        "approach_grasp",
        "close_gripper_at_grasp",
        "hold_after_close",
        "lift_after_grasp",
        "hold_after_lift",
    ]
    assert "transfer_to_drop_high" in [
        step.name for step in result.plan.steps
    ]


def test_overview_classification_and_bin_freeze_precede_tracking(
    tmp_path,
    monkeypatch,
):
    events = []
    selection = {
        "candidates": ["apple"],
        "selected_object_name": "apple",
        "target_region": {
            "horizontal": "center",
            "vertical": "middle",
            "description": "apple",
            "source": "inferred_from_image",
        },
        "visual_attributes": {
            "color": "red",
            "shape": "round",
            "container": "on_table",
        },
        "grounding_prompt": "apple",
        "accepted_sam2_class_names": [],
        "matched_instruction_alias": None,
        "visual_target_description": None,
    }
    target = make_drop_target()

    class FakeMaskSelector:
        def __init__(self, **_kwargs):
            pass

        def _load_mask(self, _target_name):
            events.append("mask")
            return np.ones((8, 8), dtype=bool)

    class FakeTracking:
        def activate(self, *_args):
            events.append("tracking")

    node = object.__new__(tracking_node.FoundationPoseGraspNode)
    node.instruction = "抓取苹果"
    node.selection_frame = (
        np.zeros((8, 8, 3), dtype=np.uint8),
        np.ones((8, 8), dtype=np.float32),
        12.5,
    )
    node.session_dir = Path(tmp_path)
    node.selected_path = Path(tmp_path) / "selected_object.json"
    node.rgbd_node = SimpleNamespace(K=np.eye(3))
    node.motion = SimpleNamespace(
        get_current_ee_pose_6d=lambda: np.zeros(6)
    )
    node.tracking = FakeTracking()
    node._status = lambda _message: events.append("status")

    monkeypatch.setattr(
        tracking_node,
        "vlm_select_target",
        lambda *_args: events.append("selection") or selection,
    )
    monkeypatch.setattr(
        tracking_node,
        "vlm_classify_food",
        lambda *_args: events.append("classification") or {
            "category": "food",
            "confidence": 0.99,
            "reason": "apple",
        },
    )
    monkeypatch.setattr(
        tracking_node,
        "resolve_drop_target",
        lambda *_args, **_kwargs: events.append("drop_target") or target,
    )
    monkeypatch.setattr(
        tracking_node,
        "run_sam2_api",
        lambda *_args, **_kwargs: events.append("sam2"),
    )
    monkeypatch.setattr(
        tracking_node,
        "FoundationPoseEstimationNode",
        FakeMaskSelector,
    )

    node._initialize_target()

    business_events = [event for event in events if event != "status"]
    assert business_events[:6] == [
        "selection",
        "classification",
        "drop_target",
        "sam2",
        "mask",
        "tracking",
    ]
    assert node.drop_target is target
    assert "drop_target" in node.selected_path.read_text(encoding="utf-8")


def test_product_object_catalog_excludes_tuna_and_pudding():
    assert len(llm_sam2.YCB_OBJECTS) == 16
    assert "tuna fish can" not in llm_sam2.YCB_OBJECTS
    assert "pudding box" not in llm_sam2.YCB_OBJECTS
    expected = {name.replace(" ", "_") for name in llm_sam2.YCB_OBJECTS}
    assert set(YCB_GRASP_NAME_MAP) == expected
    assert set(YCB_DIRECTORY_BY_OBJECT) == expected


@pytest.mark.parametrize("excluded_name", ["tuna fish can", "pudding box"])
def test_product_override_rejects_excluded_objects(monkeypatch, excluded_name):
    monkeypatch.setenv("VLM_CANDIDATE_OVERRIDE", excluded_name)

    with pytest.raises(ValueError, match="must be one of the supported YCB"):
        llm_sam2._forced_target_object("pick the object")
