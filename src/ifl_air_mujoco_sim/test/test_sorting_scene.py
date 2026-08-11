"""Tests for the canonical single-bin sorting scene."""

from copy import deepcopy
from pathlib import Path

import cv2
import mujoco
import numpy as np
from omegaconf import OmegaConf
import pytest

from env.mjcontrol_interface import MuJoCoInterface
from env.utils.populate_scene import (
    assign_placement_slots,
    select_scene_objects_for_mode,
)
from env.utils.sorting_scene import validate_sorting_layout
from my_course_pkg.tasks.sorting.locator import detect_sorting_bins


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONFIG_PATH = PACKAGE_ROOT / "env" / "config" / "base_env.yaml"
MODEL_PATH = PACKAGE_ROOT / "models" / "ur10e_2f85" / "scene.xml"


def _sorting_config():
    config = OmegaConf.to_container(
        OmegaConf.load(CONFIG_PATH),
        resolve=True,
    )
    objects = select_scene_objects_for_mode(
        config["objects"],
        scene_mode=config["scene_mode"],
        random_object_count=config["random_object_count"],
        fixed_object_names=config["fixed_object_names"],
        scene_object_categories=config["scene_object_categories"],
        assigned_object_names=config["assigned_object_names"],
    )
    objects = assign_placement_slots(
        objects,
        config["sorting_placement_slots"],
    )
    return config, objects


def test_sorting_layout_preflight_accepts_canonical_configuration():
    config, objects = _sorting_config()

    validate_sorting_layout(
        objects,
        config["classification_bins_single_random"],
    )


def test_sorting_layout_preflight_rejects_bin_object_overlap():
    config, objects = _sorting_config()
    bins = deepcopy(config["classification_bins_single_random"])
    bins["position_range"]["x"] = [-0.65, -0.55]
    bins["position_range"]["y"] = [0.05, 0.10]

    with pytest.raises(ValueError, match="overlap food_bin"):
        validate_sorting_layout(objects, bins)


def test_sorting_scene_bin_and_slots_fit_home_camera():
    config, _objects = _sorting_config()
    simulator = MuJoCoInterface(
        model_path=MODEL_PATH,
        camera_names=["camera_orbbec"],
        headless=True,
        control_timestep=config["sim"]["control_timestep"],
        render_fps=config["sorting_render_fps"],
        objects_config=config["objects"],
        random_object_count=config["random_object_count"],
        fixed_object_names=config["fixed_object_names"],
        scene_mode=config["scene_mode"],
        scene_object_categories=config["scene_object_categories"],
        assigned_object_names=config["assigned_object_names"],
        placement_slots=config["sorting_placement_slots"],
        classification_bins=config["classification_bins_single_random"],
        camera_size=config["sim"]["camera_size"],
    )
    try:
        assert mujoco.mj_name2id(
            simulator.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "food_bin",
        ) >= 0
    finally:
        simulator.close()


def test_rendered_overview_rgbd_localizes_randomized_food_bin():
    config, _objects = _sorting_config()
    simulator = MuJoCoInterface(
        model_path=MODEL_PATH,
        camera_names=["camera_orbbec"],
        headless=True,
        objects_config=config["objects"],
        random_object_count=config["random_object_count"],
        fixed_object_names=config["fixed_object_names"],
        scene_mode=config["scene_mode"],
        scene_object_categories=config["scene_object_categories"],
        assigned_object_names=config["assigned_object_names"],
        placement_slots=config["sorting_placement_slots"],
        classification_bins=config["classification_bins_single_random"],
        camera_size=config["sim"]["camera_size"],
    )
    try:
        simulator.init_renderer(width=640, height=360)
        color_rgb, depth_m = simulator.render_camera(
            "camera_orbbec",
            need_depth=True,
        )
        assert color_rgb is not None
        assert depth_m is not None

        camera_id = mujoco.mj_name2id(
            simulator.model,
            mujoco.mjtObj.mjOBJ_CAMERA,
            "camera_orbbec",
        )
        rotation_world_mujoco_camera = np.asarray(
            simulator.data.cam_xmat[camera_id],
            dtype=float,
        ).reshape(3, 3)
        transform_world_cv_camera = np.eye(4)
        transform_world_cv_camera[:3, :3] = (
            rotation_world_mujoco_camera
            @ np.diag([1.0, -1.0, -1.0])
        )
        transform_world_cv_camera[:3, 3] = simulator.data.cam_xpos[
            camera_id
        ]
        bins = detect_sorting_bins(
            cv2.cvtColor(color_rgb, cv2.COLOR_RGB2BGR),
            depth_m,
            transform_world_cv_camera,
        )

        food_bin_id = mujoco.mj_name2id(
            simulator.model,
            mujoco.mjtObj.mjOBJ_BODY,
            "food_bin",
        )
        expected_xy = np.asarray(simulator.data.xpos[food_bin_id, :2])
        detected_xy = np.asarray(bins["single_bin"]["center_xy"])
        np.testing.assert_allclose(detected_xy, expected_xy, atol=0.06)
    finally:
        simulator.close()
