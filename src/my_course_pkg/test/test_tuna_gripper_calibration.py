import json
import os
from pathlib import Path
import subprocess
import sys

import numpy as np
import pytest

from my_course_pkg.grasp.tuna_calibration import (
    load_tuna_gripper_calibration,
    locate_packaged_tuna_calibration,
)
from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.tuna_roll_grasp import interpolate_gripper_geometry


REPOSITORY_ROOT = Path(__file__).resolve().parents[3]
TOOL = (
    REPOSITORY_ROOT
    / "src"
    / "my_course_pkg"
    / "tools"
    / "calibrate_tuna_gripper_geometry.py"
)
MOUNT_TRANSFORMS = (
    REPOSITORY_ROOT
    / "src"
    / "my_course_pkg"
    / "tools"
    / "tuna_gripper_mount_transforms.json"
)
EXPANDED_URDF = (
    REPOSITORY_ROOT
    / "src"
    / "ifl_air_cell_small_ur_orbbec_robotiq_moveit_config"
    / "urdf"
    / "cell_small_ur_orbbec_robotiq2f85.urdf"
)
MESH_ROOT = (
    REPOSITORY_ROOT
    / "src"
    / "ros2_robotiq_gripper"
    / "robotiq_description"
    / "meshes"
    / "collision"
    / "2f_85"
)
COLLISION_MESHES = {
    "robotiq_85_base_link": MESH_ROOT / "robotiq_base.stl",
    "robotiq_85_left_finger_link": MESH_ROOT / "left_finger.stl",
    "robotiq_85_left_finger_tip_link": MESH_ROOT / "left_finger_tip_160_collision.stl",
    "robotiq_85_left_inner_knuckle_link": MESH_ROOT / "left_inner_knuckle.stl",
    "robotiq_85_left_knuckle_link": MESH_ROOT / "left_knuckle.stl",
    "robotiq_85_right_finger_link": MESH_ROOT / "right_finger.stl",
    "robotiq_85_right_finger_tip_link": MESH_ROOT / "right_finger_tip_160_collision.stl",
    "robotiq_85_right_inner_knuckle_link": MESH_ROOT / "right_inner_knuckle.stl",
    "robotiq_85_right_knuckle_link": MESH_ROOT / "right_knuckle.stl",
}


def tool_command(output_path):
    command = [
        sys.executable,
        str(TOOL),
        "--urdf",
        str(EXPANDED_URDF),
        "--mount-transforms",
        str(MOUNT_TRANSFORMS),
        "--output",
        str(output_path),
    ]
    for link_name, mesh_path in sorted(COLLISION_MESHES.items()):
        command.extend(["--collision-mesh", f"{link_name}={mesh_path}"])
    return command


def source_arguments():
    return {
        "urdf_path": EXPANDED_URDF,
        "collision_mesh_paths": COLLISION_MESHES,
        "mount_transforms_path": MOUNT_TRANSFORMS,
    }


def test_tool_uses_only_explicit_paths_and_is_cwd_independent(tmp_path):
    first_directory = tmp_path / "first"
    second_directory = tmp_path / "second"
    first_directory.mkdir()
    second_directory.mkdir()
    first_output = first_directory / "artifact.json"
    second_output = second_directory / "artifact.json"
    environment = {
        key: value
        for key, value in os.environ.items()
        if key not in {"AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "ROS_PACKAGE_PATH"}
    }

    subprocess.run(
        tool_command(first_output),
        cwd=first_directory,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    subprocess.run(
        tool_command(second_output),
        cwd=second_directory,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )

    assert first_output.read_bytes() == second_output.read_bytes()
    source = TOOL.read_text(encoding="utf-8")
    assert "ament_index_python" not in source
    assert "ROS_PACKAGE_PATH" not in source


def test_packaged_artifact_has_canonical_sha_fields_and_all_meshes():
    artifact_path = Path(str(locate_packaged_tuna_calibration()))
    artifact = json.loads(artifact_path.read_text(encoding="utf-8"))

    assert artifact["schema_version"] == 1
    assert len(artifact["expanded_urdf_sha256"]) == 64
    assert len(artifact["mount_transforms_sha256"]) == 64
    assert len(artifact["canonical_content_sha256"]) == 64
    assert set(artifact["collision_mesh_sha256_by_link"]) == set(COLLISION_MESHES)
    assert artifact["interpolation_error_bound_m"] <= 0.00025
    assert (
        artifact["observed_max_interpolation_error_m"]
        <= artifact["interpolation_error_bound_m"]
    )
    assert artifact["qualified_preclamp_position"] is None
    np.testing.assert_allclose(
        artifact["T_tcp_gripper_base"],
        np.array(
            [
                [1.0, 0.0, 0.0, 0.0],
                [0.0, 1.0, 0.0, 0.0],
                [0.0, 0.0, 1.0, -0.262],
                [0.0, 0.0, 0.0, 1.0],
            ]
        ),
        rtol=0.0,
        atol=1e-12,
    )


def test_every_calibration_sample_has_rigid_contact_matrix_and_support_hull():
    _table, artifact = load_tuna_gripper_calibration(**source_arguments())
    assert len(artifact["samples"]) >= 2
    qpos = []
    gaps = []
    for sample in artifact["samples"]:
        qpos.append(sample["qpos"])
        gaps.append(sample["pad_gap_m"])
        matrix = np.asarray(sample["T_tcp_lower_pad_contact"], dtype=float)
        hull = np.asarray(sample["support_hull_tcp"], dtype=float)
        np.testing.assert_allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1e-12)
        np.testing.assert_allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-9)
        assert np.linalg.det(matrix[:3, :3]) == pytest.approx(1.0)
        assert hull.ndim == 2
        assert hull.shape[1] == 3
        assert hull.shape[0] > 8
        assert np.isfinite(hull).all()
    assert np.all(np.diff(qpos) > 0.0)
    assert np.all(np.diff(gaps) < 0.0)


def test_representative_real_gripper_gaps_match_documented_values():
    table, _artifact = load_tuna_gripper_calibration(**source_arguments())
    assert interpolate_gripper_geometry(table, 0.0).pad_gap_m == pytest.approx(
        0.085517,
        abs=0.000001,
    )
    assert interpolate_gripper_geometry(table, 0.50).pad_gap_m == pytest.approx(
        0.034785,
        abs=0.00025,
    )
    assert interpolate_gripper_geometry(table, 0.79).pad_gap_m == pytest.approx(
        0.001819,
        abs=0.000001,
    )


def test_loader_fails_closed_when_current_source_sha_changes(tmp_path):
    altered_urdf = tmp_path / "altered.urdf"
    altered_urdf.write_bytes(EXPANDED_URDF.read_bytes() + b"\n<!-- changed -->\n")
    arguments = source_arguments()
    arguments["urdf_path"] = altered_urdf

    with pytest.raises(TunaGraspError) as exc_info:
        load_tuna_gripper_calibration(**arguments)

    assert exc_info.value.code is TunaErrorCode.CALIBRATION_SHA
    assert exc_info.value.diagnostics["source"] == "expanded_urdf"


def test_loader_fails_closed_when_artifact_content_is_tampered(tmp_path):
    artifact = json.loads(
        Path(str(locate_packaged_tuna_calibration())).read_text(encoding="utf-8")
    )
    artifact["samples"][0]["pad_gap_m"] += 0.001
    tampered = tmp_path / "tampered.json"
    tampered.write_text(json.dumps(artifact), encoding="utf-8")

    with pytest.raises(TunaGraspError) as exc_info:
        load_tuna_gripper_calibration(
            artifact_path=tampered,
            **source_arguments(),
        )

    assert exc_info.value.code is TunaErrorCode.CALIBRATION_SHA
    assert exc_info.value.diagnostics["source"] == "canonical_content"


def test_package_install_includes_tuna_calibration_resource(tmp_path):
    target = tmp_path / "installed"
    subprocess.run(
        [
            sys.executable,
            "-m",
            "pip",
            "install",
            "--no-deps",
            "--no-build-isolation",
            "--target",
            str(target),
            str(REPOSITORY_ROOT / "src" / "my_course_pkg"),
        ],
        check=True,
        capture_output=True,
        text=True,
    )
    environment = dict(os.environ)
    environment["PYTHONPATH"] = str(target)
    completed = subprocess.run(
        [
            sys.executable,
            "-c",
            (
                "from my_course_pkg.grasp.tuna_calibration import "
                "locate_packaged_tuna_calibration; "
                "print(locate_packaged_tuna_calibration().is_file())"
            ),
        ],
        cwd=tmp_path,
        env=environment,
        check=True,
        capture_output=True,
        text=True,
    )
    assert completed.stdout.strip() == "True"
