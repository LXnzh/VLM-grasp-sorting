import hashlib
from importlib import resources
import json
from pathlib import Path

import numpy as np

from my_course_pkg.grasp.config import (
    TUNA_CALIBRATION_MAX_INTERPOLATION_ERROR_M,
)
from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError
from my_course_pkg.grasp.tuna_roll_grasp import CalibrationPoint, CalibrationTable


TUNA_CALIBRATION_SCHEMA_VERSION = 1
TUNA_CALIBRATION_RESOURCE = "data/tuna_gripper_geometry.json"


def _canonical_json_bytes(value):
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def _sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def locate_packaged_tuna_calibration():
    resource = resources.files("my_course_pkg.grasp").joinpath(
        TUNA_CALIBRATION_RESOURCE
    )
    if not resource.is_file():
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "packaged_artifact_missing",
                "resource": TUNA_CALIBRATION_RESOURCE,
            },
        )
    return resource


def _read_path_bytes(path, label):
    path = Path(path)
    if not path.is_absolute() or not path.is_file():
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "source_path_missing",
                "source": label,
                "path": str(path),
            },
        )
    return path.read_bytes()


def _sha_mismatch(source, expected, actual):
    raise TunaGraspError(
        TunaErrorCode.CALIBRATION_SHA,
        "calibration_load",
        {
            "object_name": "tuna_fish_can",
            "reason": "sha_mismatch",
            "source": source,
            "expected_sha256": expected,
            "actual_sha256": actual,
        },
    )


def _validate_artifact_content(artifact):
    required = {
        "schema_version",
        "expanded_urdf_sha256",
        "collision_mesh_sha256_by_link",
        "mount_transforms_sha256",
        "canonical_content_sha256",
        "T_tcp_gripper_base",
        "contact_band_qpos",
        "interpolation_error_bound_m",
        "observed_max_interpolation_error_m",
        "samples",
    }
    missing = sorted(required - set(artifact))
    if missing:
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "schema_missing_fields",
                "fields": missing,
            },
        )
    if artifact["schema_version"] != TUNA_CALIBRATION_SCHEMA_VERSION:
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "schema_version",
                "expected": TUNA_CALIBRATION_SCHEMA_VERSION,
                "actual": artifact["schema_version"],
            },
        )
    expected_content_sha = artifact["canonical_content_sha256"]
    unhashed = dict(artifact)
    unhashed.pop("canonical_content_sha256")
    actual_content_sha = _sha256_bytes(_canonical_json_bytes(unhashed))
    if actual_content_sha != expected_content_sha:
        _sha_mismatch("canonical_content", expected_content_sha, actual_content_sha)
    interpolation_bound = float(artifact["interpolation_error_bound_m"])
    observed_error = float(artifact["observed_max_interpolation_error_m"])
    if (
        not np.isfinite([interpolation_bound, observed_error]).all()
        or interpolation_bound <= 0.0
        or interpolation_bound > TUNA_CALIBRATION_MAX_INTERPOLATION_ERROR_M
        or observed_error < 0.0
        or observed_error > interpolation_bound
    ):
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "interpolation_error_bound",
                "configured_max_m": TUNA_CALIBRATION_MAX_INTERPOLATION_ERROR_M,
                "artifact_bound_m": interpolation_bound,
                "observed_error_m": observed_error,
            },
        )


def load_tuna_gripper_calibration(
    *,
    artifact_path=None,
    urdf_path,
    collision_mesh_paths,
    mount_transforms_path,
):
    """Load the artifact and verify every current source before use."""
    if artifact_path is None:
        artifact_bytes = locate_packaged_tuna_calibration().read_bytes()
    else:
        artifact_bytes = _read_path_bytes(artifact_path, "artifact")
    try:
        artifact = json.loads(artifact_bytes.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "artifact_json",
            },
        ) from exc
    _validate_artifact_content(artifact)

    urdf_sha = _sha256_bytes(_read_path_bytes(urdf_path, "expanded_urdf"))
    if urdf_sha != artifact["expanded_urdf_sha256"]:
        _sha_mismatch("expanded_urdf", artifact["expanded_urdf_sha256"], urdf_sha)

    if not isinstance(collision_mesh_paths, dict):
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "collision_mesh_paths_not_mapping",
            },
        )
    expected_meshes = artifact["collision_mesh_sha256_by_link"]
    if set(collision_mesh_paths) != set(expected_meshes):
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "collision_mesh_set",
                "missing": sorted(set(expected_meshes) - set(collision_mesh_paths)),
                "extra": sorted(set(collision_mesh_paths) - set(expected_meshes)),
            },
        )
    for link_name in sorted(expected_meshes):
        actual_sha = _sha256_bytes(
            _read_path_bytes(collision_mesh_paths[link_name], f"mesh:{link_name}")
        )
        if actual_sha != expected_meshes[link_name]:
            _sha_mismatch(f"mesh:{link_name}", expected_meshes[link_name], actual_sha)

    mount_sha = _sha256_bytes(
        _read_path_bytes(mount_transforms_path, "mount_transforms")
    )
    if mount_sha != artifact["mount_transforms_sha256"]:
        _sha_mismatch(
            "mount_transforms",
            artifact["mount_transforms_sha256"],
            mount_sha,
        )

    try:
        points = tuple(
            CalibrationPoint(
                qpos=sample["qpos"],
                pad_gap_m=sample["pad_gap_m"],
                T_tcp_lower_pad_contact=sample["T_tcp_lower_pad_contact"],
                support_hull_tcp=sample["support_hull_tcp"],
            )
            for sample in artifact["samples"]
        )
        table = CalibrationTable(
            source_sha256=artifact["canonical_content_sha256"],
            max_interpolation_error_m=artifact["interpolation_error_bound_m"],
            points=points,
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise TunaGraspError(
            TunaErrorCode.CALIBRATION_SHA,
            "calibration_load",
            {
                "object_name": "tuna_fish_can",
                "reason": "calibration_samples",
            },
        ) from exc
    return table, artifact
