#!/usr/bin/env python3
"""Generate the hash-bound Tuna gripper geometry artifact without ROS."""

import argparse
import hashlib
import itertools
import json
import math
from pathlib import Path
import struct
import xml.etree.ElementTree as ET

import numpy as np


SCHEMA_VERSION = 1
DEFAULT_MAX_INTERPOLATION_ERROR_M = 0.00025


def canonical_json_bytes(value):
    return json.dumps(
        value,
        allow_nan=False,
        ensure_ascii=True,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")


def sha256_bytes(value):
    return hashlib.sha256(value).hexdigest()


def require_absolute_file(raw_path, label):
    path = Path(raw_path)
    if not path.is_absolute():
        raise ValueError(f"{label} must be an absolute filesystem path: {raw_path!r}")
    if not path.is_file():
        raise ValueError(f"{label} does not exist or is not a file: {str(path)!r}")
    return path


def require_absolute_output(raw_path):
    path = Path(raw_path)
    if not path.is_absolute():
        raise ValueError(f"--output must be an absolute filesystem path: {raw_path!r}")
    if not path.parent.is_dir():
        raise ValueError(f"--output parent directory does not exist: {str(path.parent)!r}")
    return path


def parse_vector(raw_value, length, label, default=None):
    if raw_value is None:
        if default is None:
            raise ValueError(f"{label} is required.")
        return np.array(default, dtype=float)
    values = np.array([float(item) for item in str(raw_value).split()], dtype=float)
    if values.shape != (length,) or not np.isfinite(values).all():
        raise ValueError(f"{label} must contain {length} finite numbers.")
    return values


def rpy_rotation(rpy):
    roll, pitch, yaw = rpy
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return rz @ ry @ rx


def origin_transform(element):
    result = np.eye(4, dtype=float)
    if element is None:
        return result
    result[:3, :3] = rpy_rotation(
        parse_vector(element.get("rpy"), 3, "origin rpy", default=[0.0, 0.0, 0.0])
    )
    result[:3, 3] = parse_vector(
        element.get("xyz"), 3, "origin xyz", default=[0.0, 0.0, 0.0]
    )
    return result


def axis_angle_transform(axis, angle):
    axis = np.asarray(axis, dtype=float)
    norm = float(np.linalg.norm(axis))
    if not np.isfinite(axis).all() or norm <= 1e-12:
        raise ValueError("joint axis must be finite and non-zero.")
    axis = axis / norm
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    rotation = (
        np.eye(3)
        + math.sin(angle) * skew
        + (1.0 - math.cos(angle)) * (skew @ skew)
    )
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation
    return result


def validate_rigid_transform(value, label):
    matrix = np.asarray(value, dtype=float)
    if matrix.shape != (4, 4) or not np.isfinite(matrix).all():
        raise ValueError(f"{label} must be a finite 4x4 matrix.")
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=1e-9):
        raise ValueError(f"{label} must be homogeneous.")
    if not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-8):
        raise ValueError(f"{label} rotation must be orthonormal.")
    if not np.isclose(np.linalg.det(matrix[:3, :3]), 1.0, atol=1e-8):
        raise ValueError(f"{label} rotation determinant must be +1.")
    return np.array(matrix, copy=True)


def load_stl_vertices(path):
    data = path.read_bytes()
    vertices = []
    if len(data) >= 84:
        triangle_count = struct.unpack_from("<I", data, 80)[0]
        if 84 + triangle_count * 50 == len(data):
            offset = 84
            for _ in range(triangle_count):
                values = struct.unpack_from("<12fH", data, offset)
                vertices.extend((values[3:6], values[6:9], values[9:12]))
                offset += 50
    if not vertices:
        try:
            text = data.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ValueError(f"STL is neither valid binary nor ASCII: {str(path)!r}") from exc
        for line in text.splitlines():
            fields = line.strip().split()
            if len(fields) == 4 and fields[0].lower() == "vertex":
                vertices.append(tuple(float(item) for item in fields[1:]))
    array = np.unique(np.asarray(vertices, dtype=float), axis=0)
    if array.ndim != 2 or array.shape[1] != 3 or len(array) < 3:
        raise ValueError(f"STL contains no usable triangle vertices: {str(path)!r}")
    if not np.isfinite(array).all():
        raise ValueError(f"STL contains non-finite vertices: {str(path)!r}")
    return array


class Joint:
    def __init__(self, element):
        self.name = element.get("name")
        self.kind = element.get("type")
        parent = element.find("parent")
        child = element.find("child")
        if not self.name or parent is None or child is None:
            raise ValueError("every URDF joint requires name, parent, and child.")
        self.parent = parent.get("link")
        self.child = child.get("link")
        self.origin = origin_transform(element.find("origin"))
        axis_element = element.find("axis")
        self.axis = parse_vector(
            None if axis_element is None else axis_element.get("xyz"),
            3,
            f"joint {self.name} axis",
            default=[1.0, 0.0, 0.0],
        )
        mimic = element.find("mimic")
        self.mimic_joint = None if mimic is None else mimic.get("joint")
        self.mimic_multiplier = (
            1.0 if mimic is None else float(mimic.get("multiplier", "1.0"))
        )
        self.mimic_offset = 0.0 if mimic is None else float(mimic.get("offset", "0.0"))


class UrdfKinematics:
    def __init__(self, urdf_bytes, gripper_base_link, actuated_joint):
        root = ET.fromstring(urdf_bytes)
        self.links = {element.get("name"): element for element in root.findall("link")}
        joints = [Joint(element) for element in root.findall("joint")]
        self.joints_by_name = {joint.name: joint for joint in joints}
        self.joint_by_child = {joint.child: joint for joint in joints}
        if len(self.joints_by_name) != len(joints) or len(self.joint_by_child) != len(joints):
            raise ValueError("URDF contains duplicate joint names or child links.")
        if gripper_base_link not in self.links:
            raise ValueError(f"gripper base link is missing from URDF: {gripper_base_link!r}")
        if actuated_joint not in self.joints_by_name:
            raise ValueError(f"actuated joint is missing from URDF: {actuated_joint!r}")
        self.gripper_base_link = gripper_base_link
        self.actuated_joint = actuated_joint

    def joint_position(self, joint, qpos, stack=None):
        if joint.kind == "fixed":
            return 0.0
        if joint.name == self.actuated_joint:
            return qpos
        if joint.mimic_joint is not None:
            stack = set() if stack is None else set(stack)
            if joint.name in stack:
                raise ValueError("URDF mimic joint cycle detected.")
            stack.add(joint.name)
            if joint.mimic_joint not in self.joints_by_name:
                raise ValueError(f"mimic source joint is missing: {joint.mimic_joint!r}")
            source_value = self.joint_position(
                self.joints_by_name[joint.mimic_joint],
                qpos,
                stack,
            )
            return joint.mimic_multiplier * source_value + joint.mimic_offset
        raise ValueError(
            f"non-fixed joint {joint.name!r} is neither actuated nor a mimic joint."
        )

    def T_base_link(self, link_name, qpos, stack=None):
        if link_name == self.gripper_base_link:
            return np.eye(4, dtype=float)
        stack = set() if stack is None else set(stack)
        if link_name in stack:
            raise ValueError("URDF kinematic cycle detected.")
        stack.add(link_name)
        if link_name not in self.joint_by_child:
            raise ValueError(
                f"link {link_name!r} is not a descendant of {self.gripper_base_link!r}."
            )
        joint = self.joint_by_child[link_name]
        parent = self.T_base_link(joint.parent, qpos, stack)
        if joint.kind == "fixed":
            motion = np.eye(4, dtype=float)
        elif joint.kind in {"revolute", "continuous"}:
            motion = axis_angle_transform(
                joint.axis,
                self.joint_position(joint, qpos),
            )
        else:
            raise ValueError(f"unsupported gripper joint type: {joint.kind!r}")
        return parent @ joint.origin @ motion

    def collision_origin_and_scale(self, link_name):
        if link_name not in self.links:
            raise ValueError(f"collision link is missing from URDF: {link_name!r}")
        collisions = self.links[link_name].findall("collision")
        if len(collisions) != 1:
            raise ValueError(
                f"collision link {link_name!r} must have exactly one collision element."
            )
        collision = collisions[0]
        mesh = collision.find("./geometry/mesh")
        if mesh is None:
            raise ValueError(f"collision link {link_name!r} must use one mesh.")
        scale = parse_vector(
            mesh.get("scale"),
            3,
            f"collision mesh scale for {link_name}",
            default=[1.0, 1.0, 1.0],
        )
        if np.any(scale <= 0.0):
            raise ValueError(f"collision mesh scale for {link_name!r} must be positive.")
        return origin_transform(collision.find("origin")), scale


def transform_vertices(transform, vertices):
    return (transform[:3, :3] @ vertices.T).T + transform[:3, 3]


def aabb_corners(vertices):
    minimum = np.min(vertices, axis=0)
    maximum = np.max(vertices, axis=0)
    return np.array(
        list(itertools.product(*zip(minimum, maximum))),
        dtype=float,
    )


def choose_support_point(vertices_tcp, axis, extreme, tiebreak_axis, tiebreak_extreme):
    projection = vertices_tcp @ axis
    target = np.max(projection) if extreme == "max" else np.min(projection)
    candidates = vertices_tcp[np.abs(projection - target) <= 1e-8]
    if len(candidates) == 0:
        raise ValueError("lower-pad contact support face is empty.")
    tie_projection = candidates @ tiebreak_axis
    tie_target = (
        np.max(tie_projection)
        if tiebreak_extreme == "max"
        else np.min(tie_projection)
    )
    selected = candidates[np.abs(tie_projection - tie_target) <= 1e-8]
    return np.mean(selected, axis=0)


def geometry_at_qpos(model, mount, meshes, collision_geometry, qpos):
    T_tcp_base = validate_rigid_transform(
        mount["T_tcp_gripper_base"],
        "T_tcp_gripper_base",
    )
    transformed_by_link = {}
    T_tcp_link_by_name = {}
    all_vertices = []
    for link_name in sorted(meshes):
        T_tcp_link = T_tcp_base @ model.T_base_link(link_name, qpos)
        collision_origin, scale = collision_geometry[link_name]
        vertices = meshes[link_name] * scale
        vertices_tcp = transform_vertices(T_tcp_link @ collision_origin, vertices)
        transformed_by_link[link_name] = vertices_tcp
        T_tcp_link_by_name[link_name] = T_tcp_link
        all_vertices.append(vertices_tcp)

    axis = np.asarray(mount["closing_axis_tcp"], dtype=float)
    axis = axis / np.linalg.norm(axis)
    positive = transformed_by_link[mount["positive_pad_link"]] @ axis
    negative = transformed_by_link[mount["negative_pad_link"]] @ axis
    pad_gap = float(np.min(positive) - np.max(negative))
    if pad_gap <= 0.0:
        raise ValueError(f"pad gap is non-positive at qpos={qpos!r}.")

    lower_vertices = transformed_by_link[mount["lower_pad_link"]]
    tiebreak_axis = np.asarray(mount["lower_pad_tiebreak_axis_tcp"], dtype=float)
    tiebreak_axis = tiebreak_axis / np.linalg.norm(tiebreak_axis)
    contact_point = choose_support_point(
        lower_vertices,
        axis,
        mount["lower_pad_inner_extreme"],
        tiebreak_axis,
        mount["lower_pad_tiebreak_extreme"],
    )
    contact = np.eye(4, dtype=float)
    contact[:3, :3] = T_tcp_link_by_name[mount["lower_pad_link"]][:3, :3]
    contact[:3, 3] = contact_point
    validate_rigid_transform(contact, "T_tcp_lower_pad_contact")

    # Keep the actual, deterministically ordered collision-mesh vertices.
    # An eight-corner TCP-frame AABB is conservative only in that one frame;
    # rotating its phantom corner combinations produced false table collisions
    # tens of millimetres below every real mesh vertex.  Linear world-Z support
    # is attained at a real triangle vertex, so this cloud is both exact for
    # the clearance gate and stable for pointwise qpos interpolation.
    hull = np.vstack(all_vertices)
    return {
        "qpos": float(qpos),
        "pad_gap_m": pad_gap,
        "T_tcp_lower_pad_contact": contact,
        "support_hull_tcp": hull,
    }


def rotation_angle(left, right):
    relative = left.T @ right
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    return math.acos(cosine)


def midpoint_interpolation_error(left, midpoint, right):
    expected_gap = 0.5 * (left["pad_gap_m"] + right["pad_gap_m"])
    gap_error = abs(midpoint["pad_gap_m"] - expected_gap)
    expected_contact = 0.5 * (
        left["T_tcp_lower_pad_contact"][:3, 3]
        + right["T_tcp_lower_pad_contact"][:3, 3]
    )
    contact_error = float(
        np.linalg.norm(midpoint["T_tcp_lower_pad_contact"][:3, 3] - expected_contact)
    )
    expected_hull = 0.5 * (left["support_hull_tcp"] + right["support_hull_tcp"])
    hull_error = float(
        np.max(np.linalg.norm(midpoint["support_hull_tcp"] - expected_hull, axis=1))
    )
    full_rotation = rotation_angle(
        left["T_tcp_lower_pad_contact"][:3, :3],
        right["T_tcp_lower_pad_contact"][:3, :3],
    )
    left_mid = rotation_angle(
        left["T_tcp_lower_pad_contact"][:3, :3],
        midpoint["T_tcp_lower_pad_contact"][:3, :3],
    )
    support_radius = float(
        np.max(np.linalg.norm(midpoint["support_hull_tcp"], axis=1))
    )
    rotation_error = abs(left_mid - 0.5 * full_rotation) * support_radius
    return max(gap_error, contact_error, hull_error, rotation_error)


def adaptive_samples(evaluate, qpos_min, qpos_max, maximum_error):
    samples = {
        float(qpos_min): evaluate(float(qpos_min)),
        float(qpos_max): evaluate(float(qpos_max)),
    }

    def subdivide(left_qpos, right_qpos, depth):
        midpoint_qpos = 0.5 * (left_qpos + right_qpos)
        midpoint = evaluate(midpoint_qpos)
        error = midpoint_interpolation_error(
            samples[left_qpos],
            midpoint,
            samples[right_qpos],
        )
        if error <= maximum_error:
            return
        if depth >= 20 or right_qpos - left_qpos <= 1e-8:
            raise ValueError("adaptive calibration could not meet interpolation error bound.")
        samples[midpoint_qpos] = midpoint
        subdivide(left_qpos, midpoint_qpos, depth + 1)
        subdivide(midpoint_qpos, right_qpos, depth + 1)

    subdivide(float(qpos_min), float(qpos_max), 0)
    ordered = [samples[qpos] for qpos in sorted(samples)]
    observed_error = 0.0
    for left, right in zip(ordered[:-1], ordered[1:]):
        midpoint = evaluate(0.5 * (left["qpos"] + right["qpos"]))
        observed_error = max(
            observed_error,
            midpoint_interpolation_error(left, midpoint, right),
        )
    return ordered, observed_error


def json_geometry(sample):
    return {
        "qpos": sample["qpos"],
        "pad_gap_m": sample["pad_gap_m"],
        "T_tcp_lower_pad_contact": sample["T_tcp_lower_pad_contact"].tolist(),
        "support_hull_tcp": sample["support_hull_tcp"].tolist(),
    }


def parse_collision_mesh_arguments(values):
    result = {}
    for raw_value in values:
        if "=" not in raw_value:
            raise ValueError(
                "--collision-mesh must use <link>=<absolute-path> syntax."
            )
        link_name, raw_path = raw_value.split("=", 1)
        link_name = link_name.strip()
        if not link_name or link_name in result:
            raise ValueError(f"duplicate or empty collision link: {link_name!r}")
        result[link_name] = require_absolute_file(
            raw_path,
            f"collision mesh for {link_name}",
        )
    return result


def generate_artifact(urdf_path, collision_paths, mount_path, maximum_error):
    urdf_bytes = urdf_path.read_bytes()
    mount_bytes = mount_path.read_bytes()
    mount = json.loads(mount_bytes.decode("utf-8"))
    required_mount_fields = {
        "actuated_joint",
        "closing_axis_tcp",
        "gripper_base_link",
        "lower_pad_inner_extreme",
        "lower_pad_link",
        "lower_pad_tiebreak_axis_tcp",
        "lower_pad_tiebreak_extreme",
        "negative_pad_link",
        "positive_pad_link",
        "qpos_max",
        "qpos_min",
        "required_collision_links",
        "T_tcp_gripper_base",
    }
    missing = sorted(required_mount_fields - set(mount))
    if missing:
        raise ValueError(f"mount transform file is missing fields: {missing!r}")
    for enum_name in (
        "lower_pad_inner_extreme",
        "lower_pad_tiebreak_extreme",
    ):
        if mount[enum_name] not in {"min", "max"}:
            raise ValueError(f"{enum_name} must be 'min' or 'max'.")
    required_links = set(mount["required_collision_links"])
    if set(collision_paths) != required_links:
        raise ValueError(
            "collision mesh arguments must match required_collision_links exactly; "
            f"missing={sorted(required_links - set(collision_paths))!r}, "
            f"extra={sorted(set(collision_paths) - required_links)!r}."
        )
    maximum_error = float(maximum_error)
    if not np.isfinite(maximum_error) or not 0.0 < maximum_error <= 0.00025:
        raise ValueError("maximum interpolation error must be within (0, 0.00025] m.")
    qpos_min = float(mount["qpos_min"])
    qpos_max = float(mount["qpos_max"])
    if not np.isfinite([qpos_min, qpos_max]).all() or not qpos_min < qpos_max:
        raise ValueError("mount qpos range must be finite and strictly increasing.")

    model = UrdfKinematics(
        urdf_bytes,
        mount["gripper_base_link"],
        mount["actuated_joint"],
    )
    meshes = {link: load_stl_vertices(path) for link, path in collision_paths.items()}
    collision_geometry = {
        link: model.collision_origin_and_scale(link) for link in collision_paths
    }

    def evaluate(qpos):
        return geometry_at_qpos(
            model,
            mount,
            meshes,
            collision_geometry,
            qpos,
        )

    samples, observed_error = adaptive_samples(
        evaluate,
        qpos_min,
        qpos_max,
        maximum_error,
    )
    gaps = np.array([sample["pad_gap_m"] for sample in samples], dtype=float)
    if not np.all(np.diff(gaps) < 0.0):
        raise ValueError("generated command-to-gap mapping is not strictly monotonic.")

    artifact = {
        "schema_version": SCHEMA_VERSION,
        "expanded_urdf_sha256": sha256_bytes(urdf_bytes),
        "collision_mesh_sha256_by_link": {
            link: sha256_bytes(path.read_bytes())
            for link, path in sorted(collision_paths.items())
        },
        "mount_transforms_sha256": sha256_bytes(mount_bytes),
        "T_tcp_gripper_base": validate_rigid_transform(
            mount["T_tcp_gripper_base"],
            "T_tcp_gripper_base",
        ).tolist(),
        "actuated_joint": mount["actuated_joint"],
        "gripper_base_link": mount["gripper_base_link"],
        "positive_pad_link": mount["positive_pad_link"],
        "negative_pad_link": mount["negative_pad_link"],
        "lower_pad_link": mount["lower_pad_link"],
        "contact_band_qpos": [qpos_min, qpos_max],
        "qualified_preclamp_position": None,
        "interpolation_error_bound_m": maximum_error,
        "observed_max_interpolation_error_m": observed_error,
        "samples": [json_geometry(sample) for sample in samples],
    }
    artifact["canonical_content_sha256"] = sha256_bytes(canonical_json_bytes(artifact))
    return artifact


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--urdf", required=True)
    parser.add_argument(
        "--collision-mesh",
        action="append",
        required=True,
        help="Repeat as <link>=<absolute-STL-path>.",
    )
    parser.add_argument("--mount-transforms", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument(
        "--max-interpolation-error-m",
        type=float,
        default=DEFAULT_MAX_INTERPOLATION_ERROR_M,
    )
    return parser


def main(argv=None):
    arguments = build_parser().parse_args(argv)
    urdf_path = require_absolute_file(arguments.urdf, "--urdf")
    mount_path = require_absolute_file(
        arguments.mount_transforms,
        "--mount-transforms",
    )
    output_path = require_absolute_output(arguments.output)
    collision_paths = parse_collision_mesh_arguments(arguments.collision_mesh)
    artifact = generate_artifact(
        urdf_path,
        collision_paths,
        mount_path,
        arguments.max_interpolation_error_m,
    )
    output_path.write_text(
        json.dumps(artifact, allow_nan=False, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
        newline="\n",
    )
    print(
        "Generated Tuna calibration: "
        f"samples={len(artifact['samples'])}, "
        f"max_error_m={artifact['observed_max_interpolation_error_m']:.9f}, "
        f"sha256={artifact['canonical_content_sha256']}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
