"""Conservative world-axis bounds for selected MuJoCo scene objects."""

from dataclasses import dataclass
from pathlib import Path

import numpy as np

from env.utils.ycb_assets import resolve_ycb_assets


@dataclass(frozen=True)
class MeshGeometry:
    """Mesh vertices already transformed into the object body frame."""

    vertices_body: np.ndarray


@dataclass(frozen=True)
class BoxGeometry:
    """A box expressed in the object body frame."""

    center_body: np.ndarray
    rotation_body: np.ndarray
    half_extents: np.ndarray


@dataclass(frozen=True)
class SphereGeometry:
    """A sphere expressed in the object body frame."""

    center_body: np.ndarray
    radius: float


@dataclass(frozen=True)
class CylinderGeometry:
    """A local-Z cylinder expressed in the object body frame."""

    center_body: np.ndarray
    rotation_body: np.ndarray
    radius: float
    half_height: float


@dataclass(frozen=True)
class ObjectGeometryCache:
    """All supported geometry attached directly to one selected object body."""

    object_name: str
    geometries: tuple


@dataclass(frozen=True)
class SceneClearanceBound:
    """A full-size world-axis AABB."""

    name: str
    center: np.ndarray
    size: np.ndarray


def load_obj_vertices(obj_path):
    """Load finite XYZ vertex records from an OBJ file."""
    path = Path(obj_path)
    if not path.is_file():
        raise FileNotFoundError(f"OBJ mesh not found: {path}")

    vertices = []
    with path.open("r", encoding="utf-8", errors="replace") as obj_file:
        for line_number, line in enumerate(obj_file, start=1):
            parts = line.strip().split()
            if not parts or parts[0] != "v":
                continue
            if len(parts) < 4:
                raise ValueError(
                    f"Invalid OBJ vertex record in {path} at line {line_number}: "
                    f"{line.strip()!r}"
                )
            try:
                vertices.append([float(parts[1]), float(parts[2]), float(parts[3])])
            except ValueError as exc:
                raise ValueError(
                    f"Invalid OBJ vertex record in {path} at line {line_number}: "
                    f"{line.strip()!r}"
                ) from exc

    if not vertices:
        raise ValueError(f"OBJ mesh has no vertices: {path}")
    result = np.asarray(vertices, dtype=np.float64)
    if result.ndim != 2 or result.shape[1] != 3:
        raise ValueError(f"OBJ vertex array has invalid shape for {path}: {result.shape}")
    if not np.all(np.isfinite(result)):
        raise ValueError(f"OBJ mesh contains non-finite vertices: {path}")
    return result


def _parse_vector(value, expected_length, *, context, default=None):
    if value is None:
        if default is None:
            raise ValueError(f"Missing vector for {context}")
        result = np.asarray(default, dtype=float)
    else:
        try:
            if isinstance(value, str):
                result = np.asarray(
                    [float(part) for part in value.split()],
                    dtype=float,
                )
            else:
                result = np.asarray(value, dtype=float)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"Invalid numeric vector for {context}: {value!r}") from exc
    if result.shape != (expected_length,):
        raise ValueError(
            f"Expected {expected_length} values for {context}, got {result.tolist()}"
        )
    if not np.all(np.isfinite(result)):
        raise ValueError(f"Non-finite vector for {context}: {result.tolist()}")
    return result


def _positive_values(values, expected_length, *, context):
    result = _parse_vector(values, expected_length, context=context)
    if np.any(result <= 0.0):
        raise ValueError(f"Expected positive values for {context}, got {result.tolist()}")
    return result


def _build_primitive_geometry(object_name, object_type, size):
    context = f"object {object_name!r} {object_type} size"
    if object_type == "box":
        half_extents = _positive_values(size, 3, context=context)
        return BoxGeometry(np.zeros(3), np.eye(3), half_extents)
    if object_type == "sphere":
        radius = float(_positive_values(size, 1, context=context)[0])
        return SphereGeometry(np.zeros(3), radius)
    if object_type == "cylinder":
        cylinder_size = _positive_values(size, 2, context=context)
        return CylinderGeometry(
            np.zeros(3),
            np.eye(3),
            radius=float(cylinder_size[0]),
            half_height=float(cylinder_size[1]),
        )
    raise ValueError(f"Unsupported geometry type for object {object_name!r}: {object_type!r}")


def _build_ycb_geometry_cache(object_name, ycb_dir):
    assets = resolve_ycb_assets(object_name, ycb_dir)
    mesh_paths = (assets.visual_mesh,) + assets.collision_meshes
    geometries = tuple(
        MeshGeometry(load_obj_vertices(mesh_path))
        for mesh_path in mesh_paths
    )
    return ObjectGeometryCache(object_name, geometries)


def build_object_geometry_cache(object_config):
    """Build and validate body-local geometry for one configured object."""
    object_name = str(object_config.get("name") or "").strip()
    if not object_name:
        raise ValueError("Object geometry cache requires a non-empty object name")
    object_type = str(object_config.get("type") or "").strip()
    if object_type == "mesh":
        cache = _build_ycb_geometry_cache(
            object_name,
            object_config.get("ycb_dir"),
        )
    else:
        geometry = _build_primitive_geometry(
            object_name,
            object_type,
            object_config.get("size"),
        )
        cache = ObjectGeometryCache(object_name, (geometry,))

    if not cache.geometries:
        raise ValueError(f"Object {object_name!r} has no supported geometry")
    compute_world_aabb(cache, np.zeros(3), np.eye(3))
    return cache


def _validated_transform(cache, body_position, body_rotation):
    position = np.asarray(body_position, dtype=float)
    rotation = np.asarray(body_rotation, dtype=float)
    if position.shape != (3,):
        raise ValueError(
            f"Invalid body position shape for object {cache.object_name!r}: {position.shape}"
        )
    if rotation.shape != (3, 3):
        raise ValueError(
            f"Invalid body rotation shape for object {cache.object_name!r}: {rotation.shape}"
        )
    if not np.all(np.isfinite(position)) or not np.all(np.isfinite(rotation)):
        raise ValueError(f"Non-finite body pose for object {cache.object_name!r}")
    return position, rotation


def _validated_geometry_array(value, shape, *, context):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"Invalid geometry array for {context}: shape={result.shape}")
    return result


def compute_world_aabb(cache, body_position, body_rotation, epsilon=1e-9):
    """Compute one conservative world-axis AABB from cached local geometry."""
    position, rotation = _validated_transform(cache, body_position, body_rotation)
    minimum = np.full(3, np.inf, dtype=float)
    maximum = np.full(3, -np.inf, dtype=float)

    for index, geometry in enumerate(cache.geometries):
        context = f"object {cache.object_name!r} geometry {index}"
        if isinstance(geometry, MeshGeometry):
            vertices = np.asarray(geometry.vertices_body, dtype=float)
            if (
                vertices.ndim != 2
                or vertices.shape[0] == 0
                or vertices.shape[1] != 3
                or not np.all(np.isfinite(vertices))
            ):
                raise ValueError(f"Invalid mesh vertices for {context}")
            world_vertices = (rotation @ vertices.T).T + position
            geom_minimum = world_vertices.min(axis=0)
            geom_maximum = world_vertices.max(axis=0)
        elif isinstance(geometry, BoxGeometry):
            center_body = _validated_geometry_array(
                geometry.center_body,
                (3,),
                context=f"{context} center",
            )
            rotation_body = _validated_geometry_array(
                geometry.rotation_body,
                (3, 3),
                context=f"{context} rotation",
            )
            half_extents = _validated_geometry_array(
                geometry.half_extents,
                (3,),
                context=f"{context} half extents",
            )
            if np.any(half_extents <= 0.0):
                raise ValueError(f"Non-positive box half extents for {context}")
            center_world = rotation @ center_body + position
            half_world = np.abs(rotation @ rotation_body) @ half_extents
            geom_minimum = center_world - half_world
            geom_maximum = center_world + half_world
        elif isinstance(geometry, SphereGeometry):
            center_body = _validated_geometry_array(
                geometry.center_body,
                (3,),
                context=f"{context} center",
            )
            radius = float(geometry.radius)
            if not np.isfinite(radius) or radius <= 0.0:
                raise ValueError(f"Invalid sphere radius for {context}: {radius}")
            center_world = rotation @ center_body + position
            half_world = np.full(3, radius, dtype=float)
            geom_minimum = center_world - half_world
            geom_maximum = center_world + half_world
        elif isinstance(geometry, CylinderGeometry):
            center_body = _validated_geometry_array(
                geometry.center_body,
                (3,),
                context=f"{context} center",
            )
            rotation_body = _validated_geometry_array(
                geometry.rotation_body,
                (3, 3),
                context=f"{context} rotation",
            )
            radius = float(geometry.radius)
            half_height = float(geometry.half_height)
            if (
                not np.isfinite(radius)
                or not np.isfinite(half_height)
                or radius <= 0.0
                or half_height <= 0.0
            ):
                raise ValueError(
                    f"Invalid cylinder radius/half-height for {context}: "
                    f"{radius}, {half_height}"
                )
            center_world = rotation @ center_body + position
            axis = (rotation @ rotation_body)[:, 2]
            radial_projection = np.sqrt(np.maximum(0.0, 1.0 - axis**2))
            half_world = half_height * np.abs(axis) + radius * radial_projection
            geom_minimum = center_world - half_world
            geom_maximum = center_world + half_world
        else:
            raise ValueError(
                f"Unsupported cached geometry for {context}: {type(geometry).__name__}"
            )

        if not np.all(np.isfinite(geom_minimum)) or not np.all(np.isfinite(geom_maximum)):
            raise ValueError(f"Non-finite world bounds for {context}")
        minimum = np.minimum(minimum, geom_minimum)
        maximum = np.maximum(maximum, geom_maximum)

    size = maximum - minimum
    center = 0.5 * (minimum + maximum)
    if not np.all(np.isfinite(center)) or not np.all(np.isfinite(size)):
        raise ValueError(f"Non-finite AABB for object {cache.object_name!r}")
    if np.any(size <= float(epsilon)):
        raise ValueError(
            f"Non-positive AABB extent for object {cache.object_name!r}: {size.tolist()}"
        )
    return SceneClearanceBound(cache.object_name, center, size)
