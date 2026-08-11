"""Preflight validation for the canonical VLM sorting scene."""

import math

import mujoco
import numpy as np


def _bounds(config, name):
    values = np.asarray(config.get(name, ()), dtype=float)
    if values.shape != (4,) or not np.all(np.isfinite(values)):
        raise ValueError(f"{name} must be [x_min, x_max, y_min, y_max].")
    if values[0] >= values[1] or values[2] >= values[3]:
        raise ValueError(f"{name} minimums must be below maximums.")
    return values


def _inside_rectangle(xy, rectangle, radius=0.0):
    x, y = np.asarray(xy, dtype=float)[:2]
    x_min, x_max, y_min, y_max = rectangle
    return (
        x - radius >= x_min
        and x + radius <= x_max
        and y - radius >= y_min
        and y + radius <= y_max
    )


def _circle_intersects_rectangle(xy, radius, rectangle):
    point = np.asarray(xy, dtype=float)[:2]
    closest = np.clip(
        point,
        [rectangle[0], rectangle[2]],
        [rectangle[1], rectangle[3]],
    )
    return float(np.linalg.norm(point - closest)) <= float(radius)


def validate_sorting_layout(objects_config, classification_bins):
    """Reject sorting layouts that violate table and separation contracts."""
    if not classification_bins or not classification_bins.get("enabled", False):
        return
    constraints = classification_bins.get("layout_constraints", {})
    table = _bounds(constraints, "table_bounds")
    overview = _bounds(constraints, "overview_xy_bounds")
    base = _bounds(constraints, "base_exclusion")
    object_radius = float(
        constraints.get("object_footprint_radius_m", 0.14)
    )
    safety_margin = float(constraints.get("bin_safety_margin_m", 0.04))
    reach = np.asarray(constraints.get("reach_radius_m", ()), dtype=float)
    if (
        object_radius <= 0.0
        or safety_margin < 0.0
        or reach.shape != (2,)
        or reach[0] < 0.0
        or reach[0] >= reach[1]
    ):
        raise ValueError("Invalid sorting layout radii or safety margin.")

    bins = list(classification_bins.get("bins", []))
    if len(bins) != 1:
        raise ValueError("Sorting layout requires exactly one food bin.")
    bin_config = bins[0]
    inner_size = np.asarray(bin_config.get("inner_size", ()), dtype=float)
    wall_thickness = float(bin_config.get("wall_thickness", 0.0))
    position_range = classification_bins.get("position_range", {})
    x_range = np.asarray(position_range.get("x", ()), dtype=float)
    y_range = np.asarray(position_range.get("y", ()), dtype=float)
    if (
        inner_size.shape != (2,)
        or np.any(inner_size <= 0.0)
        or wall_thickness <= 0.0
        or x_range.shape != (2,)
        or y_range.shape != (2,)
    ):
        raise ValueError("Food-bin dimensions and position range are required.")
    half_outer = inner_size / 2.0 + wall_thickness
    bin_envelope = np.array(
        [
            x_range[0] - half_outer[0],
            x_range[1] + half_outer[0],
            y_range[0] - half_outer[1],
            y_range[1] + half_outer[1],
        ],
        dtype=float,
    )
    for name, bounds in (("table", table), ("overview", overview)):
        if not (
            bin_envelope[0] >= bounds[0]
            and bin_envelope[1] <= bounds[1]
            and bin_envelope[2] >= bounds[2]
            and bin_envelope[3] <= bounds[3]
        ):
            raise ValueError(
                f"The complete food-bin footprint leaves {name} bounds."
            )

    expanded_bin = bin_envelope + np.array(
        [
            -object_radius - safety_margin,
            object_radius + safety_margin,
            -object_radius - safety_margin,
            object_radius + safety_margin,
        ]
    )
    for object_config in objects_config or ():
        position = np.asarray(object_config.get("position", ()), dtype=float)
        if position.shape != (3,) or not np.all(np.isfinite(position)):
            raise ValueError("Sorting objects require fixed finite slots.")
        xy = position[:2]
        if not _inside_rectangle(xy, table, object_radius):
            raise ValueError(
                f"Object slot for {object_config['name']!r} leaves the table."
            )
        if not _inside_rectangle(xy, overview, object_radius):
            raise ValueError(
                f"Object slot for {object_config['name']!r} leaves overview."
            )
        if _circle_intersects_rectangle(xy, object_radius, base):
            raise ValueError(
                f"Object slot for {object_config['name']!r} enters robot base."
            )
        if (
            expanded_bin[0] <= xy[0] <= expanded_bin[1]
            and expanded_bin[2] <= xy[1] <= expanded_bin[3]
        ):
            raise ValueError(
                f"Object slot for {object_config['name']!r} can overlap food_bin."
            )

    bin_centers = (
        (x_range[0], y_range[0]),
        (x_range[0], y_range[1]),
        (x_range[1], y_range[0]),
        (x_range[1], y_range[1]),
    )
    for center in bin_centers:
        radius = float(np.linalg.norm(center))
        if not reach[0] <= radius <= reach[1]:
            raise ValueError("A food-bin release target is outside reach bounds.")


def _sorting_visibility_points(objects_config, classification_bins):
    constraints = classification_bins["layout_constraints"]
    table_z = float(constraints.get("table_surface_z_m", -0.025))
    object_height = float(constraints.get("object_height_m", 0.25))
    points = []
    for object_config in objects_config:
        x, y = np.asarray(object_config["position"], dtype=float)[:2]
        for z in (table_z, table_z + object_height):
            points.append([x, y, z])

    bin_config = classification_bins["bins"][0]
    inner_size = np.asarray(bin_config["inner_size"], dtype=float)
    wall_thickness = float(bin_config["wall_thickness"])
    half_outer = inner_size / 2.0 + wall_thickness
    x_range = classification_bins["position_range"]["x"]
    y_range = classification_bins["position_range"]["y"]
    bin_height = (
        float(bin_config["floor_thickness"])
        + float(bin_config["wall_height"])
    )
    for x in (x_range[0] - half_outer[0], x_range[1] + half_outer[0]):
        for y in (y_range[0] - half_outer[1], y_range[1] + half_outer[1]):
            for z in (table_z, table_z + bin_height):
                points.append([x, y, z])
    return np.asarray(points, dtype=float)


def project_points_to_normalized_camera(
    model,
    data,
    camera_name,
    points_world,
    image_size=(1280, 720),
):
    """Project world points to MuJoCo camera normalized image coordinates."""
    camera_id = mujoco.mj_name2id(
        model,
        mujoco.mjtObj.mjOBJ_CAMERA,
        camera_name,
    )
    if camera_id < 0:
        raise ValueError(f"Sorting overview camera not found: {camera_name!r}.")
    mujoco.mj_forward(model, data)
    camera_position = np.asarray(data.cam_xpos[camera_id], dtype=float)
    camera_rotation = np.asarray(data.cam_xmat[camera_id], dtype=float).reshape(3, 3)
    points_world = np.asarray(points_world, dtype=float)
    points_camera = (points_world - camera_position) @ camera_rotation
    depth = -points_camera[:, 2]
    if np.any(depth <= 0.0):
        raise ValueError("World points are behind the requested camera.")
    width, height = (float(value) for value in image_size)
    tan_y = math.tan(math.radians(float(model.cam_fovy[camera_id])) / 2.0)
    tan_x = tan_y * width / height
    normalized_x = points_camera[:, 0] / (depth * tan_x)
    normalized_y = points_camera[:, 1] / (depth * tan_y)
    return np.column_stack([normalized_x, normalized_y])


def validate_sorting_camera_visibility(
    model,
    data,
    camera_name,
    objects_config,
    classification_bins,
    image_size=(1280, 720),
):
    """Project conservative slot/bin envelopes through the home-pose camera."""
    if not classification_bins or not classification_bins.get("enabled", False):
        return
    points_world = _sorting_visibility_points(
        objects_config,
        classification_bins,
    )
    normalized = project_points_to_normalized_camera(
        model,
        data,
        camera_name,
        points_world,
        image_size=image_size,
    )
    normalized_x = normalized[:, 0]
    normalized_y = normalized[:, 1]
    if np.any(np.abs(normalized_x) > 1.0) or np.any(np.abs(normalized_y) > 1.0):
        normalized_radius = np.maximum(
            np.abs(normalized_x),
            np.abs(normalized_y),
        )
        worst_index = int(np.argmax(normalized_radius))
        worst = float(normalized_radius[worst_index])
        raise ValueError(
            "Sorting slots or complete food-bin footprint leave the home "
            f"camera image: point={points_world[worst_index].round(4).tolist()}, "
            "normalized="
            f"[{normalized_x[worst_index]:.3f}, "
            f"{normalized_y[worst_index]:.3f}], maximum={worst:.3f}."
        )
