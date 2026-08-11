from dataclasses import dataclass
import math

import numpy as np


def _readonly_array(value, shape=None):
    array = np.array(value, dtype=float, copy=True)
    if shape is not None and array.shape != shape:
        raise ValueError(f"expected array shape {shape}; got {array.shape}.")
    if not np.isfinite(array).all():
        raise ValueError("array must contain only finite values.")
    array.setflags(write=False)
    return array


def _unit_vector(value, name):
    vector = np.asarray(value, dtype=float)
    if vector.shape != (3,) or not np.isfinite(vector).all():
        raise ValueError(f"{name} must be a finite three-vector.")
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-12:
        raise ValueError(f"{name} must be non-zero.")
    return vector / norm


def _validate_transform(value, name):
    transform = _readonly_array(value, (4, 4))
    if not np.allclose(transform[3], [0.0, 0.0, 0.0, 1.0], rtol=0.0, atol=1e-9):
        raise ValueError(f"{name} must be a homogeneous transform.")
    rotation = transform[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), rtol=0.0, atol=1e-8):
        raise ValueError(f"{name} rotation must be orthonormal.")
    if not np.isclose(np.linalg.det(rotation), 1.0, rtol=0.0, atol=1e-8):
        raise ValueError(f"{name} rotation determinant must be +1.")
    return transform


def _positive_finite(value, name):
    value = float(value)
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(f"{name} must be finite and strictly positive.")
    return value


@dataclass(frozen=True)
class QposSample:
    timestamp: float
    qpos: float

    def __post_init__(self):
        if not np.isfinite(self.timestamp) or not np.isfinite(self.qpos):
            raise ValueError("qpos sample timestamp and value must be finite.")


@dataclass(frozen=True)
class TunaBoundsSample:
    object_name: str
    frame_id: str
    source_timestamp: float
    received_monotonic: float
    center: np.ndarray
    size: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "center", _readonly_array(self.center, (3,)))
        size = _readonly_array(self.size, (3,))
        if np.any(size <= 0.0):
            raise ValueError("Tuna bounds size must be strictly positive.")
        object.__setattr__(self, "size", size)
        if not np.isfinite(self.source_timestamp) or not np.isfinite(
            self.received_monotonic
        ):
            raise ValueError("Tuna bounds timestamps must be finite.")


@dataclass(frozen=True)
class StableTunaBounds:
    source_timestamps: tuple[float, ...]
    received_monotonic: tuple[float, ...]
    center: np.ndarray
    size: np.ndarray
    bottom_z_m: float
    top_z_m: float
    radius_m: float

    def __post_init__(self):
        object.__setattr__(self, "center", _readonly_array(self.center, (3,)))
        object.__setattr__(self, "size", _readonly_array(self.size, (3,)))


@dataclass(frozen=True)
class StableTunaObservation:
    source_timestamps: tuple[float, ...]
    received_monotonic: tuple[float, ...]
    center: np.ndarray
    size: np.ndarray

    def __post_init__(self):
        object.__setattr__(self, "center", _readonly_array(self.center, (3,)))
        object.__setattr__(self, "size", _readonly_array(self.size, (3,)))


@dataclass(frozen=True)
class ValidatedQposBatch:
    timestamps: tuple[float, float, float]
    median_qpos: float
    peak_to_peak_rad: float


@dataclass(frozen=True)
class CalibrationPoint:
    qpos: float
    pad_gap_m: float
    T_tcp_lower_pad_contact: np.ndarray
    support_hull_tcp: np.ndarray

    def __post_init__(self):
        if not np.isfinite(self.qpos):
            raise ValueError("calibration qpos must be finite.")
        if not np.isfinite(self.pad_gap_m) or self.pad_gap_m <= 0.0:
            raise ValueError("calibration pad gap must be finite and positive.")
        object.__setattr__(
            self,
            "T_tcp_lower_pad_contact",
            _validate_transform(
                self.T_tcp_lower_pad_contact,
                "T_tcp_lower_pad_contact",
            ),
        )
        hull = _readonly_array(self.support_hull_tcp)
        if hull.ndim != 2 or hull.shape[1] != 3 or len(hull) < 1:
            raise ValueError("support_hull_tcp must be a non-empty Nx3 array.")
        object.__setattr__(self, "support_hull_tcp", hull)


@dataclass(frozen=True)
class CalibrationTable:
    source_sha256: str
    max_interpolation_error_m: float
    points: tuple[CalibrationPoint, ...]

    def __post_init__(self):
        if (
            not isinstance(self.source_sha256, str)
            or len(self.source_sha256) != 64
            or any(character not in "0123456789abcdef" for character in self.source_sha256)
        ):
            raise ValueError("source_sha256 must be a lowercase 64-character SHA-256.")
        error = float(self.max_interpolation_error_m)
        if not np.isfinite(error) or error < 0.0:
            raise ValueError("max_interpolation_error_m must be finite and non-negative.")
        points = tuple(self.points)
        if len(points) < 2 or not all(isinstance(point, CalibrationPoint) for point in points):
            raise ValueError("calibration requires at least two CalibrationPoint values.")
        qpos = np.array([point.qpos for point in points], dtype=float)
        gaps = np.array([point.pad_gap_m for point in points], dtype=float)
        if not np.all(np.diff(qpos) > 0.0):
            raise ValueError("calibration qpos samples must be strictly increasing.")
        if not np.all(np.diff(gaps) < 0.0):
            raise ValueError("calibration pad gaps must be strictly decreasing.")
        object.__setattr__(self, "max_interpolation_error_m", error)
        object.__setattr__(self, "points", points)


@dataclass(frozen=True)
class InterpolatedGripperGeometry:
    measured_qpos: float
    pad_gap_m: float
    T_tcp_lower_pad_contact: np.ndarray
    support_hull_tcp: np.ndarray
    interpolation_error_m: float

    def __post_init__(self):
        object.__setattr__(
            self,
            "T_tcp_lower_pad_contact",
            _validate_transform(
                self.T_tcp_lower_pad_contact,
                "T_tcp_lower_pad_contact",
            ),
        )
        hull = _readonly_array(self.support_hull_tcp)
        if hull.ndim != 2 or hull.shape[1] != 3:
            raise ValueError("support_hull_tcp must be Nx3.")
        object.__setattr__(self, "support_hull_tcp", hull)


@dataclass(frozen=True)
class FrozenPivotContract:
    calibration_sha256: str
    sample_timestamps: tuple[float, float, float]
    qpos_peak_to_peak_rad: float
    measured_qpos: float
    measured_pad_gap_m: float
    T_world_tcp_preclamp: np.ndarray
    T_tcp_pivot_frozen: np.ndarray
    T_world_pivot: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            "T_world_tcp_preclamp",
            _validate_transform(self.T_world_tcp_preclamp, "T_world_tcp_preclamp"),
        )
        object.__setattr__(
            self,
            "T_tcp_pivot_frozen",
            _validate_transform(self.T_tcp_pivot_frozen, "T_tcp_pivot_frozen"),
        )
        object.__setattr__(
            self,
            "T_world_pivot",
            _validate_transform(self.T_world_pivot, "T_world_pivot"),
        )


@dataclass(frozen=True)
class RetentionContract:
    full_close_target: float
    sample_timestamps: tuple[float, float, float]
    qpos_peak_to_peak_rad: float
    measured_post_close_qpos: float
    measured_post_close_pad_gap_m: float
    calibration_sha256: str


@dataclass(frozen=True)
class RollWaypoint:
    index: int
    total: int
    angle_deg: float
    increment_deg: float
    T_world_tcp: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            "T_world_tcp",
            _validate_transform(self.T_world_tcp, "roll waypoint T_world_tcp"),
        )


@dataclass(frozen=True)
class LiftWaypoint:
    index: int
    total: int
    distance_m: float
    increment_m: float
    T_world_tcp: np.ndarray

    def __post_init__(self):
        object.__setattr__(
            self,
            "T_world_tcp",
            _validate_transform(self.T_world_tcp, "lift waypoint T_world_tcp"),
        )


@dataclass(frozen=True)
class GateResult:
    name: str
    passed: bool
    residual: float
    limit: float


@dataclass(frozen=True)
class ExpectedAABB:
    center: np.ndarray
    size: np.ndarray
    angle_deg: float

    def __post_init__(self):
        object.__setattr__(self, "center", _readonly_array(self.center, (3,)))
        object.__setattr__(self, "size", _readonly_array(self.size, (3,)))


@dataclass(frozen=True)
class FinalizationAttemptMetadata:
    attempt: int
    evidence_after_timestamp: float
    dependency: str
    elapsed_sec: float
    result_received: bool


@dataclass(frozen=True)
class FinalizationCommitMetadata:
    suffix_generation_id: str
    attempt_count: int
    pivot_contract_sha256: str
    plan_revision: int


def generate_radial_directions(count):
    count = int(count)
    if count not in {4, 8}:
        raise ValueError("Tuna radial direction count must be exactly 4 or 8.")
    return tuple(
        _readonly_array(
            [math.cos(2.0 * math.pi * index / count), math.sin(2.0 * math.pi * index / count), 0.0],
            (3,),
        )
        for index in range(count)
    )


def tool_rotation_for_radial(radial_direction, angle_to_horizontal_deg):
    radial = _unit_vector(radial_direction, "radial_direction")
    if abs(radial[2]) > 1e-9:
        raise ValueError("radial_direction must lie in the world-horizontal plane.")
    angle_deg = float(angle_to_horizontal_deg)
    if not np.isfinite(angle_deg) or not 0.0 < angle_deg < 90.0:
        raise ValueError("angle_to_horizontal_deg must be within (0, 90).")
    angle = math.radians(angle_deg)
    world_up = np.array([0.0, 0.0, 1.0], dtype=float)
    tangential = _unit_vector(np.cross(world_up, radial), "tangential_axis")
    tool_z = -math.cos(angle) * radial - math.sin(angle) * world_up
    tool_x = -math.sin(angle) * radial + math.cos(angle) * world_up
    rotation = np.column_stack((tool_x, tangential, tool_z))
    if np.linalg.det(rotation) < 0.0:
        raise ValueError("constructed Tuna tool frame is left-handed.")
    return _readonly_array(rotation, (3, 3))


def build_contact_support_pose(
    center_xy,
    radius_m,
    bottom_z_m,
    radial_direction,
    contact_height_m,
    tool_z_angle_to_horizontal_deg,
    T_tcp_lower_pad_contact,
):
    center_xy = _readonly_array(center_xy, (2,))
    radius_m = _positive_finite(radius_m, "radius_m")
    bottom_z_m = float(bottom_z_m)
    if not np.isfinite(bottom_z_m):
        raise ValueError("bottom_z_m must be finite.")
    contact_height_m = _positive_finite(contact_height_m, "contact_height_m")
    radial = _unit_vector(radial_direction, "radial_direction")
    rotation = tool_rotation_for_radial(radial, tool_z_angle_to_horizontal_deg)
    T_tcp_contact = _validate_transform(
        T_tcp_lower_pad_contact,
        "T_tcp_lower_pad_contact",
    )
    contact_world = np.array(
        [
            center_xy[0] + radius_m * radial[0],
            center_xy[1] + radius_m * radial[1],
            bottom_z_m + contact_height_m,
        ],
        dtype=float,
    )
    result = np.eye(4, dtype=float)
    result[:3, :3] = rotation
    result[:3, 3] = contact_world - rotation @ T_tcp_contact[:3, 3]
    return _validate_transform(result, "T_world_tcp_contact_support")


def build_pregrasp_pose(T_world_tcp_contact, approach_dist_m):
    contact = _validate_transform(T_world_tcp_contact, "T_world_tcp_contact")
    distance = _positive_finite(approach_dist_m, "approach_dist_m")
    result = np.array(contact, copy=True)
    result[:3, 3] -= distance * contact[:3, 2]
    return _validate_transform(result, "T_world_tcp_pregrasp")


def validate_tuna_bounds_samples(
    samples,
    *,
    required_count,
    source_boundary,
    reception_boundary,
    stability_tolerance_m,
    table_z_m,
    table_consistency_tolerance_m,
):
    samples = tuple(samples)
    if len(samples) != int(required_count) or not all(
        isinstance(sample, TunaBoundsSample) for sample in samples
    ):
        raise ValueError(
            f"exactly {required_count} TunaBoundsSample values are required."
        )
    if any(sample.object_name != "tuna_fish_can" for sample in samples):
        raise ValueError("bounds samples must have exact tuna_fish_can identity.")
    if any(sample.frame_id != "world" for sample in samples):
        raise ValueError("bounds samples must be expressed in the world frame.")
    source_timestamps = tuple(float(sample.source_timestamp) for sample in samples)
    reception_timestamps = tuple(
        float(sample.received_monotonic) for sample in samples
    )
    if not all(
        source_timestamps[index] < source_timestamps[index + 1]
        for index in range(len(samples) - 1)
    ):
        raise ValueError("bounds source timestamps must be distinct and increasing.")
    if not all(
        reception_timestamps[index] < reception_timestamps[index + 1]
        for index in range(len(samples) - 1)
    ):
        raise ValueError("bounds reception timestamps must be distinct and increasing.")
    if any(timestamp <= float(source_boundary) for timestamp in source_timestamps):
        raise ValueError("bounds samples are stale relative to the source boundary.")
    if any(timestamp <= float(reception_boundary) for timestamp in reception_timestamps):
        raise ValueError("bounds samples are stale relative to the reception boundary.")

    tolerance = _positive_finite(stability_tolerance_m, "stability_tolerance_m")
    centers = np.stack([sample.center for sample in samples])
    sizes = np.stack([sample.size for sample in samples])
    if np.any(np.std(centers, axis=0) > tolerance + 1e-12) or np.any(
        np.std(sizes, axis=0) > tolerance + 1e-12
    ):
        raise ValueError("bounds samples are unstable.")
    center = np.mean(centers, axis=0)
    size = np.mean(sizes, axis=0)
    if size[2] >= 0.60 * min(size[0], size[1]):
        raise ValueError("bounds do not describe a flat Tuna can.")
    if abs(size[0] - size[1]) > 0.010:
        raise ValueError("bounds horizontal dimensions are inconsistent with a can.")
    bottom_z = float(center[2] - 0.5 * size[2])
    table_z = float(table_z_m)
    table_tolerance = _positive_finite(
        table_consistency_tolerance_m,
        "table_consistency_tolerance_m",
    )
    if not np.isfinite(table_z) or abs(bottom_z - table_z) > table_tolerance:
        raise ValueError("Tuna bottom is inconsistent with planning-scene table Z.")
    return StableTunaBounds(
        source_timestamps=source_timestamps,
        received_monotonic=reception_timestamps,
        center=center,
        size=size,
        bottom_z_m=bottom_z,
        top_z_m=float(center[2] + 0.5 * size[2]),
        radius_m=float(0.25 * (size[0] + size[1])),
    )


def validate_tuna_observation_samples(
    samples,
    *,
    required_count,
    source_boundary,
    reception_boundary,
    stability_tolerance_m,
):
    """Validate fresh exact-name bounds without assuming the can stays flat."""
    samples = tuple(samples)
    if len(samples) != int(required_count) or not all(
        isinstance(sample, TunaBoundsSample) for sample in samples
    ):
        raise ValueError(
            f"exactly {required_count} TunaBoundsSample values are required."
        )
    if any(sample.object_name != "tuna_fish_can" for sample in samples):
        raise ValueError("bounds samples must have exact tuna_fish_can identity.")
    if any(sample.frame_id != "world" for sample in samples):
        raise ValueError("bounds samples must be expressed in the world frame.")
    source_timestamps = tuple(float(sample.source_timestamp) for sample in samples)
    reception_timestamps = tuple(
        float(sample.received_monotonic) for sample in samples
    )
    if not all(
        source_timestamps[index] < source_timestamps[index + 1]
        for index in range(len(samples) - 1)
    ):
        raise ValueError("bounds source timestamps must be distinct and increasing.")
    if not all(
        reception_timestamps[index] < reception_timestamps[index + 1]
        for index in range(len(samples) - 1)
    ):
        raise ValueError("bounds reception timestamps must be distinct and increasing.")
    if any(timestamp <= float(source_boundary) for timestamp in source_timestamps):
        raise ValueError("bounds samples are stale relative to the source boundary.")
    if any(timestamp <= float(reception_boundary) for timestamp in reception_timestamps):
        raise ValueError("bounds samples are stale relative to the reception boundary.")
    tolerance = _positive_finite(stability_tolerance_m, "stability_tolerance_m")
    centers = np.stack([sample.center for sample in samples])
    sizes = np.stack([sample.size for sample in samples])
    if np.any(np.std(centers, axis=0) > tolerance + 1e-12) or np.any(
        np.std(sizes, axis=0) > tolerance + 1e-12
    ):
        raise ValueError("bounds samples are unstable.")
    return StableTunaObservation(
        source_timestamps=source_timestamps,
        received_monotonic=reception_timestamps,
        center=np.mean(centers, axis=0),
        size=np.mean(sizes, axis=0),
    )


def validate_qpos_samples(samples, command_boundary, tolerance_rad):
    samples = tuple(samples)
    if len(samples) != 3 or not all(isinstance(sample, QposSample) for sample in samples):
        raise ValueError("exactly three QposSample values are required.")
    boundary = float(command_boundary)
    tolerance = _positive_finite(tolerance_rad, "tolerance_rad")
    timestamps = tuple(float(sample.timestamp) for sample in samples)
    if not all(timestamp > boundary for timestamp in timestamps):
        raise ValueError("all qpos samples must be strictly newer than the command boundary.")
    if not (timestamps[0] < timestamps[1] < timestamps[2]):
        raise ValueError("qpos sample timestamps must be distinct and strictly increasing.")
    values = np.array([sample.qpos for sample in samples], dtype=float)
    peak_to_peak = float(np.ptp(values))
    if peak_to_peak > tolerance + 1e-12:
        raise ValueError("qpos samples are not stable within the configured tolerance.")
    return ValidatedQposBatch(
        timestamps=timestamps,
        median_qpos=float(np.median(values)),
        peak_to_peak_rad=peak_to_peak,
    )


def _axis_angle_rotation(axis, angle_rad):
    axis = _unit_vector(axis, "rotation axis")
    x, y, z = axis
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    identity = np.eye(3)
    return identity + math.sin(angle_rad) * skew + (1.0 - math.cos(angle_rad)) * (skew @ skew)


def _interpolate_rotation(left, right, fraction):
    relative = left.T @ right
    cosine = float(np.clip((np.trace(relative) - 1.0) / 2.0, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle <= 1e-12:
        return np.array(left, copy=True)
    if abs(math.sin(angle)) <= 1e-9:
        # Calibration intervals are adaptively small; a 180-degree jump is a
        # malformed artifact rather than an interpolation case.
        raise ValueError("calibration rotation interval is singular.")
    axis = np.array(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ],
        dtype=float,
    ) / (2.0 * math.sin(angle))
    return left @ _axis_angle_rotation(axis, fraction * angle)


def interpolate_gripper_geometry(calibration, measured_qpos):
    if not isinstance(calibration, CalibrationTable):
        raise TypeError("calibration must be a CalibrationTable.")
    qpos = float(measured_qpos)
    if not np.isfinite(qpos):
        raise ValueError("measured_qpos must be finite.")
    points = calibration.points
    if qpos < points[0].qpos or qpos > points[-1].qpos:
        raise ValueError("measured_qpos lies outside the calibrated contact band.")
    for left, right in zip(points[:-1], points[1:]):
        if left.qpos <= qpos <= right.qpos:
            fraction = (qpos - left.qpos) / (right.qpos - left.qpos)
            if left.support_hull_tcp.shape != right.support_hull_tcp.shape:
                raise ValueError("adjacent calibration support hulls must have equal shape.")
            matrix = np.eye(4, dtype=float)
            matrix[:3, :3] = _interpolate_rotation(
                left.T_tcp_lower_pad_contact[:3, :3],
                right.T_tcp_lower_pad_contact[:3, :3],
                fraction,
            )
            matrix[:3, 3] = (
                (1.0 - fraction) * left.T_tcp_lower_pad_contact[:3, 3]
                + fraction * right.T_tcp_lower_pad_contact[:3, 3]
            )
            hull = (
                (1.0 - fraction) * left.support_hull_tcp
                + fraction * right.support_hull_tcp
            )
            gap = (1.0 - fraction) * left.pad_gap_m + fraction * right.pad_gap_m
            return InterpolatedGripperGeometry(
                measured_qpos=qpos,
                pad_gap_m=float(gap),
                T_tcp_lower_pad_contact=matrix,
                support_hull_tcp=hull,
                interpolation_error_m=calibration.max_interpolation_error_m,
            )
    raise AssertionError("calibration interval lookup failed")


def freeze_pivot_contract(
    T_world_tcp_preclamp,
    samples,
    command_boundary,
    calibration,
    qpos_stability_tolerance_rad,
    max_interpolation_error_m,
):
    batch = validate_qpos_samples(
        samples,
        command_boundary,
        qpos_stability_tolerance_rad,
    )
    geometry = interpolate_gripper_geometry(calibration, batch.median_qpos)
    maximum_error = _positive_finite(
        max_interpolation_error_m,
        "max_interpolation_error_m",
    )
    if geometry.interpolation_error_m > maximum_error:
        raise ValueError("calibration interpolation error exceeds the Tuna safety bound.")
    T_world_tcp = _validate_transform(T_world_tcp_preclamp, "T_world_tcp_preclamp")
    T_world_pivot = T_world_tcp @ geometry.T_tcp_lower_pad_contact
    return FrozenPivotContract(
        calibration_sha256=calibration.source_sha256,
        sample_timestamps=batch.timestamps,
        qpos_peak_to_peak_rad=batch.peak_to_peak_rad,
        measured_qpos=batch.median_qpos,
        measured_pad_gap_m=geometry.pad_gap_m,
        T_world_tcp_preclamp=T_world_tcp,
        T_tcp_pivot_frozen=geometry.T_tcp_lower_pad_contact,
        T_world_pivot=T_world_pivot,
    )


def freeze_retention_contract(
    full_close_target,
    samples,
    command_boundary,
    calibration,
    qpos_stability_tolerance_rad,
):
    target = float(full_close_target)
    if not np.isfinite(target):
        raise ValueError("full_close_target must be finite.")
    batch = validate_qpos_samples(
        samples,
        command_boundary,
        qpos_stability_tolerance_rad,
    )
    geometry = interpolate_gripper_geometry(calibration, batch.median_qpos)
    return RetentionContract(
        full_close_target=target,
        sample_timestamps=batch.timestamps,
        qpos_peak_to_peak_rad=batch.peak_to_peak_rad,
        measured_post_close_qpos=batch.median_qpos,
        measured_post_close_pad_gap_m=geometry.pad_gap_m,
        calibration_sha256=calibration.source_sha256,
    )


def aperture_gate(current_pad_gap_m, frozen_pad_gap_m, tolerance_m):
    current = float(current_pad_gap_m)
    frozen = float(frozen_pad_gap_m)
    tolerance = _positive_finite(tolerance_m, "tolerance_m")
    if not np.isfinite(current) or not np.isfinite(frozen):
        raise ValueError("aperture values must be finite.")
    residual = abs(current - frozen)
    return GateResult("aperture_drift", residual <= tolerance, residual, tolerance)


def generate_roll_waypoints(
    pivot_contract,
    roll_axis_world,
    total_angle_deg,
    max_microsegment_angle_deg,
    start_angle_deg=0.0,
):
    if not isinstance(pivot_contract, FrozenPivotContract):
        raise TypeError("pivot_contract must be a FrozenPivotContract.")
    axis = _unit_vector(roll_axis_world, "roll_axis_world")
    total_angle = _positive_finite(total_angle_deg, "total_angle_deg")
    start_angle = float(start_angle_deg)
    if not np.isfinite(start_angle) or start_angle < 0.0:
        raise ValueError("start_angle_deg must be finite and non-negative.")
    if total_angle <= start_angle:
        raise ValueError("total_angle_deg must be greater than start_angle_deg.")
    max_increment = _positive_finite(
        max_microsegment_angle_deg,
        "max_microsegment_angle_deg",
    )
    segment_angle = total_angle - start_angle
    count = int(math.ceil(segment_angle / max_increment))
    increment = segment_angle / count
    start = pivot_contract.T_world_tcp_preclamp
    start_position = start[:3, 3]
    pivot_position = pivot_contract.T_world_pivot[:3, 3]
    waypoints = []
    for index in range(1, count + 1):
        angle_deg = start_angle + increment * index
        delta = _axis_angle_rotation(axis, math.radians(angle_deg))
        result = np.eye(4, dtype=float)
        result[:3, :3] = delta @ start[:3, :3]
        result[:3, 3] = pivot_position + delta @ (start_position - pivot_position)
        actual_pivot = result @ pivot_contract.T_tcp_pivot_frozen
        if not np.allclose(
            actual_pivot[:3, 3],
            pivot_position,
            rtol=0.0,
            atol=1e-9,
        ):
            raise ValueError("roll waypoint does not preserve the frozen pivot origin.")
        waypoints.append(
            RollWaypoint(
                index=index,
                total=count,
                angle_deg=angle_deg,
                increment_deg=increment,
                T_world_tcp=result,
            )
        )
    return tuple(waypoints)


def generate_lift_waypoints(T_world_tcp_start, distance_m, max_increment_m):
    start = _validate_transform(T_world_tcp_start, "T_world_tcp_start")
    distance = _positive_finite(distance_m, "distance_m")
    maximum = _positive_finite(max_increment_m, "max_increment_m")
    count = int(math.ceil(distance / maximum))
    increment = distance / count
    result = []
    for index in range(1, count + 1):
        waypoint = np.array(start, copy=True)
        waypoint[2, 3] += increment * index
        result.append(
            LiftWaypoint(
                index=index,
                total=count,
                distance_m=increment * index,
                increment_m=increment,
                T_world_tcp=waypoint,
            )
        )
    return tuple(result)


def required_width(height_m, diameter_m, delta_deg):
    height = _positive_finite(height_m, "height_m")
    diameter = _positive_finite(diameter_m, "diameter_m")
    delta = math.radians(float(delta_deg))
    if not np.isfinite(delta):
        raise ValueError("delta_deg must be finite.")
    return height * abs(math.cos(delta)) + diameter * abs(math.sin(delta))


def cylinder_aabb_height(height_m, diameter_m, roll_angle_deg):
    return required_width(height_m, diameter_m, roll_angle_deg)


def support_hull_clearance_lower_bound(
    T_world_tcp,
    support_hull_tcp,
    table_z_m,
    interpolation_error_m,
):
    transform = _validate_transform(T_world_tcp, "T_world_tcp")
    hull = np.asarray(support_hull_tcp, dtype=float)
    if hull.ndim != 2 or hull.shape[1] != 3 or not np.isfinite(hull).all():
        raise ValueError("support_hull_tcp must be a finite Nx3 array.")
    table_z = float(table_z_m)
    error = float(interpolation_error_m)
    if not np.isfinite(table_z) or not np.isfinite(error) or error < 0.0:
        raise ValueError("table height and interpolation error must be finite.")
    world_vertices = (transform[:3, :3] @ hull.T).T + transform[:3, 3]
    return float(np.min(world_vertices[:, 2]) - table_z - error)


def evaluate_initial_straddle(open_pad_gap_m, projected_width_m, margin_m):
    residual = float(open_pad_gap_m) - float(projected_width_m) - float(margin_m)
    return GateResult("initial_straddle", residual >= 0.0, residual, 0.0)


def evaluate_lower_finger_access(minimum_sweep_clearance_m, required_clearance_m):
    residual = float(minimum_sweep_clearance_m) - float(required_clearance_m)
    return GateResult("lower_finger_sweep", residual >= 0.0, residual, 0.0)


def evaluate_contact_limited_close(measured_pad_gap_m, required_width_m, tolerance_m):
    residual = float(measured_pad_gap_m) - float(required_width_m)
    tolerance = _positive_finite(tolerance_m, "tolerance_m")
    return GateResult(
        "contact_limited_close",
        residual >= -tolerance,
        residual,
        tolerance,
    )


def classify_roll_residual(
    radial_residual_m,
    tangential_residual_m,
    vertical_residual_m,
    expected_height_increase_m,
    actual_height_increase_m,
    tolerance_m,
    stage="roll",
):
    values = np.array(
        [
            radial_residual_m,
            tangential_residual_m,
            vertical_residual_m,
            expected_height_increase_m,
            actual_height_increase_m,
            tolerance_m,
        ],
        dtype=float,
    )
    if not np.isfinite(values).all() or tolerance_m <= 0.0:
        raise ValueError("roll residual inputs must be finite with positive tolerance.")
    if stage == "preclamp":
        return (
            "preclamp_displacement"
            if np.max(np.abs(values[:3])) > tolerance_m
            else "success"
        )
    if actual_height_increase_m < expected_height_increase_m - tolerance_m:
        return "slide_without_roll"
    if np.max(np.abs(values[:3])) > tolerance_m:
        return "excessive_slip"
    return "success"
