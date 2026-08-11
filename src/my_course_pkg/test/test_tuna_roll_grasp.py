import numpy as np
import pytest

from my_course_pkg.grasp.tuna_roll_grasp import (
    CalibrationPoint,
    CalibrationTable,
    QposSample,
    aperture_gate,
    build_contact_support_pose,
    build_pregrasp_pose,
    classify_roll_residual,
    cylinder_aabb_height,
    evaluate_contact_limited_close,
    evaluate_initial_straddle,
    evaluate_lower_finger_access,
    freeze_pivot_contract,
    freeze_retention_contract,
    generate_lift_waypoints,
    generate_radial_directions,
    generate_roll_waypoints,
    interpolate_gripper_geometry,
    required_width,
    support_hull_clearance_lower_bound,
    tool_rotation_for_radial,
    validate_qpos_samples,
)


def transform(translation=(0.0, 0.0, 0.0)):
    value = np.eye(4, dtype=float)
    value[:3, 3] = translation
    return value


def calibration_table():
    hull = np.array(
        [
            [-0.01, -0.01, -0.01],
            [-0.01, 0.01, -0.01],
            [0.01, -0.01, 0.01],
            [0.01, 0.01, 0.01],
        ],
        dtype=float,
    )
    return CalibrationTable(
        source_sha256="a" * 64,
        max_interpolation_error_m=0.0002,
        points=(
            CalibrationPoint(
                qpos=0.40,
                pad_gap_m=0.045,
                T_tcp_lower_pad_contact=transform((0.0, 0.0, 0.080)),
                support_hull_tcp=hull,
            ),
            CalibrationPoint(
                qpos=0.50,
                pad_gap_m=0.035,
                T_tcp_lower_pad_contact=transform((0.0, 0.0, 0.082)),
                support_hull_tcp=hull + np.array([0.0, 0.0, 0.002]),
            ),
            CalibrationPoint(
                qpos=0.60,
                pad_gap_m=0.025,
                T_tcp_lower_pad_contact=transform((0.0, 0.0, 0.084)),
                support_hull_tcp=hull + np.array([0.0, 0.0, 0.004]),
            ),
        ),
    )


@pytest.mark.parametrize("count", [4, 8])
def test_radial_directions_cover_circle_in_stable_order(count):
    directions = generate_radial_directions(count)
    assert len(directions) == count
    np.testing.assert_allclose(directions[0], [1.0, 0.0, 0.0], atol=1e-12)
    for direction in directions:
        assert np.linalg.norm(direction) == pytest.approx(1.0)
        assert direction[2] == pytest.approx(0.0)


def test_radial_directions_reject_shared_profile_style_counts():
    with pytest.raises(ValueError, match="4 or 8"):
        generate_radial_directions(6)


@pytest.mark.parametrize("pitch_deg", [35.0, 40.0, 45.0])
def test_tool_frame_uses_tcp_x_for_closing_and_tcp_z_for_inward_down_approach(
    pitch_deg,
):
    radial = np.array([0.0, 1.0, 0.0])
    rotation = tool_rotation_for_radial(radial, pitch_deg)
    np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-12)
    assert np.linalg.det(rotation) == pytest.approx(1.0)

    tcp_x = rotation[:, 0]
    tcp_z = rotation[:, 2]
    assert np.dot(tcp_z, radial) < 0.0
    assert tcp_z[2] < 0.0
    actual_angle = np.rad2deg(np.arctan2(abs(tcp_z[2]), np.linalg.norm(tcp_z[:2])))
    assert actual_angle == pytest.approx(pitch_deg)
    assert np.dot(tcp_x, tcp_z) == pytest.approx(0.0, abs=1e-12)
    np.testing.assert_allclose(+tcp_x, -(-tcp_x), atol=1e-12)


def test_contact_pose_anchors_calibrated_lower_pad_at_requested_live_bounds_point():
    T_tcp_contact = transform((0.012, 0.0, 0.080))
    pose = build_contact_support_pose(
        center_xy=np.array([0.30, -0.20]),
        radius_m=0.043,
        bottom_z_m=0.70,
        radial_direction=np.array([1.0, 0.0, 0.0]),
        contact_height_m=0.009,
        tool_z_angle_to_horizontal_deg=40.0,
        T_tcp_lower_pad_contact=T_tcp_contact,
    )
    actual_contact = pose @ T_tcp_contact
    np.testing.assert_allclose(
        actual_contact[:3, 3],
        [0.343, -0.20, 0.709],
        rtol=0.0,
        atol=1e-12,
    )


def test_pregrasp_subtracts_tool_z_and_moves_outward_and_upward():
    contact = build_contact_support_pose(
        center_xy=np.array([0.0, 0.0]),
        radius_m=0.043,
        bottom_z_m=0.70,
        radial_direction=np.array([1.0, 0.0, 0.0]),
        contact_height_m=0.009,
        tool_z_angle_to_horizontal_deg=40.0,
        T_tcp_lower_pad_contact=transform(),
    )
    pregrasp = build_pregrasp_pose(contact, 0.1)
    expected = contact[:3, 3] - 0.1 * contact[:3, 2]
    np.testing.assert_allclose(pregrasp[:3, 3], expected, atol=1e-12)
    assert pregrasp[0, 3] > contact[0, 3]
    assert pregrasp[2, 3] > contact[2, 3]


def test_qpos_validator_requires_three_new_distinct_stable_samples():
    samples = (
        QposSample(10.1, 0.499),
        QposSample(10.2, 0.500),
        QposSample(10.3, 0.501),
    )
    result = validate_qpos_samples(samples, command_boundary=10.0, tolerance_rad=0.002)
    assert result.median_qpos == pytest.approx(0.500)
    assert result.peak_to_peak_rad == pytest.approx(0.002)
    assert result.timestamps == (10.1, 10.2, 10.3)


@pytest.mark.parametrize(
    "samples",
    [
        (QposSample(10.0, 0.5), QposSample(10.2, 0.5), QposSample(10.3, 0.5)),
        (QposSample(10.1, 0.5), QposSample(10.1, 0.5), QposSample(10.3, 0.5)),
        (QposSample(10.1, 0.5), QposSample(10.3, 0.5), QposSample(10.2, 0.5)),
        (QposSample(10.1, 0.498), QposSample(10.2, 0.500), QposSample(10.3, 0.501)),
    ],
)
def test_qpos_validator_rejects_stale_duplicate_reordered_or_unstable(samples):
    with pytest.raises(ValueError):
        validate_qpos_samples(samples, command_boundary=10.0, tolerance_rad=0.002)


def test_calibration_interpolation_uses_measured_qpos_not_command_target():
    interpolated = interpolate_gripper_geometry(calibration_table(), 0.45)
    assert interpolated.measured_qpos == pytest.approx(0.45)
    assert interpolated.pad_gap_m == pytest.approx(0.040)
    np.testing.assert_allclose(
        interpolated.T_tcp_lower_pad_contact[:3, 3],
        [0.0, 0.0, 0.081],
        atol=1e-12,
    )


def test_pivot_is_frozen_from_actual_qpos_and_invariant_through_microsegments():
    table = calibration_table()
    T_world_tcp = transform((0.40, -0.20, 0.75))
    samples = (
        QposSample(1.1, 0.449),
        QposSample(1.2, 0.450),
        QposSample(1.3, 0.451),
    )
    contract = freeze_pivot_contract(
        T_world_tcp,
        samples,
        command_boundary=1.0,
        calibration=table,
        qpos_stability_tolerance_rad=0.002,
        max_interpolation_error_m=0.00025,
    )
    waypoints = generate_roll_waypoints(
        contract,
        roll_axis_world=np.array([0.0, 1.0, 0.0]),
        total_angle_deg=10.0,
        max_microsegment_angle_deg=2.5,
    )
    assert [waypoint.angle_deg for waypoint in waypoints] == [2.5, 5.0, 7.5, 10.0]
    pivot_origin = contract.T_world_pivot[:3, 3]
    for waypoint in waypoints:
        actual_pivot = waypoint.T_world_tcp @ contract.T_tcp_pivot_frozen
        np.testing.assert_allclose(actual_pivot[:3, 3], pivot_origin, atol=1e-12)

    with pytest.raises(ValueError, match="positive"):
        generate_roll_waypoints(contract, [0.0, 1.0, 0.0], -10.0, 2.5)


def test_pivot_and_retention_aperture_references_are_distinct():
    table = calibration_table()
    pivot = freeze_pivot_contract(
        transform(),
        (QposSample(1.1, 0.45), QposSample(1.2, 0.45), QposSample(1.3, 0.45)),
        command_boundary=1.0,
        calibration=table,
        qpos_stability_tolerance_rad=0.002,
        max_interpolation_error_m=0.00025,
    )
    retention = freeze_retention_contract(
        full_close_target=0.79,
        samples=(
            QposSample(2.1, 0.55),
            QposSample(2.2, 0.55),
            QposSample(2.3, 0.55),
        ),
        command_boundary=2.0,
        calibration=table,
        qpos_stability_tolerance_rad=0.002,
    )
    assert pivot.measured_pad_gap_m == pytest.approx(0.040)
    assert retention.measured_post_close_pad_gap_m == pytest.approx(0.030)
    assert aperture_gate(0.0404, pivot.measured_pad_gap_m, 0.0005).passed
    assert not aperture_gate(
        0.0306,
        retention.measured_post_close_pad_gap_m,
        0.0005,
    ).passed


def test_lift_waypoints_are_at_most_ten_mm_and_keep_rolled_orientation():
    rolled = transform((0.4, -0.2, 0.8))
    angle = np.deg2rad(30.0)
    rolled[:3, :3] = np.array(
        [
            [np.cos(angle), 0.0, np.sin(angle)],
            [0.0, 1.0, 0.0],
            [-np.sin(angle), 0.0, np.cos(angle)],
        ]
    )
    waypoints = generate_lift_waypoints(rolled, 0.030, 0.010)
    assert len(waypoints) == 3
    assert max(waypoint.increment_m for waypoint in waypoints) <= 0.010 + 1e-12
    for waypoint in waypoints:
        np.testing.assert_allclose(waypoint.T_world_tcp[:3, :3], rolled[:3, :3])
    assert waypoints[-1].T_world_tcp[2, 3] == pytest.approx(0.830)


def test_cylinder_aabb_height_and_oblique_width_include_axial_and_radial_terms():
    diameter = 0.085
    height = 0.03354
    assert cylinder_aabb_height(height, diameter, 0.0) == pytest.approx(height)
    expected_30 = height * np.cos(np.deg2rad(30.0)) + diameter * np.sin(
        np.deg2rad(30.0)
    )
    assert cylinder_aabb_height(height, diameter, 30.0) == pytest.approx(expected_30)
    assert required_width(height, diameter, 30.0) == pytest.approx(expected_30)
    assert required_width(height, diameter, 30.0) > height / np.cos(np.deg2rad(30.0))


def test_three_close_gates_are_independent_and_named():
    initial = evaluate_initial_straddle(0.086, 0.0855, 0.001)
    sweep = evaluate_lower_finger_access(0.0002, 0.0005)
    close = evaluate_contact_limited_close(0.071, 0.072, 0.0005)
    assert initial.name == "initial_straddle"
    assert sweep.name == "lower_finger_sweep"
    assert close.name == "contact_limited_close"
    assert not initial.passed
    assert not sweep.passed
    assert not close.passed


def test_support_hull_clearance_subtracts_calibration_error():
    hull = np.array([[0.0, 0.0, -0.010], [0.0, 0.0, 0.020]])
    clearance = support_hull_clearance_lower_bound(
        T_world_tcp=transform((0.0, 0.0, 0.720)),
        support_hull_tcp=hull,
        table_z_m=0.700,
        interpolation_error_m=0.00025,
    )
    assert clearance == pytest.approx(0.00975)


def test_roll_residual_classifies_preclamp_no_roll_slip_and_success():
    assert classify_roll_residual(
        radial_residual_m=0.003,
        tangential_residual_m=0.0,
        vertical_residual_m=0.0,
        expected_height_increase_m=0.0,
        actual_height_increase_m=0.0,
        tolerance_m=0.002,
        stage="preclamp",
    ) == "preclamp_displacement"
    assert classify_roll_residual(0.0, 0.0, -0.010, 0.020, 0.001, 0.005) == "slide_without_roll"
    assert classify_roll_residual(0.006, 0.0, 0.0, 0.020, 0.020, 0.005) == "excessive_slip"
    assert classify_roll_residual(0.001, 0.001, 0.001, 0.020, 0.019, 0.005) == "success"
