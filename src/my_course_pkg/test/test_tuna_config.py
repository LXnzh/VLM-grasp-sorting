import numpy as np
import pytest

from my_course_pkg.grasp import config


TUNA_ENV_NAMES = (
    "GRASP_TUNA_RADIAL_DIRECTION_COUNT",
    "GRASP_TUNA_CONTACT_HEIGHTS_M",
    "GRASP_TUNA_TOOL_Z_ANGLES_TO_HORIZONTAL_DEG",
    "GRASP_TUNA_APPROACH_DIST_M",
    "GRASP_TUNA_INITIAL_BOUNDS_STABILITY_TOLERANCE_M",
    "GRASP_TUNA_PRECLAMP_BOUNDS_TOLERANCE_M",
    "GRASP_TUNA_BOUNDS_TOLERANCE_M",
    "GRASP_TUNA_MIN_TABLE_CLEARANCE_M",
    "GRASP_TUNA_CALIBRATION_MAX_INTERPOLATION_ERROR_M",
    "GRASP_TUNA_PRECLAMP_POSITION",
    "GRASP_TUNA_MIN_CONTACT_DEFLECTION_M",
    "GRASP_TUNA_QPOS_STABILITY_TOLERANCE_RAD",
    "GRASP_TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M",
    "GRASP_TUNA_PIVOT_APERTURE_DRIFT_TOLERANCE_M",
    "GRASP_TUNA_RETENTION_APERTURE_DRIFT_TOLERANCE_M",
    "GRASP_TUNA_ROLL_MICROSEGMENT_MAX_ANGLE_DEG",
    "GRASP_TUNA_LIFT_MICROSEGMENT_MAX_TRANSLATION_M",
    "GRASP_TUNA_LIFT_FOLLOW_TOLERANCE_M",
    "GRASP_TUNA_ROLL_CHECKPOINT_ANGLES_DEG",
    "GRASP_TUNA_MIN_STRADDLE_MARGIN_M",
    "GRASP_TUNA_TEST_LIFT_M",
    "GRASP_TUNA_LIFT_OBSERVATION_SPACING_M",
    "GRASP_TUNA_POST_CLOSE_HOLD_SEC",
    "GRASP_TUNA_MAX_PHYSICAL_COMMANDS",
    "GRASP_TUNA_DEBUG_STOP_AFTER",
)


@pytest.fixture(autouse=True)
def clear_tuna_environment(monkeypatch):
    for name in TUNA_ENV_NAMES:
        monkeypatch.delenv(name, raising=False)


def test_tuna_defaults_are_isolated_and_conservative():
    tuna = config.read_tuna_config()

    assert tuna.radial_direction_count == 4
    assert tuna.contact_heights_m == (0.008, 0.009, 0.010)
    assert tuna.tool_z_angles_to_horizontal_deg == (35.0, 40.0, 45.0)
    assert tuna.approach_dist_m == pytest.approx(0.1)
    assert tuna.initial_bounds_stability_tolerance_m == pytest.approx(0.002)
    assert tuna.preclamp_bounds_tolerance_m == pytest.approx(0.002)
    assert tuna.bounds_tolerance_m == pytest.approx(0.005)
    assert tuna.min_table_clearance_m == pytest.approx(0.005)
    assert tuna.calibration_max_interpolation_error_m == pytest.approx(0.00025)
    assert tuna.preclamp_position is None
    assert tuna.min_contact_deflection_m == pytest.approx(0.0005)
    assert tuna.qpos_stability_tolerance_rad == pytest.approx(0.002)
    assert tuna.setpoint_hysteresis_tolerance_m == pytest.approx(0.0005)
    assert tuna.pivot_aperture_drift_tolerance_m == pytest.approx(0.0005)
    assert tuna.retention_aperture_drift_tolerance_m == pytest.approx(0.0005)
    assert tuna.roll_microsegment_max_angle_deg == pytest.approx(2.5)
    assert tuna.lift_microsegment_max_translation_m == pytest.approx(0.010)
    assert tuna.lift_follow_tolerance_m == pytest.approx(0.002)
    assert tuna.roll_checkpoint_angles_deg == (10.0, 20.0, 30.0)
    assert tuna.min_straddle_margin_m == pytest.approx(0.001)
    assert tuna.test_lift_m == pytest.approx(0.030)
    assert tuna.lift_observation_spacing_m == pytest.approx(0.050)
    assert tuna.post_close_hold_sec == pytest.approx(0.5)
    assert tuna.max_physical_commands == 96
    assert tuna.debug_stop_after == "generation"


def test_tuna_preclamp_is_explicit_and_must_be_inside_shared_command_range(
    monkeypatch,
):
    monkeypatch.setenv("GRASP_TUNA_PRECLAMP_POSITION", "0.50")
    assert config.read_tuna_config().preclamp_position == pytest.approx(0.50)

    for invalid in ("0.0", "0.79", "nan", "not-a-number"):
        monkeypatch.setenv("GRASP_TUNA_PRECLAMP_POSITION", invalid)
        with pytest.raises(ValueError, match="GRASP_TUNA_PRECLAMP_POSITION"):
            config.read_tuna_config()


@pytest.mark.parametrize(
    ("name", "invalid"),
    [
        ("GRASP_TUNA_INITIAL_BOUNDS_STABILITY_TOLERANCE_M", "0.0021"),
        ("GRASP_TUNA_PRECLAMP_BOUNDS_TOLERANCE_M", "0.0021"),
        ("GRASP_TUNA_BOUNDS_TOLERANCE_M", "0.0051"),
        ("GRASP_TUNA_CALIBRATION_MAX_INTERPOLATION_ERROR_M", "0.000251"),
        ("GRASP_TUNA_QPOS_STABILITY_TOLERANCE_RAD", "0.0021"),
        ("GRASP_TUNA_SETPOINT_HYSTERESIS_TOLERANCE_M", "0.000501"),
        ("GRASP_TUNA_PIVOT_APERTURE_DRIFT_TOLERANCE_M", "0.000501"),
        ("GRASP_TUNA_RETENTION_APERTURE_DRIFT_TOLERANCE_M", "0.000501"),
        ("GRASP_TUNA_ROLL_MICROSEGMENT_MAX_ANGLE_DEG", "2.5001"),
        ("GRASP_TUNA_LIFT_MICROSEGMENT_MAX_TRANSLATION_M", "0.01001"),
        ("GRASP_TUNA_LIFT_FOLLOW_TOLERANCE_M", "0.00201"),
        ("GRASP_TUNA_LIFT_OBSERVATION_SPACING_M", "0.0501"),
        ("GRASP_TUNA_MAX_PHYSICAL_COMMANDS", "97"),
    ],
)
def test_tuna_safety_overrides_may_not_loosen_maxima(monkeypatch, name, invalid):
    monkeypatch.setenv(name, invalid)
    with pytest.raises(ValueError, match=name):
        config.read_tuna_config()


@pytest.mark.parametrize(
    ("name", "invalid"),
    [
        ("GRASP_TUNA_MIN_TABLE_CLEARANCE_M", "0.0049"),
        ("GRASP_TUNA_MIN_CONTACT_DEFLECTION_M", "0.00049"),
        ("GRASP_TUNA_MIN_STRADDLE_MARGIN_M", "0.0009"),
    ],
)
def test_tuna_safety_overrides_may_not_loosen_minima(monkeypatch, name, invalid):
    monkeypatch.setenv(name, invalid)
    with pytest.raises(ValueError, match=name):
        config.read_tuna_config()


@pytest.mark.parametrize("value", ["0", "3", "5", "9", "nan"])
def test_tuna_radial_direction_count_is_exactly_four_or_eight(monkeypatch, value):
    monkeypatch.setenv("GRASP_TUNA_RADIAL_DIRECTION_COUNT", value)
    with pytest.raises(ValueError, match="GRASP_TUNA_RADIAL_DIRECTION_COUNT"):
        config.read_tuna_config()


def test_tuna_eight_direction_mode_is_explicit_opt_in(monkeypatch):
    monkeypatch.setenv("GRASP_TUNA_RADIAL_DIRECTION_COUNT", "8")
    assert config.read_tuna_config().radial_direction_count == 8


@pytest.mark.parametrize(
    ("name", "invalid"),
    [
        ("GRASP_TUNA_CONTACT_HEIGHTS_M", "0.008,0.010"),
        ("GRASP_TUNA_CONTACT_HEIGHTS_M", "0.007,0.009,0.010"),
        ("GRASP_TUNA_TOOL_Z_ANGLES_TO_HORIZONTAL_DEG", "35,45"),
        ("GRASP_TUNA_TOOL_Z_ANGLES_TO_HORIZONTAL_DEG", "30,40,45"),
        ("GRASP_TUNA_ROLL_CHECKPOINT_ANGLES_DEG", "10,30"),
        ("GRASP_TUNA_ROLL_CHECKPOINT_ANGLES_DEG", "10,20,35"),
    ],
)
def test_tuna_discrete_design_grids_cannot_be_changed(monkeypatch, name, invalid):
    monkeypatch.setenv(name, invalid)
    with pytest.raises(ValueError, match=name):
        config.read_tuna_config()


@pytest.mark.parametrize("value", ["", "invalid", "roll_segment_4", "NONE "])
def test_tuna_debug_stage_rejects_unknown_or_noncanonical_values(monkeypatch, value):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", value)
    if value == "NONE ":
        assert config.read_tuna_config().debug_stop_after == "none"
    else:
        with pytest.raises(ValueError, match="GRASP_TUNA_DEBUG_STOP_AFTER"):
            config.read_tuna_config()


@pytest.mark.parametrize(
    "stage",
    [
        "generation",
        "pregrasp",
        "contact_support",
        "preclamp",
        "roll_segment_1",
        "roll_segment_2",
        "roll_segment_3",
        "close",
        "test_lift",
        "normal_lift",
        "none",
    ],
)
def test_tuna_debug_stage_accepts_exact_enum(monkeypatch, stage):
    monkeypatch.setenv("GRASP_TUNA_DEBUG_STOP_AFTER", stage)
    assert config.read_tuna_config().debug_stop_after == stage


@pytest.mark.parametrize(
    "name",
    [
        "GRASP_TUNA_APPROACH_DIST_M",
        "GRASP_TUNA_POST_CLOSE_HOLD_SEC",
        "GRASP_TUNA_TEST_LIFT_M",
    ],
)
@pytest.mark.parametrize("value", ["0", "-0.1", "nan", "inf"])
def test_tuna_positive_values_reject_nonpositive_or_nonfinite(
    monkeypatch,
    name,
    value,
):
    monkeypatch.setenv(name, value)
    with pytest.raises(ValueError, match=name):
        config.read_tuna_config()


def test_public_tuna_defaults_are_not_numpy_mutable_values():
    for value in (
        config.TUNA_CONTACT_HEIGHTS_M,
        config.TUNA_TOOL_Z_ANGLES_TO_HORIZONTAL_DEG,
        config.TUNA_ROLL_CHECKPOINT_ANGLES_DEG,
    ):
        assert isinstance(value, tuple)
        assert all(np.isfinite(item) for item in value)
