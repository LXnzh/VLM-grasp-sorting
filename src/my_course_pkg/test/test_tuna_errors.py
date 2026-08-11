import math

import pytest

from my_course_pkg.grasp.tuna_errors import TunaErrorCode, TunaGraspError


def test_tuna_error_codes_cover_distinct_control_flow_failures():
    assert TunaErrorCode.DEPENDENCY_TRANSIENT is not TunaErrorCode.PLANNING
    assert TunaErrorCode.PIVOT_INVALIDATED is not TunaErrorCode.RETENTION
    assert (
        TunaErrorCode.RETENTION_APERTURE_DRIFT
        is not TunaErrorCode.PIVOT_INVALIDATED
    )
    assert {code.value for code in TunaErrorCode} >= {
        "configuration",
        "calibration_sha",
        "dependency_transient",
        "pivot_invalidated",
        "retention_aperture_drift",
        "bounds",
        "planning",
        "collision",
        "contact",
        "motion",
        "retention",
    }


def test_tuna_grasp_error_is_structured_and_serializable():
    error = TunaGraspError(
        TunaErrorCode.PIVOT_INVALIDATED,
        "roll_segment_2",
        {
            "object_name": "tuna_fish_can",
            "aperture_drift_m": 0.0007,
            "samples": [0.031, 0.0317],
            "committed": True,
        },
    )

    assert error.code is TunaErrorCode.PIVOT_INVALIDATED
    assert error.stage == "roll_segment_2"
    assert error.retryable is False
    assert error.to_dict() == {
        "code": "pivot_invalidated",
        "stage": "roll_segment_2",
        "diagnostics": {
            "object_name": "tuna_fish_can",
            "aperture_drift_m": 0.0007,
            "samples": [0.031, 0.0317],
            "committed": True,
        },
        "retryable": False,
    }


@pytest.mark.parametrize(
    "diagnostics",
    [
        {"value": math.nan},
        {"value": math.inf},
        {"nested": [{"value": -math.inf}]},
        {1: "non-string-key"},
        {"value": object()},
    ],
)
def test_tuna_grasp_error_rejects_nonfinite_or_nonserializable_diagnostics(
    diagnostics,
):
    with pytest.raises((TypeError, ValueError), match="diagnostics"):
        TunaGraspError(TunaErrorCode.MOTION, "roll", diagnostics)


def test_tuna_grasp_error_requires_enum_and_nonempty_stage():
    with pytest.raises(TypeError, match="TunaErrorCode"):
        TunaGraspError("motion", "roll", {})
    with pytest.raises(ValueError, match="stage"):
        TunaGraspError(TunaErrorCode.MOTION, " ", {})


def test_retry_control_uses_enum_and_flag_not_message_text():
    transient = TunaGraspError(
        TunaErrorCode.DEPENDENCY_TRANSIENT,
        "finalize",
        {"dependency": "compute_ik", "result_received": False},
        retryable=True,
    )
    semantic = TunaGraspError(
        TunaErrorCode.PLANNING,
        "finalize",
        {"dependency": "compute_ik", "result_received": True},
        retryable=False,
    )

    assert transient.retryable is True
    assert semantic.retryable is False
    assert "timeout" not in str(transient).lower()
