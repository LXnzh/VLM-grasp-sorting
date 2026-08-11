import json
import math
from enum import Enum


class TunaErrorCode(Enum):
    CONFIGURATION = "configuration"
    CALIBRATION_SHA = "calibration_sha"
    DEPENDENCY_TRANSIENT = "dependency_transient"
    PIVOT_INVALIDATED = "pivot_invalidated"
    RETENTION_APERTURE_DRIFT = "retention_aperture_drift"
    BOUNDS = "bounds"
    PLANNING = "planning"
    COLLISION = "collision"
    CONTACT = "contact"
    MOTION = "motion"
    RETENTION = "retention"
    COMMAND_BUDGET = "command_budget"


def _validate_diagnostics(value, path="diagnostics"):
    if value is None or isinstance(value, (str, bool, int)):
        return
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{path} must contain only finite numbers.")
        return
    if isinstance(value, list):
        for index, item in enumerate(value):
            _validate_diagnostics(item, f"{path}[{index}]")
        return
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise TypeError(f"{path} keys must be strings.")
            _validate_diagnostics(item, f"{path}.{key}")
        return
    raise TypeError(
        f"{path} must be JSON-serializable without implicit conversion; "
        f"got {type(value).__name__}."
    )


class TunaGraspError(RuntimeError):
    def __init__(self, code, stage, diagnostics, retryable=False):
        if not isinstance(code, TunaErrorCode):
            raise TypeError(f"code must be a TunaErrorCode; got {code!r}.")
        stage = str(stage)
        if not stage.strip():
            raise ValueError("stage must be a non-empty string.")
        if not isinstance(diagnostics, dict):
            raise TypeError("diagnostics must be a dictionary.")
        _validate_diagnostics(diagnostics)
        # This final standard-library check guards future changes to the
        # recursive validator and deliberately disallows NaN/Infinity.
        json.dumps(diagnostics, allow_nan=False, sort_keys=True)

        self.code = code
        self.stage = stage
        self.diagnostics = diagnostics
        self.retryable = bool(retryable)
        super().__init__(f"Tuna grasp failed [{code.value}] at {stage}")

    def to_dict(self):
        return {
            "code": self.code.value,
            "stage": self.stage,
            "diagnostics": self.diagnostics,
            "retryable": self.retryable,
        }
