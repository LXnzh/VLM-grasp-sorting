import os
from pathlib import Path, PurePosixPath

import numpy as np

from my_course_pkg.paths import FOUNDATIONPOSE_RESULT_JSON, SELECTED_OBJECT_JSON


def env_float(name, default):
    return float(os.environ.get(name, default))


def env_int(name, default):
    return int(os.environ.get(name, default))


def env_bool(name, default=False):
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}


def env_csv(name, default):
    return [item.strip() for item in os.environ.get(name, default).split(",") if item.strip()]


GRASP_EXECUTION_MODES = frozenset({"lift_return", "safe_place"})


def read_grasp_execution_mode():
    mode = os.environ.get("GRASP_EXECUTION_MODE", "lift_return").strip().lower()
    if mode not in GRASP_EXECUTION_MODES:
        valid_modes = ", ".join(sorted(GRASP_EXECUTION_MODES))
        raise ValueError(
            "GRASP_EXECUTION_MODE must be one of "
            f"{valid_modes}; got {mode!r}."
        )
    return mode


GRASP_EXECUTION_MODE = read_grasp_execution_mode()


def read_return_release_clearance_m():
    clearance_m = env_float("GRASP_RETURN_RELEASE_CLEARANCE_M", "0.02")
    if not np.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError(
            "GRASP_RETURN_RELEASE_CLEARANCE_M must be finite and non-negative; "
            f"got {clearance_m!r}."
        )
    return clearance_m


GRASP_RETURN_RELEASE_CLEARANCE_M = read_return_release_clearance_m()


def read_vertical_grasp_z_offset():
    offset_m = env_float("GRASP_VERTICAL_Z_OFFSET", "0.0")
    if not np.isfinite(offset_m):
        raise ValueError(
            "GRASP_VERTICAL_Z_OFFSET must be finite; "
            f"got {offset_m!r}."
        )
    return offset_m


def read_vertical_min_tcp_above_target_bottom_m():
    clearance_m = env_float(
        "GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M",
        "0.008",
    )
    if not np.isfinite(clearance_m) or clearance_m < 0.0:
        raise ValueError(
            "GRASP_VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M must be finite and "
            f"non-negative; got {clearance_m!r}."
        )
    return clearance_m


def read_top_down_grasp_z_offset():
    offset_m = env_float("GRASP_TOP_DOWN_Z_OFFSET", "0.0")
    if not np.isfinite(offset_m):
        raise ValueError(
            "GRASP_TOP_DOWN_Z_OFFSET must be finite; "
            f"got {offset_m!r}."
        )
    return offset_m


DEFAULT_HAMMER_GRASP_BALANCE_POINT = np.array(
    [-0.030227, -0.009931, 0.015676],
    dtype=float,
)


def read_hammer_grasp_balance_point():
    point = np.array(
        [
            env_float(
                "GRASP_HAMMER_BALANCE_X",
                str(DEFAULT_HAMMER_GRASP_BALANCE_POINT[0]),
            ),
            env_float(
                "GRASP_HAMMER_BALANCE_Y",
                str(DEFAULT_HAMMER_GRASP_BALANCE_POINT[1]),
            ),
            env_float(
                "GRASP_HAMMER_BALANCE_Z",
                str(DEFAULT_HAMMER_GRASP_BALANCE_POINT[2]),
            ),
        ],
        dtype=float,
    )
    if not np.isfinite(point).all():
        raise ValueError(
            "GRASP_HAMMER_BALANCE_X/Y/Z must be finite; "
            f"got {point.tolist()!r}."
        )
    return point


def read_hammer_grasp_balance_weight():
    weight = env_float("GRASP_HAMMER_BALANCE_WEIGHT", "250.0")
    if not np.isfinite(weight) or weight < 0.0:
        raise ValueError(
            "GRASP_HAMMER_BALANCE_WEIGHT must be finite and non-negative; "
            f"got {weight!r}."
        )
    return weight


def read_hammer_max_top_down_angle_deg():
    angle_deg = env_float(
        "GRASP_HAMMER_MAX_TOP_DOWN_ANGLE_DEG",
        "10.0",
    )
    if not np.isfinite(angle_deg) or not 0.0 <= angle_deg <= 180.0:
        raise ValueError(
            "GRASP_HAMMER_MAX_TOP_DOWN_ANGLE_DEG must be finite and within "
            f"[0, 180]; got {angle_deg!r}."
        )
    return angle_deg


def read_final_approach_position_tolerance_m():
    tolerance_m = env_float(
        "GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M",
        "0.018",
    )
    if not np.isfinite(tolerance_m) or tolerance_m < 0.0:
        raise ValueError(
            "GRASP_FINAL_APPROACH_POSITION_TOLERANCE_M must be "
            "finite and non-negative; "
            f"got {tolerance_m!r}."
        )
    return tolerance_m


def _read_positive_finite_float(name, default):
    raw_value = os.environ.get(name, default)
    try:
        value = float(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{name} must be finite and strictly positive; got {raw_value!r}."
        ) from exc
    if not np.isfinite(value) or value <= 0.0:
        raise ValueError(
            f"{name} must be finite and strictly positive; got {value!r}."
        )
    return value


def _read_positive_integer(name, default, *, minimum=1, require_odd=False):
    raw_value = os.environ.get(name, default)
    try:
        value = int(raw_value)
    except (TypeError, ValueError) as exc:
        raise ValueError(
            f"{name} must be an integer greater than or equal to {minimum}; "
            f"got {raw_value!r}."
        ) from exc
    if value < minimum or (require_odd and value % 2 == 0):
        odd_requirement = " and odd" if require_odd else ""
        raise ValueError(
            f"{name} must be an integer greater than or equal to {minimum}"
            f"{odd_requirement}; got {value!r}."
        )
    return value


def read_vertical_approach_xy_tolerance_m():
    return _read_positive_finite_float(
        "GRASP_VERTICAL_APPROACH_XY_TOLERANCE_M",
        "0.0015",
    )


def read_vertical_descent_xy_tolerance_m():
    return _read_positive_finite_float(
        "GRASP_VERTICAL_DESCENT_XY_TOLERANCE_M",
        "0.0020",
    )


def read_vertical_approach_z_tolerance_m():
    return _read_positive_finite_float(
        "GRASP_VERTICAL_APPROACH_Z_TOLERANCE_M",
        "0.003",
    )


def read_vertical_approach_waypoint_max_dist_m():
    return _read_positive_finite_float(
        "GRASP_VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M",
        "0.005",
    )


def read_vertical_reanchor_max_offset_m():
    return _read_positive_finite_float(
        "GRASP_VERTICAL_REANCHOR_MAX_OFFSET_M",
        "0.03",
    )


def read_vertical_reanchor_max_iterations():
    return _read_positive_integer(
        "GRASP_VERTICAL_REANCHOR_MAX_ITERATIONS",
        "3",
    )


def read_vertical_reanchor_settle_sec():
    return _read_positive_finite_float(
        "GRASP_VERTICAL_REANCHOR_SETTLE_SEC",
        "0.4",
    )


def read_vertical_reanchor_sample_count():
    return _read_positive_integer(
        "GRASP_VERTICAL_REANCHOR_SAMPLE_COUNT",
        "3",
        minimum=3,
        require_odd=True,
    )


DEFAULT_DROP_POSITION = np.array([-0.55, -0.45, 0.35], dtype=float)
DEFAULT_PLACE_PREFERRED_XY = np.array([-0.38, -0.65], dtype=float)


def read_drop_position():
    return np.array(
        [
            env_float("DROP_X", str(DEFAULT_DROP_POSITION[0])),
            env_float("DROP_Y", str(DEFAULT_DROP_POSITION[1])),
            env_float("DROP_Z", str(DEFAULT_DROP_POSITION[2])),
        ],
        dtype=float,
    )


FOUNDATIONPOSE_OBJECT_CAMERA_POSE_JSON = FOUNDATIONPOSE_RESULT_JSON
SELECTED_OBJECT_PATH = SELECTED_OBJECT_JSON
DEFAULT_GRASP_ROOT = PurePosixPath("/home/ws/grasps")
GRASP_ROOT = (
    Path(os.environ["GRASP_ROOT"]).expanduser()
    if "GRASP_ROOT" in os.environ
    else DEFAULT_GRASP_ROOT
)

DEFAULT_CAMERA_FRAME = "camera_orbbec"
CAMERA_FRAME = os.environ.get("GRASP_CAMERA_FRAME", DEFAULT_CAMERA_FRAME)
CAMERA_POSE_CONVENTION = os.environ.get(
    "GRASP_CAMERA_POSE_CONVENTION",
    "opencv_to_mujoco",
).strip().lower()

GRIPPER_EFFORT = env_float("GRASP_GRIPPER_EFFORT", "140.0")
GRIPPER_SETTLE_SEC = env_float("GRASP_GRIPPER_SETTLE_SEC", "2.0")
GRIPPER_CLOSE_SETTLE_SEC = env_float("GRASP_GRIPPER_CLOSE_SETTLE_SEC", "3.0")
GRIPPER_OPEN_POSITION = env_float("GRASP_GRIPPER_OPEN", "0.0")
GRIPPER_CLOSED_POSITION = env_float("GRASP_GRIPPER_CLOSED", "0.79")
GRIPPER_OPEN_POSITION_TOLERANCE = env_float(
    "GRASP_GRIPPER_OPEN_POSITION_TOLERANCE",
    "0.05",
)
GRIPPER_OPEN_MAX_POSITION = env_float(
    "GRASP_GRIPPER_OPEN_MAX_POSITION",
    str(GRIPPER_OPEN_POSITION + GRIPPER_OPEN_POSITION_TOLERANCE),
)
GRIPPER_CLOSE_MIN_POSITION = env_float(
    "GRASP_GRIPPER_CLOSE_MIN_POSITION",
    "0.01",
)
GRIPPER_OPEN_ATTEMPTS = max(1, env_int("GRASP_GRIPPER_OPEN_ATTEMPTS", "2"))
GRIPPER_OPEN_RETRY_DELAY_SEC = env_float(
    "GRASP_GRIPPER_OPEN_RETRY_DELAY_SEC",
    "0.2",
)
GRIPPER_COMMAND_MODE = os.environ.get("GRASP_GRIPPER_COMMAND_MODE", "action").strip().lower()
GRIPPER_ACTION_NAMES = env_csv(
    "GRASP_GRIPPER_ACTIONS",
    "/robotiq_2f_urcap_adapter/gripper_command",
)
GRIPPER_ACTION_WAIT_SEC = env_float("GRASP_GRIPPER_ACTION_WAIT_SEC", "15.0")
GRASP_DEBUG_STOP_AT_GRASP = env_bool("GRASP_DEBUG_STOP_AT_GRASP")
GRASP_DEBUG_STOP_AT_PREGRASP = env_bool("GRASP_DEBUG_STOP_AT_PREGRASP")
GRASP_DEBUG_STOP_AFTER_CLOSE = env_bool("GRASP_DEBUG_STOP_AFTER_CLOSE")
GRASP_DEBUG_STOP_AFTER_LIFT = env_bool("GRASP_DEBUG_STOP_AFTER_LIFT")
GRASP_DEBUG_STOP_BEFORE_RELEASE = env_bool("GRASP_DEBUG_STOP_BEFORE_RELEASE")
GRASP_CANONICALIZE_TABLETOP_OBJECT_POSE = env_bool(
    "GRASP_CANONICALIZE_TABLETOP_OBJECT_POSE",
    True,
)
GRASP_TABLETOP_CANONICAL_TILT_WARN_DEG = env_float(
    "GRASP_TABLETOP_CANONICAL_TILT_WARN_DEG",
    "15.0",
)

APPROACH_DIST = env_float("GRASP_APPROACH_DIST", "0.1")
WAYPOINT_MAX_DIST = env_float("GRASP_WAYPOINT_MAX_DIST", "0.02")
INTERPOLATE_AVG_SPEED = env_float("GRASP_INTERPOLATE_AVG_SPEED", "0.2")
INTERPOLATE_FILTER_DISTANCE = env_float("GRASP_INTERPOLATE_FILTER_DISTANCE", "2.0")
INTERPOLATE_FILTER_ANGLE_DEG = env_float("GRASP_INTERPOLATE_FILTER_ANGLE_DEG", "180.0")
FINAL_APPROACH_POSITION_TOLERANCE_M = (
    read_final_approach_position_tolerance_m()
)
FINAL_APPROACH_SETTLE_TIMEOUT_SEC = env_float(
    "GRASP_FINAL_APPROACH_SETTLE_TIMEOUT_SEC", "1.5"
)
FINAL_APPROACH_COMMAND_PERIOD_SEC = env_float(
    "GRASP_FINAL_APPROACH_COMMAND_PERIOD_SEC", "0.05"
)
VERTICAL_APPROACH_XY_TOLERANCE_M = read_vertical_approach_xy_tolerance_m()
VERTICAL_DESCENT_XY_TOLERANCE_M = read_vertical_descent_xy_tolerance_m()
VERTICAL_APPROACH_Z_TOLERANCE_M = read_vertical_approach_z_tolerance_m()
VERTICAL_APPROACH_WAYPOINT_MAX_DIST_M = (
    read_vertical_approach_waypoint_max_dist_m()
)
VERTICAL_REANCHOR_MAX_OFFSET_M = read_vertical_reanchor_max_offset_m()
VERTICAL_REANCHOR_MAX_ITERATIONS = read_vertical_reanchor_max_iterations()
VERTICAL_REANCHOR_SETTLE_SEC = read_vertical_reanchor_settle_sec()
VERTICAL_REANCHOR_SAMPLE_COUNT = read_vertical_reanchor_sample_count()
GRASP_LIFT_HOLD_SEC = env_float("GRASP_LIFT_HOLD_SEC", "5.0")
GRASP_LIFT_HEIGHT = env_float("GRASP_LIFT_HEIGHT", "0.2")
DROP_HIGH_HOLD_SEC = env_float("GRASP_DROP_HIGH_HOLD_SEC", "1.0")
RELEASE_PRE_OPEN_HOLD_SEC = env_float("GRASP_RELEASE_PRE_OPEN_HOLD_SEC", "1.0")
GRASP_Z_OFFSET = env_float("GRASP_Z_OFFSET", "-0.02")
PEAR_GRASP_Z_OFFSET = env_float("GRASP_PEAR_Z_OFFSET", "0.005")
VERTICAL_GRASP_Z_OFFSET = read_vertical_grasp_z_offset()
VERTICAL_MIN_TCP_ABOVE_TARGET_BOTTOM_M = (
    read_vertical_min_tcp_above_target_bottom_m()
)
TOP_DOWN_GRASP_Z_OFFSET = read_top_down_grasp_z_offset()
HAMMER_GRASP_BALANCE_POINT = read_hammer_grasp_balance_point()
HAMMER_GRASP_BALANCE_WEIGHT = read_hammer_grasp_balance_weight()
HAMMER_MAX_TOP_DOWN_ANGLE_DEG = read_hammer_max_top_down_angle_deg()
SIDE_GRASP_Z_OFFSET = env_float("SIDE_GRASP_Z_OFFSET", "0.0")
SIDE_GRASP_CANDIDATE_COUNT = max(
    1,
    # Keep enough valid side grasps for MoveIt to reject collision-prone
    # approaches without exhausting the geometric candidate set.
    env_int("GRASP_MAX_SIDE_CANDIDATES", "16"),
)
SIDE_GRASP_PREFERRED_TILT_DEG = env_float(
    "GRASP_SIDE_PREFERRED_TILT_DEG",
    "45.0",
)
SIDE_GRASP_MIN_TOOL_Z_DOWN_ANGLE_DEG = env_float(
    "GRASP_SIDE_MIN_TOOL_Z_DOWN_ANGLE_DEG",
    "38.0",
)
SIDE_GRASP_MAX_TOOL_Z_DOWN_ANGLE_DEG = env_float(
    "GRASP_SIDE_MAX_TOOL_Z_DOWN_ANGLE_DEG",
    "58.0",
)
SIDE_GRASP_MAX_SIDE_AXIS_ANGLE_DEG = env_float(
    "GRASP_SIDE_MAX_SIDE_AXIS_ANGLE_DEG",
    "75.0",
)
SIDE_GRASP_MIN_HEIGHT_M = env_float(
    "GRASP_SIDE_MIN_HEIGHT_M",
    "0.06",
)
SIDE_GRASP_MAX_HEIGHT_M = env_float(
    "GRASP_SIDE_MAX_HEIGHT_M",
    "0.11",
)
SIDE_GRASP_TARGET_HEIGHT_M = env_float(
    "GRASP_SIDE_TARGET_HEIGHT_M",
    "0.082",
)
SIDE_GRASP_STEEP_TILT_PENALTY = env_float(
    "GRASP_SIDE_STEEP_TILT_PENALTY",
    "3.0",
)
SIDE_GRASP_HEIGHT_WEIGHT = env_float(
    "GRASP_SIDE_HEIGHT_WEIGHT",
    "250.0",
)
SIDE_GRASP_MAX_CENTERLINE_OFFSET_M = env_float(
    "GRASP_SIDE_MAX_CENTERLINE_OFFSET_M",
    "0.025",
)
SIDE_GRASP_CENTERLINE_WEIGHT = env_float(
    "GRASP_SIDE_CENTERLINE_WEIGHT",
    "300.0",
)
SIDE_GRASP_MIN_WORLD_Z_ABOVE_OBJECT_M = env_float(
    "GRASP_SIDE_MIN_WORLD_Z_ABOVE_OBJECT_M",
    "0.04",
)
SIDE_GRASP_CLEARANCE_ENABLED = env_bool("GRASP_SIDE_CLEARANCE_ENABLED", True)
SIDE_GRASP_APPROACH_CORRIDOR_RADIUS_M = env_float(
    "GRASP_SIDE_APPROACH_CORRIDOR_RADIUS_M",
    "0.08",
)
SIDE_GRASP_APPROACH_CLEARANCE_MARGIN_M = env_float(
    "GRASP_SIDE_APPROACH_CLEARANCE_MARGIN_M",
    "0.03",
)


def read_side_grasp_approach_vertical_margin_m():
    return env_float("GRASP_SIDE_APPROACH_VERTICAL_MARGIN_M", "0.12")


SIDE_GRASP_APPROACH_VERTICAL_MARGIN_M = (
    read_side_grasp_approach_vertical_margin_m()
)
GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC = env_float(
    "GRASP_SCENE_DESCRIPTION_TIMEOUT_SEC",
    "0.5",
)
GRASP_SCENE_CLEARANCE_TOPIC = (
    os.environ.get("GRASP_SCENE_CLEARANCE_TOPIC", "/scene_clearance_bounds").strip()
    or "/scene_clearance_bounds"
)
SIDE_GRASP_SYMMETRY_YAW_DEG = [
    float(value)
    for value in env_csv("GRASP_SIDE_SYMMETRY_YAW_DEG", "0,90,180,270")
]
SIDE_GRASP_SYMMETRY_CENTER_XY_BY_OBJECT = {
    # YCB can meshes are not centered at object-local XY origin.  Symmetry
    # expansion must rotate candidate poses about the cylinder axis, otherwise
    # 90/180/270 deg copies are translated far away from the can.
    "tomato_soup_can": np.array([-0.0091685, 0.0840175], dtype=float),
}
DROP_POSITION = read_drop_position()
DROP_RELEASE_Z_OFFSET = env_float("GRASP_DROP_RELEASE_Z_OFFSET", "0.0")
GRASP_PLACE_ENABLED = env_bool("GRASP_PLACE_ENABLED", True)
GRASP_PLACE_USE_SCENE = env_bool("GRASP_PLACE_USE_SCENE", True)
GRASP_PLACE_REQUIRE_SCENE = env_bool("GRASP_PLACE_REQUIRE_SCENE", True)
GRASP_PLACE_X_MIN = env_float("GRASP_PLACE_X_MIN", "-0.55")
GRASP_PLACE_X_MAX = env_float("GRASP_PLACE_X_MAX", "-0.20")
GRASP_PLACE_Y_MIN = env_float("GRASP_PLACE_Y_MIN", "-0.85")
GRASP_PLACE_Y_MAX = env_float("GRASP_PLACE_Y_MAX", "-0.50")
GRASP_PLACE_GRID_STEP_M = env_float("GRASP_PLACE_GRID_STEP_M", "0.05")
GRASP_PLACE_OBJECT_CLEARANCE_M = env_float(
    "GRASP_PLACE_OBJECT_CLEARANCE_M",
    "0.08",
)
GRASP_PLACE_EDGE_MARGIN_M = env_float("GRASP_PLACE_EDGE_MARGIN_M", "0.06")
GRASP_PLACE_PREFERRED_X = env_float(
    "GRASP_PLACE_PREFERRED_X",
    str(DEFAULT_PLACE_PREFERRED_XY[0]),
)
GRASP_PLACE_PREFERRED_Y = env_float(
    "GRASP_PLACE_PREFERRED_Y",
    str(DEFAULT_PLACE_PREFERRED_XY[1]),
)
GRASP_PLACE_BASE_EXCLUSION_ENABLED = env_bool(
    "GRASP_PLACE_BASE_EXCLUSION_ENABLED",
    True,
)
GRASP_PLACE_BASE_EXCLUSION_X_MIN = env_float(
    "GRASP_PLACE_BASE_EXCLUSION_X_MIN",
    "-0.05",
)
GRASP_PLACE_BASE_EXCLUSION_X_MAX = env_float(
    "GRASP_PLACE_BASE_EXCLUSION_X_MAX",
    "0.35",
)
GRASP_PLACE_BASE_EXCLUSION_Y_MIN = env_float(
    "GRASP_PLACE_BASE_EXCLUSION_Y_MIN",
    "-0.45",
)
GRASP_PLACE_BASE_EXCLUSION_Y_MAX = env_float(
    "GRASP_PLACE_BASE_EXCLUSION_Y_MAX",
    "0.10",
)
DROP_OFFSET = np.array(
    [
        env_float("GRASP_DROP_OFFSET_X", "0.2"),
        env_float("GRASP_DROP_OFFSET_Y", "0.3"),
        env_float("GRASP_DROP_OFFSET_Z", "0.0"),
    ],
    dtype=float,
)
SIDE_DROP_OFFSET = np.array(
    [
        env_float("GRASP_SIDE_DROP_OFFSET_X", "0.1"),
        env_float("GRASP_SIDE_DROP_OFFSET_Y", "0.1"),
        env_float("GRASP_SIDE_DROP_OFFSET_Z", "0.0"),
    ],
    dtype=float,
)

YCB_GRASP_NAME_MAP = {
    "tomato_soup_can": "005_tomato_soup_can",
    "gelatin_box": "009_gelatin_box",
    "banana": "011_banana",
    "apple": "013_apple",
    "lemon": "014_lemon",
    "peach": "015_peach",
    "pear": "016_pear",
    "orange": "017_orange",
    "plum": "018_plum",
    "sponge": "026_sponge",
    "hammer": "048_hammer",
    "baseball": "055_baseball",
    "tennis_ball": "056_tennis_ball",
    "racquetball": "057_racquetball",
    "foam_brick": "061_foam_brick",
    "rubiks_cube": "077_rubiks_cube",
}

VERTICAL_GRASP_OBJECTS = {
    "gelatin_box",
    "sponge",
    "foam_brick",
    "rubiks_cube",
}
SIDE_GRASP_OBJECTS = {
    "tomato_soup_can",
}
SIDE_GRASP_GEOMETRY_CENTER_BY_OBJECT = {
    # YCB can meshes are not centered at the object origin. Side-grasp
    # centerline checks should use the visible cylinder center instead.
    "tomato_soup_can": np.array([-0.009169, 0.084018, 0.051006], dtype=float),
}
TABLETOP_CANONICAL_OBJECTS = {
    # The YCB can grasp library uses object-local +Z as the cylinder height.
    # FoundationPose can report unstable roll/pitch for a texture-symmetric can
    # standing on a tabletop, so keep its translation but plan grasps with an
    # upright object frame by default.
    "tomato_soup_can",
}
CENTERED_GRASP_OBJECTS = {
    "banana",
    "hammer",
}

GRASP_CATEGORY_BY_OBJECT = {
    **{name: "cylindrical_can" for name in SIDE_GRASP_OBJECTS},
    **{name: "round_top" for name in {
        "apple", "lemon", "peach", "pear", "orange", "plum",
        "baseball", "tennis_ball", "racquetball",
    }},
    **{name: "box" for name in VERTICAL_GRASP_OBJECTS},
    "banana": "banana",
    "hammer": "tool_top",
}
GRASP_PROFILE_BY_CATEGORY = {
    "cylindrical_can": "side",
    "round_top": "round_top",
    "box": "vertical",
    "banana": "centered",
    "tool_top": "top_down",
}
GRASP_PROFILE_BY_OBJECT = {
    name: GRASP_PROFILE_BY_CATEGORY[category]
    for name, category in GRASP_CATEGORY_BY_OBJECT.items()
}

OBJECT_GEOMETRY_BY_NAME = {
    "apple": {
        "center": [0.000859, -0.003784, 0.035552],
        "bbox_size": [0.075448, 0.074871, 0.071889],
    },
    "lemon": {
        "center": [-0.010579, 0.021654, 0.026274],
        "bbox_size": [0.060588, 0.059299, 0.053017],
    },
    "peach": {
        "center": [-0.014270, 0.005633, 0.029108],
        "bbox_size": [0.062123, 0.062632, 0.058645],
    },
    "pear": {
        "center": [-0.033320, 0.017995, 0.032652],
        "bbox_size": [0.066546, 0.100455, 0.065663],
    },
    "orange": {
        "center": [-0.006935, -0.018360, 0.035428],
        "bbox_size": [0.072158, 0.073986, 0.071352],
    },
    "plum": {
        "center": [-0.007818, 0.019424, 0.026223],
        "bbox_size": [0.057190, 0.054941, 0.053040],
    },
    "baseball": {
        "center": [-0.010081, -0.048245, 0.036077],
        "bbox_size": [0.073078, 0.073712, 0.072563],
    },
    "tennis_ball": {
        "center": [0.008212, -0.044278, 0.033132],
        "bbox_size": [0.066975, 0.067030, 0.066457],
    },
    "racquetball": {
        "center": [-0.009053, -0.122232, 0.027596],
        "bbox_size": [0.055778, 0.056056, 0.055574],
    },
}
ROUND_TOP_MAX_APPROACH_ANGLE_DEG = env_float("GRASP_ROUND_TOP_MAX_APPROACH_ANGLE_DEG", "20.0")
ROUND_TOP_SYMMETRY_YAW_DEG = tuple(range(0, 360, 45))
ROUND_TOP_MAX_CENTER_OFFSET_M = env_float("GRASP_ROUND_TOP_MAX_CENTER_OFFSET_M", "0.010")
ROUND_TOP_MIN_NORMALIZED_HEIGHT = env_float("GRASP_ROUND_TOP_MIN_NORMALIZED_HEIGHT", "0.0")
ROUND_TOP_MAX_NORMALIZED_HEIGHT = env_float("GRASP_ROUND_TOP_MAX_NORMALIZED_HEIGHT", "0.20")
ROUND_TOP_TARGET_NORMALIZED_HEIGHT = env_float("GRASP_ROUND_TOP_TARGET_NORMALIZED_HEIGHT", "0.08")
ROUND_TOP_MAX_GRIPPER_OPENING_M = env_float("GRASP_ROUND_TOP_MAX_GRIPPER_OPENING_M", "0.08516")
ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M = env_float(
    "GRASP_ROUND_TOP_TCP_TO_LOWEST_FINGER_Z_M", "-0.104")
ROUND_TOP_TABLE_CLEARANCE_M = env_float("GRASP_ROUND_TOP_TABLE_CLEARANCE_M", "0.005")
ROUND_TOP_CANDIDATE_COUNT = max(1, env_int("GRASP_ROUND_TOP_CANDIDATE_COUNT", "8"))
ROUND_TOP_CLEARANCE_ENABLED = env_bool("GRASP_ROUND_TOP_CLEARANCE_ENABLED", True)
ROUND_TOP_CLEARANCE_REQUIRE_SCENE = env_bool(
    "GRASP_ROUND_TOP_CLEARANCE_REQUIRE_SCENE",
    True,
)
PEAR_MAX_CLOSING_AXIS_ERROR_DEG = env_float(
    "GRASP_PEAR_MAX_CLOSING_AXIS_ERROR_DEG",
    "5.0",
)
PEAR_MIN_OPENING_MARGIN_M = env_float(
    "GRASP_PEAR_MIN_OPENING_MARGIN_M",
    "0.005",
)
PEAR_MAX_FINGER_HEIGHT_DELTA_M = env_float(
    "GRASP_PEAR_MAX_FINGER_HEIGHT_DELTA_M",
    "0.002",
)
PEAR_MAX_ORIENTATION_CORRECTION_DEG = env_float(
    "GRASP_PEAR_MAX_ORIENTATION_CORRECTION_DEG",
    "20.0",
)
PEAR_CLOSE_POSITION_TOLERANCE_RAD = env_float(
    "GRASP_PEAR_CLOSE_POSITION_TOLERANCE_RAD",
    "0.010",
)
# Keep round-top approach-clearance knobs independent from side grasps. The
# initial conservative values describe the same physical gripper envelope, but
# the vertical swept volume must remain separately measurable and tunable.
ROUND_TOP_APPROACH_CORRIDOR_RADIUS_M = env_float(
    "GRASP_ROUND_TOP_APPROACH_CORRIDOR_RADIUS_M",
    "0.08",
)
ROUND_TOP_APPROACH_CLEARANCE_MARGIN_M = env_float(
    "GRASP_ROUND_TOP_APPROACH_CLEARANCE_MARGIN_M",
    "0.03",
)
ROUND_TOP_APPROACH_VERTICAL_MARGIN_M = env_float(
    "GRASP_ROUND_TOP_APPROACH_VERTICAL_MARGIN_M",
    # Covers the measured 0.104 m TCP-to-lowest-finger reach plus margin.
    "0.12",
)
