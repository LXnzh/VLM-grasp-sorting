from my_course_pkg.grasp.config import (
    INTERPOLATE_AVG_SPEED,
    INTERPOLATE_FILTER_ANGLE_DEG,
    INTERPOLATE_FILTER_DISTANCE,
)
from my_course_pkg.tasks.tracking.guarded_executor import (
    GuardedMotionExecutor,
)


class _RecordingArmClient:
    def __init__(self):
        self.calls = []

    def interpolate_to_pose(self, pose_msg, **kwargs):
        self.calls.append((pose_msg, kwargs))
        return "motion-result"


def test_guarded_move_to_pose_matches_stable_executor_contract():
    guard_calls = []
    executor = object.__new__(GuardedMotionExecutor)
    executor.arm_api2_client = _RecordingArmClient()
    executor.motion_guard = lambda version, **kwargs: guard_calls.append(
        (version, kwargs)
    )
    executor.movement_version = 7
    executor.guard_enabled = True
    pose_msg = object()

    result = executor.move_to_pose(pose_msg)

    assert result is None
    assert guard_calls == [(7, {"require_stable": False})]
    assert executor.arm_api2_client.calls == [
        (
            pose_msg,
            {
                "avg_speed": INTERPOLATE_AVG_SPEED,
                "filter_distance": INTERPOLATE_FILTER_DISTANCE,
                "filter_angle_deg": INTERPOLATE_FILTER_ANGLE_DEG,
            },
        )
    ]
