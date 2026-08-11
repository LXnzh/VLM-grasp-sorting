
from . import guarded_executor as _guarded_executor_module
from . import node as _node_module
from . import recovery as _recovery_module
from . import worker as _worker_module
from .guarded_executor import GuardedMotionExecutor
from .node import (
    FoundationPoseGraspNode,
    _DirectMaskFoundationPose,
    _env_enabled,
    _parse_main_args,
    main,
)
from .recovery import ( 
    TargetLostError,
    TargetRecoveryMixin,
    _mask_is_usable_away_from_image_border,
    select_recovery_mask,
)
from .worker import HighRateTrackingWorker, TargetMovedError, TrackedFrame


__all__ = [
    "FoundationPoseGraspNode",
    "GuardedMotionExecutor",
    "HighRateTrackingWorker",
    "TargetLostError",
    "TargetMovedError",
    "TrackedFrame",
    "main",
    "select_recovery_mask",
]


_LEGACY_MODULES = (
    _node_module,
    _recovery_module,
    _worker_module,
    _guarded_executor_module,
)


def __getattr__(name):
    """Expose legacy implementation imports for downstream test patches."""
    for module in _LEGACY_MODULES:
        try:
            return getattr(module, name)
        except AttributeError:
            continue
    raise AttributeError(
        f"module {__name__!r} has no attribute {name!r}"
    )


def __dir__():
    names = set(globals())
    for module in _LEGACY_MODULES:
        names.update(dir(module))
    return sorted(names)


if __name__ == "__main__":
    main()
