import sys


_FACADE_MODULE = "my_course_pkg.tasks.tracking.foundationpose_grasp"


def legacy_override(name, default):
    """Return a legacy façade override when one has been installed."""
    facade = sys.modules.get(_FACADE_MODULE)
    if facade is None:
        return default
    return getattr(facade, name, default)
