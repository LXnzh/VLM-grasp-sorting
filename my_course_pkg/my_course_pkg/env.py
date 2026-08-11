"""Small, side-effect-free environment parsing helpers."""

import os


def env_bool(name: str, default: bool = False) -> bool:
    """Read a conventional boolean environment variable.

    Missing values retain the caller's boolean default. Any other non-empty
    value is false, matching the package's historic configuration behavior.
    """
    value = os.environ.get(name)
    if value is None:
        return bool(default)
    return value.strip().lower() in {"1", "true", "yes", "on"}
