"""Deterministic user-instruction aliases for perception target selection."""

from dataclasses import dataclass
import re


@dataclass(frozen=True)
class TargetAlias:
    """Separates a canonical YCB identity from its visual grounding language."""

    matched_instruction_alias: str
    selected_object_name: str
    grounding_prompt: str
    accepted_sam2_class_names: tuple[str, ...]
    visual_target_description: str


_BLUE_RACQUETBALL_ALIAS = TargetAlias(
    matched_instruction_alias="blue racquetball",
    selected_object_name="racquetball",
    grounding_prompt="blue ball",
    accepted_sam2_class_names=("blue ball",),
    visual_target_description=(
        "small smooth blue ball; not a yellow/green tennis ball"
    ),
)
_RACQUETBALL_PATTERN = re.compile(
    r"^(?!.*\bblueberry\s+racquetball\b)"
    r".*\b(?:blue\s+racquetball|blue\s+ball|racquetball)\b",
    flags=re.IGNORECASE,
)


def resolve_target_alias(user_instruction: str) -> TargetAlias | None:
    """Return a fixed alias match without performing any I/O."""

    if not isinstance(user_instruction, str):
        return None
    if _RACQUETBALL_PATTERN.search(user_instruction) is None:
        return None
    return _BLUE_RACQUETBALL_ALIAS
