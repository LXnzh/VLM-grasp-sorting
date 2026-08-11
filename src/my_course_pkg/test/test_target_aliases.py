import json

import pytest

from my_course_pkg.perception import llm_sam2
from my_course_pkg.perception.target_aliases import resolve_target_alias


@pytest.mark.parametrize(
    "instruction",
    [
        "pick up the racquetball",
        "pick up the blue ball",
        "please grasp the blue   ball now",
        "pick up the blue racquetball",
        "BLUE RACQUETBALL",
        "please grasp the Blue   Racquetball now",
    ],
)
def test_blue_racquetball_alias_resolves_canonical_identity_and_visual_prompt(
    instruction,
):
    alias = resolve_target_alias(instruction)

    assert alias is not None
    assert alias.matched_instruction_alias == "blue racquetball"
    assert alias.selected_object_name == "racquetball"
    assert alias.grounding_prompt == "blue ball"
    assert alias.accepted_sam2_class_names == ("blue ball",)
    assert "yellow/green tennis ball" in alias.visual_target_description


@pytest.mark.parametrize(
    "instruction",
    [
        "pick up the blue racquetballs",
        "pick up the blue_racquetball",
        "pick up the blueberry racquetball",
    ],
)
def test_blue_racquetball_alias_rejects_nonmatching_instructions(instruction):
    assert resolve_target_alias(instruction) is None


@pytest.mark.parametrize(
    "instruction",
    [
        "pick up the racquetball",
        "pick up the blue ball",
        "pick up the blue racquetball",
    ],
)
def test_llm_sam2_alias_uses_blue_ball_prompt_and_writes_audit_metadata(
    instruction,
    tmp_path,
    monkeypatch,
):
    captured = {}

    def fail_vlm_selection(_img_path, _prompt, _label):
        raise AssertionError("fixed alias must bypass VLM target selection")

    def fake_sam2_api(img_path, text_prompt):
        captured["img_path"] = img_path
        captured["text_prompt"] = text_prompt
        return {"annotations": []}

    monkeypatch.delenv("VLM_CANDIDATE_OVERRIDE", raising=False)
    monkeypatch.setattr(llm_sam2, "_run_vlm_json", fail_vlm_selection)
    monkeypatch.setattr(llm_sam2, "run_sam2_api", fake_sam2_api)
    monkeypatch.setattr(
        llm_sam2,
        "vlm_classify_food",
        lambda *_args: {
            "category": "non_food",
            "confidence": 1.0,
            "reason": "test",
        },
    )

    node = llm_sam2.LlmSam2Node(img_path=tmp_path / "rgb.png")
    node.output_dir = tmp_path / "vlm_sam2"
    result = node.run(instruction)

    assert captured["text_prompt"] == "blue ball."
    assert result["candidates"] == ["racquetball"]
    assert result["selected_object_name"] == "racquetball"
    assert result["sam2_text_prompt"] == "blue ball."

    selected = json.loads(
        (node.output_dir / "selected_object.json").read_text(encoding="utf-8")
    )
    assert selected["user_instruction"] == instruction
    assert selected["selected_object_name"] == "racquetball"
    assert selected["matched_instruction_alias"] == "blue racquetball"
    assert selected["grounding_prompt"] == "blue ball"
    assert selected["accepted_sam2_class_names"] == ["blue ball"]
    assert selected["target_category"] == "non_food"


def test_vlm_candidate_override_takes_precedence_over_fixed_alias(
    tmp_path,
    monkeypatch,
):
    captured = {}

    def fake_sam2_api(_img_path, text_prompt):
        captured["text_prompt"] = text_prompt
        return {"annotations": []}

    monkeypatch.setenv("VLM_CANDIDATE_OVERRIDE", "apple")
    monkeypatch.setattr(llm_sam2, "run_sam2_api", fake_sam2_api)
    monkeypatch.setattr(
        llm_sam2,
        "vlm_classify_food",
        lambda *_args: {
            "category": "food",
            "confidence": 1.0,
            "reason": "test",
        },
    )

    node = llm_sam2.LlmSam2Node(img_path=tmp_path / "rgb.png")
    node.output_dir = tmp_path / "vlm_sam2"
    result = node.run("pick up the blue racquetball")

    assert result["candidates"] == ["apple"]
    assert result["selected_object_name"] == "apple"
    assert captured["text_prompt"] == "apple."

    selected = json.loads(
        (node.output_dir / "selected_object.json").read_text(encoding="utf-8")
    )
    assert "matched_instruction_alias" not in selected
    assert "accepted_sam2_class_names" not in selected
