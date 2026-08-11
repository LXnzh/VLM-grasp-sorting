import json

import numpy as np
import pytest
from PIL import Image
from pycocotools import mask as mask_utils

from my_course_pkg.perception.foundationpose import (
    FoundationPoseEstimationNode,
    _sam2_class_matches_target,
)


MASK_INDEX_ENV = "FOUNDATIONPOSE_MASK_INDEX"
AUTO_MASK_VERIFY_ENV = "FOUNDATIONPOSE_AUTO_MASK_VERIFY"


def _write_selected(path, target, **extra):
    payload = {
        "candidates": [target],
        "selected_object_name": target,
    }
    payload.update(extra)
    path.write_text(
        json.dumps(payload),
        encoding="utf-8",
    )


def _write_blue_racquetball_alias(path):
    _write_selected(
        path,
        "racquetball",
        matched_instruction_alias="blue racquetball",
        grounding_prompt="blue ball",
        accepted_sam2_class_names=["blue ball"],
        visual_target_description=(
            "small smooth blue ball; not a yellow/green tennis ball"
        ),
    )


def _mask_at(row, col):
    mask = np.zeros((4, 5), dtype=np.uint8)
    mask[row: row + 2, col: col + 2] = 1
    return mask


def _annotation(
    class_name,
    mask,
    *,
    bbox=None,
    grounding_score=None,
    score=None,
):
    rle = mask_utils.encode(np.asfortranarray(mask))
    rle["counts"] = rle["counts"].decode("ascii")

    annotation = {
        "class_name": class_name,
        "segmentation": rle,
    }
    if bbox is not None:
        annotation["bbox"] = bbox
    if grounding_score is not None:
        annotation["grounding_score"] = grounding_score
    if score is not None:
        annotation["score"] = score
    return annotation


def _write_sam2_response(path, annotations):
    path.write_text(
        json.dumps(
            {
                "annotations": annotations,
            }
        ),
        encoding="utf-8",
    )


def _write_rgb(path, size=(20, 16)):
    image = Image.new("RGB", size, color=(120, 120, 120))
    image.save(path)


def _tomato_candidates():
    first_tomato_mask = _mask_at(0, 0)
    second_tomato_mask = _mask_at(1, 2)
    return (
        first_tomato_mask,
        second_tomato_mask,
        [
            _annotation(
                "tomato soup",
                first_tomato_mask,
                bbox=[0, 0, 2, 2],
                grounding_score=0.56,
                score=0.98,
            ),
            _annotation(
                "tomato soup",
                second_tomato_mask,
                bbox=[2, 1, 4, 3],
                grounding_score=0.42,
                score=0.99,
            ),
        ],
    )


def test_load_mask_accepts_sam2_truncated_tomato_soup_label(tmp_path):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    _write_selected(selected_json, "tomato soup can")
    expected_mask = _mask_at(1, 2)
    _write_sam2_response(
        sam2_json,
        [_annotation("tomato soup", expected_mask)],
    )

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        output_dir=tmp_path / "foundationpose",
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, expected_mask.astype(bool))


def test_load_mask_rejects_no_target_matches_and_writes_metadata(tmp_path):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(
        sam2_json,
        [_annotation("banana", _mask_at(0, 0))],
    )

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        output_dir=output_dir,
    )

    with pytest.raises(RuntimeError, match="did not detect the target object"):
        node._load_mask(node._load_target())

    metadata = json.loads((output_dir / "mask_candidates.json").read_text())
    assert metadata["target"] == "tomato soup can"
    assert metadata["candidate_count"] == 0
    assert metadata["candidates"] == []


def test_load_mask_rejects_multiple_matches_without_override(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    rgb_path = tmp_path / "rgb.png"
    _write_selected(selected_json, "tomato soup can")
    _first_mask, _second_mask, annotations = _tomato_candidates()
    _write_sam2_response(sam2_json, annotations)
    _write_rgb(rgb_path, size=(5, 4))
    monkeypatch.setenv(AUTO_MASK_VERIFY_ENV, "0")

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
    )

    with pytest.raises(RuntimeError, match="returned 2 masks"):
        node._load_mask(node._load_target())

    metadata = json.loads((output_dir / "mask_candidates.json").read_text())
    assert metadata["candidate_count"] == 2
    assert metadata["override_env"] == MASK_INDEX_ENV
    assert metadata["candidates"][0]["index"] == 1
    assert metadata["candidates"][0]["source_annotation_index"] == 0
    assert metadata["candidates"][0]["grounding_score"] == pytest.approx(0.56)
    assert metadata["candidates"][1]["index"] == 2
    assert metadata["candidates"][1]["source_annotation_index"] == 1
    assert metadata["candidates"][1]["area_px"] == 4
    assert (output_dir / "mask_candidates_overlay.jpg").exists()
    assert not (output_dir / "mask_verification_result.json").exists()


def test_load_mask_verifier_selects_high_confidence_candidate(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask, second_mask, annotations = _tomato_candidates()
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(sam2_json, annotations)
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(target, candidates, card_path):
        assert target == "tomato soup can"
        assert [candidate["metadata"]["index"] for candidate in candidates] == [1, 2]
        assert card_path.exists()
        return {
            "raw_response": '{"selected_index": 2, "confidence": "high"}',
            "parsed": {"selected_index": 2, "confidence": "high"},
            "selected_index": 2,
            "confidence": "high",
        }

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, second_mask.astype(bool))
    assert not np.array_equal(mask, first_mask.astype(bool))
    selected = json.loads((output_dir / "selected_mask_metadata.json").read_text())
    assert selected["selected_by"] == "vlm_candidate_verifier"
    assert selected["candidate"]["index"] == 2
    verification = json.loads(
        (output_dir / "mask_verification_result.json").read_text()
    )
    assert verification["decision"] == "selected"
    assert verification["selected_index"] == 2
    assert verification["selected_candidate"]["index"] == 2
    assert (output_dir / "mask_verification_candidates.jpg").exists()


def test_load_mask_verifier_low_confidence_falls_back_to_highest_detection_score(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask, second_mask, annotations = _tomato_candidates()
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(sam2_json, annotations)
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        return {
            "raw_response": '{"selected_index": null, "confidence": "low"}',
            "parsed": {"selected_index": None, "confidence": "low"},
            "selected_index": None,
            "confidence": "low",
        }

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, first_mask.astype(bool))
    assert not np.array_equal(mask, second_mask.astype(bool))
    verification = json.loads(
        (output_dir / "mask_verification_result.json").read_text()
    )
    assert verification["decision"] == "uncertain"
    assert verification["selected_candidate"] is None
    selected = json.loads((output_dir / "selected_mask_metadata.json").read_text())
    assert selected["selected_by"] == "highest_detection_score_fallback"
    assert selected["candidate"]["index"] == 1


def test_load_mask_verifier_invalid_index_uses_highest_detection_score_fallback(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask, _second_mask, annotations = _tomato_candidates()
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(sam2_json, annotations)
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        return {
            "raw_response": '{"selected_index": 99, "confidence": "high"}',
            "parsed": {"selected_index": 99, "confidence": "high"},
            "selected_index": 99,
            "confidence": "high",
        }

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, first_mask.astype(bool))
    verification = json.loads(
        (output_dir / "mask_verification_result.json").read_text()
    )
    assert verification["decision"] == "error"
    assert verification["selected_index"] == 99
    assert verification["selected_candidate"] is None
    selected = json.loads((output_dir / "selected_mask_metadata.json").read_text())
    assert selected["selected_by"] == "highest_detection_score_fallback"
    assert selected["candidate"]["index"] == 1


def test_load_mask_verifier_exception_uses_highest_detection_score_fallback(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask, _second_mask, annotations = _tomato_candidates()
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(sam2_json, annotations)
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        raise RuntimeError("verifier unavailable")

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, first_mask.astype(bool))
    verification = json.loads(
        (output_dir / "mask_verification_result.json").read_text()
    )
    assert verification["decision"] == "error"
    assert verification["error"] == "verifier unavailable"
    selected = json.loads((output_dir / "selected_mask_metadata.json").read_text())
    assert selected["selected_by"] == "highest_detection_score_fallback"
    assert selected["candidate"]["index"] == 1


def test_load_mask_fallback_prefers_any_finite_grounding_score_over_score(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask = _mask_at(0, 0)
    second_mask = _mask_at(1, 2)
    _write_selected(selected_json, "apple")
    _write_sam2_response(
        sam2_json,
        [
            _annotation("apple", first_mask, grounding_score=0.20, score=0.10),
            _annotation("apple", second_mask, score=0.99),
        ],
    )
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        return {"selected_index": None, "confidence": "low"}

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, first_mask.astype(bool))


def test_load_mask_fallback_uses_score_when_all_grounding_scores_are_missing(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask = _mask_at(0, 0)
    second_mask = _mask_at(1, 2)
    _write_selected(selected_json, "apple")
    _write_sam2_response(
        sam2_json,
        [
            _annotation("apple", first_mask, score=0.40),
            _annotation("apple", second_mask, score=0.70),
        ],
    )
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        return {"selected_index": None, "confidence": "low"}

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, second_mask.astype(bool))


@pytest.mark.parametrize(
    "first_score,second_score",
    [(None, None), (0.70, 0.70)],
)
def test_load_mask_fallback_ties_choose_lower_candidate_index(
    tmp_path,
    monkeypatch,
    first_score,
    second_score,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    first_mask = _mask_at(0, 0)
    second_mask = _mask_at(1, 2)
    _write_selected(selected_json, "apple")
    _write_sam2_response(
        sam2_json,
        [
            _annotation("apple", first_mask, score=first_score),
            _annotation("apple", second_mask, score=second_score),
        ],
    )
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        return {"selected_index": None, "confidence": "low"}

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, first_mask.astype(bool))


def test_load_mask_override_selects_requested_filtered_candidate(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    first_tomato_mask = _mask_at(0, 0)
    second_tomato_mask = _mask_at(1, 2)
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(
        sam2_json,
        [
            _annotation("banana", _mask_at(0, 2)),
            _annotation("tomato soup", first_tomato_mask),
            _annotation("tomato soup", second_tomato_mask),
        ],
    )
    monkeypatch.setenv(MASK_INDEX_ENV, "2")

    def verifier_should_not_run(_target, _candidates, _card_path):
        raise AssertionError("manual mask override should bypass the verifier")

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        output_dir=output_dir,
        mask_candidate_verifier=verifier_should_not_run,
    )

    mask = node._load_mask(node._load_target())

    np.testing.assert_array_equal(mask, second_tomato_mask.astype(bool))
    selected = json.loads((output_dir / "selected_mask_metadata.json").read_text())
    assert selected["selected_by"] == MASK_INDEX_ENV
    assert selected["candidate"]["index"] == 2
    assert selected["candidate"]["source_annotation_index"] == 2


def test_load_mask_rejects_invalid_override_value(tmp_path, monkeypatch):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(
        sam2_json,
        [_annotation("tomato soup", _mask_at(1, 2))],
    )
    monkeypatch.setenv(MASK_INDEX_ENV, "not-an-int")

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        output_dir=output_dir,
    )

    with pytest.raises(RuntimeError, match="must be an integer"):
        node._load_mask(node._load_target())

    assert not (output_dir / "selected_mask_metadata.json").exists()


def test_run_removes_stale_pose_result_when_mask_selection_fails(
    tmp_path,
    monkeypatch,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    pose_result = output_dir / "pose_result.json"
    pose_error = output_dir / "pose_error.json"
    output_dir.mkdir()
    pose_result.write_text('{"success": true}', encoding="utf-8")
    pose_error.write_text('{"success": false}', encoding="utf-8")
    _write_selected(selected_json, "tomato soup can")
    _write_sam2_response(
        sam2_json,
        [
            _annotation("tomato soup", _mask_at(0, 0)),
            _annotation("tomato soup", _mask_at(1, 2)),
        ],
    )
    monkeypatch.setenv(AUTO_MASK_VERIFY_ENV, "0")

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        output_dir=output_dir,
    )

    with pytest.raises(RuntimeError, match="returned 2 masks"):
        node.run()

    assert not pose_result.exists()
    assert not pose_error.exists()
    assert not (output_dir / "selected_mask_metadata.json").exists()


def test_sam2_class_matching_accepts_dynamic_alias_class_names():
    assert _sam2_class_matches_target(
        "blue_ball",
        "racquetball",
        accepted_sam2_class_names=["blue ball"],
    )
    assert not _sam2_class_matches_target("blue ball", "racquetball")


def test_load_mask_accepts_blue_ball_for_canonical_racquetball_mesh(tmp_path):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    mesh_root = tmp_path / "meshes"
    racquetball_mesh = mesh_root / "057_racquetball"
    racquetball_mesh.mkdir(parents=True)
    (racquetball_mesh / "textured.obj").write_text(
        "v 0 0 0\nv 1 1 1\nv -1 -1 -1\nf 1 2 3\n",
        encoding="utf-8",
    )
    (racquetball_mesh / "textured.mtl").write_text(
        "newmtl material\nmap_Kd texture_map.png\n",
        encoding="utf-8",
    )
    (racquetball_mesh / "texture_map.png").write_bytes(b"fixture")
    _write_blue_racquetball_alias(selected_json)
    expected_mask = _mask_at(1, 2)
    _write_sam2_response(
        sam2_json,
        [_annotation("blue ball", expected_mask)],
    )

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        mesh_root=mesh_root,
        output_dir=output_dir,
    )

    target = node._load_target()
    mask = node._load_mask(target)

    assert target == "racquetball"
    np.testing.assert_array_equal(mask, expected_mask.astype(bool))
    assert node._mesh_dir(target) == racquetball_mesh


@pytest.mark.parametrize("verifier_mode", ["low_confidence", "invalid_index", "error"])
def test_blue_racquetball_alias_never_uses_highest_score_fallback(
    tmp_path,
    monkeypatch,
    verifier_mode,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    rgb_path = tmp_path / "rgb.png"
    output_dir = tmp_path / "foundationpose"
    _write_blue_racquetball_alias(selected_json)
    _write_sam2_response(
        sam2_json,
        [
            _annotation(
                "blue ball",
                _mask_at(0, 0),
                bbox=[0, 0, 2, 2],
                grounding_score=0.90,
            ),
            _annotation(
                "blue ball",
                _mask_at(1, 2),
                bbox=[2, 1, 4, 3],
                grounding_score=0.80,
            ),
        ],
    )
    _write_rgb(rgb_path)
    monkeypatch.delenv(MASK_INDEX_ENV, raising=False)
    monkeypatch.delenv(AUTO_MASK_VERIFY_ENV, raising=False)

    def fake_verifier(_target, _candidates, _card_path):
        if verifier_mode == "error":
            raise RuntimeError("verifier unavailable")
        if verifier_mode == "invalid_index":
            return {
                "selected_index": 99,
                "confidence": "high",
            }
        return {
            "selected_index": None,
            "confidence": "low",
        }

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        rgb_path=rgb_path,
        output_dir=output_dir,
        mask_candidate_verifier=fake_verifier,
    )

    with pytest.raises(RuntimeError, match="Strict alias mask selection failed"):
        node._load_mask(node._load_target())

    assert not (output_dir / "selected_mask_metadata.json").exists()
    verification = json.loads(
        (output_dir / "mask_verification_result.json").read_text(encoding="utf-8")
    )
    assert verification["decision"] in {"uncertain", "error"}


def test_blue_racquetball_alias_mask_failure_removes_stale_pose_before_service(
    tmp_path,
):
    selected_json = tmp_path / "selected.json"
    sam2_json = tmp_path / "sam2.json"
    output_dir = tmp_path / "foundationpose"
    pose_result = output_dir / "pose_result.json"
    output_dir.mkdir()
    pose_result.write_text('{"success": true}', encoding="utf-8")
    _write_blue_racquetball_alias(selected_json)
    _write_sam2_response(
        sam2_json,
        [_annotation("tennis ball", _mask_at(0, 0))],
    )

    node = FoundationPoseEstimationNode(
        selected_json=selected_json,
        sam2_response_json=sam2_json,
        output_dir=output_dir,
    )

    with pytest.raises(RuntimeError, match="did not detect the target object"):
        node.run()

    assert not pose_result.exists()
