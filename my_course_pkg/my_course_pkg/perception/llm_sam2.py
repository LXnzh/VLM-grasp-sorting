import base64
import json
import os
import re
import time
import requests
from pathlib import Path

from my_course_pkg.paths import RGB_PATH, VLM_SAM2_OUTPUT_DIR
from my_course_pkg.tasks.sorting.vlm_classifier import classify_food

from my_course_pkg.perception.http import (
    RETRYABLE_STATUS_CODES as _DEFAULT_RETRYABLE_STATUS_CODES,
    post_with_retries,
    short_response_text,
)

try:
    from openai import APIStatusError, BadRequestError, OpenAI
except ModuleNotFoundError:
    class BadRequestError(Exception):
        pass

    class APIStatusError(Exception):
        pass

    OpenAI = None


YCB_OBJECTS = [
    "tomato soup can",
    "tuna fish can",
    "pudding box",
    "gelatin box",
    "banana",
    "apple",
    "lemon",
    "peach",
    "pear",
    "orange",
    "plum",
    "sponge",
    "hammer",
    "baseball",
    "tennis ball",
    "racquetball",
    "foam brick",
    "rubiks cube",
]

# Chinese aliases are deliberately kept beside the permitted YCB labels so a
# direct user instruction such as “抓取苹果” has the same authority as
# “pick up the apple”.  The VLM may still infer a visual position, but it must
# not replace an explicitly named object with a visually similar one.
CHINESE_OBJECT_ALIASES = {
    "banana": ("香蕉",),
    "apple": ("苹果",),
    "lemon": ("柠檬",),
    "peach": ("桃子",),
    "pear": ("梨",),
    "orange": ("橙子", "橘子"),
    "plum": ("李子",),
    "tomato soup can": ("番茄汤罐", "番茄罐头"),
    "tuna fish can": ("金枪鱼罐", "金枪鱼罐头"),
    "pudding box": ("布丁盒",),
    "gelatin box": ("果冻盒",),
}
RGB_IMAGE_PATH = RGB_PATH
VLM_BASE_URL = os.environ.get(
    "VLM_BASE_URL",
    "https://ki-toolbox.scc.kit.edu/api/v1",
)
VLM_MODEL = os.environ.get("VLM_MODEL", "azure.gpt-5.4")
RETRYABLE_STATUS_CODES = set(_DEFAULT_RETRYABLE_STATUS_CODES)


def _get_vlm_client() -> OpenAI:
    if OpenAI is None:
        raise RuntimeError(
            "Missing Python package 'openai'. Install it in the same environment "
            "used by ros2, for example: python3 -m pip install openai"
        )
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError(
            "Missing VLM API key. Set VLM_API_KEY or OPENAI_API_KEY before running "
            "the LLM/SAM2 pipeline."
        )
    return OpenAI(api_key=api_key, base_url=VLM_BASE_URL)


def _short_response_text(response) -> str:
    """Compatibility wrapper for remote-service error diagnostics."""
    return short_response_text(response)


def _post_with_retries(url: str, *, attempts: int = 3, **kwargs) -> requests.Response:
    """Compatibility wrapper retaining SAM2's retry contract."""
    return post_with_retries(
        url,
        attempts=attempts,
        response_text=_short_response_text,
        retryable_status_codes=RETRYABLE_STATUS_CODES,
        **kwargs,
    )


def _is_image_unsupported_error(exc: BadRequestError) -> bool:
    message = str(exc).lower()
    body = getattr(exc, "body", None)
    if body is not None:
        message += f" {body}".lower()

    unsupported_markers = (
        "image_url",
        "image input",
        "vision",
        "multimodal",
        "does not support image",
        "unsupported content type",
        "invalid content type",
    )
    return any(marker in message for marker in unsupported_markers)


def _parse_json_object(content: str) -> dict:
    try:
        return json.loads(content)
    except json.JSONDecodeError:
        start = content.find("{")
        end = content.rfind("}")
        if start == -1 or end == -1 or end <= start:
            raise
        return json.loads(content[start:end + 1])


def save_base64_image(image_base64: str, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(base64.b64decode(image_base64))


def _image_data_url(img_path: str) -> str:
    with open(img_path, "rb") as handle:
        return (
            "data:image/png;base64,"
            + base64.b64encode(handle.read()).decode("ascii")
        )


def _run_vlm_json(img_path: str, prompt: str, output_label: str) -> dict:
    image_data_url = _image_data_url(img_path)
    attempts = 3
    vlm_client = _get_vlm_client()
    for attempt in range(1, attempts + 1):
        try:
            response = vlm_client.chat.completions.create(
                model=VLM_MODEL,
                messages=[
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": prompt},
                            {"type": "image_url", "image_url": {"url": image_data_url}},
                        ],
                    }
                ],
                temperature=0,
            )
            break
        except BadRequestError as exc:
            if _is_image_unsupported_error(exc):
                raise RuntimeError(
                    f"The configured VLM model/API does not support image input: "
                    f"base_url={VLM_BASE_URL}, model={VLM_MODEL}. "
                    "This pipeline needs a vision-capable OpenAI-compatible API. "
                    "Set VLM_BASE_URL and VLM_MODEL to a model that accepts image_url."
                ) from exc
            raise
        except APIStatusError as exc:
            if exc.status_code not in RETRYABLE_STATUS_CODES or attempt == attempts:
                raise RuntimeError(
                    "VLM service request failed. "
                    f"HTTP {exc.status_code}: {_short_response_text(exc.response)}"
                ) from exc
            wait_sec = 2 ** (attempt - 1)
            print(
                f"VLM service returned HTTP {exc.status_code}; "
                f"retrying in {wait_sec}s..."
            )
            time.sleep(wait_sec)

    content = response.choices[0].message.content
    print(f"Raw VLM {output_label} output:")
    print(content)
    return _parse_json_object(content)


def vlm_select_candidates(img_path: str, user_instruction: str):
    """Deprecated compatibility wrapper returning target candidates."""
    return vlm_select_target(img_path, user_instruction)["candidates"]


def _instruction_region_axes(user_instruction: str) -> set[str]:
    """Return image axes that the user explicitly constrained."""
    instruction = str(user_instruction or "").strip().lower()
    horizontal_patterns = (
        r"\bleft(?:most)?\b",
        r"\bright(?:most)?\b",
        r"\bcent(?:er|re)(?:most)?\b",
        r"\bmiddle\s+(?:one|object|item|apple|fruit|can|box)\b",
        r"左",
        r"右",
        r"中间(?:的)?(?:一个|物体|物品|苹果|水果|罐|盒)",
    )
    vertical_patterns = (
        r"\btop(?:most)?\b",
        r"\bbottom(?:most)?\b",
        r"\bupper\b",
        r"\blower\b",
        r"\bmiddle\s+(?:row|level|height)\b",
        r"上(?:面|方|边|部)?",
        r"下(?:面|方|边|部)?",
        r"中间(?:一)?(?:行|层|高度)",
    )
    axes = set()
    if any(re.search(pattern, instruction) for pattern in horizontal_patterns):
        axes.add("horizontal")
    if any(re.search(pattern, instruction) for pattern in vertical_patterns):
        axes.add("vertical")
    return axes


def _normalize_target_region(region, user_instruction: str = ""):
    if not isinstance(region, dict):
        region = {}
    horizontal = str(region.get("horizontal", "unknown")).strip().lower()
    vertical = str(region.get("vertical", "unknown")).strip().lower()
    if horizontal not in {"left", "center", "right", "unknown"}:
        horizontal = "unknown"
    if vertical not in {"top", "middle", "bottom", "unknown"}:
        vertical = "unknown"
    source = (
        str(region.get("source", "inferred_from_image")).strip().lower()
        or "inferred_from_image"
    )
    if source not in {"user_instruction", "inferred_from_image", "unknown"}:
        source = "inferred_from_image"

    # A single source value describes the whole region.  When the user names
    # only one axis (for example, "the left"), do not turn the VLM's inferred
    # value on the other axis into a strict user constraint.
    if source == "user_instruction":
        explicit_axes = _instruction_region_axes(user_instruction)
        if "horizontal" not in explicit_axes:
            horizontal = "unknown"
        if "vertical" not in explicit_axes:
            vertical = "unknown"

    return {
        "horizontal": horizontal,
        "vertical": vertical,
        "description": str(region.get("description", "")).strip(),
        "source": source,
    }


def _normalize_visual_attributes(attributes):
    if not isinstance(attributes, dict):
        attributes = {}
    return {
        "color": str(attributes.get("color", "unknown")).strip() or "unknown",
        "shape": str(attributes.get("shape", "unknown")).strip() or "unknown",
        "container": str(attributes.get("container", "unknown")).strip() or "unknown",
    }


def _valid_ycb_object(value: str) -> str | None:
    """Return the canonical YCB name for an explicit override, if valid."""
    normalized = " ".join(str(value).strip().lower().split())
    if normalized in YCB_OBJECTS:
        return normalized
    return None


def _explicit_object_from_instruction(user_instruction: str) -> str | None:
    """Honor an unambiguous object name explicitly spoken by the user."""
    instruction = str(user_instruction or "").lower()
    matches = []
    for object_name in YCB_OBJECTS:
        english_pattern = r"\b" + re.escape(object_name) + r"s?\b"
        chinese_aliases = CHINESE_OBJECT_ALIASES.get(object_name, ())
        if (
            re.search(english_pattern, instruction)
            or any(alias in instruction for alias in chinese_aliases)
        ):
            matches.append(object_name)
    return matches[0] if len(matches) == 1 else None


def _forced_target_object(user_instruction: str) -> str | None:
    """Read the GUI override first, then a single explicit spoken object."""
    override = os.environ.get("VLM_CANDIDATE_OVERRIDE", "").strip()
    if override:
        target = _valid_ycb_object(override)
        if target is None:
            raise ValueError(
                "VLM_CANDIDATE_OVERRIDE must be one of the supported YCB "
                f"objects, got {override!r}."
            )
        return target
    return _explicit_object_from_instruction(user_instruction)


def vlm_select_target(img_path: str, user_instruction: str):
    allowed_objects = ", ".join(YCB_OBJECTS)
    forced_target = _forced_target_object(user_instruction)

    prompt = f"""
You are a robot perception assistant.

You will receive:
1. a camera image
2. a user instruction
3. a list of allowed YCB objects

Your task is to select the target object or possible target objects from the allowed list.
You must also infer where the chosen target is in the image, even if the user
does not explicitly mention a position.

Allowed YCB objects:
{allowed_objects}

User instruction:
{user_instruction}

Rules:
1. Only choose object names from the allowed YCB object list.
2. Use both the image and the user instruction. If the instruction explicitly
   names an allowed object (for example "apple" or "苹果"), that exact object
   is the target; do not substitute a visually similar object such as peach.
3. If the instruction clearly refers to one object, return exactly one candidate.
4. If the instruction is ambiguous, return multiple reasonable candidates.
5. Select selected_object_name from candidates[0] unless there is a clear best match.
6. target_region.horizontal must be one of: left, center, right, unknown.
7. target_region.vertical must be one of: top, middle, bottom, unknown.
8. Use target_region.source="user_instruction" only when the user says a
   position; otherwise use "inferred_from_image". When it is
   "user_instruction", set an axis to "unknown" unless the user explicitly
   stated that axis. For example, "the left" means horizontal="left" and
   vertical="unknown".
9. visual_attributes.container should be "on_table", "in_bin", or "unknown".
10. Do not explain your reasoning.
11. Output only valid JSON.
12. JSON format:
{{
  "candidates": ["object name"],
  "selected_object_name": "object name",
  "target_region": {{
    "horizontal": "left|center|right|unknown",
    "vertical": "top|middle|bottom|unknown",
    "description": "short phrase",
    "source": "user_instruction|inferred_from_image|unknown"
  }},
  "visual_attributes": {{
    "color": "short color phrase",
    "shape": "short shape phrase",
    "container": "on_table|in_bin|unknown"
  }}
}}
"""

    result = _run_vlm_json(img_path, prompt, "target-selection")
    if forced_target is not None:
        candidates = [forced_target]
        selected_object_name = forced_target
    else:
        candidates = result["candidates"]
        candidates = [x for x in candidates if x in YCB_OBJECTS]
        if not candidates:
            raise ValueError(f"No valid YCB candidates returned: {result}")
        selected_object_name = result.get("selected_object_name")
        if selected_object_name not in candidates:
            selected_object_name = candidates[0]

    return {
        "candidates": candidates,
        "selected_object_name": selected_object_name,
        "target_region": _normalize_target_region(
            result.get("target_region"), user_instruction
        ),
        "visual_attributes": _normalize_visual_attributes(
            result.get("visual_attributes")
        ),
        "raw_vlm_selection": result,
        "target_override": forced_target,
    }


def vlm_classify_food(img_path: str, target_object: str) -> dict:
    """Classify a visible target without consulting an object/category lookup table."""
    return classify_food(img_path, target_object, _run_vlm_json)


def build_sam2_text_prompt(candidates):
    return ". ".join(candidates) + "."


def run_sam2_api(
    img_path: str,
    text_prompt: str,
    output_dir: Path | str = VLM_SAM2_OUTPUT_DIR,
):
    sam2_server_url = "http://172.22.222.226:5000/predict"

    image_path = Path(img_path)
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    with image_path.open("rb") as image_file:
        response = _post_with_retries(
            sam2_server_url,
            data={
                "text_prompt": text_prompt,
                "box_threshold": "0.35",
                "text_threshold": "0.25",
                "return_images": "true",
            },
            files={
                "image": (
                    image_path.name,
                    image_file,
                    "application/octet-stream",
                )
            },
            timeout=300,
        )

    payload = response.json()

    (output_dir / "response.json").write_text(
        json.dumps(payload, indent=2),
        encoding="utf-8",
    )

    images = payload.get("images", {})

    if "groundingdino_annotated_image" in images:
        save_base64_image(
            images["groundingdino_annotated_image"],
            output_dir / "groundingdino_annotated_image.jpg",
        )

    if "grounded_sam2_annotated_image_with_mask" in images:
        save_base64_image(
            images["grounded_sam2_annotated_image_with_mask"],
            output_dir / "grounded_sam2_annotated_image_with_mask.jpg",
        )

    return payload


class LlmSam2Node:
    """Pipeline wrapper for VLM object selection + SAM2 segmentation."""

    def __init__(self, img_path: Path | str = RGB_IMAGE_PATH):
        self.img_path = str(img_path)
        self.output_dir = VLM_SAM2_OUTPUT_DIR

    def run(self, user_instruction: str) -> dict:
        selection = vlm_select_target(self.img_path, user_instruction)
        candidates = selection["candidates"]
        selected_object_name = selection["selected_object_name"]
        text_prompt = build_sam2_text_prompt([selected_object_name])

        print("User instruction:", user_instruction)
        print("VLM candidates:", candidates)
        print("VLM selected object:", selected_object_name)
        print("VLM inferred target region:", selection["target_region"])
        print("VLM visual attributes:", selection["visual_attributes"])
        print("SAM2 target prompt:", text_prompt)

        payload = run_sam2_api(self.img_path, text_prompt)
        classification = vlm_classify_food(
            self.img_path,
            selected_object_name,
        )
        print(
            "VLM food classification: "
            f"{classification['category']} "
            f"(confidence={classification['confidence']:.2f})"
        )

        self.output_dir.mkdir(parents=True, exist_ok=True)
        (self.output_dir / "selected_object.json").write_text(
            json.dumps(
                {
                    "user_instruction": user_instruction,
                    "candidates": candidates,
                    "selected_object_name": selected_object_name,
                    "target_region": selection["target_region"],
                    "visual_attributes": selection["visual_attributes"],
                    "sam2_text_prompt": text_prompt,
                    "target_category": classification["category"],
                    "classification_confidence": classification["confidence"],
                    "classification_reason": classification["reason"],
                },
                indent=2,
            ),
            encoding="utf-8",
        )

        return {
            "candidates": candidates,
            "selected_object_name": selected_object_name,
            "target_region": selection["target_region"],
            "visual_attributes": selection["visual_attributes"],
            "sam2_text_prompt": text_prompt,
            "sam2_payload": payload,
            "classification": classification,
        }


if __name__ == "__main__":
    user_instruction = input("Please enter the robot instruction: ")
    LlmSam2Node().run(user_instruction)
