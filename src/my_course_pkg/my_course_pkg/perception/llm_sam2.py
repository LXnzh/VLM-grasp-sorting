import base64
import json
import os
import time
import requests
from pathlib import Path
from openai import APIStatusError, BadRequestError, OpenAI

from my_course_pkg.paths import RGB_PATH, VLM_SAM2_OUTPUT_DIR
from my_course_pkg.perception.target_aliases import resolve_target_alias


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
#export VLM_CANDIDATE_OVERRIDE=baseball
# source /home/ws/install/setup.bash
# ros2 run my_course_pkg pipeline
#unset VLM_CANDIDATE_OVERRIDE

RGB_IMAGE_PATH = RGB_PATH
VLM_BASE_URL = os.environ.get(
    "VLM_BASE_URL",
    "https://ki-toolbox.scc.kit.edu/api/v1",
)
VLM_MODEL = os.environ.get("VLM_MODEL", "azure.gpt-5-mini")
RETRYABLE_STATUS_CODES = {429, 500, 502, 503, 504}


def _get_vlm_client() -> OpenAI:
    api_key = os.environ.get("VLM_API_KEY") or os.environ.get("OPENAI_API_KEY")
    if not api_key:
        raise RuntimeError(
            "Missing VLM API key. Set VLM_API_KEY or OPENAI_API_KEY before running "
            "the LLM/SAM2 pipeline."
        )
    return OpenAI(api_key=api_key, base_url=VLM_BASE_URL)


def _short_response_text(response) -> str:
    if response is None:
        return ""

    text = getattr(response, "text", None)
    if text is None:
        try:
            text = response.read().decode("utf-8", errors="replace")
        except Exception:
            text = str(response)

    return " ".join(str(text).split())[:300]


def _post_with_retries(url: str, *, attempts: int = 3, **kwargs) -> requests.Response:
    last_error = None
    for attempt in range(1, attempts + 1):
        try:
            response = requests.post(url, **kwargs)
            if response.status_code not in RETRYABLE_STATUS_CODES:
                response.raise_for_status()
                return response
            last_error = RuntimeError(
                f"HTTP {response.status_code}: {_short_response_text(response)}"
            )
        except requests.RequestException as exc:
            last_error = exc

        if attempt < attempts:
            wait_sec = 2 ** (attempt - 1)
            print(f"Request to {url} failed ({last_error}); retrying in {wait_sec}s...")
            time.sleep(wait_sec)

    raise RuntimeError(
        f"Request to {url} failed after {attempts} attempts. Last error: {last_error}"
    ) from last_error


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
        return json.loads(content[start : end + 1])


def save_base64_image(image_base64: str, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_bytes(base64.b64decode(image_base64))


def vlm_select_candidates(img_path: str, user_instruction: str):
    override = os.environ.get("VLM_CANDIDATE_OVERRIDE")
    if override:
        candidates = [
            candidate.strip().lower().replace("_", " ")
            for candidate in override.split(",")
            if candidate.strip()
        ]
        candidates = [candidate for candidate in candidates if candidate in YCB_OBJECTS]
        if not candidates:
            raise ValueError(
                f"VLM_CANDIDATE_OVERRIDE did not contain valid YCB objects: {override}"
            )
        return candidates

    with open(img_path, "rb") as handle:
        image_data_url = (
            "data:image/png;base64,"
            + base64.b64encode(handle.read()).decode("ascii")
        )

    allowed_objects = ", ".join(YCB_OBJECTS)

    prompt = f"""
You are a robot perception assistant.

You will receive:
1. a camera image
2. a user instruction
3. a list of allowed YCB objects

Your task is to select the target object or possible target objects from the allowed list.

Allowed YCB objects:
{allowed_objects}

User instruction:
{user_instruction}

Rules:
1. Only choose object names from the allowed YCB object list.
2. Use both the image and the user instruction.
3. If the instruction clearly refers to one object, return exactly one candidate.
4. If the instruction is ambiguous, return multiple reasonable candidates.
5. Do not describe the image.
6. Do not explain your reasoning.
7. Output only valid JSON.
8. JSON format:
{{"candidates": ["object name"]}}
"""

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
    print("Raw VLM output:")
    print(content)

    result = _parse_json_object(content)
    candidates = result["candidates"]

    candidates = [x for x in candidates if x in YCB_OBJECTS]

    if not candidates:
        raise ValueError(f"No valid YCB candidates returned: {content}")

    return candidates


def build_sam2_text_prompt(candidates):
    return ". ".join(candidates) + "."


def run_sam2_api(img_path: str, text_prompt: str):
    sam2_server_url = "http://172.22.222.226:5000/predict"

    image_path = Path(img_path)
    output_dir = VLM_SAM2_OUTPUT_DIR
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
        alias = None
        if not os.environ.get("VLM_CANDIDATE_OVERRIDE"):
            alias = resolve_target_alias(user_instruction)

        if alias is None:
            candidates = vlm_select_candidates(self.img_path, user_instruction)
            grounding_prompts = candidates
        else:
            candidates = [alias.selected_object_name]
            grounding_prompts = [alias.grounding_prompt]

        text_prompt = build_sam2_text_prompt(grounding_prompts)
        selected_object_name = candidates[0]

        print("User instruction:", user_instruction)
        print("VLM candidates:", candidates)

        payload = run_sam2_api(self.img_path, text_prompt)

        self.output_dir.mkdir(parents=True, exist_ok=True)
        selected_object_payload = {
            "user_instruction": user_instruction,
            "candidates": candidates,
            "selected_object_name": selected_object_name,
            "sam2_text_prompt": text_prompt,
        }
        alias_payload = {}
        if alias is not None:
            alias_payload = {
                "matched_instruction_alias": alias.matched_instruction_alias,
                "grounding_prompt": alias.grounding_prompt,
                "accepted_sam2_class_names": list(
                    alias.accepted_sam2_class_names
                ),
                "visual_target_description": alias.visual_target_description,
            }
            selected_object_payload.update(alias_payload)

        (self.output_dir / "selected_object.json").write_text(
            json.dumps(selected_object_payload, indent=2),
            encoding="utf-8",
        )

        result = {
            "candidates": candidates,
            "selected_object_name": selected_object_name,
            "sam2_text_prompt": text_prompt,
            "sam2_payload": payload,
        }
        result.update(alias_payload)
        return result


if __name__ == "__main__":
    user_instruction = input("Please enter the robot instruction: ")
    LlmSam2Node().run(user_instruction)
