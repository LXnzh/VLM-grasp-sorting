import os
from pathlib import Path


DEFAULT_OUTPUT_DIR = Path("/home/ws/src/my_course_pkg/my_course_pkg/outputs_local")
OUTPUT_DIR = Path(os.environ.get("MY_COURSE_OUTPUT_DIR", DEFAULT_OUTPUT_DIR))

RGBD_FRAME_DIR = OUTPUT_DIR / "rgbd_frame"
RGB_PATH = RGBD_FRAME_DIR / "rgb.png"
DEPTH_PATH = RGBD_FRAME_DIR / "depth.npy"

VLM_SAM2_OUTPUT_DIR = OUTPUT_DIR / "vlm_sam2_test"
SELECTED_OBJECT_JSON = VLM_SAM2_OUTPUT_DIR / "selected_object.json"
SAM2_RESPONSE_JSON = VLM_SAM2_OUTPUT_DIR / "response.json"

FOUNDATIONPOSE_OUTPUT_DIR = OUTPUT_DIR / "foundationpose"
FOUNDATIONPOSE_RESULT_JSON = FOUNDATIONPOSE_OUTPUT_DIR / "pose_result.json"
