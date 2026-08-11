"""Camera calibration helpers shared by perception pipelines."""

import math

import numpy as np


DEFAULT_VERTICAL_FOV_DEG = 51.38


def camera_matrix(width: int, height: int) -> np.ndarray:
    """
    Return the RGB-D camera intrinsic matrix for a frame size.

    The camera model intentionally remains the same as the original
    FoundationPose wrapper: square pixels, an image-centred principal point,
    and a 51.38 degree vertical field of view.
    """
    focal = (float(height) / 2.0) / math.tan(
        math.radians(DEFAULT_VERTICAL_FOV_DEG) / 2.0
    )
    return np.array(
        [
            [focal, 0.0, float(width) / 2.0],
            [0.0, focal, float(height) / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
