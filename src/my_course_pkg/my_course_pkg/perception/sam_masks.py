"""SAM2 mask decoding primitives."""

import numpy as np
from pycocotools import mask as mask_utils


def decode_annotation_mask(
    annotation: dict,
    *,
    decoder=mask_utils,
) -> np.ndarray:
    """Decode the SAM2 JSON RLE representation into a boolean image mask."""
    rle = {
        "counts": annotation["segmentation"]["counts"].encode(),
        "size": annotation["segmentation"]["size"],
    }
    return decoder.decode(rle).astype(bool)
