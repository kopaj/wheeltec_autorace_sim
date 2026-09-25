#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np


# ============================================================
# Paths
# ============================================================

REPO_ROOT = Path(__file__).resolve().parents[1]

TEXTURE_DIR = (
    REPO_ROOT
    / "wheeltec_autorace_description"
    / "models"
    / "racetrack"
    / "materials"
    / "textures"
)

INPUT_PATH = TEXTURE_DIR / "course_original.png"
OUTPUT_PATH = TEXTURE_DIR / "course.png"


# ============================================================
# Track colors
# ============================================================

# Very dark purple background.
#
# OpenCV uses BGR order, not RGB.
#
# RGB: (24, 0, 32)
# HEX: #180020
BACKGROUND_BGR = np.array(
    [32, 0, 24],
    dtype=np.uint8
)

LANE_BGR = np.array(
    [255, 255, 255],
    dtype=np.uint8
)


def main():

    # --------------------------------------------------------
    # Load source image
    # --------------------------------------------------------

    image = cv2.imread(str(INPUT_PATH))

    if image is None:
        raise RuntimeError(
            f"Could not load input image: {INPUT_PATH}"
        )

    print(
        f"Input image: "
        f"{image.shape[1]} x {image.shape[0]} px"
    )


    # --------------------------------------------------------
    # Convert source image to HSV
    # --------------------------------------------------------

    hsv = cv2.cvtColor(
        image,
        cv2.COLOR_BGR2HSV
    )


    # --------------------------------------------------------
    # Detect original WHITE lane
    # --------------------------------------------------------
    #
    # White pixels:
    #   low saturation
    #   high brightness
    #

    white_lower = np.array(
        [0, 0, 175],
        dtype=np.uint8
    )

    white_upper = np.array(
        [179, 80, 255],
        dtype=np.uint8
    )

    white_mask = cv2.inRange(
        hsv,
        white_lower,
        white_upper
    )


    # --------------------------------------------------------
    # Detect original YELLOW lane
    # --------------------------------------------------------
    #
    # OpenCV HSV hue range is 0..179.
    #

    yellow_lower = np.array(
        [12, 80, 110],
        dtype=np.uint8
    )

    yellow_upper = np.array(
        [42, 255, 255],
        dtype=np.uint8
    )

    yellow_mask = cv2.inRange(
        hsv,
        yellow_lower,
        yellow_upper
    )


    # --------------------------------------------------------
    # Combine both lane masks
    # --------------------------------------------------------

    lane_mask = cv2.bitwise_or(
        white_mask,
        yellow_mask
    )


    # --------------------------------------------------------
    # Remove small gaps / anti-aliasing artifacts
    # --------------------------------------------------------

    kernel = np.ones(
        (3, 3),
        dtype=np.uint8
    )

    lane_mask = cv2.morphologyEx(
        lane_mask,
        cv2.MORPH_CLOSE,
        kernel,
        iterations=1
    )


    # --------------------------------------------------------
    # Create output image
    # --------------------------------------------------------

    output = np.empty_like(image)

    # Everything starts as dark purple.
    output[:, :] = BACKGROUND_BGR

    # Both detected lanes become pure white.
    output[lane_mask > 0] = LANE_BGR


    # --------------------------------------------------------
    # Statistics
    # --------------------------------------------------------

    lane_pixels = np.count_nonzero(
        lane_mask
    )

    total_pixels = lane_mask.size

    lane_ratio = (
        lane_pixels / total_pixels
    ) * 100.0

    print(
        f"Detected lane pixels: "
        f"{lane_pixels}"
    )

    print(
        f"Lane coverage: "
        f"{lane_ratio:.2f}%"
    )


    # --------------------------------------------------------
    # Write generated track
    # --------------------------------------------------------

    success = cv2.imwrite(
        str(OUTPUT_PATH),
        output
    )

    if not success:
        raise RuntimeError(
            f"Could not save output image: {OUTPUT_PATH}"
        )

    print(
        f"Generated texture: {OUTPUT_PATH}"
    )

    print(
        "Background RGB: (24, 0, 32) / #180020"
    )

    print(
        "Lane RGB: (255, 255, 255)"
    )


if __name__ == "__main__":
    main()