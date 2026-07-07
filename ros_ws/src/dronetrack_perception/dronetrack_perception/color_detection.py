"""Pure red-blob detection (HSV threshold + circularity gate).

rclpy-free so it unit-tests with plain Python + cv2 (see
test/test_color_detection.py). The node (color_detection_node.py) owns ROS
I/O and message construction; this module owns the vision.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import List, Tuple

import cv2
import numpy as np


@dataclass
class RedDetectParams:
    # Two HSV bands because red wraps hue 0/180 in OpenCV.
    hue_lo_1: int = 0
    hue_hi_1: int = 10
    hue_lo_2: int = 170
    hue_hi_2: int = 180
    sat_min: int = 90
    val_min: int = 60
    min_area_px: float = 12.0
    min_circularity: float = 0.65
    min_diameter_px: float = 4.0
    morph_kernel: int = 3


@dataclass
class RedBlob:
    circularity: float
    cx: float
    cy: float
    diameter: float


def red_mask(frame_bgr, p: RedDetectParams):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    lo1 = (p.hue_lo_1, p.sat_min, p.val_min)
    hi1 = (p.hue_hi_1, 255, 255)
    lo2 = (p.hue_lo_2, p.sat_min, p.val_min)
    hi2 = (p.hue_hi_2, 255, 255)
    mask = cv2.inRange(hsv, lo1, hi1) | cv2.inRange(hsv, lo2, hi2)
    if p.morph_kernel > 0:
        mask = cv2.morphologyEx(
            mask, cv2.MORPH_OPEN, np.ones((p.morph_kernel, p.morph_kernel), np.uint8))
    return mask


def find_red_circles(frame_bgr, p: RedDetectParams) -> List[RedBlob]:
    """Return round red blobs, most-circular first.

    Circularity = contour area / enclosing-circle area (→1.0 for a disc,
    much lower for irregular red clutter). This is the false-positive gate
    that separates the ball from arbitrary red objects in frame.
    """
    mask = red_mask(frame_bgr, p)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    blobs: List[RedBlob] = []
    for c in contours:
        area = cv2.contourArea(c)
        if area < p.min_area_px:
            continue
        (cx, cy), radius = cv2.minEnclosingCircle(c)
        diameter = 2.0 * radius
        if diameter < p.min_diameter_px:
            continue
        circularity = float(area / (np.pi * radius * radius + 1e-6))
        if circularity < p.min_circularity:
            continue
        blobs.append(RedBlob(circularity, float(cx), float(cy), float(diameter)))
    blobs.sort(key=lambda b: b.circularity, reverse=True)
    return blobs
