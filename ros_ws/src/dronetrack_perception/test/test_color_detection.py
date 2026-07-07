"""Unit tests for color_detection (needs cv2; skips cleanly if absent). Run:

    python3 src/dronetrack_perception/test/test_color_detection.py
"""

import os
import sys

try:
    import cv2
    import numpy as np
    HAVE_CV2 = True
except ImportError:  # pragma: no cover
    HAVE_CV2 = False

try:
    from dronetrack_perception.color_detection import (
        RedDetectParams, RedBlob, find_red_circles,
    )
except ImportError:  # pragma: no cover - direct-run convenience
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
    from dronetrack_perception.color_detection import (  # noqa: E402
        RedDetectParams, RedBlob, find_red_circles,
    )

RED_BGR = (40, 40, 220)   # saturated red
BLUE_BGR = (220, 80, 30)


def _white(w=640, h=480):
    return np.full((h, w, 3), 255, np.uint8)


def test_finds_a_red_ball():
    if not HAVE_CV2:
        print("  (skipped: no cv2)"); return
    img = _white()
    cv2.circle(img, (325, 210), 30, RED_BGR, -1)
    blobs = find_red_circles(img, RedDetectParams())
    assert blobs, "should detect the red ball"
    b = blobs[0]
    assert abs(b.cx - 325) < 3 and abs(b.cy - 210) < 3
    assert abs(b.diameter - 60) < 6
    assert b.circularity > 0.8, f"circle should be highly circular, got {b.circularity:.2f}"


def test_rejects_non_round_red():
    if not HAVE_CV2:
        print("  (skipped: no cv2)"); return
    img = _white()
    # A long red bar: red, but not round.
    cv2.rectangle(img, (100, 230), (260, 250), RED_BGR, -1)
    blobs = find_red_circles(img, RedDetectParams())
    assert not blobs, f"elongated red bar must be rejected, got {blobs}"


def test_ball_wins_over_bigger_irregular_red():
    if not HAVE_CV2:
        print("  (skipped: no cv2)"); return
    # Mirrors the live bench frame: a big irregular red distractor plus the
    # smaller round ball. The ball must be the (only) accepted detection.
    img = _white()
    pts = np.array([[60, 150], [180, 120], [140, 260], [90, 300], [40, 200]], np.int32)
    cv2.fillPoly(img, [pts], RED_BGR)             # big blobby red thing
    cv2.circle(img, (400, 220), 24, RED_BGR, -1)  # the ball
    blobs = find_red_circles(img, RedDetectParams())
    assert blobs, "ball should be detected"
    assert abs(blobs[0].cx - 400) < 4 and abs(blobs[0].cy - 220) < 4, \
        "most-circular blob should be the ball, not the big distractor"
    for b in blobs:
        assert b.circularity >= 0.65


def test_ignores_blue():
    if not HAVE_CV2:
        print("  (skipped: no cv2)"); return
    img = _white()
    cv2.circle(img, (320, 240), 40, BLUE_BGR, -1)
    assert not find_red_circles(img, RedDetectParams()), "blue must not match a red detector"


def test_small_speck_below_min_area():
    if not HAVE_CV2:
        print("  (skipped: no cv2)"); return
    img = _white()
    cv2.circle(img, (300, 300), 1, RED_BGR, -1)  # ~area 3px
    assert not find_red_circles(img, RedDetectParams(min_area_px=12.0))


def test_confidence_is_circularity_and_sorted():
    if not HAVE_CV2:
        print("  (skipped: no cv2)"); return
    img = _white()
    cv2.circle(img, (150, 150), 35, RED_BGR, -1)   # rounder (bigger, less pixelation)
    cv2.circle(img, (480, 320), 8, RED_BGR, -1)    # smaller, more pixelated
    blobs = find_red_circles(img, RedDetectParams(min_diameter_px=4.0))
    assert len(blobs) >= 2
    assert blobs[0].circularity >= blobs[1].circularity, "must be sorted most-circular first"


def _run_all():
    failures = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print(f"PASS {name}")
            except AssertionError as exc:
                failures += 1
                print(f"FAIL {name}: {exc}")
    return failures


if __name__ == "__main__":
    sys.exit(1 if _run_all() else 0)
