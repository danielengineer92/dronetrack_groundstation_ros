"""Pixel-to-bearing camera geometry for the tracker.

Kept free of rclpy so it unit-tests with plain Python (see
test/test_camera_geometry.py). Two mutually exclusive modes:

- **legacy** (``calibrated=False``): the historical distortion-free pinhole
  with the principal point pinned to the image center —
  ``bearing = atan((pixel - dim/2) / f)``. Pure python, no cv2, and
  numerically identical to the pre-calibration tracker, which keeps the
  Gazebo sim (a true pinhole camera) byte-for-byte unchanged.
- **calibrated** (``calibrated=True``): measured intrinsics fx/fy/cx/cy and
  optional distortion coefficients. Distorted pixels are undistorted with
  OpenCV (one point per detection — microseconds) and the bearing comes from
  the normalized coordinates: ``bearing = atan(x_normalized)``.

cv2/numpy are imported lazily and only on the calibrated+distorted path, so
uncalibrated deployments and the test suite never require them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

VALID_MODELS = ("plumb_bob", "rational", "fisheye")

# Expected dist_coeffs lengths per model (OpenCV conventions). plumb_bob may
# also be given with 4 coefficients (k1,k2,p1,p2) which OpenCV accepts.
_MODEL_COEFF_LENGTHS = {
    "plumb_bob": (4, 5),
    "rational": (8,),
    "fisheye": (4,),
}


@dataclass(frozen=True)
class CameraGeometry:
    image_width: int
    image_height: int
    fx: float
    fy: float
    cx: float
    cy: float
    dist_coeffs: Tuple[float, ...]
    model: str
    calibrated: bool
    # Lazily-built cv2 arrays (numpy K 3x3 and D vector), cached after the
    # first distorted-path call. `field(default=...)` keeps the dataclass
    # frozen while the list contents stay mutable for caching.
    _cv_cache: list = field(default_factory=list, repr=False, compare=False)


def build_geometry(
    *,
    image_width: int,
    image_height: int,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    dist_coeffs: Sequence[float],
    distortion_model: str,
    legacy_fx: float,
    legacy_fy: float,
) -> CameraGeometry:
    """Build the geometry once at node startup.

    ``fx<=0 or fy<=0`` selects the legacy path carrying the FOV-derived
    focal lengths (today's behavior, sim-safe). ``cx/cy<=0`` default to the
    image center. Unknown model strings raise ValueError; a coefficient
    count that does not match the model raises ValueError (bad paste is a
    config error we want loud at startup, not silently wrong bearings).
    """
    image_width = int(image_width)
    image_height = int(image_height)

    if fx <= 0.0 or fy <= 0.0:
        return CameraGeometry(
            image_width=image_width,
            image_height=image_height,
            fx=float(legacy_fx),
            fy=float(legacy_fy),
            cx=image_width / 2.0,
            cy=image_height / 2.0,
            dist_coeffs=(),
            model="plumb_bob",
            calibrated=False,
        )

    model = str(distortion_model).strip().lower()
    if model not in VALID_MODELS:
        raise ValueError(
            f"distortion_model '{distortion_model}' not one of {VALID_MODELS}"
        )

    coeffs = tuple(float(c) for c in (dist_coeffs or ()))
    if coeffs:
        allowed = _MODEL_COEFF_LENGTHS[model]
        if len(coeffs) not in allowed:
            raise ValueError(
                f"dist_coeffs has {len(coeffs)} values but model '{model}' "
                f"expects {allowed} — model/coefficients mismatch (bad paste?)"
            )
        if not all(math.isfinite(c) for c in coeffs):
            raise ValueError("dist_coeffs contains non-finite values")

    if not all(math.isfinite(v) for v in (fx, fy)):
        raise ValueError("camera_fx/camera_fy must be finite")

    return CameraGeometry(
        image_width=image_width,
        image_height=image_height,
        fx=float(fx),
        fy=float(fy),
        cx=float(cx) if cx > 0.0 else image_width / 2.0,
        cy=float(cy) if cy > 0.0 else image_height / 2.0,
        dist_coeffs=coeffs,
        model=model,
        calibrated=True,
    )


def describe(geom: CameraGeometry) -> str:
    """One-line summary for the tracker's startup log."""
    if not geom.calibrated:
        return (
            f"legacy FOV pinhole (fx={geom.fx:.1f}, fy={geom.fy:.1f}, "
            f"principal=image center)"
        )
    dist = f"{len(geom.dist_coeffs)} coeffs" if geom.dist_coeffs else "no distortion"
    return (
        f"CALIBRATED {geom.model} (fx={geom.fx:.1f}, fy={geom.fy:.1f}, "
        f"cx={geom.cx:.1f}, cy={geom.cy:.1f}, {dist})"
    )


def bearing_from_pixel(
    geom: CameraGeometry, center_x_px: float, center_y_px: float
) -> Tuple[float, float]:
    """Pixel center -> (bearing_x_rad, bearing_y_rad).

    Legacy and undistorted-calibrated paths are pure python. The distorted
    path undistorts the single point with OpenCV and converts the returned
    ideal normalized coordinates to bearings.
    """
    if not geom.dist_coeffs:
        # Pinhole (legacy or calibrated-without-distortion): closed form.
        bx = math.atan((float(center_x_px) - geom.cx) / max(geom.fx, 1e-6))
        by = math.atan((float(center_y_px) - geom.cy) / max(geom.fy, 1e-6))
        return bx, by

    xn, yn = _undistort_normalized(geom, float(center_x_px), float(center_y_px))
    return math.atan(xn), math.atan(yn)


def _undistort_normalized(
    geom: CameraGeometry, u: float, v: float
) -> Tuple[float, float]:
    import cv2  # lazy: only calibrated+distorted deployments need it
    import numpy as np

    if not geom._cv_cache:
        k = np.array(
            [[geom.fx, 0.0, geom.cx], [0.0, geom.fy, geom.cy], [0.0, 0.0, 1.0]],
            dtype=np.float64,
        )
        d = np.array(geom.dist_coeffs, dtype=np.float64)
        geom._cv_cache.append((k, d))
    k, d = geom._cv_cache[0]

    pts = _np_point(u, v)
    if geom.model == "fisheye":
        import cv2  # noqa: F811 (already imported; keeps branch self-contained)

        out = cv2.fisheye.undistortPoints(pts, k, d)
    else:
        out = cv2.undistortPoints(pts, k, d)
    xn = float(out[0, 0, 0])
    yn = float(out[0, 0, 1])
    return xn, yn


def _np_point(u: float, v: float):
    import numpy as np

    return np.array([[[u, v]]], dtype=np.float64)
