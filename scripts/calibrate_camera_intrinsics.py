#!/usr/bin/env python3
"""Offline intrinsic calibration from captured board JPEGs.

Supports plain checkerboards AND ChArUco boards (better for wide lenses:
partial views at the frame edges still contribute corners). Fits BOTH the
standard plumb-bob (5-coeff) and rational (8-coeff) models on the same
detected corners, compares RMS reprojection error and the max residual in
the outer 15% edge band (where a 120-degree lens distorts most), prints a
data-driven RECOMMENDED MODEL, and emits a ready-to-paste YAML block for
the tracker_node section of configs/pi_hardware.yaml.

    # plain checkerboard (pattern = INNER corners):
    python3 scripts/calibrate_camera_intrinsics.py \
        --images calib_frames --pattern 9x6 --square-mm 25

    # ChArUco (pattern = SQUARES, e.g. a 10x7-square board):
    python3 scripts/calibrate_camera_intrinsics.py \
        --images calib_frames --board charuco --pattern 10x7 \
        --square-mm 25 --marker-mm 19 --aruco-dict DICT_4X4_50

No ROS required. Needs apt python3-opencv (cv2 4.x with aruco contrib).
"""

import argparse
import glob
import math
import os
import sys

import cv2
import numpy as np

EDGE_BAND = 0.15


def parse_pattern(text: str):
    try:
        cols, rows = text.lower().split("x")
        return int(cols), int(rows)
    except Exception:
        raise argparse.ArgumentTypeError(
            f"pattern '{text}' must look like 9x6 (INNER corners, cols x rows)")


def make_charuco_board(pattern, square_mm, marker_mm, dict_name):
    dict_id = getattr(cv2.aruco, dict_name, None)
    if dict_id is None:
        raise SystemExit(f"Unknown --aruco-dict {dict_name} (e.g. DICT_4X4_50)")
    adict = cv2.aruco.getPredefinedDictionary(dict_id)
    # cv2 4.6 legacy API; 4.7+ renamed to CharucoBoard(...)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(
            pattern[0], pattern[1], square_mm / 1000.0, marker_mm / 1000.0, adict)
    else:  # pragma: no cover - newer OpenCV
        board = cv2.aruco.CharucoBoard(
            (pattern[0], pattern[1]), square_mm / 1000.0, marker_mm / 1000.0, adict)
    return adict, board


def detect_charuco(paths, pattern, square_mm, marker_mm, dict_name, min_corners=8):
    adict, board = make_charuco_board(pattern, square_mm, marker_mm, dict_name)
    all_corners, all_ids, used, size = [], [], [], None
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  skip (unreadable): {path}")
            continue
        h, w = img.shape[:2]
        if size is None:
            size = (w, h)
        elif (w, h) != size:
            raise SystemExit(
                f"ERROR: mixed resolutions — {path} is {w}x{h}, first image was "
                f"{size[0]}x{size[1]}. Calibrate one resolution at a time.")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = cv2.aruco.detectMarkers(gray, adict)
        if ids is None or len(ids) < 4:
            print(f"  skip (few markers): {os.path.basename(path)}")
            continue
        n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
            corners, ids, gray, board)
        if n is None or n < min_corners:
            print(f"  skip ({n or 0} charuco corners): {os.path.basename(path)}")
            continue
        all_corners.append(ch_corners)
        all_ids.append(ch_ids)
        used.append(path)
    return board, all_corners, all_ids, used, size


def fit_charuco(name, flags, board, all_corners, all_ids, size):
    ret, K, D, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, size, None, None, flags=flags)
    # Per-image + edge-band residuals from the board's known corner positions.
    board_pts = board.chessboardCorners.reshape(-1, 3)
    per_image, edge_residuals = [], []
    w, h = size
    for i, (ch_corners, ch_ids) in enumerate(zip(all_corners, all_ids)):
        obj = board_pts[ch_ids.reshape(-1)]
        proj, _ = cv2.projectPoints(obj.reshape(-1, 1, 3), rvecs[i], tvecs[i], K, D)
        err = np.linalg.norm(
            proj.reshape(-1, 2) - ch_corners.reshape(-1, 2), axis=1)
        per_image.append(float(np.sqrt(np.mean(err ** 2))))
        pts = ch_corners.reshape(-1, 2)
        in_edge = ((pts[:, 0] < w * EDGE_BAND) | (pts[:, 0] > w * (1 - EDGE_BAND))
                   | (pts[:, 1] < h * EDGE_BAND) | (pts[:, 1] > h * (1 - EDGE_BAND)))
        if in_edge.any():
            edge_residuals.extend(err[in_edge].tolist())
    max_edge = float(max(edge_residuals)) if edge_residuals else float("nan")
    n_coeffs = {"plumb_bob": 5, "rational": 8}[name]
    return {
        "name": name,
        "rms": float(ret),
        "K": K,
        "D": D.reshape(-1)[:n_coeffs],
        "per_image": per_image,
        "max_edge_px": max_edge,
    }


def detect_corners(paths, pattern):
    objp = np.zeros((pattern[0] * pattern[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern[0], 0:pattern[1]].T.reshape(-1, 2)

    objpoints, imgpoints, used, size = [], [], [], None
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"  skip (unreadable): {path}")
            continue
        h, w = img.shape[:2]
        if size is None:
            size = (w, h)
        elif (w, h) != size:
            raise SystemExit(
                f"ERROR: mixed resolutions — {path} is {w}x{h}, first image was "
                f"{size[0]}x{size[1]}. Calibrate one resolution at a time.")
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(
            gray, pattern,
            cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        if not found:
            print(f"  skip (no board): {os.path.basename(path)}")
            continue
        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        objpoints.append(objp)
        imgpoints.append(corners)
        used.append(path)
    return objpoints, imgpoints, used, size


def edge_coverage(imgpoints, size):
    w, h = size
    hit = {"left": False, "right": False, "top": False, "bottom": False}
    for corners in imgpoints:
        pts = corners.reshape(-1, 2)
        hit["left"] |= bool((pts[:, 0] < w * EDGE_BAND).any())
        hit["right"] |= bool((pts[:, 0] > w * (1 - EDGE_BAND)).any())
        hit["top"] |= bool((pts[:, 1] < h * EDGE_BAND).any())
        hit["bottom"] |= bool((pts[:, 1] > h * (1 - EDGE_BAND)).any())
    return hit


def fit_model(name, flags, objpoints, imgpoints, size):
    ret, K, D, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, size, None, None, flags=flags)
    per_image = []
    edge_residuals = []
    w, h = size
    for i, (objp, imgp) in enumerate(zip(objpoints, imgpoints)):
        proj, _ = cv2.projectPoints(objp, rvecs[i], tvecs[i], K, D)
        err = np.linalg.norm(proj.reshape(-1, 2) - imgp.reshape(-1, 2), axis=1)
        per_image.append(float(np.sqrt(np.mean(err ** 2))))
        pts = imgp.reshape(-1, 2)
        in_edge = ((pts[:, 0] < w * EDGE_BAND) | (pts[:, 0] > w * (1 - EDGE_BAND))
                   | (pts[:, 1] < h * EDGE_BAND) | (pts[:, 1] > h * (1 - EDGE_BAND)))
        if in_edge.any():
            edge_residuals.extend(err[in_edge].tolist())
    max_edge = float(max(edge_residuals)) if edge_residuals else float("nan")
    n_coeffs = {"plumb_bob": 5, "rational": 8}[name]
    return {
        "name": name,
        "rms": float(ret),
        "K": K,
        "D": D.reshape(-1)[:n_coeffs],
        "per_image": per_image,
        "max_edge_px": max_edge,
    }


def fit_fisheye(objpoints, imgpoints, size):
    # fisheye needs (N,1,3) float64 objpoints and CALIB_FIX_SKEW is typical.
    obj = [o.reshape(-1, 1, 3).astype(np.float64) for o in objpoints]
    img = [i.astype(np.float64) for i in imgpoints]
    K = np.zeros((3, 3))
    D = np.zeros((4, 1))
    flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC
             | cv2.fisheye.CALIB_FIX_SKEW)
    try:
        ret, K, D, _, _ = cv2.fisheye.calibrate(
            obj, img, size, K, D, None, None, flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
    except cv2.error as exc:
        return {"name": "fisheye", "error": str(exc).splitlines()[-1]}
    return {"name": "fisheye", "rms": float(ret), "K": K,
            "D": D.reshape(-1), "max_edge_px": float("nan"), "per_image": []}


def recommend(std, rat):
    if (rat["max_edge_px"] <= 1.0 or math.isnan(rat["max_edge_px"])) and rat["rms"] <= 0.5:
        if std["rms"] - rat["rms"] <= 0.1 and (
                math.isnan(std["max_edge_px"]) or math.isnan(rat["max_edge_px"])
                or std["max_edge_px"] - rat["max_edge_px"] <= 0.1):
            return std, "plumb_bob is within 0.1 px of rational everywhere — fewer params, less overfit"
        return rat, "rational meets RMS<=0.5px and edge residual<=1.0px"
    if std["rms"] <= rat["rms"] and std["max_edge_px"] <= rat["max_edge_px"]:
        return std, "standard fit no worse than rational"
    return rat, "rational is the better of the two (check thresholds below)"


def yaml_block(model):
    K, D = model["K"], model["D"]
    coeffs = ", ".join(f"{c:.6f}" for c in D)
    return (
        f"    camera_fx: {K[0, 0]:.2f}\n"
        f"    camera_fy: {K[1, 1]:.2f}\n"
        f"    camera_cx: {K[0, 2]:.2f}\n"
        f"    camera_cy: {K[1, 2]:.2f}\n"
        f"    distortion_model: \"{model['name']}\"\n"
        f"    dist_coeffs: [{coeffs}]"
    )


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--images", default="calib_frames")
    ap.add_argument("--board", choices=("checker", "charuco"), default="checker")
    ap.add_argument("--pattern", type=parse_pattern, default=(9, 6),
                    help="checker: INNER corners COLSxROWS (9x6 = a 10x7-square "
                         "board). charuco: SQUARES COLSxROWS (e.g. 10x7).")
    ap.add_argument("--square-mm", type=float, default=25.0)
    ap.add_argument("--marker-mm", type=float, default=19.0,
                    help="charuco only: ArUco marker side length")
    ap.add_argument("--aruco-dict", default="DICT_4X4_50",
                    help="charuco only: dictionary name printed on the board")
    ap.add_argument("--try-fisheye", action="store_true")
    ap.add_argument("--min-images", type=int, default=12)
    args = ap.parse_args()

    paths = sorted(glob.glob(os.path.join(args.images, "*.jpg"))
                   + glob.glob(os.path.join(args.images, "*.png")))
    if not paths:
        raise SystemExit(f"No images in {args.images}")
    print(f"Scanning {len(paths)} images for a {args.pattern[0]}x{args.pattern[1]} "
          f"{args.board} board ...")

    fisheye_obj, fisheye_img = None, None
    if args.board == "charuco":
        board, all_corners, all_ids, used, size = detect_charuco(
            paths, args.pattern, args.square_mm, args.marker_mm, args.aruco_dict)
        imgpoints = all_corners
        board_pts = board.chessboardCorners.reshape(-1, 3)
        fisheye_obj = [board_pts[i.reshape(-1)].astype(np.float64) for i in all_ids]
        fisheye_img = all_corners
    else:
        objpoints, imgpoints, used, size = detect_corners(paths, args.pattern)
        # Scale object points by square size (meters don't matter for
        # intrinsics, but keep it honest for the extrinsics/report).
        scale = args.square_mm / 1000.0
        objpoints = [o * scale for o in objpoints]
        fisheye_obj, fisheye_img = objpoints, imgpoints

    print(f"Usable: {len(used)}/{len(paths)} at {size[0]}x{size[1]}")
    if len(used) < args.min_images:
        raise SystemExit(
            f"ERROR: only {len(used)} usable images (< {args.min_images}). "
            "Capture more varied views (scripts/capture_calib_frames.py).")
    if size != (640, 480):
        print(f"WARNING: resolution {size} != 640x480 — these intrinsics only "
              "apply to the resolution the detector runs at!")

    cov = edge_coverage(imgpoints, size)
    missing = [k for k, v in cov.items() if not v]
    if missing:
        print(f"WARNING: no corners ever reached the outer {int(EDGE_BAND*100)}% "
              f"band on: {', '.join(missing)} — the distortion fit is "
              "unconstrained exactly where the wide lens needs it. "
              "Strongly consider re-capturing with the board at those edges.")

    print("\nFitting standard (plumb_bob, 5 coeffs) ...")
    if args.board == "charuco":
        std = fit_charuco("plumb_bob", 0, board, all_corners, all_ids, size)
        print("Fitting rational (8 coeffs) ...")
        rat = fit_charuco("rational", cv2.CALIB_RATIONAL_MODEL,
                          board, all_corners, all_ids, size)
    else:
        std = fit_model("plumb_bob", 0, objpoints, imgpoints, size)
        print("Fitting rational (8 coeffs) ...")
        rat = fit_model("rational", cv2.CALIB_RATIONAL_MODEL, objpoints, imgpoints, size)

    print("\n=== Model comparison ===")
    for m in (std, rat):
        worst = max(m["per_image"]) if m["per_image"] else float("nan")
        print(f"  {m['name']:>10}: RMS {m['rms']:.3f} px | max edge-band residual "
              f"{m['max_edge_px']:.3f} px | worst image RMS {worst:.3f} px")

    winner, why = recommend(std, rat)

    if rat["rms"] > 0.5 or (not math.isnan(rat["max_edge_px"]) and rat["max_edge_px"] > 1.5):
        print("\n!!! Even the rational model fits poorly (RMS>0.5px or edge>1.5px).")
        print("!!! This lens may be a true fisheye projection.")
        if args.try_fisheye:
            fe = fit_fisheye(fisheye_obj, fisheye_img, size)
            if "error" in fe:
                print(f"    fisheye fit FAILED: {fe['error']}")
            else:
                print(f"    fisheye: RMS {fe['rms']:.3f} px")
                if fe["rms"] < winner["rms"]:
                    winner, why = fe, "fisheye RMS beats both rectilinear models"
        else:
            print("!!! Re-run with --try-fisheye to fit cv2.fisheye as well.")

    print(f"\nRECOMMENDED MODEL: {winner['name']}  ({why})")
    print("\nPaste into the `tracker_node:` -> `ros__parameters:` section of")
    print("  configs/pi_hardware.yaml          <- hardware source of truth")
    print("  (deploy_pi.sh stages it onto the Pi; do NOT paste into")
    print("   configs/pi.yaml — that is the SIM config and must stay legacy)\n")
    print(yaml_block(winner))
    print("\nThen: deploy, restart, and confirm the tracker startup log says")
    print(f"  'Camera geometry: CALIBRATED {winner['name']} ...'")


if __name__ == "__main__":
    main()
