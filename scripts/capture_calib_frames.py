#!/usr/bin/env python3
"""Capture checkerboard frames for intrinsic calibration.

Subscribes to the compressed camera topic and saves the raw JPEG bytes of
frames where the FULL checkerboard is detected, auto-paced and gated so you
end up with varied, usable views instead of 40 near-duplicates. A live
preview (cv2.imshow) draws the detected corners and a coverage HUD steering
you toward the frame regions a 120-degree lens needs most: edges + corners.

Typical use (laptop, ROS env sourced, Pi camera streaming):

    python3 scripts/capture_calib_frames.py --out-dir calib_frames --count 40

Then run scripts/calibrate_camera_intrinsics.py on the output directory.
"""

import argparse
import os
import sys
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage

EXPECTED_SIZE = (640, 480)


def parse_pattern(text: str):
    try:
        cols, rows = text.lower().split("x")
        return int(cols), int(rows)
    except Exception:
        raise argparse.ArgumentTypeError(
            f"pattern '{text}' must look like 9x6 (INNER corners, cols x rows)")


def make_charuco(args):
    dict_id = getattr(cv2.aruco, args.aruco_dict, None)
    if dict_id is None:
        raise SystemExit(f"Unknown --aruco-dict {args.aruco_dict}")
    adict = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        board = cv2.aruco.CharucoBoard_create(
            args.pattern[0], args.pattern[1],
            args.square_mm / 1000.0, args.marker_mm / 1000.0, adict)
    else:  # pragma: no cover - newer OpenCV
        board = cv2.aruco.CharucoBoard(
            (args.pattern[0], args.pattern[1]),
            args.square_mm / 1000.0, args.marker_mm / 1000.0, adict)
    return adict, board


class _Capture(Node):
    def __init__(self, args):
        super().__init__("calib_frame_capture")
        self.args = args
        self.pattern = args.pattern
        self.charuco = None
        if args.board == "charuco":
            self.charuco = make_charuco(args)
        self.saved = 0
        self.last_save_time = 0.0
        self.last_saved_centroid = None
        self.last_saved_area = 0.0
        self.frames_seen = 0
        self.size_warned = False
        # Coverage bands (fraction of saves whose corners touched each region).
        self.coverage = {"left": 0, "right": 0, "top": 0, "bottom": 0, "corners": 0}
        os.makedirs(args.out_dir, exist_ok=True)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(CompressedImage, args.topic, self._on_frame, qos)
        self.get_logger().info(
            f"Capturing from {args.topic} -> {args.out_dir} "
            f"(pattern {self.pattern[0]}x{self.pattern[1]} inner corners, "
            f"target {args.count} frames, "
            f"{'MANUAL (SPACE to save)' if args.manual else f'auto every {args.interval_s}s'})"
        )

    # ---- helpers -----------------------------------------------------------
    def _board_moved_enough(self, corners, frame_shape) -> bool:
        h, w = frame_shape[:2]
        centroid = corners.reshape(-1, 2).mean(axis=0)
        area = float(cv2.contourArea(cv2.convexHull(corners.reshape(-1, 1, 2))))
        if self.last_saved_centroid is None:
            return True
        dx = abs(centroid[0] - self.last_saved_centroid[0]) / w
        dy = abs(centroid[1] - self.last_saved_centroid[1]) / h
        darea = abs(area - self.last_saved_area) / max(self.last_saved_area, 1.0)
        return max(dx, dy) >= self.args.min_move_frac or darea >= 0.25

    def _update_coverage(self, corners, frame_shape):
        h, w = frame_shape[:2]
        pts = corners.reshape(-1, 2)
        edge = 0.15
        if (pts[:, 0] < w * edge).any():
            self.coverage["left"] += 1
        if (pts[:, 0] > w * (1 - edge)).any():
            self.coverage["right"] += 1
        if (pts[:, 1] < h * edge).any():
            self.coverage["top"] += 1
        if (pts[:, 1] > h * (1 - edge)).any():
            self.coverage["bottom"] += 1
        in_corner = ((pts[:, 0] < w * edge) | (pts[:, 0] > w * (1 - edge))) & (
            (pts[:, 1] < h * edge) | (pts[:, 1] > h * (1 - edge)))
        if in_corner.any():
            self.coverage["corners"] += 1

    def _coverage_hud(self) -> str:
        def mark(k):
            return "#" if self.coverage[k] > 0 else "."
        return (f"cov L{mark('left')} R{mark('right')} T{mark('top')} "
                f"B{mark('bottom')} corners:{self.coverage['corners']}")

    def _save(self, msg, corners, frame):
        path = os.path.join(self.args.out_dir, f"calib_{self.saved:03d}.jpg")
        with open(path, "wb") as f:
            f.write(bytes(msg.data))  # raw JPEG passthrough, no re-encode
        self.saved += 1
        self.last_save_time = time.monotonic()
        self.last_saved_centroid = corners.reshape(-1, 2).mean(axis=0)
        self.last_saved_area = float(
            cv2.contourArea(cv2.convexHull(corners.reshape(-1, 1, 2))))
        self._update_coverage(corners, frame.shape)
        print(f"saved {self.saved}/{self.args.count}  {self._coverage_hud()}")

    # ---- frame handler -----------------------------------------------------
    def _on_frame(self, msg: CompressedImage):
        self.frames_seen += 1
        frame = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            return
        h, w = frame.shape[:2]
        if (w, h) != EXPECTED_SIZE and not self.size_warned:
            self.size_warned = True
            self.get_logger().warning(
                f"Frame is {w}x{h}, expected {EXPECTED_SIZE[0]}x{EXPECTED_SIZE[1]} — "
                "intrinsics only apply at the resolution they were calibrated at!")

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.charuco is not None:
            adict, board = self.charuco
            m_corners, m_ids, _ = cv2.aruco.detectMarkers(gray, adict)
            found, corners = False, None
            if m_ids is not None and len(m_ids) >= 4:
                n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
                    m_corners, m_ids, gray, board)
                # Partial views are the point of ChArUco: 8 corners is enough
                # to contribute, so edge-of-frame shots still count.
                if n is not None and n >= 8:
                    found, corners = True, ch_corners
        else:
            found, corners = cv2.findChessboardCorners(
                gray, self.pattern,
                cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK)

        key = -1
        if not self.args.no_preview:
            disp = frame.copy()
            if found:
                if self.charuco is not None:
                    cv2.aruco.drawDetectedCornersCharuco(disp, corners)
                else:
                    cv2.drawChessboardCorners(disp, self.pattern, corners, found)
            hud = (f"{'BOARD' if found else 'NO BOARD'}  saved {self.saved}/"
                   f"{self.args.count}  {self._coverage_hud()}")
            cv2.putText(disp, hud, (8, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                        (0, 255, 0) if found else (0, 0, 255), 2)
            cv2.imshow("calib capture (q=quit, SPACE=save in --manual)", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                raise SystemExit(0)

        if not found:
            return

        if self.args.manual:
            if key == ord(" "):
                self._save(msg, corners, frame)
        else:
            since = time.monotonic() - self.last_save_time
            if since >= self.args.interval_s and self._board_moved_enough(
                    corners, frame.shape):
                self._save(msg, corners, frame)

        if self.saved >= self.args.count:
            print("Done: target frame count reached.")
            raise SystemExit(0)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--topic", default="/drone/camera/image_raw/compressed")
    ap.add_argument("--out-dir", default="calib_frames")
    ap.add_argument("--board", choices=("checker", "charuco"), default="checker")
    ap.add_argument("--pattern", type=parse_pattern, default=(9, 6),
                    help="checker: INNER corners COLSxROWS (default 9x6). "
                         "charuco: SQUARES COLSxROWS (e.g. 10x7).")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="charuco only: square side length")
    ap.add_argument("--marker-mm", type=float, default=19.0,
                    help="charuco only: ArUco marker side length")
    ap.add_argument("--aruco-dict", default="DICT_4X4_50",
                    help="charuco only: dictionary name")
    ap.add_argument("--count", type=int, default=40)
    ap.add_argument("--interval-s", type=float, default=1.5)
    ap.add_argument("--manual", action="store_true",
                    help="save on SPACE instead of auto-cadence")
    ap.add_argument("--no-preview", action="store_true",
                    help="headless: no imshow window (auto mode only)")
    ap.add_argument("--min-move-frac", type=float, default=0.08,
                    help="board must move this fraction of the frame between saves")
    args = ap.parse_args()

    if args.manual and args.no_preview:
        ap.error("--manual needs the preview window for keyboard input")

    rclpy.init()
    node = _Capture(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        print(f"Captured {node.saved} frames into {args.out_dir} "
              f"({node.frames_seen} frames seen). {node._coverage_hud()}")
        if node.saved and (node.coverage["corners"] == 0
                           or 0 in (node.coverage["left"], node.coverage["right"],
                                    node.coverage["top"], node.coverage["bottom"])):
            print("WARNING: some frame edges/corners were never covered — "
                  "wide-lens distortion is fit worst exactly there. "
                  "Consider capturing more views at the edges.")
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
