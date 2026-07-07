#!/usr/bin/env python3
"""Calibrate the monocular distance estimate against a known real distance.

The tracker estimates range as ``distance_m = distance_calibration_k /
bbox_diameter_px``. This tool measures ``k`` for YOUR camera + YOLO model +
ball: hold (or place) the ball at a tape-measured distance from the camera,
run this while the stack is up, and it samples live detections and prints the
calibrated ``k`` plus the exact ``pi.yaml`` lines to set.

The real DroneTrack ball is 8.5 in = 0.2159 m diameter (the default below);
the Gazebo sim ball is 0.20 m — pass --ball-diameter-m 0.20 when calibrating
in SITL.

Usage (through the stack's DDS environment):

    # ball exactly 3.00 m from the camera lens, sample ~5 s of detections:
    scripts/ros_wsl.sh env python3 scripts/calibrate_distance.py --true-distance-m 3.00

    # SITL check against the sim ball:
    scripts/ros_wsl.sh env python3 scripts/calibrate_distance.py \
        --true-distance-m <gz truth> --ball-diameter-m 0.20

For best results run it at 2-3 different distances spanning your working
range (e.g. 2 m, 5 m, 8 m) and use the mean of the reported k values — a
constant k across distances also confirms the bbox scales like a true
pinhole; a drifting k means the detector's box-fatness varies with size.
"""

from __future__ import annotations

import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from drone_interfaces.msg import DetectionArray

BALL_8_5_INCH_M = 0.2159  # 8.5 in — the real DroneTrack ball


class _Collector(Node):
    def __init__(self, topic: str, target_class: str, min_confidence: float):
        super().__init__("distance_calibrator")
        self.samples: list[float] = []
        self.image_width = 0
        self.image_height = 0
        self.target_class = target_class
        self.min_confidence = min_confidence
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        self.create_subscription(DetectionArray, topic, self._on_detections, qos)

    def _on_detections(self, msg: DetectionArray) -> None:
        self.image_width = int(msg.image_width) or self.image_width
        self.image_height = int(msg.image_height) or self.image_height
        candidates = [
            d for d in msg.detections
            if d.confidence >= self.min_confidence
            and (not self.target_class or d.class_name == self.target_class)
        ]
        if not candidates:
            return
        best = max(candidates, key=lambda d: d.confidence)
        diameter_px = 0.5 * (float(best.pixel_width) + float(best.pixel_height))
        if diameter_px > 1.0:
            self.samples.append(diameter_px)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument("--true-distance-m", type=float, required=True,
                        help="Tape-measured camera-to-ball distance (m).")
    parser.add_argument("--ball-diameter-m", type=float, default=BALL_8_5_INCH_M,
                        help=f"Real ball diameter in m (default {BALL_8_5_INCH_M} = 8.5 in; "
                             "sim ball is 0.20).")
    parser.add_argument("--samples", type=int, default=100,
                        help="Detections to collect (default 100 ≈ 7 s at 15 fps).")
    parser.add_argument("--timeout-s", type=float, default=30.0,
                        help="Give up after this long (default 30 s).")
    parser.add_argument("--topic", default="/groundstation/vision/detections",
                        help="DetectionArray topic (default: raw YOLO output, no gate).")
    parser.add_argument("--target-class", default="red_ball",
                        help="Only use detections of this class ('' = any).")
    parser.add_argument("--min-confidence", type=float, default=0.3)
    parser.add_argument("--hfov-deg", type=float, default=99.7,
                        help="Camera horizontal FOV for the fat-factor report "
                             "(ignored when --fx is given).")
    parser.add_argument("--fx", type=float, default=0.0,
                        help="Measured focal length in px (camera_fx from "
                             "calibrate_camera_intrinsics.py); replaces the "
                             "FOV-derived fx in the fat-factor cross-check.")
    args = parser.parse_args()

    if args.true_distance_m <= 0.0 or args.ball_diameter_m <= 0.0:
        parser.error("--true-distance-m and --ball-diameter-m must be > 0")

    rclpy.init()
    node = _Collector(args.topic, args.target_class, args.min_confidence)
    print(f"Sampling {args.samples} detections from {args.topic} "
          f"(class='{args.target_class or 'any'}', conf>={args.min_confidence}) ...")
    start = time.monotonic()
    try:
        while len(node.samples) < args.samples:
            rclpy.spin_once(node, timeout_sec=0.5)
            if time.monotonic() - start > args.timeout_s:
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

    n = len(node.samples)
    if n < 10:
        print(f"\nFAILED: only {n} usable detections. Is the stack running, the ball "
              "in view, and the topic/class right?", file=sys.stderr)
        return 1

    med = statistics.median(node.samples)
    mean = statistics.fmean(node.samples)
    spread = statistics.pstdev(node.samples)
    k = args.true_distance_m * med

    print(f"\nCollected {n} samples in {time.monotonic() - start:.1f} s "
          f"(image {node.image_width}x{node.image_height})")
    print(f"bbox diameter: median={med:.1f} px, mean={mean:.1f} px, stdev={spread:.1f} px")
    if spread > 0.15 * med:
        print("WARNING: noisy diameters (stdev > 15% of median) — is the ball or "
              "camera moving? Calibrate with both still.")

    print(f"\ndistance_calibration_k = true_distance * median_diameter"
          f" = {args.true_distance_m:.3f} * {med:.1f} = {k:.1f}")

    if node.image_width > 0:
        if args.fx > 0.0:
            fx = args.fx
            fx_source = "measured camera_fx"
        else:
            fx = node.image_width / (2.0 * math.tan(math.radians(args.hfov_deg) / 2.0))
            fx_source = f"fx from --hfov-deg {args.hfov_deg}"
        pinhole_k = args.ball_diameter_m * fx
        print(f"pinhole predicts k = ball_diameter * fx = {args.ball_diameter_m:.4f} * "
              f"{fx:.0f} ({fx_source}) = {pinhole_k:.1f}"
              f"  ->  bbox fat-factor = {k / pinhole_k:.2f}x")
        print("(fat-factor is a property of the YOLO model; re-calibrate after "
              "retraining or swapping models)")

    print(f"\nSet in configs/pi.yaml under tracker_node:")
    print(f"    distance_calibration_k: {k:.1f}")
    print(f"    ball_diameter_m: {args.ball_diameter_m:.4f}")
    print("\nRepeat at 2-3 distances across your working range; k should be stable.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
