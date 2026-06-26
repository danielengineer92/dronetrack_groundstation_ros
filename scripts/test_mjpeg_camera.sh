#!/usr/bin/env bash
# Probe a V4L2 camera in MJPEG mode and measure the real capture FPS.
# Run on the Pi, for example:
#   bash scripts/test_mjpeg_camera.sh --device 0 --width 640 --height 480 --fps 90
#   bash scripts/test_mjpeg_camera.sh --device /dev/video0 --width 1920 --height 1080 --fps 90
set -eo pipefail

python3 - "$@" <<'PY'
from __future__ import annotations

import argparse
import os
import shutil
import statistics
import subprocess
import sys
import time
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Force MJPEG on a V4L2 camera and measure real capture FPS."
    )
    parser.add_argument("--device", default="0", help="Camera index or device path, default: 0")
    parser.add_argument("--width", type=int, default=640, help="Requested frame width")
    parser.add_argument("--height", type=int, default=480, help="Requested frame height")
    parser.add_argument("--fps", type=float, default=90.0, help="Requested camera FPS")
    parser.add_argument("--seconds", type=float, default=5.0, help="Measurement duration")
    parser.add_argument("--warmup", type=int, default=30, help="Frames to discard before timing")
    parser.add_argument("--buffer-size", type=int, default=1, help="OpenCV capture buffer size")
    parser.add_argument(
        "--skip-v4l2-list",
        action="store_true",
        help="Do not print v4l2-ctl --list-formats-ext output first",
    )
    parser.add_argument(
        "--v4l2-stream",
        action="store_true",
        help="Also run a direct v4l2-ctl MJPEG stream test before OpenCV",
    )
    parser.add_argument(
        "--stream-count",
        type=int,
        default=300,
        help="Frame count for --v4l2-stream",
    )
    parser.add_argument(
        "--save-frame",
        default="",
        help="Optional path to save the last captured frame as JPEG/PNG",
    )
    return parser.parse_args()


def v4l2_device(device: str) -> str:
    device = str(device)
    return f"/dev/video{device}" if device.isdigit() else device


def cv2_device(device: str):
    device = str(device)
    return int(device) if device.isdigit() else device


def run_command(cmd: list[str], *, check: bool = False) -> int:
    print(f"\n$ {' '.join(cmd)}")
    try:
        completed = subprocess.run(cmd, text=True, check=check)
        return int(completed.returncode)
    except FileNotFoundError:
        print(f"command not found: {cmd[0]}")
        return 127
    except subprocess.CalledProcessError as exc:
        return int(exc.returncode)


def decode_fourcc(value: float) -> str:
    raw = int(value)
    chars = []
    for index in range(4):
        byte = (raw >> (8 * index)) & 0xFF
        chars.append(chr(byte) if 32 <= byte <= 126 else "?")
    return "".join(chars)


def video_writer_fourcc(cv2, code: str) -> int:
    return cv2.VideoWriter_fourcc(*code)


def main() -> int:
    args = parse_args()
    dev = v4l2_device(args.device)

    print("== DroneTrack MJPEG camera probe ==")
    print(f"device={args.device} ({dev}) requested={args.width}x{args.height}@{args.fps:g} MJPEG")

    have_v4l2_ctl = shutil.which("v4l2-ctl") is not None
    if not args.skip_v4l2_list:
        if have_v4l2_ctl:
            run_command(["v4l2-ctl", "-d", dev, "--list-formats-ext"])
        else:
            print("\nv4l2-ctl not found. Install with: sudo apt install v4l-utils")

    if args.v4l2_stream:
        if not have_v4l2_ctl:
            print("\nSkipping --v4l2-stream because v4l2-ctl is not installed.")
        else:
            run_command([
                "v4l2-ctl",
                "-d",
                dev,
                f"--set-fmt-video=width={args.width},height={args.height},pixelformat=MJPG",
                f"--set-parm={args.fps:g}",
                "--stream-mmap",
                f"--stream-count={args.stream_count}",
                "--stream-to=/dev/null",
            ])

    try:
        import cv2
    except ImportError:
        print("\nPython cannot import cv2. Install OpenCV on the Pi first.", file=sys.stderr)
        print("For ROS Jazzy systems this is usually: sudo apt install python3-opencv", file=sys.stderr)
        return 2

    cap = cv2.VideoCapture(cv2_device(args.device), cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"\nCould not open camera {args.device!r} with OpenCV/V4L2.", file=sys.stderr)
        return 3

    requested_fourcc = video_writer_fourcc(cv2, "MJPG")
    settings = [
        (cv2.CAP_PROP_FOURCC, requested_fourcc, "fourcc=MJPG"),
        (cv2.CAP_PROP_FRAME_WIDTH, float(args.width), f"width={args.width}"),
        (cv2.CAP_PROP_FRAME_HEIGHT, float(args.height), f"height={args.height}"),
        (cv2.CAP_PROP_FPS, float(args.fps), f"fps={args.fps:g}"),
    ]
    if args.buffer_size >= 0:
        settings.append((cv2.CAP_PROP_BUFFERSIZE, float(args.buffer_size), f"buffer_size={args.buffer_size}"))

    print("\nApplying OpenCV/V4L2 settings:")
    for prop, value, label in settings:
        ok = cap.set(prop, value)
        print(f"  {label:<18} {'ok' if ok else 'not accepted'}")

    actual_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    actual_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    actual_fps = float(cap.get(cv2.CAP_PROP_FPS))
    actual_fourcc = decode_fourcc(cap.get(cv2.CAP_PROP_FOURCC))
    print(
        "\nNegotiated by OpenCV/V4L2: "
        f"{actual_width}x{actual_height}@{actual_fps:.2f}, fourcc={actual_fourcc!r}"
    )

    warmup_ok = 0
    last_frame = None
    for _ in range(max(args.warmup, 0)):
        ok, frame = cap.read()
        if ok and frame is not None:
            warmup_ok += 1
            last_frame = frame

    print(f"Warmup frames read: {warmup_ok}/{max(args.warmup, 0)}")
    print(f"Measuring for {args.seconds:g}s ...")

    timestamps: list[float] = []
    failures = 0
    deadline = time.perf_counter() + max(args.seconds, 0.1)
    while time.perf_counter() < deadline:
        ok, frame = cap.read()
        now = time.perf_counter()
        if ok and frame is not None:
            timestamps.append(now)
            last_frame = frame
        else:
            failures += 1

    cap.release()

    if len(timestamps) < 2:
        print(f"\nCaptured only {len(timestamps)} frames; failures={failures}", file=sys.stderr)
        return 4

    elapsed = timestamps[-1] - timestamps[0]
    measured_fps = (len(timestamps) - 1) / elapsed if elapsed > 0 else 0.0
    intervals = [b - a for a, b in zip(timestamps, timestamps[1:])]
    avg_interval_ms = statistics.mean(intervals) * 1000.0
    min_interval_ms = min(intervals) * 1000.0
    max_interval_ms = max(intervals) * 1000.0

    print("\nMeasured OpenCV capture:")
    print(f"  frames={len(timestamps)} failures={failures} elapsed={elapsed:.3f}s")
    print(f"  fps={measured_fps:.2f}")
    print(
        "  frame_interval_ms="
        f"avg {avg_interval_ms:.2f}, min {min_interval_ms:.2f}, max {max_interval_ms:.2f}"
    )
    if last_frame is not None:
        print(f"  last_frame_shape={tuple(last_frame.shape)}")

    if args.save_frame:
        output = Path(args.save_frame).expanduser()
        output.parent.mkdir(parents=True, exist_ok=True)
        ok = cv2.imwrite(str(output), last_frame)
        print(f"  saved_frame={output} ({'ok' if ok else 'failed'})")

    print("\nInterpretation:")
    if actual_fourcc != "MJPG":
        print("  OpenCV did not negotiate MJPG. Check v4l2-ctl modes and try a listed resolution/FPS.")
    elif measured_fps < args.fps * 0.75:
        print("  MJPG was selected, but measured FPS is far below request. Check exposure, USB bandwidth, CPU load, and subscribers.")
    else:
        print("  MJPG mode is working at roughly the requested rate.")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
PY