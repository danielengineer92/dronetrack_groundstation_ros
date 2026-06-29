#!/usr/bin/env bash
# Publish native V4L2 MJPEG frames directly as sensor_msgs/CompressedImage.
# This bypasses the current path: camera MJPG -> OpenCV BGR Image -> JPEG re-encode.
#
# Run on the Pi after sourcing ROS, for example:
#   source ros_ws/install/setup.bash
#   bash scripts/test_native_mjpeg_stream.sh --device /dev/video0 --width 640 --height 480 --fps 60
#
# Then on the laptop point YOLO at the topic this publishes, or publish to the
# default /drone/camera/image_raw/compressed topic with the normal compressor off.
set -eo pipefail

python3 - "$@" <<'PY'
from __future__ import annotations

import argparse
import os
import shutil
import signal
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish native camera MJPEG bytes directly to ROS CompressedImage."
    )
    parser.add_argument("--device", default="/dev/video0", help="V4L2 device path or index, default: /dev/video0")
    parser.add_argument("--width", type=int, default=640, help="Requested frame width")
    parser.add_argument("--height", type=int, default=480, help="Requested frame height")
    parser.add_argument("--fps", type=float, default=60.0, help="Requested camera FPS")
    parser.add_argument(
        "--topic",
        default="/drone/camera/image_raw/compressed",
        help="CompressedImage topic to publish. Use the default only when the normal compressor is off.",
    )
    parser.add_argument("--frame-id", default="camera_optical_frame", help="ROS header frame_id")
    parser.add_argument("--seconds", type=float, default=0.0, help="Stop after this many seconds; 0 runs until Ctrl-C")
    parser.add_argument("--report-period", type=float, default=2.0, help="Seconds between progress reports")
    parser.add_argument("--chunk-size", type=int, default=65536, help="Bytes to read per v4l2-ctl stdout read")
    parser.add_argument("--buffer-count", type=int, default=4, help="v4l2 mmap buffer count")
    parser.add_argument("--stream-count", type=int, default=2147483647, help="v4l2 frame count limit")
    parser.add_argument(
        "--wait-for-subscriber",
        action="store_true",
        help="Wait for a subscriber before starting the V4L2 stream.",
    )
    parser.add_argument(
        "--save-first-frame",
        default="",
        help="Optional path to save the first native JPEG frame for inspection.",
    )
    parser.add_argument(
        "--no-list-formats",
        action="store_true",
        help="Skip printing v4l2-ctl --list-formats-ext before streaming.",
    )
    return parser.parse_args()


def normalize_device(device: str) -> str:
    device = str(device)
    return f"/dev/video{device}" if device.isdigit() else device


def require_ros_imports():
    try:
        import rclpy
        from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
        from sensor_msgs.msg import CompressedImage
    except ImportError as exc:
        print("Failed to import ROS Python modules.", file=sys.stderr)
        print("Run: source ros_ws/install/setup.bash", file=sys.stderr)
        print(f"Import error: {exc}", file=sys.stderr)
        raise SystemExit(2) from exc

    return rclpy, HistoryPolicy, QoSProfile, ReliabilityPolicy, CompressedImage


@dataclass
class Counters:
    frames: int = 0
    bytes_total: int = 0
    dropped_prefix_bytes: int = 0
    saved_first_frame: bool = False


def stderr_reader(proc: subprocess.Popen[bytes], lines: deque[str], stop_event: threading.Event) -> None:
    assert proc.stderr is not None
    while not stop_event.is_set():
        line = proc.stderr.readline()
        if not line:
            break
        try:
            text = line.decode("utf-8", errors="replace").rstrip()
        except Exception:
            text = repr(line)
        if text:
            lines.append(text)
            while len(lines) > 20:
                lines.popleft()


def find_jpeg_frames(buffer: bytearray, counters: Counters):
    while True:
        soi = buffer.find(b"\xff\xd8")
        if soi < 0:
            if len(buffer) > 2:
                counters.dropped_prefix_bytes += len(buffer) - 2
                del buffer[:-2]
            return
        if soi > 0:
            counters.dropped_prefix_bytes += soi
            del buffer[:soi]

        eoi = buffer.find(b"\xff\xd9", 2)
        if eoi < 0:
            return

        frame = bytes(buffer[: eoi + 2])
        del buffer[: eoi + 2]
        yield frame


def main() -> int:
    args = parse_args()
    dev = normalize_device(args.device)

    if shutil.which("v4l2-ctl") is None:
        print("v4l2-ctl not found. Install on the Pi with: sudo apt install v4l-utils", file=sys.stderr)
        return 2

    rclpy, HistoryPolicy, QoSProfile, ReliabilityPolicy, CompressedImage = require_ros_imports()

    if not args.no_list_formats:
        subprocess.run(["v4l2-ctl", "-d", dev, "--list-formats-ext"], check=False)

    rclpy.init()
    node = rclpy.create_node("native_mjpeg_stream_test")
    qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )
    pub = node.create_publisher(CompressedImage, args.topic, qos)

    print("== Native MJPEG -> ROS CompressedImage test ==")
    print(f"device={dev} requested={args.width}x{args.height}@{args.fps:g} pixelformat=MJPG")
    print(f"topic={args.topic} frame_id={args.frame_id} qos=BEST_EFFORT/KEEP_LAST(1)")
    print("This script publishes the camera's JPEG bytes directly; it does not decode or re-encode frames.")

    if args.wait_for_subscriber:
        print("Waiting for a subscriber before starting camera stream ...")
        while rclpy.ok() and pub.get_subscription_count() == 0:
            rclpy.spin_once(node, timeout_sec=0.1)

    cmd = [
        "v4l2-ctl",
        "-d",
        dev,
        f"--set-fmt-video=width={args.width},height={args.height},pixelformat=MJPG",
        f"--set-parm={args.fps:g}",
        f"--stream-mmap={args.buffer_count}",
        f"--stream-count={args.stream_count}",
        "--stream-to=-",
    ]
    print("\n$ " + " ".join(cmd))
    proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
    assert proc.stdout is not None

    stop_event = threading.Event()
    stderr_lines: deque[str] = deque(maxlen=20)
    stderr_thread = threading.Thread(target=stderr_reader, args=(proc, stderr_lines, stop_event), daemon=True)
    stderr_thread.start()

    counters = Counters()
    pending = bytearray()
    t0 = time.monotonic()
    last_report_t = t0
    last_report_frames = 0
    last_report_bytes = 0
    recent_intervals: deque[float] = deque(maxlen=120)
    last_frame_t = 0.0
    deadline = t0 + args.seconds if args.seconds > 0 else None

    def shutdown_proc() -> None:
        stop_event.set()
        if proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                proc.kill()

    try:
        while rclpy.ok():
            if deadline is not None and time.monotonic() >= deadline:
                break

            chunk = os.read(proc.stdout.fileno(), max(1024, args.chunk_size))
            if not chunk:
                if proc.poll() is not None:
                    break
                continue

            pending.extend(chunk)
            for jpeg in find_jpeg_frames(pending, counters):
                now = time.monotonic()
                if last_frame_t > 0.0:
                    recent_intervals.append(now - last_frame_t)
                last_frame_t = now

                msg = CompressedImage()
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.header.frame_id = args.frame_id
                msg.format = "jpeg"
                msg.data = jpeg
                pub.publish(msg)
                counters.frames += 1
                counters.bytes_total += len(jpeg)

                if args.save_first_frame and not counters.saved_first_frame:
                    with open(os.path.expanduser(args.save_first_frame), "wb") as handle:
                        handle.write(jpeg)
                    counters.saved_first_frame = True
                    print(f"Saved first native JPEG frame to {args.save_first_frame}")

                rclpy.spin_once(node, timeout_sec=0.0)

                if now - last_report_t >= max(args.report_period, 0.25):
                    dt = now - last_report_t
                    frames_delta = counters.frames - last_report_frames
                    bytes_delta = counters.bytes_total - last_report_bytes
                    fps = frames_delta / dt if dt > 0 else 0.0
                    avg_kb = (bytes_delta / max(frames_delta, 1)) / 1024.0
                    interval_ms = 0.0
                    if recent_intervals:
                        interval_ms = (sum(recent_intervals) / len(recent_intervals)) * 1000.0
                    print(
                        "native_mjpeg | "
                        f"pub_fps={fps:.1f}, avg_jpeg={avg_kb:.1f} KB, "
                        f"avg_interval={interval_ms:.1f} ms, subscribers={pub.get_subscription_count()}, "
                        f"total_frames={counters.frames}"
                    )
                    last_report_t = now
                    last_report_frames = counters.frames
                    last_report_bytes = counters.bytes_total

    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        shutdown_proc()
        stderr_thread.join(timeout=1.0)
        elapsed = max(1e-6, time.monotonic() - t0)
        avg_fps = counters.frames / elapsed
        avg_kb = (counters.bytes_total / max(counters.frames, 1)) / 1024.0
        print("\nSummary:")
        print(f"  frames={counters.frames} elapsed={elapsed:.2f}s avg_pub_fps={avg_fps:.2f}")
        print(f"  avg_jpeg={avg_kb:.1f} KB total_mb={counters.bytes_total / (1024 * 1024):.2f}")
        print(f"  dropped_prefix_bytes={counters.dropped_prefix_bytes} leftover_buffer={len(pending)}")
        if stderr_lines:
            print("  recent v4l2-ctl stderr:")
            for line in stderr_lines:
                print(f"    {line}")
        node.destroy_node()
        rclpy.shutdown()

    return 0 if counters.frames > 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
PY