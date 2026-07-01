#!/usr/bin/env python3
"""Mission state-sequence integration test.

Subscribes to /drone/mission/state and asserts the mission executor walks through
the expected sequence when autonomy + mission are requested against a running
SITL stack.

Prerequisites:
  1. PX4 SITL running (docker or native): `sudo docker run --rm -it --network host px4io/px4-sitl:latest`
  2. SITL mission stack running: `bash scripts/sim_sitl.sh`
  3. PX4 armed: `commander arm` in the pxh> shell (set `param set COM_DISARM_PRFLT -1` first)

Usage:
  # In a shell sourced with `source scripts/env.sh sim`:
  python3 tests/test_mission_states.py

The test publishes autonomy+mission requests, then waits up to TIMEOUT_S for
each expected state transition. Exits 0 on success, 1 on timeout/wrong sequence.
"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String

EXPECTED_SEQUENCE = [
    "IDLE",
    "PREFLIGHT",
    "TAKEOFF",
    "PRIME_OFFBOARD",
    "TRACK_CENTER",
    "DO_ORBIT",
    "LAND",
    "COMPLETE",
]

# States the executor only emits under some conditions (e.g. PREFLIGHT is skipped
# when preflight checks pass instantly). The sequence matcher may skip these.
OPTIONAL_STATES = frozenset({"PREFLIGHT"})

TIMEOUT_S = 120.0
SETTLE_S = 3.0


class MissionStateChecker(Node):
    def __init__(self) -> None:
        super().__init__("mission_state_checker")

        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        self.autonomy_pub = self.create_publisher(Bool, "/drone/autonomy/request", qos)
        self.mission_pub = self.create_publisher(Bool, "/drone/mission/request", qos)

        self.create_subscription(String, "/drone/mission/state", self._on_state, qos)

        self.observed: list[str] = []
        self.expected_idx = 0
        self.start_time = 0.0
        self.triggered = False
        self.done = False
        self.success = False

    def _on_state(self, msg: String) -> None:
        # The executor publishes "NAME: detail" (e.g. "TAKEOFF: takeoff
        # requested, altitude=..."); match on the NAME only.
        state = msg.data.split(":", 1)[0].strip().upper()
        if not state:
            return

        if self.observed and self.observed[-1] == state:
            return

        self.observed.append(state)
        self.get_logger().info(f"State transition: {state}")

        # Advance through the expected sequence, skipping optional states this
        # mission plan didn't emit. Interleaved/unexpected states are ignored
        # (we simply keep waiting for the next required state).
        while self.expected_idx < len(EXPECTED_SEQUENCE):
            expected = EXPECTED_SEQUENCE[self.expected_idx]
            if state == expected:
                self.expected_idx += 1
                break
            if expected in OPTIONAL_STATES:
                self.expected_idx += 1  # optional state not seen; skip it
                continue
            break

        if self.expected_idx >= len(EXPECTED_SEQUENCE):
            self.get_logger().info("All expected states observed!")
            self.done = True
            self.success = True

    def trigger_mission(self) -> None:
        if self.triggered:
            return
        self.triggered = True
        self.start_time = time.monotonic()
        self.get_logger().info("Publishing autonomy_request=true, mission_request=true")
        self.autonomy_pub.publish(Bool(data=True))
        time.sleep(0.5)
        self.mission_pub.publish(Bool(data=True))

    def check_timeout(self) -> bool:
        if self.start_time > 0 and time.monotonic() - self.start_time > TIMEOUT_S:
            return True
        return False


def main() -> int:
    rclpy.init()
    node = MissionStateChecker()

    node.get_logger().info(f"Waiting {SETTLE_S}s for node discovery...")
    start = time.monotonic()
    while time.monotonic() - start < SETTLE_S:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.trigger_mission()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=0.25)
        if node.check_timeout():
            node.get_logger().error(
                f"TIMEOUT after {TIMEOUT_S}s. "
                f"Observed: {node.observed}. "
                f"Expected next: {EXPECTED_SEQUENCE[node.expected_idx] if node.expected_idx < len(EXPECTED_SEQUENCE) else 'DONE'}"
            )
            break

    node.destroy_node()
    rclpy.try_shutdown()

    if node.success:
        print(f"\nPASS: Mission walked through expected sequence: {' -> '.join(EXPECTED_SEQUENCE)}")
        return 0
    else:
        print(f"\nFAIL: Observed states: {' -> '.join(node.observed)}")
        print(f"Expected sequence: {' -> '.join(EXPECTED_SEQUENCE)}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
