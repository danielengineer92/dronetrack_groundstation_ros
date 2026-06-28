"""Pure decision helpers for the autonomy manager.

Kept free of rclpy so the gate logic can be unit tested with plain Python
(see test/test_autonomy_logic.py). The node still owns all I/O and timing.
"""

from __future__ import annotations


def scan_yaw_authorized(
    *,
    allow_scan_without_lock: bool,
    mission_is_scan: bool,
    request_fresh: bool,
    safety_ok: bool,
    target_locked: bool,
) -> bool:
    """Whether to grant pre-lock, yaw-only autonomy for a SCAN mission step.

    This is the Option-B allowance: while the operator-authored mission is in a
    SCAN step and the vehicle is safe, let autonomy enable so the drone can
    yaw-sweep to *search* for a target, even though nothing is locked yet. It is
    strictly gated:

    - ``allow_scan_without_lock`` must be enabled (default OFF);
    - the autonomy request must be fresh and telemetry safety must pass;
    - a fresh, active SCAN mission command must be present;
    - the target must NOT already be locked (a locked target is normal TRACKING).

    Movement stays yaw-only because control_node, while the mission mode is SCAN,
    holds the captured local-NED anchor and only rotates — no translation.
    """
    if not allow_scan_without_lock:
        return False
    if target_locked:
        return False
    if not (request_fresh and safety_ok):
        return False
    return mission_is_scan
