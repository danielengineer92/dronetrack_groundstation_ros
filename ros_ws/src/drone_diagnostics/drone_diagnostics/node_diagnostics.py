"""Lightweight diagnostics helpers for ROS 2 Python nodes.

The goal is intentionally simple: every node should emit the same useful
heartbeat data without depending on heavyweight diagnostic_updater packages.
"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Optional


@dataclass
class _TopicStats:
    topic: str
    label: str
    direction: str
    stale_seconds: float
    expect_connections: bool = True
    count: int = 0
    last_time: Optional[float] = None
    last_summary: str = ""
    _last_rate_time: float = 0.0
    _last_rate_count: int = 0

    def mark(self, increment: int = 1, summary: str = "") -> None:
        self.count += increment
        self.last_time = time.monotonic()
        if summary:
            self.last_summary = summary

    def rate(self, now: float) -> float:
        if self._last_rate_time <= 0.0:
            self._last_rate_time = now
            self._last_rate_count = self.count
            return 0.0

        elapsed = now - self._last_rate_time
        if elapsed <= 0.0:
            return 0.0

        rate = (self.count - self._last_rate_count) / elapsed
        self._last_rate_time = now
        self._last_rate_count = self.count
        return rate

    def age(self, now: float) -> Optional[float]:
        if self.last_time is None:
            return None
        return now - self.last_time


class NodeDiagnostics:
    """Heartbeat, topic rate, stale-data, and connection logging for one node."""

    def __init__(
        self,
        node,
        *,
        heartbeat_period: Optional[float] = None,
        stale_seconds: Optional[float] = None,
        connection_check: Optional[bool] = None,
    ) -> None:
        self.node = node
        self._inputs: dict[str, _TopicStats] = {}
        self._outputs: dict[str, _TopicStats] = {}
        self._start_time = time.monotonic()

        self._safe_declare_parameter("diagnostic_heartbeat_period", 5.0)
        self._safe_declare_parameter("diagnostic_stale_seconds", 2.0)
        self._safe_declare_parameter("diagnostic_connection_check", True)

        self.heartbeat_period = float(
            heartbeat_period
            if heartbeat_period is not None
            else self.node.get_parameter("diagnostic_heartbeat_period").value
        )
        self.default_stale_seconds = float(
            stale_seconds
            if stale_seconds is not None
            else self.node.get_parameter("diagnostic_stale_seconds").value
        )
        self.connection_check = bool(
            connection_check
            if connection_check is not None
            else self.node.get_parameter("diagnostic_connection_check").value
        )

        if self.heartbeat_period > 0.0:
            self._timer = self.node.create_timer(self.heartbeat_period, self.report)
        else:
            self._timer = None

        self.node.get_logger().info(
            f"Diagnostics enabled | heartbeat={self.heartbeat_period:.1f}s, "
            f"stale>{self.default_stale_seconds:.1f}s, connection_check={self.connection_check}"
        )

    def _safe_declare_parameter(self, name: str, default_value) -> None:
        try:
            self.node.declare_parameter(name, default_value)
        except Exception:
            # Parameter may have been declared by the node or by another helper.
            pass

    def add_input(
        self,
        topic: str,
        label: str = "input",
        *,
        stale_seconds: Optional[float] = None,
        expect_publishers: bool = True,
    ) -> None:
        self._inputs[topic] = _TopicStats(
            topic=topic,
            label=label,
            direction="IN",
            stale_seconds=float(stale_seconds if stale_seconds is not None else self.default_stale_seconds),
            expect_connections=expect_publishers,
        )
        self.node.get_logger().info(f"Subscribed diagnostic watch | {label}: {topic}")

    def add_output(
        self,
        topic: str,
        label: str = "output",
        *,
        stale_seconds: Optional[float] = None,
        expect_subscribers: bool = True,
    ) -> None:
        self._outputs[topic] = _TopicStats(
            topic=topic,
            label=label,
            direction="OUT",
            stale_seconds=float(stale_seconds if stale_seconds is not None else self.default_stale_seconds),
            expect_connections=expect_subscribers,
        )
        self.node.get_logger().info(f"Publisher diagnostic watch | {label}: {topic}")

    def mark_received(self, topic: str, *, summary: str = "", count: int = 1) -> None:
        stats = self._inputs.get(topic)
        if stats is not None:
            stats.mark(count, summary)

    def mark_published(self, topic: str, *, summary: str = "", count: int = 1) -> None:
        stats = self._outputs.get(topic)
        if stats is not None:
            stats.mark(count, summary)

    def age_seconds(self, topic: str) -> Optional[float]:
        now = time.monotonic()
        stats = self._inputs.get(topic) or self._outputs.get(topic)
        if stats is None:
            return None
        return stats.age(now)

    def format_age(self, topic: str) -> str:
        age = self.age_seconds(topic)
        if age is None:
            return "never"
        return f"{age:.2f}s"

    def report(self) -> None:
        now = time.monotonic()
        uptime = now - self._start_time
        parts: list[str] = [f"uptime={uptime:.1f}s"]
        warn_parts: list[str] = []

        for stats in self._inputs.values():
            rate = stats.rate(now)
            age = stats.age(now)
            publishers = self.node.count_publishers(stats.topic) if self.connection_check else -1
            age_text = "never" if age is None else f"{age:.2f}s"
            conn_text = f", publishers={publishers}" if self.connection_check else ""
            extra = f", {stats.last_summary}" if stats.last_summary else ""
            parts.append(
                f"{stats.direction}:{stats.label} count={stats.count}, rate={rate:.1f}Hz, "
                f"age={age_text}{conn_text}{extra}"
            )

            if self.connection_check and stats.expect_connections and publishers == 0:
                warn_parts.append(f"no publisher for {stats.topic}")
            if age is None:
                warn_parts.append(f"no data yet on {stats.topic}")
            elif age > stats.stale_seconds:
                warn_parts.append(f"stale {stats.topic}: age={age:.2f}s > {stats.stale_seconds:.2f}s")

        for stats in self._outputs.values():
            rate = stats.rate(now)
            subscribers = self.node.count_subscribers(stats.topic) if self.connection_check else -1
            conn_text = f", subscribers={subscribers}" if self.connection_check else ""
            extra = f", {stats.last_summary}" if stats.last_summary else ""
            parts.append(
                f"{stats.direction}:{stats.label} count={stats.count}, rate={rate:.1f}Hz{conn_text}{extra}"
            )

            if self.connection_check and stats.expect_connections and subscribers == 0:
                warn_parts.append(f"no subscriber for {stats.topic}")

        message = "Diagnostics heartbeat | " + " | ".join(parts)
        if warn_parts:
            self.node.get_logger().warning(message + " | WARN: " + "; ".join(warn_parts))
        else:
            self.node.get_logger().info(message + " | OK")
