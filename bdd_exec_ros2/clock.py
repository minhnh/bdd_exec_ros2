# Copyright 2026 Minh Nguyen
# SPDX-License-Identifier: Apache-2.0
"""ROS simulation-clock reset notification and optional setup synchronization."""

import math
from collections.abc import Callable

from rclpy.clock import Clock, JumpThreshold
from rclpy.clock_type import ClockType
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.task import Future


class SimulationClockSync:
    """Report backward jumps and optionally await reset followed by progress.

    Call prepare_wait() before setup, wait_for_progress() afterward, and
    clear_wait() in finally. Monitoring remains active until close().
    """

    def __init__(self, node: Node, on_reset: Callable[[int], None] | None = None):
        self._on_reset = on_reset
        self._node = node
        self._clock = node.get_clock()
        self._future: Future | None = None
        self._timer = None
        self._reset_seen = False
        self._jump = self._clock.create_jump_callback(
            JumpThreshold(
                min_forward=Duration(nanoseconds=1),
                min_backward=Duration(nanoseconds=-1),
                on_clock_change=False,
            ),
            post_callback=self._on_jump,
        )

    def prepare_wait(self):
        """Create one setup wait and start recording reset evidence."""
        if self._jump is None:
            raise RuntimeError("Clock synchronization is closed")
        if self._future is not None:
            raise RuntimeError("Call clear_wait() before preparing another clock wait")
        self._reset_seen = False
        self._future = Future()

    def clear_wait(self):
        """Release setup wait resources without stopping reset monitoring."""
        future, timer = self._future, self._timer
        self._future = self._timer = None
        if timer is not None:
            self._node.destroy_timer(timer)
        if future is not None and not future.done():
            future.cancel()

    def _on_jump(self, jump):
        future = self._future
        if jump.delta.nanoseconds < 0:
            if future is not None:
                self._reset_seen = True
            if self._on_reset is not None:
                self._on_reset(self._clock.now().nanoseconds - jump.delta.nanoseconds)
        elif (
            jump.delta.nanoseconds > 0
            and self._timer is not None
            and self._reset_seen
            and future is not None
            and not future.done()
        ):
            future.set_result(None)

    def wait_for_progress(self, timeout: float) -> Future:
        """Await a post-setup forward update after an observed backward reset."""
        future = self._future
        if self._jump is None:
            raise RuntimeError("Clock synchronization is closed")
        if future is None or future.done() or self._timer is not None:
            raise RuntimeError("Call prepare_wait() before starting a new clock wait")
        if not math.isfinite(timeout) or timeout <= 0:
            raise ValueError("Clock wait timeout must be finite and positive")

        def timed_out():
            if not future.done():
                future.set_exception(
                    TimeoutError(
                        f"scene clock did not reset and advance within {timeout}s after starting simulation"
                    )
                )

        # Missing or paused simulation time must not disable the setup watchdog.
        self._timer = self._node.create_timer(
            timeout, timed_out, clock=Clock(clock_type=ClockType.STEADY_TIME)
        )
        return future

    def close(self):
        if self._jump is not None:
            self._jump.unregister()
            self._jump = None
        self.clear_wait()
