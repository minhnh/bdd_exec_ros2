# Copyright 2026 Minh Nguyen
# SPDX-License-Identifier: Apache-2.0
from types import SimpleNamespace

import pytest
from rclpy.clock import ROSClock
from rclpy.time import Time as RosTime

from bdd_exec_ros2.clock import SimulationClockSync


@pytest.fixture
def clock_wait():
    clock = ROSClock()
    clock._set_ros_time_is_active(True)
    clock.set_ros_time_override(RosTime(seconds=100))
    callbacks, destroyed, resets = [], [], []

    def timer(period, callback, **kwargs):
        assert kwargs["clock"].clock_type.name == "STEADY_TIME"
        callbacks.append(callback)
        return callback

    node = SimpleNamespace(
        get_clock=lambda: clock, create_timer=timer, destroy_timer=destroyed.append
    )
    sync = SimulationClockSync(node, on_reset=resets.append)
    yield sync, clock, callbacks, destroyed, resets
    sync.close()


@pytest.mark.parametrize("reset_during_setup", [False, True])
def test_wait_requires_reset_and_post_setup_progress(clock_wait, reset_during_setup):
    sync, clock, _, destroyed, resets = clock_wait
    sync.prepare_wait()
    if reset_during_setup:
        clock.set_ros_time_override(RosTime())
        clock.set_ros_time_override(RosTime(seconds=0.05))
    future = sync.wait_for_progress(1)
    assert not future.done()  # Progress during setup cannot complete the wait.
    if not reset_during_setup:
        clock.set_ros_time_override(RosTime(seconds=101))
        assert not future.done()
        clock.set_ros_time_override(RosTime())
        assert not future.done()
    clock.set_ros_time_override(RosTime(seconds=0.1))
    assert future.done() and future.result() is None
    assert resets == [(100 if reset_during_setup else 101) * 1_000_000_000]
    sync.clear_wait()
    assert len(destroyed) == 1


@pytest.mark.parametrize("initial_time", [0, 0.01])
def test_zero_or_near_zero_does_not_establish_reset(clock_wait, initial_time):
    sync, clock, callbacks, destroyed, _ = clock_wait
    clock.set_ros_time_override(RosTime(seconds=initial_time))
    sync.prepare_wait()
    future = sync.wait_for_progress(1)
    clock.set_ros_time_override(RosTime(seconds=0.02))
    assert not future.done()
    callbacks[-1]()
    with pytest.raises(TimeoutError, match="scene clock"):
        future.result()
    sync.clear_wait()
    sync.clear_wait()
    assert len(destroyed) == 1


def test_wait_lifecycle_and_old_timeout(clock_wait):
    sync, clock, callbacks, _, resets = clock_wait
    with pytest.raises(RuntimeError, match="prepare_wait"):
        sync.wait_for_progress(1)
    clock.set_ros_time_override(RosTime(seconds=99))
    assert resets == [100_000_000_000]  # Monitoring does not require a prepared wait.
    sync.prepare_wait()
    with pytest.raises(RuntimeError, match="clear_wait"):
        sync.prepare_wait()
    with pytest.raises(ValueError, match="positive"):
        sync.wait_for_progress(0)
    first = sync.wait_for_progress(1)
    with pytest.raises(RuntimeError, match="prepare_wait"):
        sync.wait_for_progress(1)
    clock.set_ros_time_override(RosTime(seconds=98))
    clock.set_ros_time_override(RosTime(seconds=98.1))
    assert first.done()
    with pytest.raises(RuntimeError, match="prepare_wait"):
        sync.wait_for_progress(1)
    sync.clear_wait()
    sync.prepare_wait()
    second = sync.wait_for_progress(1)
    callbacks[0]()  # A queued callback from the previous wait must not affect this one.
    assert not second.done()
    sync.clear_wait()
    assert second.cancelled()
    with pytest.raises(RuntimeError, match="prepare_wait"):
        sync.wait_for_progress(1)
    sync.close()
    sync.close()
    with pytest.raises(RuntimeError, match="closed"):
        sync.wait_for_progress(1)
    with pytest.raises(RuntimeError, match="closed"):
        sync.prepare_wait()
