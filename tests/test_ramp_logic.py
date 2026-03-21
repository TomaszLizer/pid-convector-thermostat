"""Tests for the ramp tick dead-zone handling logic.

The ramp logic lives in climate.py's _async_ramp_tick method. Since that
method depends on Home Assistant, we extract and replicate the core ramp
algorithm here for isolated testing.
"""

import pytest

# Default constants matching the real component
DEFAULT_RAMP_INTERVAL = 2.0  # seconds


def ramp_tick(current, target, ramp_rate, dead_zone, output_min):
    """Pure-function replica of _async_ramp_tick's core logic.

    Returns (new_actual_output, fan_speed_sent_to_hardware).
    fan_speed_sent_to_hardware applies the output_min clamp.
    """
    if abs(target - current) < 0.5:
        # Close enough, snap to target
        fan_speed = target
        if target > 0 and output_min > 0:
            fan_speed = max(target, output_min)
        return target, fan_speed

    # Calculate step for this tick
    max_step = ramp_rate * DEFAULT_RAMP_INTERVAL
    if target > current:
        new_output = min(current + max_step, target)
    else:
        new_output = max(current - max_step, target)

    # Handle intermediate values inside the dead zone range (THE FIX)
    if 0 < new_output < dead_zone:
        if target == 0:
            if output_min > 0:
                # Ramping to off: allow smooth descent through dead zone.
                if current <= output_min + 0.5:
                    # Already at output_min — final off step
                    new_output = 0
                else:
                    # Not yet at output_min — clamp for soft landing
                    new_output = output_min
            else:
                # No output_min — let ramp step naturally
                pass
        else:
            new_output = dead_zone

    new_output = round(new_output, 1)

    # Simulate _async_set_fan_speed's output_min enforcement
    fan_speed = new_output
    if fan_speed > 0 and output_min > 0:
        fan_speed = round(max(fan_speed, output_min), 1)

    # _async_set_fan_speed overwrites _actual_output with clamped value
    actual = fan_speed if fan_speed > 0 else 0.0
    return actual, fan_speed


def simulate_ramp(start, target, ramp_rate=3.0, dead_zone=15.0,
                  output_min=10.0, max_ticks=50):
    """Simulate multiple ramp ticks and return the sequence of fan speeds."""
    current = start
    history = [current]
    for _ in range(max_ticks):
        if current == target:
            break
        current, fan_speed = ramp_tick(current, target, ramp_rate,
                                       dead_zone, output_min)
        history.append(fan_speed)
    return history


class TestRampShutdown:
    """Test smooth ramp-down to zero (the main fix)."""

    def test_shutdown_from_45_percent(self):
        """Fan at 45% ramping to 0 should go through output_min before off."""
        seq = simulate_ramp(45.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        # Should end at 0
        assert seq[-1] == 0.0

        # Should pass through output_min (10) at least once before off
        assert 10.0 in seq

        # The step before 0 should be output_min
        off_idx = seq.index(0.0)
        assert seq[off_idx - 1] == 10.0

        # Should never have a jump larger than max_step (6) except the
        # dead_zone clamping steps
        for i in range(1, len(seq)):
            step = abs(seq[i] - seq[i - 1])
            # Allow up to dead_zone jump (for the soft-landing clamp)
            assert step <= 15.0, f"Step from {seq[i-1]} to {seq[i]} = {step}"

    def test_shutdown_from_25_percent(self):
        """Fan at 25% ramping to 0."""
        seq = simulate_ramp(25.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 0.0
        assert 10.0 in seq
        # Expected: 25 → 19 → 10 → 0
        assert len(seq) == 4  # start + 3 ticks

    def test_shutdown_from_15_percent(self):
        """Fan at 15% (at dead zone) ramping to 0."""
        seq = simulate_ramp(15.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 0.0
        assert 10.0 in seq
        # Expected: 15 → 10 → 0
        assert len(seq) == 3

    def test_shutdown_from_output_min(self):
        """Fan at output_min ramping to 0 should turn off in one tick."""
        seq = simulate_ramp(10.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 0.0
        # Expected: 10 → 0 (one tick)
        assert len(seq) == 2

    def test_no_stuck_at_output_min(self):
        """Ramp should not get stuck at output_min forever."""
        seq = simulate_ramp(45.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        # Count how many times output_min appears (should be exactly 1)
        count_at_min = sum(1 for v in seq[1:] if v == 10.0)
        assert count_at_min == 1, f"Stuck at output_min: {seq}"

    def test_shutdown_completes_in_reasonable_time(self):
        """From 45% to off should take < 20 ticks (40 seconds)."""
        seq = simulate_ramp(45.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        ticks = len(seq) - 1  # exclude start
        assert ticks <= 20
        total_time = ticks * DEFAULT_RAMP_INTERVAL
        assert total_time <= 40.0


class TestRampStartup:
    """Test ramp-up from 0 to target."""

    def test_startup_to_30_percent(self):
        """Fan at 0% ramping to 30% should start at dead_zone."""
        seq = simulate_ramp(0.0, 30.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 30.0
        # First non-zero speed should be dead_zone (15), not 6
        assert seq[1] == 15.0

    def test_startup_to_20_percent(self):
        """Fan at 0% ramping to 20%."""
        seq = simulate_ramp(0.0, 20.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 20.0
        assert seq[1] == 15.0  # dead zone floor

    def test_startup_to_50_percent(self):
        """Fan at 0% ramping to 50%."""
        seq = simulate_ramp(0.0, 50.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 50.0
        assert seq[1] == 15.0  # starts at dead zone


class TestRampSteadyState:
    """Test ramp between two non-zero targets."""

    def test_ramp_down_within_range(self):
        """From 50% to 30% — should step smoothly."""
        seq = simulate_ramp(50.0, 30.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 30.0
        # All values should be above dead zone
        for v in seq:
            assert v >= 15.0 or v == 0.0

    def test_ramp_up_within_range(self):
        """From 20% to 40% — should step smoothly."""
        seq = simulate_ramp(20.0, 40.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 40.0
        for v in seq:
            assert v >= 15.0 or v == 0.0


class TestOldBugRegression:
    """Regression tests: the old code would snap to 0 in the dead zone."""

    def test_no_snap_from_19_to_0(self):
        """Old bug: 19% → ramp step = 13 (in DZ) → snapped to 0.
        Now should go 19 → 10 → 0."""
        seq = simulate_ramp(19.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        # Should not have a direct 19→0 jump
        assert seq == [19.0, 10.0, 0.0]

    def test_no_snap_from_25_to_0(self):
        """Old bug: 25→19→0 (snapped). Now: 25→19→10→0."""
        seq = simulate_ramp(25.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=10.0)

        # Must visit output_min before off
        off_idx = seq.index(0.0)
        assert seq[off_idx - 1] == 10.0

    def test_higher_ramp_rate_still_soft_lands(self):
        """Even with ramp_rate=5 (step=10), should soft-land at output_min."""
        seq = simulate_ramp(45.0, 0.0, ramp_rate=5.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 0.0
        off_idx = seq.index(0.0)
        assert seq[off_idx - 1] == 10.0

    def test_ramp_rate_10_aggressive(self):
        """Very aggressive ramp_rate=10 (step=20). Still soft-lands."""
        seq = simulate_ramp(45.0, 0.0, ramp_rate=10.0,
                            dead_zone=15.0, output_min=10.0)

        assert seq[-1] == 0.0
        off_idx = seq.index(0.0)
        assert seq[off_idx - 1] == 10.0


class TestEdgeCases:
    """Edge cases for the ramp logic."""

    def test_output_min_zero(self):
        """When output_min=0, no soft landing — ramp steps naturally to 0."""
        seq = simulate_ramp(30.0, 0.0, ramp_rate=3.0,
                            dead_zone=15.0, output_min=0.0)

        assert seq[-1] == 0.0
        # Each step should be at most max_step (6)
        for i in range(1, len(seq)):
            assert abs(seq[i] - seq[i - 1]) <= 6.1  # allow rounding

    def test_dead_zone_zero(self):
        """When dead_zone=0, dead zone logic never triggers (0 < x < 0 is never true).
        In practice, dead_zone=0 means the fan would never enter the dead zone
        path and ramp handles it purely by stepping."""
        seq = simulate_ramp(30.0, 0.0, ramp_rate=3.0,
                            dead_zone=0.0, output_min=0.0)  # both 0 for clean test

        assert seq[-1] == 0.0

    def test_already_at_target(self):
        """When current == target, no change."""
        seq = simulate_ramp(30.0, 30.0)
        assert seq == [30.0]

    def test_snap_when_close(self):
        """When diff < 0.5, snap to target."""
        actual, fan = ramp_tick(30.3, 30.0, 3.0, 15.0, 10.0)
        assert actual == 30.0
