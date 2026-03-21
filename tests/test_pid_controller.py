"""Tests for PID controller — focused on derivative timing behavior.

These tests mock time.time() to control timing precisely and verify
that the derivative term behaves correctly under various scenarios:
- Normal operation with periodic sensor changes
- Keep-alive ticks between sensor changes (same temp repeated)
- Startup with seed_setpoint
- Rapid sensor replay during HA startup
"""

import sys
import os
from unittest.mock import patch

import pytest

# Add the custom component to the path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'custom_components', 'pid_convector_thermostat'))
from pid_controller import PID


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def make_pid(**overrides):
    """Create a PID with production-like defaults."""
    defaults = dict(
        kp=15.0, ki=0.01, kd=300.0, ke=0.0,
        out_min=0.0, out_max=70.0,
        sampling_period=0, min_deriv_dt=2.0,
    )
    defaults.update(overrides)
    return PID(**defaults)


# ---------------------------------------------------------------------------
# Basic derivative behavior
# ---------------------------------------------------------------------------

class TestDerivativeBasics:
    """Verify derivative computes correctly when sensor changes."""

    @patch('pid_controller.time')
    def test_derivative_on_temperature_drop(self, mock_time):
        """Derivative should be positive when temperature drops (opposing the change)."""
        pid = make_pid(kp=0, ki=0, kd=300)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(19.9, 20.0)

        # D = -(Kd * input_diff) / dt = -(300 * -0.1) / 10 = 3.0
        assert pid.derivative == pytest.approx(3.0, abs=0.01), \
            f"Expected D=3.0, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_derivative_on_temperature_rise(self, mock_time):
        """Derivative should be negative when temperature rises."""
        pid = make_pid(kp=0, ki=0, kd=300)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(20.1, 20.0)

        # D = -(300 * 0.1) / 10 = -3.0
        assert pid.derivative == pytest.approx(-3.0, abs=0.01), \
            f"Expected D=-3.0, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_derivative_zero_when_temp_unchanged(self, mock_time):
        """Derivative should stay 0 if temperature never changes."""
        pid = make_pid(kp=0, ki=0, kd=300)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 20.0
        pid.calc(20.0, 20.0)

        assert pid.derivative == pytest.approx(0.0, abs=0.001)


# ---------------------------------------------------------------------------
# Keep-alive tick behavior (the critical real-world pattern)
# ---------------------------------------------------------------------------

class TestKeepAliveTicks:
    """Simulate HA's keep-alive pattern: calc() called every 10s,
    but sensor only changes every ~60s.

    The derivative should:
    1. Update when sensor actually changes
    2. HOLD its value on ticks where sensor is unchanged
    3. Use calc dt (time since last calc) for the derivative computation
    """

    @patch('pid_controller.time')
    def test_derivative_holds_between_sensor_changes(self, mock_time):
        """D should hold its value across keep-alive ticks with no sensor change."""
        pid = make_pid(kp=0, ki=0, kd=300)

        # t=0: first reading
        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        # t=10: temp changes — derivative should compute
        mock_time.return_value = 10.0
        pid.calc(19.9, 20.0)
        d_after_change = pid.derivative
        assert d_after_change != 0, "Derivative should be non-zero after temp change"

        # t=20, t=30: keep-alive ticks, same temp — derivative should HOLD
        mock_time.return_value = 20.0
        pid.calc(19.9, 20.0)
        assert pid.derivative == d_after_change, \
            f"D should hold at {d_after_change}, got {pid.derivative}"

        mock_time.return_value = 30.0
        pid.calc(19.9, 20.0)
        assert pid.derivative == d_after_change, \
            f"D should still hold at {d_after_change}, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_derivative_updates_on_next_sensor_change(self, mock_time):
        """After holding, D should update when sensor changes again."""
        pid = make_pid(kp=0, ki=0, kd=300)

        # t=0: first reading
        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        # t=10: temp drops to 19.9 — D kicks in
        mock_time.return_value = 10.0
        pid.calc(19.9, 20.0)
        d1 = pid.derivative

        # t=20, t=30, t=40, t=50: keep-alive ticks, same temp
        for t in [20, 30, 40, 50]:
            mock_time.return_value = float(t)
            pid.calc(19.9, 20.0)

        # t=60: sensor changes to 19.8
        # dt = 60 - 50 = 10 (time since last calc)
        mock_time.return_value = 60.0
        pid.calc(19.8, 20.0)

        # D = -(300 * (19.8 - 19.9)) / 10 = -(300 * -0.1) / 10 = 3.0
        assert pid.derivative == pytest.approx(3.0, abs=0.01), \
            f"Expected D=3.0, got {pid.derivative} (dt={pid.dt})"
        assert pid.dt == pytest.approx(10.0, abs=0.1), \
            f"dt should be 10s, got {pid.dt}"

    @patch('pid_controller.time')
    def test_full_realistic_sequence(self, mock_time):
        """Simulate a realistic 3-minute sequence with 10s ticks and sensor updates at ~60s."""
        pid = make_pid(kp=15, ki=0.01, kd=300, ke=0)

        # Temp sequence: sensor reports every 60s, ticks every 10s
        readings = {
            0: 20.0,
            60: 19.9,
            120: 19.8,
        }

        results = []
        for t in range(0, 130, 10):
            mock_time.return_value = float(t)
            temp = readings.get(t, None)
            if temp is None:
                temp = max((k for k in readings if k <= t), key=lambda k: k)
                temp = readings[temp]
            output, updated = pid.calc(temp, 20.0)
            results.append({
                't': t, 'temp': temp, 'output': output,
                'p': pid.proportional, 'i': pid.integral,
                'd': pid.derivative, 'dt': pid.dt,
            })

        # After t=60, derivative should be non-zero
        r60 = next(r for r in results if r['t'] == 60)
        assert r60['d'] != 0, f"At t=60 (temp changed 20.0->19.9), D should be non-zero, got {r60['d']}"

        # Between t=70 and t=110, derivative should hold its value from t=60
        for r in results:
            if 70 <= r['t'] <= 110:
                assert r['d'] == r60['d'], \
                    f"At t={r['t']}, D should hold at {r60['d']}, got {r['d']}"

        # At t=120, derivative should update with new sensor reading
        r120 = next(r for r in results if r['t'] == 120)
        assert r120['d'] != 0, "At t=120, D should be non-zero"


# ---------------------------------------------------------------------------
# Startup / seed behavior
# ---------------------------------------------------------------------------

class TestStartupSeed:
    """Test seed_setpoint() and derivative suppression during startup."""

    @patch('pid_controller.time')
    def test_seeded_startup_no_derivative_spike(self, mock_time):
        """After seed, the first sensor event should not spike derivative.
        The seed sets _input but _last_input=None, so derivative is suppressed."""
        pid = make_pid(kp=0, ki=0, kd=300)
        pid.seed_setpoint(20.0, 20.1)

        # First calc — same as seeded value, no change
        mock_time.return_value = 100.0
        pid.calc(20.1, 20.0)
        assert pid.derivative == 0.0

        # Second calc — value changes, but _last_input was seeded (20.1)
        # dt = 0.05s -> effective_dt = max(0.05, 2.0) = 2.0
        # input_diff = 20.72 - 20.1 = 0.62
        # D = -(300 * 0.62) / 2.0 = -93  <- still large but clamped by min_deriv_dt
        mock_time.return_value = 100.05
        pid.calc(20.72, 20.0)
        # The derivative fires here but is clamped by min_deriv_dt
        d_val = pid.derivative
        # Should NOT be as extreme as -(300 * 0.62) / 0.05 = -3720
        assert abs(d_val) < 100, \
            f"With min_deriv_dt=2.0, D should be clamped, got {d_val}"

    @patch('pid_controller.time')
    def test_seeded_startup_rapid_replay_clamped(self, mock_time):
        """Rapid startup replay: multiple sensor values in quick succession.
        min_deriv_dt should clamp derivative magnitude."""
        pid = make_pid(kp=0, ki=0, kd=300)
        pid.seed_setpoint(20.0, 20.0)

        # Rapid replay: 3 events in 100ms
        mock_time.return_value = 100.0
        pid.calc(20.0, 20.0)  # same as seed, no change
        assert pid.derivative == 0.0

        mock_time.return_value = 100.05
        pid.calc(20.5, 20.0)  # _last_input=20.0 (seeded), diff=0.5
        # D = -(300 * 0.5) / max(0.05, 2.0) = -75
        assert abs(pid.derivative) <= 75.1

        mock_time.return_value = 100.10
        pid.calc(20.1, 20.0)  # _last_input=20.5, diff=-0.4
        # D = -(300 * -0.4) / max(0.05, 2.0) = 60
        assert abs(pid.derivative) <= 60.1

    @patch('pid_controller.time')
    def test_derivative_activates_after_seed_plus_two_readings(self, mock_time):
        """After seed + 2 real sensor changes with good spacing, derivative works."""
        pid = make_pid(kp=0, ki=0, kd=300)
        pid.seed_setpoint(20.0, 20.0)

        # First change at t=10 (good spacing from seed)
        mock_time.return_value = 10.0
        pid.calc(20.1, 20.0)  # _last_input=20.0 (seeded)
        # D = -(300 * 0.1) / 10 = -3.0 (but _last_calc_time is None from seed, so dt=0)
        # Actually: seed doesn't set _last_calc_time, so this is the first calc
        # _last_calc_time = None, dt = 0, derivative branch skipped

        # Second change at t=20
        mock_time.return_value = 20.0
        pid.calc(19.9, 20.0)  # _last_input=20.1, dt=10
        # D = -(300 * -0.2) / 10 = 6.0
        assert pid.derivative == pytest.approx(6.0, abs=0.1), \
            f"After seed + 2 readings, D should work, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_no_seed_derivative_works_after_two_readings(self, mock_time):
        """Without seed, derivative should work after 2 readings."""
        pid = make_pid(kp=0, ki=0, kd=300)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(19.9, 20.0)

        # D = -(300 * -0.1) / 10 = 3.0
        assert pid.derivative == pytest.approx(3.0, abs=0.01), \
            f"Without seed, D should work after 2 readings, got {pid.derivative}"


# ---------------------------------------------------------------------------
# dt timing
# ---------------------------------------------------------------------------

class TestDtTiming:
    """Test that dt is correctly the time between calc() calls."""

    @patch('pid_controller.time')
    def test_dt_is_calc_interval(self, mock_time):
        """dt should be time between calc() calls."""
        pid = make_pid()

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(20.0, 20.0)
        assert pid.dt == pytest.approx(10.0, abs=0.1)

        mock_time.return_value = 20.0
        pid.calc(20.0, 20.0)
        assert pid.dt == pytest.approx(10.0, abs=0.1)

    @patch('pid_controller.time')
    def test_dt_varies_with_call_frequency(self, mock_time):
        """dt follows actual time between calls."""
        pid = make_pid()

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 5.0
        pid.calc(20.0, 20.0)
        assert pid.dt == pytest.approx(5.0, abs=0.1)

        mock_time.return_value = 25.0
        pid.calc(20.0, 20.0)
        assert pid.dt == pytest.approx(20.0, abs=0.1)


# ---------------------------------------------------------------------------
# Min deriv dt floor
# ---------------------------------------------------------------------------

class TestMinDerivDt:
    """Test that the min_deriv_dt floor clamps rapid calc() calls."""

    @patch('pid_controller.time')
    def test_rapid_sensor_changes_clamped(self, mock_time):
        """Two calc() calls 0.5s apart should use min_deriv_dt=2.0 for derivative."""
        pid = make_pid(kp=0, ki=0, kd=300, min_deriv_dt=2.0)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 0.5
        pid.calc(19.9, 20.0)

        # Without floor: D = -(300 * -0.1) / 0.5 = 60
        # With floor (2.0): D = -(300 * -0.1) / 2.0 = 15
        assert pid.derivative == pytest.approx(15.0, abs=0.1), \
            f"With min_deriv_dt=2.0, D should be 15.0, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_normal_spacing_not_clamped(self, mock_time):
        """Sensor changes 10s apart should NOT be clamped."""
        pid = make_pid(kp=0, ki=0, kd=300, min_deriv_dt=2.0)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(19.9, 20.0)

        # D = -(300 * -0.1) / 10 = 3.0 (no clamping)
        assert pid.derivative == pytest.approx(3.0, abs=0.01), \
            f"D should be 3.0 (no clamping), got {pid.derivative}"


# ---------------------------------------------------------------------------
# The actual HA scenario: sensor event + keep-alive interleaved
# ---------------------------------------------------------------------------

class TestHARealisticScenario:
    """Simulate the real HA behavior:
    - Sensor fires state_changed event (triggers calc immediately)
    - Keep-alive timer fires every 10s (triggers calc with same temp)
    - Sensor changes every ~30-90s
    """

    @patch('pid_controller.time')
    def test_sensor_event_then_keepalive_ticks(self, mock_time):
        """Sensor event at t=0, then keep-alive at t=10, t=20.
        Sensor changes at t=25 (event-driven, not aligned to timer)."""
        pid = make_pid(kp=0, ki=0, kd=300)

        # t=0: sensor event — 20.0
        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        # t=10: keep-alive tick — same temp
        mock_time.return_value = 10.0
        pid.calc(20.0, 20.0)
        assert pid.derivative == 0.0, "D should be 0 (no temp change yet)"

        # t=20: keep-alive tick — same temp
        mock_time.return_value = 20.0
        pid.calc(20.0, 20.0)
        assert pid.derivative == 0.0

        # t=25: sensor event — temp changed to 19.9
        mock_time.return_value = 25.0
        pid.calc(19.9, 20.0)

        # dt = 25 - 20 = 5 (time since last calc)
        # D = -(300 * -0.1) / 5 = 6.0
        assert pid.dt == pytest.approx(5.0, abs=0.1), \
            f"dt should be 5, got {pid.dt}"
        assert pid.derivative == pytest.approx(6.0, abs=0.01), \
            f"D should be 6.0, got {pid.derivative}"

        # t=30: keep-alive tick — same temp, D should hold
        mock_time.return_value = 30.0
        pid.calc(19.9, 20.0)
        assert pid.derivative == pytest.approx(6.0, abs=0.01), \
            f"D should hold at 6.0, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_warmup_then_normal_operation(self, mock_time):
        """Full startup-to-steady-state scenario with seed."""
        pid = make_pid(kp=15, ki=0, kd=300)
        pid.seed_setpoint(20.0, 20.0)

        # t=0: first calc after seed — same temp, no derivative
        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)
        assert pid.derivative == 0.0, "D=0, same as seed"

        # t=10: keep-alive, same temp
        mock_time.return_value = 10.0
        pid.calc(20.0, 20.0)
        assert pid.derivative == 0.0

        # t=20: sensor change to 19.9
        mock_time.return_value = 20.0
        pid.calc(19.9, 20.0)

        # dt = 10, input_diff = 19.9 - 20.0 = -0.1
        # D = -(300 * -0.1) / 10 = 3.0
        assert pid.derivative == pytest.approx(3.0, abs=0.1), \
            f"D should be 3.0, got {pid.derivative}"

        # Keep-alive ticks — D holds
        for t in [30, 40, 50]:
            mock_time.return_value = float(t)
            pid.calc(19.9, 20.0)
            assert pid.derivative == pytest.approx(3.0, abs=0.1), \
                f"At t={t}, D should hold at 3.0, got {pid.derivative}"

        # t=60: next sensor change
        mock_time.return_value = 60.0
        pid.calc(19.8, 20.0)

        # dt = 10, input_diff = 19.8 - 19.9 = -0.1
        # D = -(300 * -0.1) / 10 = 3.0
        assert pid.derivative == pytest.approx(3.0, abs=0.1), \
            f"D should be 3.0, got {pid.derivative}"


# ---------------------------------------------------------------------------
# Edge cases
# ---------------------------------------------------------------------------

class TestEdgeCases:
    """Edge cases that could break derivative."""

    @patch('pid_controller.time')
    def test_same_temp_forever_no_derivative(self, mock_time):
        """If temperature never changes, derivative stays 0."""
        pid = make_pid(kp=0, ki=0, kd=300)

        for t in range(0, 300, 10):
            mock_time.return_value = float(t)
            pid.calc(20.0, 20.0)

        assert pid.derivative == 0.0

    @patch('pid_controller.time')
    def test_clear_samples_resets_state(self, mock_time):
        """After clear_samples(), derivative needs fresh readings."""
        pid = make_pid(kp=0, ki=0, kd=300)

        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        mock_time.return_value = 10.0
        pid.calc(19.9, 20.0)
        assert pid.derivative != 0.0

        pid.clear_samples()

        # After clear, first calc has _input=None, so it's treated as first reading
        mock_time.return_value = 20.0
        pid.calc(19.9, 20.0)
        # _last_input is None -> derivative not computed, holds from clear
        # But clear_samples doesn't reset derivative... let's check what happens

        mock_time.return_value = 30.0
        pid.calc(19.8, 20.0)
        # Now _last_input=19.9, dt=10
        # D = -(300 * -0.1) / 10 = 3.0
        assert pid.derivative == pytest.approx(3.0, abs=0.1), \
            f"After clear + 2 readings, D should be 3.0, got {pid.derivative}"

    @patch('pid_controller.time')
    def test_integral_accumulates_every_tick(self, mock_time):
        """Integral should grow on every calc() call, not just sensor changes."""
        pid = make_pid(kp=0, ki=1.0, kd=0)

        mock_time.return_value = 0.0
        pid.calc(19.0, 20.0)  # error = 1.0

        mock_time.return_value = 10.0
        pid.calc(19.0, 20.0)
        i_after_10 = pid.integral

        mock_time.return_value = 20.0
        pid.calc(19.0, 20.0)
        i_after_20 = pid.integral

        assert i_after_10 > 0, "Integral should be positive"
        assert i_after_20 > i_after_10, \
            f"Integral should grow: {i_after_10} -> {i_after_20}"
        assert i_after_10 == pytest.approx(10.0, abs=0.1)
        assert i_after_20 == pytest.approx(20.0, abs=0.1)

    @patch('pid_controller.time')
    def test_derivative_with_sensor_event_between_keepalive_ticks(self, mock_time):
        """Sensor event arrives between keep-alive ticks.
        Derivative dt should be short (time since last tick), not since last sensor change."""
        pid = make_pid(kp=0, ki=0, kd=300)

        # t=0: first reading
        mock_time.return_value = 0.0
        pid.calc(20.0, 20.0)

        # t=10: keep-alive tick
        mock_time.return_value = 10.0
        pid.calc(20.0, 20.0)

        # t=13: sensor event — temp changed
        mock_time.return_value = 13.0
        pid.calc(19.9, 20.0)

        # dt = 13 - 10 = 3 (time since last calc, not since last sensor change)
        # D = -(300 * -0.1) / 3 = 10.0
        assert pid.dt == pytest.approx(3.0, abs=0.1)
        assert pid.derivative == pytest.approx(10.0, abs=0.1), \
            f"D should use calc dt=3, got D={pid.derivative}"
