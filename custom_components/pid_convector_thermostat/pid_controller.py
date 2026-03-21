"""PID Controller for PID Convector Thermostat.

Based on Arduino PID Library by Brett Beauregard.
See https://github.com/br3ttb/Arduino-PID-Library

Original implementation from HASmartThermostat by ScratMan.
See https://github.com/ScratMan/HASmartThermostat
"""

import logging
from time import time

_LOGGER = logging.getLogger(__name__)


class PID:
    """A proportional-integral-derivative controller with outdoor temperature compensation."""

    def __init__(self, kp, ki, kd, ke=0, out_min=float('-inf'), out_max=float('+inf'),
                 sampling_period=0, min_deriv_dt=2.0, cold_tolerance=0.3, hot_tolerance=0.3):
        """Initialize the PID controller.

        :param kp: Proportional coefficient.
        :param ki: Integral coefficient.
        :param kd: Derivative coefficient.
        :param ke: Outdoor temperature compensation coefficient.
        :param out_min: Lower output limit.
        :param out_max: Upper output limit.
        :param sampling_period: Minimum time between PID calculations in seconds.
        :param min_deriv_dt: Minimum derivative interval in seconds. Clamps dt
            to prevent spikes from sensor jitter or startup replay.
        :param cold_tolerance: Temperature below setpoint before action (PID OFF mode).
        :param hot_tolerance: Temperature above setpoint before action (PID OFF mode).
        """
        if kp is None:
            raise ValueError('kp must be specified')
        if ki is None:
            raise ValueError('ki must be specified')
        if kd is None:
            raise ValueError('kd must be specified')
        if out_min >= out_max:
            raise ValueError('out_min must be less than out_max')

        self._Kp = kp
        self._Ki = ki
        self._Kd = kd
        self._Ke = ke
        self._out_min = out_min
        self._out_max = out_max
        self._proportional = 0.0
        self._integral = 0.0
        self._derivative = 0.0
        self._last_set_point = 0
        self._set_point = 0
        self._input = None
        self._last_input = None
        self._calc_time = None
        self._last_calc_time = None
        self._error = 0
        self._input_diff = 0
        self._dext = 0
        self._dt = 0
        self._last_output = 0
        self._output = 0
        self._proportional = 0
        self._derivative = 0
        self._external = 0
        self._mode = 'AUTO'
        self._sampling_period = sampling_period
        self._min_deriv_dt = min_deriv_dt
        self._cold_tolerance = cold_tolerance
        self._hot_tolerance = hot_tolerance

    @property
    def mode(self):
        return self._mode

    @mode.setter
    def mode(self, mode):
        assert mode.upper() in ['AUTO', 'OFF']
        self._mode = mode.upper()

    @property
    def out_max(self):
        return self._out_max

    @out_max.setter
    def out_max(self, out_max):
        self._out_max = out_max

    @property
    def out_min(self):
        return self._out_min

    @out_min.setter
    def out_min(self, out_min):
        self._out_min = out_min

    @property
    def sampling_period(self):
        return self._sampling_period

    @property
    def error(self):
        return self._error

    @property
    def proportional(self):
        return self._proportional

    @property
    def integral(self):
        return self._integral

    @integral.setter
    def integral(self, i):
        assert isinstance(i, float), "Integral should be a float"
        self._integral = i

    @property
    def derivative(self):
        return self._derivative

    @property
    def external(self):
        return self._external

    @property
    def dt(self):
        return self._dt

    def set_pid_param(self, kp=None, ki=None, kd=None, ke=None):
        """Set PID parameters."""
        if kp is not None and isinstance(kp, (int, float)):
            self._Kp = kp
        if ki is not None and isinstance(ki, (int, float)):
            self._Ki = ki
        if kd is not None and isinstance(kd, (int, float)):
            self._Kd = kd
        if ke is not None and isinstance(ke, (int, float)):
            self._Ke = ke

    def clear_samples(self):
        """Clear the samples values and timestamp to restart PID from clean state."""
        self._input = None
        self._last_input = None
        self._calc_time = None
        self._last_calc_time = None

    def seed_setpoint(self, set_point, input_val=None):
        """Seed the PID state so the first calc() starts cleanly.

        Seeds the setpoint so the anti-windup logic doesn't see a spurious
        change from 0 -> target_temp (which would reset the integral).

        If input_val is provided, also seeds the input value so the first
        calc() doesn't treat the startup sensor read as a "change".

        Must be called after state restore and before the first calc().
        """
        self._set_point = set_point
        self._last_set_point = set_point
        if input_val is not None:
            self._input = input_val
        self._last_input = None
        self._last_calc_time = None

    def calc(self, input_val, set_point, ext_temp=None):
        """Compute PID output.

        Timing: a single timeline (_calc_time / _last_calc_time) is used for
        both integral dt and derivative dt. The derivative only recomputes when
        the sensor value actually changes; between changes it holds its previous
        value. The min_deriv_dt floor prevents spikes when calc() is called in
        rapid succession (e.g. HA startup event replay).

        Args:
            input_val (float): The current temperature.
            set_point (float): The target temperature.
            ext_temp (float): Outdoor temperature for compensation.

        Returns:
            Tuple of (output, updated): output clamped between out_min and out_max,
            and boolean indicating if a new value was computed.
        """
        now = time()
        if self._sampling_period != 0 and self._calc_time is not None and \
                now - self._calc_time < self._sampling_period:
            return self._output, False

        # Advance timing
        self._last_calc_time = self._calc_time
        self._calc_time = now

        # Compute dt
        if self._last_calc_time is not None:
            self._dt = self._calc_time - self._last_calc_time
        else:
            self._dt = 0

        # Detect whether the sensor value actually changed
        sensor_changed = (self._input is None or input_val != self._input)
        if sensor_changed:
            self._last_input = self._input
            self._input = input_val

        self._last_output = self._output
        self._last_set_point = self._set_point
        self._set_point = set_point

        if self.mode == 'OFF':
            if input_val <= set_point - self._cold_tolerance:
                self._output = self._out_max
                _LOGGER.debug("PID OFF: input below set point -> max output")
                return self._output, True
            elif input_val >= set_point + self._hot_tolerance:
                self._output = self._out_min
                _LOGGER.debug("PID OFF: input above set point -> min output")
                return self._output, True
            else:
                return self._output, False

        # Compute error
        self._error = set_point - input_val

        if ext_temp is not None:
            self._dext = set_point - ext_temp
        else:
            self._dext = 0

        # External temperature compensation
        self._external = self._Ke * self._dext

        # Anti-windup: only integrate when not saturated and set point is stable
        if self._out_min <= self._last_output <= self._out_max and \
                self._last_set_point == self._set_point:
            self._integral += self._Ki * self._error * self._dt
            self._integral = max(
                min(self._integral, self._out_max - self._external),
                self._out_min - self._external
            )
        if ext_temp is not None and self._last_set_point != self._set_point:
            self._integral = 0

        self._proportional = self._Kp * self._error

        # Derivative: only recompute when sensor changed AND we have a valid
        # previous reading. Between changes, hold the previous derivative value.
        if sensor_changed and self._last_input is not None and self._dt > 0:
            self._input_diff = self._input - self._last_input
            # Floor dt to prevent spikes from rapid calc() calls
            effective_dt = max(self._dt, self._min_deriv_dt)
            self._derivative = -(self._Kd * self._input_diff) / effective_dt
        # else: hold previous derivative value

        # Compute PID output
        output = self._proportional + self._integral + self._derivative + self._external
        self._output = max(min(output, self._out_max), self._out_min)
        return self._output, True
