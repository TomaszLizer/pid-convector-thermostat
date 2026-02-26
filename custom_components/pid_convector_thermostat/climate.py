"""PID Convector Thermostat - Climate Platform.

A PID-controlled climate entity for VDC fan convectors (trench heaters)
with dead-zone fan control, pause signals, ramp rate, and anti-cycling.
"""

import asyncio
import logging
import time
from abc import ABC

import voluptuous as vol

from homeassistant.core import HomeAssistant, CoreState, Event, EventStateChangedData, callback
from homeassistant.helpers import entity_platform
from homeassistant.const import (
    ATTR_ENTITY_ID,
    ATTR_TEMPERATURE,
    CONF_NAME,
    CONF_UNIQUE_ID,
    EVENT_HOMEASSISTANT_START,
    PRECISION_HALVES,
    STATE_ON,
    STATE_OFF,
    STATE_UNKNOWN,
)
from homeassistant.components.light import (
    DOMAIN as LIGHT_DOMAIN,
    SERVICE_TURN_ON as SERVICE_TURN_LIGHT_ON,
    SERVICE_TURN_OFF as SERVICE_TURN_LIGHT_OFF,
    ATTR_BRIGHTNESS_PCT,
)
from homeassistant.util import slugify
import homeassistant.helpers.config_validation as cv
from homeassistant.helpers.event import (
    async_track_state_change_event,
    async_track_time_interval,
)
from homeassistant.helpers.reload import async_setup_reload_service
from homeassistant.helpers.restore_state import RestoreEntity

from homeassistant.components.climate import PLATFORM_SCHEMA, ClimateEntity, ClimateEntityFeature
from homeassistant.components.climate import (
    ATTR_PRESET_MODE,
    HVACMode,
    HVACAction,
)

from . import DOMAIN, PLATFORMS
from . import const
from .pid_controller import PID

_LOGGER = logging.getLogger(__name__)

# --- Preset speed schema ---
PRESET_SPEEDS_SCHEMA = vol.Schema({
    vol.Optional(const.PRESET_LOW, default=const.DEFAULT_PRESET_LOW): vol.Coerce(float),
    vol.Optional(const.PRESET_MEDIUM, default=const.DEFAULT_PRESET_MEDIUM): vol.Coerce(float),
    vol.Optional(const.PRESET_HIGH, default=const.DEFAULT_PRESET_HIGH): vol.Coerce(float),
})

# --- Platform schema ---
PLATFORM_SCHEMA = PLATFORM_SCHEMA.extend(
    {
        vol.Required(const.CONF_HEATER): cv.entity_id,
        vol.Required(const.CONF_SENSOR): cv.entity_id,
        vol.Optional(const.CONF_OUTDOOR_SENSOR): cv.entity_id,
        vol.Optional(const.CONF_WINDOW_SENSOR): cv.entity_id,
        vol.Optional(const.CONF_PAUSE_SIGNAL): cv.entity_id,
        vol.Optional(CONF_NAME, default=const.DEFAULT_NAME): cv.string,
        vol.Optional(CONF_UNIQUE_ID, default='none'): cv.string,
        vol.Optional(const.CONF_MIN_TEMP, default=7.0): vol.Coerce(float),
        vol.Optional(const.CONF_MAX_TEMP, default=35.0): vol.Coerce(float),
        vol.Optional(const.CONF_TARGET_TEMP): vol.Coerce(float),
        vol.Optional(const.CONF_KP, default=const.DEFAULT_KP): vol.Coerce(float),
        vol.Optional(const.CONF_KI, default=const.DEFAULT_KI): vol.Coerce(float),
        vol.Optional(const.CONF_KD, default=const.DEFAULT_KD): vol.Coerce(float),
        vol.Optional(const.CONF_KE, default=const.DEFAULT_KE): vol.Coerce(float),
        vol.Optional(const.CONF_DEAD_ZONE, default=const.DEFAULT_DEAD_ZONE): vol.Coerce(float),
        vol.Optional(const.CONF_OUTPUT_MAX, default=const.DEFAULT_OUTPUT_MAX): vol.Coerce(float),
        vol.Optional(const.CONF_NIGHT_MAX, default=const.DEFAULT_NIGHT_MAX): vol.Coerce(float),
        vol.Optional(const.CONF_PRESET_SPEEDS, default={}): PRESET_SPEEDS_SCHEMA,
        vol.Required(const.CONF_KEEP_ALIVE): vol.All(cv.time_period, cv.positive_timedelta),
        vol.Optional(const.CONF_SAMPLING_PERIOD, default=const.DEFAULT_SAMPLING_PERIOD): vol.All(
            cv.time_period, cv.positive_timedelta),
        vol.Optional(const.CONF_SENSOR_STALL, default=const.DEFAULT_SENSOR_STALL): vol.All(
            cv.time_period, cv.positive_timedelta),
        vol.Optional(const.CONF_OUTPUT_SAFETY, default=const.DEFAULT_OUTPUT_SAFETY): vol.Coerce(float),
        vol.Optional(const.CONF_MIN_ON_DURATION, default=const.DEFAULT_MIN_ON_DURATION): vol.Coerce(int),
        vol.Optional(const.CONF_MIN_OFF_DURATION, default=const.DEFAULT_MIN_OFF_DURATION): vol.Coerce(int),
        vol.Optional(const.CONF_RAMP_RATE, default=const.DEFAULT_RAMP_RATE): vol.Coerce(float),
        vol.Optional(const.CONF_HOT_TOLERANCE, default=const.DEFAULT_TOLERANCE): vol.Coerce(float),
        vol.Optional(const.CONF_COLD_TOLERANCE, default=const.DEFAULT_TOLERANCE): vol.Coerce(float),
        vol.Optional(const.CONF_INITIAL_HVAC_MODE): vol.In(
            [HVACMode.HEAT, HVACMode.FAN_ONLY, HVACMode.OFF]
        ),
        vol.Optional(const.CONF_DEBUG, default=False): cv.boolean,
    }
)

# Presets available per HVAC mode
HEAT_PRESETS = [const.PRESET_AUTO, const.PRESET_NIGHT, const.PRESET_LOW,
                const.PRESET_MEDIUM, const.PRESET_HIGH]
FAN_ONLY_PRESETS = [const.PRESET_LOW, const.PRESET_MEDIUM, const.PRESET_HIGH]
PID_PRESETS = {const.PRESET_AUTO, const.PRESET_NIGHT}


async def async_setup_platform(hass, config, async_add_entities, discovery_info=None):
    """Set up the PID Convector Thermostat platform."""
    await async_setup_reload_service(hass, DOMAIN, PLATFORMS)

    platform = entity_platform.current_platform.get()
    assert platform

    preset_speeds = config.get(const.CONF_PRESET_SPEEDS, {})

    thermostat = PidConvectorThermostat(
        name=config.get(CONF_NAME),
        unique_id=config.get(CONF_UNIQUE_ID),
        heater_entity_id=config.get(const.CONF_HEATER),
        sensor_entity_id=config.get(const.CONF_SENSOR),
        outdoor_sensor_id=config.get(const.CONF_OUTDOOR_SENSOR),
        window_sensor_id=config.get(const.CONF_WINDOW_SENSOR),
        pause_signal_id=config.get(const.CONF_PAUSE_SIGNAL),
        min_temp=config.get(const.CONF_MIN_TEMP),
        max_temp=config.get(const.CONF_MAX_TEMP),
        target_temp=config.get(const.CONF_TARGET_TEMP),
        kp=config.get(const.CONF_KP),
        ki=config.get(const.CONF_KI),
        kd=config.get(const.CONF_KD),
        ke=config.get(const.CONF_KE),
        dead_zone=config.get(const.CONF_DEAD_ZONE),
        output_max=config.get(const.CONF_OUTPUT_MAX),
        night_max=config.get(const.CONF_NIGHT_MAX),
        preset_speeds=preset_speeds,
        keep_alive=config.get(const.CONF_KEEP_ALIVE),
        sampling_period=config.get(const.CONF_SAMPLING_PERIOD),
        sensor_stall=config.get(const.CONF_SENSOR_STALL),
        output_safety=config.get(const.CONF_OUTPUT_SAFETY),
        min_on_duration=config.get(const.CONF_MIN_ON_DURATION),
        min_off_duration=config.get(const.CONF_MIN_OFF_DURATION),
        ramp_rate=config.get(const.CONF_RAMP_RATE),
        cold_tolerance=config.get(const.CONF_COLD_TOLERANCE),
        hot_tolerance=config.get(const.CONF_HOT_TOLERANCE),
        initial_hvac_mode=config.get(const.CONF_INITIAL_HVAC_MODE),
        debug=config.get(const.CONF_DEBUG),
    )
    async_add_entities([thermostat])

    platform.async_register_entity_service(
        "set_pid_gain",
        {
            vol.Optional("kp"): vol.Coerce(float),
            vol.Optional("ki"): vol.Coerce(float),
            vol.Optional("kd"): vol.Coerce(float),
            vol.Optional("ke"): vol.Coerce(float),
        },
        "async_set_pid_gain",
    )
    platform.async_register_entity_service(
        "clear_integral",
        {},
        "async_clear_integral",
    )


class PidConvectorThermostat(ClimateEntity, RestoreEntity, ABC):
    """PID-controlled climate entity for VDC fan convectors."""

    def __init__(self, **kwargs):
        """Initialize the thermostat."""
        self._name = kwargs['name']
        self._unique_id = kwargs['unique_id']
        self._heater_entity_id = kwargs['heater_entity_id']
        self._sensor_entity_id = kwargs['sensor_entity_id']
        self._outdoor_sensor_id = kwargs.get('outdoor_sensor_id')
        self._window_sensor_id = kwargs.get('window_sensor_id')
        self._pause_signal_id = kwargs.get('pause_signal_id')

        if self._unique_id == 'none':
            self._unique_id = slugify(f"{DOMAIN}_{self._name}_{self._heater_entity_id}")

        # Temperature limits
        self._min_temp = kwargs['min_temp']
        self._max_temp = kwargs['max_temp']
        self._target_temp = kwargs.get('target_temp')

        # PID parameters
        self._kp = kwargs['kp']
        self._ki = kwargs['ki']
        self._kd = kwargs['kd']
        self._ke = kwargs['ke']
        self._cold_tolerance = abs(kwargs.get('cold_tolerance', const.DEFAULT_TOLERANCE))
        self._hot_tolerance = abs(kwargs.get('hot_tolerance', const.DEFAULT_TOLERANCE))

        # Output configuration
        self._dead_zone = kwargs['dead_zone']
        self._output_max = kwargs['output_max']
        self._night_max = kwargs['night_max']
        self._preset_speeds = {
            const.PRESET_LOW: kwargs['preset_speeds'].get(const.PRESET_LOW, const.DEFAULT_PRESET_LOW),
            const.PRESET_MEDIUM: kwargs['preset_speeds'].get(const.PRESET_MEDIUM, const.DEFAULT_PRESET_MEDIUM),
            const.PRESET_HIGH: kwargs['preset_speeds'].get(const.PRESET_HIGH, const.DEFAULT_PRESET_HIGH),
        }

        # Timing
        self._keep_alive = kwargs['keep_alive']
        self._sampling_period = kwargs['sampling_period'].total_seconds()
        self._sensor_stall = kwargs['sensor_stall'].total_seconds()
        self._output_safety = kwargs['output_safety']
        self._min_on_duration = kwargs['min_on_duration']
        self._min_off_duration = kwargs['min_off_duration']
        self._ramp_rate = kwargs['ramp_rate']
        self._debug = kwargs.get('debug', False)

        # HVAC state
        self._hvac_mode = kwargs.get('initial_hvac_mode') or None
        self._attr_preset_mode = const.PRESET_AUTO

        # Internal state
        self._active = False
        self._current_temp = None
        self._cur_temp_time = None
        self._previous_temp = None
        self._previous_temp_time = None
        self._ext_temp = None
        self._last_sensor_update = time.time()
        self._temp_lock = asyncio.Lock()

        # PID state
        self._p = self._i = self._d = self._e = self._dt = 0.0
        self._control_output = 0.0  # PID target output
        self._actual_output = 0.0   # Current fan speed (after ramp)
        self._target_output = 0.0   # Desired fan speed (after dead-zone)

        # Anti-cycling state
        self._fan_is_on = False
        self._last_on_off_change = 0.0

        # Ramp state
        self._ramp_unsub = None

        # Pause state
        self._paused = False
        self._pause_reason = None

        # Features
        self._support_flags = (
            ClimateEntityFeature.TARGET_TEMPERATURE
            | ClimateEntityFeature.PRESET_MODE
            | ClimateEntityFeature.TURN_OFF
            | ClimateEntityFeature.TURN_ON
        )
        self._enable_turn_on_off_backwards_compatibility = False
        self._attr_hvac_modes = [HVACMode.HEAT, HVACMode.FAN_ONLY, HVACMode.OFF]

        # Unit
        self._unit = None

        # PID controller (created after we know output range)
        self._pid_controller = PID(
            self._kp, self._ki, self._kd, self._ke,
            out_min=0.0, out_max=self._output_max,
            sampling_period=self._sampling_period,
            cold_tolerance=self._cold_tolerance,
            hot_tolerance=self._hot_tolerance,
        )
        self._pid_controller.mode = "AUTO"

    # --- HA lifecycle ---

    async def async_added_to_hass(self):
        """Run when entity about to be added."""
        await super().async_added_to_hass()
        self._unit = self.hass.config.units.temperature_unit

        # Track sensor changes
        self.async_on_remove(
            async_track_state_change_event(
                self.hass, self._sensor_entity_id, self._async_sensor_changed))

        if self._outdoor_sensor_id:
            self.async_on_remove(
                async_track_state_change_event(
                    self.hass, self._outdoor_sensor_id, self._async_ext_sensor_changed))

        if self._window_sensor_id:
            self.async_on_remove(
                async_track_state_change_event(
                    self.hass, self._window_sensor_id, self._async_pause_changed))

        if self._pause_signal_id:
            self.async_on_remove(
                async_track_state_change_event(
                    self.hass, self._pause_signal_id, self._async_pause_changed))

        # Keep alive timer
        if self._keep_alive:
            self.async_on_remove(
                async_track_time_interval(
                    self.hass, self._async_control_heating, self._keep_alive))

        # Start ramp timer (every 2 seconds)
        if self._ramp_rate > 0:
            from datetime import timedelta
            self.async_on_remove(
                async_track_time_interval(
                    self.hass, self._async_ramp_tick,
                    timedelta(seconds=const.DEFAULT_RAMP_INTERVAL)))

        @callback
        def _async_startup(*_):
            """Init on startup."""
            sensor_state = self.hass.states.get(self._sensor_entity_id)
            if sensor_state and sensor_state.state not in (STATE_UNKNOWN, None, 'unavailable'):
                self._async_update_temp(sensor_state)
            if self._outdoor_sensor_id:
                ext_state = self.hass.states.get(self._outdoor_sensor_id)
                if ext_state and ext_state.state not in (STATE_UNKNOWN, None, 'unavailable'):
                    self._async_update_ext_temp(ext_state)
            # Check initial pause state
            self._update_pause_state()

        if self.hass.state == CoreState.running:
            _async_startup()
        else:
            self.hass.bus.async_listen_once(EVENT_HOMEASSISTANT_START, _async_startup)

        # Restore previous state
        old_state = await self.async_get_last_state()
        if old_state is not None:
            if old_state.attributes.get(ATTR_TEMPERATURE) is not None:
                self._target_temp = float(old_state.attributes[ATTR_TEMPERATURE])
            elif self._target_temp is None:
                self._target_temp = self._min_temp

            if old_state.attributes.get(ATTR_PRESET_MODE) is not None:
                self._attr_preset_mode = old_state.attributes[ATTR_PRESET_MODE]

            if isinstance(old_state.attributes.get('pid_i'), (float, int)):
                self._i = float(old_state.attributes['pid_i'])
                self._pid_controller.integral = self._i

            for attr in ('kp', 'ki', 'kd', 'ke'):
                val = old_state.attributes.get(attr)
                if val is not None:
                    setattr(self, f'_{attr}', float(val))
            self._pid_controller.set_pid_param(self._kp, self._ki, self._kd, self._ke)

            if not self._hvac_mode and old_state.state:
                self._set_hvac_mode_internal(old_state.state)
        else:
            if self._target_temp is None:
                self._target_temp = self._min_temp
            _LOGGER.warning("%s: No previous state, target temp set to %s",
                            self.entity_id, self._target_temp)

        if not self._hvac_mode:
            self._hvac_mode = HVACMode.OFF

        await self._async_control_heating(calc_pid=True)

    # --- Properties ---

    @property
    def should_poll(self):
        return False

    @property
    def name(self):
        return self._name

    @property
    def unique_id(self):
        return self._unique_id

    @property
    def temperature_unit(self):
        return self._unit or self.hass.config.units.temperature_unit

    @property
    def current_temperature(self):
        return self._current_temp

    @property
    def target_temperature(self):
        return self._target_temp

    @property
    def target_temperature_step(self):
        return PRECISION_HALVES

    @property
    def hvac_mode(self):
        return self._hvac_mode

    @property
    def hvac_action(self):
        if self._hvac_mode == HVACMode.OFF:
            return HVACAction.OFF
        if self._paused:
            return HVACAction.IDLE
        if self._fan_is_on and self._actual_output > 0:
            if self._hvac_mode == HVACMode.HEAT:
                return HVACAction.HEATING
            return HVACAction.FAN
        return HVACAction.IDLE

    @property
    def preset_mode(self):
        return self._attr_preset_mode

    @property
    def preset_modes(self):
        if self._hvac_mode == HVACMode.HEAT:
            return HEAT_PRESETS
        elif self._hvac_mode == HVACMode.FAN_ONLY:
            return FAN_ONLY_PRESETS
        return []

    @property
    def supported_features(self):
        return self._support_flags

    @property
    def min_temp(self):
        return self._min_temp

    @property
    def max_temp(self):
        return self._max_temp

    @property
    def extra_state_attributes(self):
        attrs = {}
        if self._debug:
            attrs.update({
                "pid_p": round(self._p, 2),
                "pid_i": round(self._i, 2),
                "pid_d": round(self._d, 2),
                "pid_e": round(self._e, 2),
                "pid_dt": round(self._dt, 2),
                "control_output": round(self._control_output, 1),
                "target_output": round(self._target_output, 1),
                "actual_output": round(self._actual_output, 1),
                "dead_zone_active": self._target_output == 0 and self._control_output > 0,
                "paused": self._paused,
                "pause_reason": self._pause_reason,
                "fan_is_on": self._fan_is_on,
            })
        # Always expose PID gains (useful for set_pid_gain service)
        attrs.update({
            "kp": self._kp,
            "ki": self._ki,
            "kd": self._kd,
            "ke": self._ke,
            "dead_zone": self._dead_zone,
        })
        return attrs

    # --- HVAC mode changes ---

    def _set_hvac_mode_internal(self, hvac_mode):
        """Set HVAC mode without side effects."""
        if hvac_mode in (HVACMode.HEAT, 'heat'):
            self._hvac_mode = HVACMode.HEAT
            self._pid_controller.out_max = self._get_current_output_max()
        elif hvac_mode in (HVACMode.FAN_ONLY, 'fan_only'):
            self._hvac_mode = HVACMode.FAN_ONLY
        elif hvac_mode in (HVACMode.OFF, 'off'):
            self._hvac_mode = HVACMode.OFF
        else:
            _LOGGER.warning("%s: Unknown HVAC mode: %s", self.entity_id, hvac_mode)

    async def async_set_hvac_mode(self, hvac_mode: HVACMode) -> None:
        """Set new target HVAC mode."""
        old_mode = self._hvac_mode

        if hvac_mode == HVACMode.HEAT:
            self._hvac_mode = HVACMode.HEAT
            # Transition presets
            if self._attr_preset_mode not in HEAT_PRESETS:
                self._attr_preset_mode = const.PRESET_AUTO
            self._pid_controller.out_max = self._get_current_output_max()
            self._pid_controller.out_min = 0.0
            await self._async_control_heating(calc_pid=True)

        elif hvac_mode == HVACMode.FAN_ONLY:
            self._hvac_mode = HVACMode.FAN_ONLY
            # Transition presets
            if self._attr_preset_mode not in FAN_ONLY_PRESETS:
                self._attr_preset_mode = const.PRESET_LOW
            # Set fixed speed immediately
            await self._async_apply_fixed_speed()

        elif hvac_mode == HVACMode.OFF:
            self._hvac_mode = HVACMode.OFF
            self._control_output = 0.0
            self._target_output = 0.0
            self._pid_controller.clear_samples()
            self._previous_temp = None
            self._previous_temp_time = None
            await self._async_set_fan_speed(0)

        else:
            _LOGGER.error("%s: Unrecognized HVAC mode: %s", self.entity_id, hvac_mode)
            return

        self.async_write_ha_state()

    async def async_set_temperature(self, **kwargs):
        """Set new target temperature."""
        temperature = kwargs.get(ATTR_TEMPERATURE)
        if temperature is None:
            return
        self._target_temp = temperature
        if self._hvac_mode == HVACMode.HEAT:
            await self._async_control_heating(calc_pid=True)
        self.async_write_ha_state()

    async def async_set_preset_mode(self, preset_mode: str):
        """Set new preset mode."""
        if self._hvac_mode == HVACMode.HEAT and preset_mode not in HEAT_PRESETS:
            _LOGGER.warning("%s: Preset %s not available in HEAT mode", self.entity_id, preset_mode)
            return
        if self._hvac_mode == HVACMode.FAN_ONLY and preset_mode not in FAN_ONLY_PRESETS:
            _LOGGER.warning("%s: Preset %s not available in FAN_ONLY mode", self.entity_id, preset_mode)
            return

        self._attr_preset_mode = preset_mode

        if self._hvac_mode == HVACMode.HEAT:
            # Update PID output max based on preset
            self._pid_controller.out_max = self._get_current_output_max()
            if preset_mode in PID_PRESETS:
                await self._async_control_heating(calc_pid=True)
            else:
                # Fixed speed presets in heat mode
                await self._async_apply_fixed_speed()
        elif self._hvac_mode == HVACMode.FAN_ONLY:
            await self._async_apply_fixed_speed()

        self.async_write_ha_state()

    # --- Services ---

    async def async_set_pid_gain(self, **kwargs):
        """Set PID parameters via service call."""
        for param, value in kwargs.items():
            if value is not None:
                setattr(self, f'_{param}', float(value))
        self._pid_controller.set_pid_param(self._kp, self._ki, self._kd, self._ke)
        if self._hvac_mode == HVACMode.HEAT and self._attr_preset_mode in PID_PRESETS:
            await self._async_control_heating(calc_pid=True)

    async def async_clear_integral(self, **kwargs):
        """Clear the integral value."""
        self._pid_controller.integral = 0.0
        self._i = 0.0
        self.async_write_ha_state()

    # --- Sensor handlers ---

    @callback
    async def _async_sensor_changed(self, event: Event[EventStateChangedData]):
        """Handle temperature sensor changes."""
        new_state = event.data["new_state"]
        if new_state is None or new_state.state in (STATE_UNKNOWN, 'unavailable'):
            return
        self._previous_temp_time = self._cur_temp_time
        self._cur_temp_time = time.time()
        self._async_update_temp(new_state)
        _LOGGER.debug("%s: Temperature update: %s", self.entity_id, self._current_temp)

        if self._hvac_mode == HVACMode.HEAT and self._attr_preset_mode in PID_PRESETS:
            await self._async_control_heating(calc_pid=True)
        self.async_write_ha_state()

    @callback
    async def _async_ext_sensor_changed(self, event: Event[EventStateChangedData]):
        """Handle outdoor temperature sensor changes."""
        new_state = event.data["new_state"]
        if new_state is None or new_state.state in (STATE_UNKNOWN, 'unavailable'):
            return
        self._async_update_ext_temp(new_state)
        _LOGGER.debug("%s: Outdoor temp update: %s", self.entity_id, self._ext_temp)

    @callback
    def _async_update_temp(self, state):
        """Update current temperature from sensor state."""
        try:
            self._previous_temp = self._current_temp
            self._current_temp = float(state.state)
            self._last_sensor_update = time.time()
        except ValueError as ex:
            _LOGGER.debug("%s: Unable to parse temperature: %s", self.entity_id, ex)

    @callback
    def _async_update_ext_temp(self, state):
        """Update outdoor temperature from sensor state."""
        try:
            self._ext_temp = float(state.state)
        except ValueError as ex:
            _LOGGER.debug("%s: Unable to parse outdoor temperature: %s", self.entity_id, ex)

    # --- Pause handling ---

    def _update_pause_state(self):
        """Check current state of all pause signals."""
        reasons = []
        if self._window_sensor_id:
            state = self.hass.states.get(self._window_sensor_id)
            if state and state.state == STATE_ON:
                reasons.append("window_open")
        if self._pause_signal_id:
            state = self.hass.states.get(self._pause_signal_id)
            if state and state.state == STATE_ON:
                reasons.append("pause_signal")
        self._paused = len(reasons) > 0
        self._pause_reason = ", ".join(reasons) if reasons else None

    @callback
    async def _async_pause_changed(self, event: Event[EventStateChangedData]):
        """Handle window/pause signal changes."""
        new_state = event.data["new_state"]
        if new_state is None:
            return

        was_paused = self._paused
        self._update_pause_state()

        if self._paused and not was_paused:
            # Entering pause — stop fan immediately (bypass deadband and ramp)
            _LOGGER.info("%s: Pausing — reason: %s", self.entity_id, self._pause_reason)
            self._target_output = 0.0
            await self._async_set_fan_speed_immediate(0)
        elif not self._paused and was_paused:
            # Exiting pause — clear derivative history, resume with deadband
            _LOGGER.info("%s: Resuming from pause", self.entity_id)
            self._pid_controller.clear_samples()
            self._previous_temp = None
            self._previous_temp_time = None
            # Deadband applies on resume — _last_on_off_change is already set
            if self._hvac_mode == HVACMode.HEAT and self._attr_preset_mode in PID_PRESETS:
                await self._async_control_heating(calc_pid=True)
            elif self._hvac_mode == HVACMode.FAN_ONLY or self._attr_preset_mode not in PID_PRESETS:
                await self._async_apply_fixed_speed()

        self.async_write_ha_state()

    # --- Core control loop ---

    async def _async_control_heating(self, now=None, calc_pid=False):
        """Run the PID control loop."""
        async with self._temp_lock:
            if not self._active and self._current_temp is not None and self._target_temp is not None:
                self._active = True
                _LOGGER.info("%s: Activating — temp=%s, target=%s",
                             self.entity_id, self._current_temp, self._target_temp)

            if not self._active or self._hvac_mode == HVACMode.OFF:
                return

            if self._paused:
                return

            # Fixed speed preset — no PID needed
            if self._attr_preset_mode not in PID_PRESETS:
                return

            # Sensor stall check
            if self._sensor_stall > 0 and \
                    time.time() - self._last_sensor_update > self._sensor_stall:
                _LOGGER.warning("%s: Sensor stall detected, setting safety output %s",
                                self.entity_id, self._output_safety)
                self._control_output = self._output_safety
            else:
                await self._calc_pid_output()

            # Apply dead-zone mapping
            self._apply_dead_zone()

            # Apply anti-cycling deadband and request fan speed
            await self._apply_output_with_deadband()

            self.async_write_ha_state()

    async def _calc_pid_output(self):
        """Calculate PID output."""
        if self._current_temp is None or self._target_temp is None:
            return

        output, update = self._pid_controller.calc(
            self._current_temp, self._target_temp,
            ext_temp=self._ext_temp)

        self._control_output = round(output, 1)
        self._p = round(self._pid_controller.proportional, 2)
        self._i = round(self._pid_controller.integral, 2)
        self._d = round(self._pid_controller.derivative, 2)
        self._e = round(self._pid_controller.external, 2)
        self._dt = round(self._pid_controller.dt, 2)

        if update:
            _LOGGER.debug(
                "%s: PID output=%.1f (error=%.2f, dt=%.1f, p=%.2f, i=%.2f, d=%.2f, e=%.2f)",
                self.entity_id, self._control_output,
                self._pid_controller.error, self._dt,
                self._p, self._i, self._d, self._e)

    def _apply_dead_zone(self):
        """Map PID output through dead-zone: below threshold → 0, above → pass through."""
        if abs(self._control_output) < self._dead_zone:
            self._target_output = 0.0
        else:
            self._target_output = abs(self._control_output)

    def _get_current_output_max(self):
        """Get the output maximum for the current preset."""
        if self._attr_preset_mode == const.PRESET_NIGHT:
            return self._night_max
        return self._output_max

    # --- Anti-cycling deadband ---

    async def _apply_output_with_deadband(self):
        """Apply anti-cycling deadband before changing fan state."""
        now = time.time()
        time_since_change = now - self._last_on_off_change

        want_on = self._target_output > 0
        currently_on = self._fan_is_on

        if want_on and not currently_on:
            # Want to turn ON — check min_off_duration
            if time_since_change < self._min_off_duration:
                _LOGGER.debug("%s: Deadband: suppressing ON (off for %.0fs, need %ds)",
                              self.entity_id, time_since_change, self._min_off_duration)
                return
            self._fan_is_on = True
            self._last_on_off_change = now
            _LOGGER.info("%s: Fan ON (target=%.1f%%)", self.entity_id, self._target_output)

        elif not want_on and currently_on:
            # Want to turn OFF — check min_on_duration
            if time_since_change < self._min_on_duration:
                _LOGGER.debug("%s: Deadband: suppressing OFF (on for %.0fs, need %ds)",
                              self.entity_id, time_since_change, self._min_on_duration)
                return
            self._fan_is_on = False
            self._last_on_off_change = now
            self._target_output = 0.0
            _LOGGER.info("%s: Fan OFF (dead-zone)", self.entity_id)

        # If ramp is disabled, set speed immediately
        if self._ramp_rate <= 0:
            await self._async_set_fan_speed(self._target_output)
        # Otherwise the ramp timer will handle gradual changes

    # --- Ramp rate control ---

    @callback
    async def _async_ramp_tick(self, now=None):
        """Periodic ramp tick — gradually move actual output toward target."""
        if self._hvac_mode == HVACMode.OFF:
            return
        if self._paused:
            return

        target = self._target_output
        current = self._actual_output

        if abs(target - current) < 0.5:
            # Close enough, snap to target
            if current != target:
                self._actual_output = target
                await self._async_set_fan_speed(target)
            return

        # Calculate step for this tick
        max_step = self._ramp_rate * const.DEFAULT_RAMP_INTERVAL
        if target > current:
            new_output = min(current + max_step, target)
        else:
            new_output = max(current - max_step, target)

        # Ensure we don't go below dead_zone (unless targeting 0)
        if 0 < new_output < self._dead_zone:
            if target == 0:
                new_output = 0
            else:
                new_output = self._dead_zone

        self._actual_output = round(new_output, 1)
        await self._async_set_fan_speed(self._actual_output)

    # --- Fan speed control ---

    async def _async_set_fan_speed(self, value: float):
        """Set fan speed via light brightness. 0 = turn off."""
        value = round(max(0, value), 1)
        self._actual_output = value

        if value <= 0:
            _LOGGER.debug("%s: Setting fan OFF", self.entity_id)
            data = {ATTR_ENTITY_ID: self._heater_entity_id}
            await self.hass.services.async_call(
                LIGHT_DOMAIN, SERVICE_TURN_LIGHT_OFF, data)
        else:
            _LOGGER.debug("%s: Setting fan to %.1f%%", self.entity_id, value)
            data = {ATTR_ENTITY_ID: self._heater_entity_id, ATTR_BRIGHTNESS_PCT: value}
            await self.hass.services.async_call(
                LIGHT_DOMAIN, SERVICE_TURN_LIGHT_ON, data)

    async def _async_set_fan_speed_immediate(self, value: float):
        """Set fan speed immediately, bypassing ramp. Used for pause."""
        self._actual_output = round(max(0, value), 1)
        self._target_output = self._actual_output
        if value <= 0:
            self._fan_is_on = False
            self._last_on_off_change = time.time()
        await self._async_set_fan_speed(value)

    async def _async_apply_fixed_speed(self):
        """Apply fixed speed from current preset."""
        if self._paused:
            return
        speed = self._preset_speeds.get(self._attr_preset_mode, 0)
        self._control_output = speed
        self._target_output = speed
        self._fan_is_on = speed > 0
        if self._ramp_rate <= 0:
            await self._async_set_fan_speed(speed)
        # If ramp is enabled, ramp timer will handle it

    @property
    def _is_device_active(self):
        """Check if the heater entity is currently on."""
        try:
            state = self.hass.states.get(self._heater_entity_id)
            if state is None:
                return False
            return state.state != STATE_OFF
        except Exception:
            return False
