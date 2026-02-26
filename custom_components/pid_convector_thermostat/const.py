"""Constants for PID Convector Thermostat."""

DOMAIN = "pid_convector_thermostat"

DEFAULT_NAME = "PID Convector Thermostat"

# PID defaults
DEFAULT_KP = 15.0
DEFAULT_KI = 0.01
DEFAULT_KD = 300.0
DEFAULT_KE = 0.0

# Output defaults
DEFAULT_DEAD_ZONE = 20.0
DEFAULT_OUTPUT_MAX = 70.0
DEFAULT_NIGHT_MAX = 40.0

# Preset speed defaults
DEFAULT_PRESET_LOW = 25.0
DEFAULT_PRESET_MEDIUM = 45.0
DEFAULT_PRESET_HIGH = 65.0

# Timing defaults
DEFAULT_SAMPLING_PERIOD = "00:00:00"
DEFAULT_SENSOR_STALL = "06:00:00"
DEFAULT_OUTPUT_SAFETY = 0.0
DEFAULT_MIN_ON_DURATION = 30  # seconds
DEFAULT_MIN_OFF_DURATION = 30  # seconds
DEFAULT_RAMP_RATE = 20.0  # %/second
DEFAULT_RAMP_INTERVAL = 2.0  # seconds between ramp ticks

# Tolerance defaults
DEFAULT_TOLERANCE = 0.3

# Config keys
CONF_HEATER = "heater"
CONF_SENSOR = "target_sensor"
CONF_OUTDOOR_SENSOR = "outdoor_sensor"
CONF_WINDOW_SENSOR = "window_sensor"
CONF_PAUSE_SIGNAL = "pause_signal"
CONF_MIN_TEMP = "min_temp"
CONF_MAX_TEMP = "max_temp"
CONF_TARGET_TEMP = "target_temp"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_KE = "ke"
CONF_DEAD_ZONE = "dead_zone"
CONF_OUTPUT_MAX = "output_max"
CONF_NIGHT_MAX = "night_max"
CONF_PRESET_SPEEDS = "preset_speeds"
CONF_KEEP_ALIVE = "keep_alive"
CONF_SAMPLING_PERIOD = "sampling_period"
CONF_SENSOR_STALL = "sensor_stall"
CONF_OUTPUT_SAFETY = "output_safety"
CONF_MIN_ON_DURATION = "min_on_duration"
CONF_MIN_OFF_DURATION = "min_off_duration"
CONF_RAMP_RATE = "ramp_rate"
CONF_DEBUG = "debug"
CONF_INITIAL_HVAC_MODE = "initial_hvac_mode"
CONF_HOT_TOLERANCE = "hot_tolerance"
CONF_COLD_TOLERANCE = "cold_tolerance"

# Preset names
PRESET_AUTO = "auto"
PRESET_NIGHT = "night"
PRESET_LOW = "low"
PRESET_MEDIUM = "medium"
PRESET_HIGH = "high"
