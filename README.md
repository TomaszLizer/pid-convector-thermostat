# PID Convector Thermostat

[![hacs_badge](https://img.shields.io/badge/HACS-Custom-41BDF5.svg)](https://github.com/hacs/integration)

A Home Assistant custom component for PID-controlled VDC fan convectors (trench heaters, fan coil units). Purpose-built for variable-speed fan heating systems that require:

- **Dead-zone fan control with hysteresis** — fan shuts off completely below a configurable threshold, with separate on/off thresholds to prevent oscillation
- **Minimum output floor** — motor safety: fan never runs below a configurable minimum speed
- **Soft start/ramp rate** — gradual fan speed changes to reduce mechanical stress and noise
- **Anti-cycling deadband** — prevents rapid on/off switching
- **Pause signals** — instantly stops fans when windows open or boiler is off
- **PID with outdoor compensation** — accurate temperature control with weather-aware tuning
- **Proper derivative on keep-alive** — derivative term holds between sensor updates instead of collapsing to zero

## Why this component?

Standard HA thermostat components (generic_thermostat, smart_thermostat) don't handle the unique requirements of VDC fan convectors:

| Problem | This Component's Solution |
|---|---|
| Fan motor stalls at very low speeds (e.g., <15%) | **Dead zone with hysteresis**: output below threshold -> fan OFF, with separate on/off thresholds to prevent oscillation at boundary |
| Fan runs too slow even above dead zone | **Output min**: configurable minimum fan speed floor (e.g., 10%) for motor safety |
| PID output stuck at minimum overheats room | Dead zone allows PID to naturally request 0% through the dead zone |
| Rapid on/off cycling damages motors | **Anti-cycling deadband**: configurable minimum on/off durations |
| Abrupt speed changes are noisy | **Ramp rate**: gradual speed transitions (configurable %/second) |
| Open window + running fan = wasted energy | **Pause signals**: instant fan stop on window open, smooth resume on close |
| Boiler cycling causes unnecessary fan operation | **Pause signals**: generic pause input for boiler state, etc. |
| Derivative term disappears between sensor updates | **Separate timing**: derivative uses sensor-change intervals, integral uses calc intervals |

## Installation

### HACS (Recommended)

1. Open HACS → Integrations → ⋮ → Custom repositories
2. Add `https://github.com/TomaszLizer/pid-convector-thermostat` as Integration
3. Install "PID Convector Thermostat"
4. Restart Home Assistant

### Manual

1. Copy `custom_components/pid_convector_thermostat/` to your HA `custom_components/` directory
2. Restart Home Assistant

## Configuration

Add to your `configuration.yaml`:

```yaml
climate:
  - platform: pid_convector_thermostat
    name: "Living Room Heater"
    unique_id: climate_living_room_heater
    heater: light.trench_heater_living_room    # Light entity controlling fan speed via brightness
    target_sensor: sensor.living_room_temperature
    outdoor_sensor: sensor.outdoor_temperature  # Optional
    window_sensor: binary_sensor.living_room_window  # Optional
    pause_signal: binary_sensor.boiler_heating  # Optional
    min_temp: 15
    max_temp: 25
    target_temp: 21
    kp: 15
    ki: 0.01
    kd: 300
    ke: 0.5
    dead_zone: 15          # PID output below 15% -> fan OFF (with hysteresis: OFF below 9%)
    output_min: 10         # Fan never runs below 10% (motor safety)
    output_max: 70         # Maximum fan speed in auto mode
    night_max: 40          # Maximum fan speed in night mode
    min_on_duration: 30    # Minimum seconds fan stays ON
    min_off_duration: 30   # Minimum seconds fan stays OFF
    ramp_rate: 20          # Maximum speed change: 20%/second
    keep_alive:
      seconds: 10
    debug: true
    preset_speeds:         # Optional, defaults shown
      low: 25
      medium: 45
      high: 65
```

## Parameters

### Required

| Parameter | Description |
|---|---|
| `heater` | `entity_id` of a light entity that controls fan speed via `brightness_pct` |
| `target_sensor` | `entity_id` of a temperature sensor |
| `keep_alive` | Update interval for the control loop (e.g., `seconds: 60`) |

### PID Tuning

| Parameter | Default | Description |
|---|---|---|
| `kp` | 15.0 | Proportional gain |
| `ki` | 0.01 | Integral gain |
| `kd` | 300.0 | Derivative gain |
| `ke` | 0.0 | Outdoor temperature compensation gain |

### Output Control

| Parameter | Default | Description |
|---|---|---|
| `dead_zone` | 20 | PID output below this (%) -> fan OFF. Uses hysteresis: ON at `dead_zone`, OFF at `dead_zone * 0.6` |
| `output_min` | 0 | Minimum fan speed (%) when running. Non-zero output is clamped to `[output_min, 100]` for motor safety |
| `output_max` | 70 | Maximum fan speed (%) in `auto` preset |
| `night_max` | 40 | Maximum fan speed (%) in `night` preset |
| `preset_speeds` | `{low: 25, medium: 45, high: 65}` | Fixed speeds for manual presets |

### Anti-cycling & Ramp

| Parameter | Default | Description |
|---|---|---|
| `min_on_duration` | 30 | Minimum seconds fan stays ON before turning OFF |
| `min_off_duration` | 30 | Minimum seconds fan stays OFF before turning ON |
| `ramp_rate` | 20 | Maximum speed change in %/second (0 = instant) |

### Safety

| Parameter | Default | Description |
|---|---|---|
| `sensor_stall` | `06:00:00` | If sensor doesn't update for this duration, enter safety mode |
| `output_safety` | 0 | Fan speed (%) during sensor stall (0 = fan off) |

### Optional Sensors

| Parameter | Description |
|---|---|
| `outdoor_sensor` | Outdoor temperature sensor for `ke` compensation |
| `window_sensor` | Binary sensor — fan pauses immediately when ON |
| `pause_signal` | Binary sensor — generic pause (e.g., boiler not heating) |

### Other

| Parameter | Default | Description |
|---|---|---|
| `min_temp` | 7 | Minimum settable temperature |
| `max_temp` | 35 | Maximum settable temperature |
| `target_temp` | — | Initial target temperature |
| `sampling_period` | 0 | Minimum time between PID calculations (0 = every sensor update) |
| `cold_tolerance` | 0.3 | Tolerance below setpoint (PID OFF mode) |
| `hot_tolerance` | 0.3 | Tolerance above setpoint (PID OFF mode) |
| `initial_hvac_mode` | — | Force HVAC mode on start (`heat`, `fan_only`, `off`) |
| `debug` | false | Expose PID internals as state attributes |

## HVAC Modes

| Mode | Description |
|---|---|
| **Heat** | PID-controlled heating with dead-zone fan control |
| **Fan Only** | Fixed fan speed (no PID, no heating demand) — summer air circulation |
| **Off** | Everything off |

## Presets

Presets change dynamically based on HVAC mode:

| Preset | In Heat Mode | In Fan Only Mode |
|---|---|---|
| **auto** | PID control, max = `output_max` | — |
| **night** | PID control, max = `night_max` (quiet) | — |
| **low** | Fixed speed from `preset_speeds.low` | Fixed speed from `preset_speeds.low` |
| **medium** | Fixed speed from `preset_speeds.medium` | Fixed speed from `preset_speeds.medium` |
| **high** | Fixed speed from `preset_speeds.high` | Fixed speed from `preset_speeds.high` |

## Dead Zone Behavior

The dead zone uses **hysteresis** to prevent oscillation at the boundary. With the default hysteresis factor of 0.6:

- **Turn ON threshold**: PID output >= `dead_zone` (e.g., 15%)
- **Turn OFF threshold**: PID output < `dead_zone * 0.6` (e.g., 9%)
- **Between thresholds**: fan maintains its current state

```
Example: dead_zone: 15, output_min: 10

PID Output:    0%  ···  8%  |  9-14%  |  15%  ···  70%
                            |         |
Fan OFF:       0%   0%  0%  |  hold   |
Fan ON:                     |  hold   |  15%  ···  70%  (clamped to >= output_min)
                    ↑       | hyster. |   ↑
              below OFF     |  band   | above ON threshold
              threshold     |         |
```

When `output_min` is set (e.g., 10), any non-zero fan speed is clamped to at least that value, ensuring the motor never runs at dangerously low speeds.

## Pause Behavior

| Event | Action |
|---|---|
| **Window opens / Pause activates** | Fan stops **immediately** (bypasses deadband and ramp) |
| **Window closes / Pause deactivates** | Resume with **deadband** (waits `min_off_duration`) and **ramp** (soft start) |

PID integral is preserved during pause; derivative history is cleared to prevent transient spikes on resume.

## Services

### `pid_convector_thermostat.set_pid_gain`

Adjust PID gains without restarting HA. Values are saved in entity state and restored after restart.

```yaml
service: pid_convector_thermostat.set_pid_gain
data:
  kp: 12.0
  ki: 0.008
target:
  entity_id: climate.living_room_heater
```

### `pid_convector_thermostat.clear_integral`

Reset the PID integral to zero. Useful during tuning.

```yaml
service: pid_convector_thermostat.clear_integral
target:
  entity_id: climate.living_room_heater
```

## Debug Attributes

When `debug: true`, the following attributes are exposed:

| Attribute | Description |
|---|---|
| `pid_p` | Proportional component |
| `pid_i` | Integral component |
| `pid_d` | Derivative component |
| `pid_e` | External (outdoor) compensation component |
| `pid_dt` | Time since last PID calculation (integral interval) |
| `pid_deriv_dt` | Time since last actual sensor change (derivative interval) |
| `pid_tick` | PID calculation counter (for verifying keep-alive is firing) |
| `control_output` | Raw PID output (before dead zone) |
| `target_output` | Desired fan speed (after dead zone) |
| `actual_output` | Current fan speed (after ramp) |
| `dead_zone_active` | True when PID wants heating but output is in dead zone |
| `dead_zone_on_threshold` | The dead zone ON threshold value |
| `dead_zone_off_threshold` | The dead zone OFF threshold value (dead_zone * 0.6) |
| `paused` | True when paused by window/signal |
| `pause_reason` | Reason for pause (window_open, pause_signal) |
| `fan_is_on` | Current fan on/off state |

PID gains (`kp`, `ki`, `kd`, `ke`) and `dead_zone` are always exposed regardless of debug setting.

## HomeKit Compatibility

This component uses standard `ClimateEntity` with `HVACMode.HEAT`, `HVACMode.FAN_ONLY`, and `HVACMode.OFF`. The HomeKit bridge maps these natively:

- **Heat/Off toggle** works out of the box
- **Target temperature** slider works out of the box
- **Presets** (auto/night/low/medium/high) can be exposed as HomeKit scenes

## Credits

PID controller based on [Arduino PID Library](https://github.com/br3ttb/Arduino-PID-Library) by Brett Beauregard, adapted from [HASmartThermostat](https://github.com/ScratMan/HASmartThermostat) by ScratMan.

## License

MIT
