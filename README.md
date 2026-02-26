# PID Convector Thermostat

[![hacs_badge](https://img.shields.io/badge/HACS-Custom-41BDF5.svg)](https://github.com/hacs/integration)

A Home Assistant custom component for PID-controlled VDC fan convectors (trench heaters, fan coil units). Purpose-built for variable-speed fan heating systems that require:

- **Dead-zone fan control** — fan shuts off completely below a configurable threshold (no motor stall at low speeds)
- **Soft start/ramp rate** — gradual fan speed changes to reduce mechanical stress and noise
- **Anti-cycling deadband** — prevents rapid on/off switching
- **Pause signals** — instantly stops fans when windows open or boiler is off
- **PID with outdoor compensation** — accurate temperature control with weather-aware tuning

## Why this component?

Standard HA thermostat components (generic_thermostat, smart_thermostat) don't handle the unique requirements of VDC fan convectors:

| Problem | This Component's Solution |
|---|---|
| Fan motor stalls at very low speeds (e.g., <15%) | **Dead zone**: output below threshold → fan OFF (0%), above → runs at that speed |
| PID output stuck at minimum overheats room | Dead zone allows PID to naturally request 0% through the dead zone |
| Rapid on/off cycling damages motors | **Anti-cycling deadband**: configurable minimum on/off durations |
| Abrupt speed changes are noisy | **Ramp rate**: gradual speed transitions (configurable %/second) |
| Open window + running fan = wasted energy | **Pause signals**: instant fan stop on window open, smooth resume on close |
| Boiler cycling causes unnecessary fan operation | **Pause signals**: generic pause input for boiler state, etc. |

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
    dead_zone: 20          # PID output below 20% → fan OFF
    output_max: 70         # Maximum fan speed in auto mode
    night_max: 40          # Maximum fan speed in night mode
    min_on_duration: 30    # Minimum seconds fan stays ON
    min_off_duration: 30   # Minimum seconds fan stays OFF
    ramp_rate: 20          # Maximum speed change: 20%/second
    keep_alive:
      seconds: 60
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
| `dead_zone` | 20 | PID output below this (%) → fan OFF. At or above → fan runs at that speed |
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

```
PID Output:    0%  ···  19%  |  20%  ···  70%
                             |
Fan Speed:     0%   0%   0%  |  20%  ···  70%
                    ↑        |   ↑
              dead zone      | minimum safe speed
```

The PID operates in range [0, output_max]. When its output falls below `dead_zone`, the fan turns **completely off** (light.turn_off). When output rises above `dead_zone`, the fan starts at that speed — already above the minimum safe motor speed.

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
| `pid_dt` | Time delta between last two PID calculations |
| `control_output` | Raw PID output (before dead zone) |
| `target_output` | Desired fan speed (after dead zone) |
| `actual_output` | Current fan speed (after ramp) |
| `dead_zone_active` | True when PID wants heating but output is in dead zone |
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
