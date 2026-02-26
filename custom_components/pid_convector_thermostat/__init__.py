"""PID Convector Thermostat for Home Assistant.

A PID-controlled climate platform for VDC fan convectors (trench heaters)
with dead-zone fan control, pause signals, ramp rate, and anti-cycling.
"""

DOMAIN = "pid_convector_thermostat"
PLATFORMS = ["climate"]
