# Calibration Guide

## Wheel Circumference

Wheel circumference is required for speed and distance calculation.

Recommended procedure:

1. Mark the tire and floor.
2. Roll the bicycle forward by one full wheel revolution.
3. Measure distance between marks.
4. Enter the value in meters in the firmware configuration.

## Hall Sensor Debounce

The debounce time prevents false pulses.

If the value is too low, noise may be counted as additional wheel revolutions. If the value is too high, valid pulses at high speed may be ignored.

## Stop Timeout

The stop timeout determines when speed should be forced to zero after the last pulse.

## Altitude Calibration

Pressure-based altitude depends on reference sea-level pressure. For better results, use local weather pressure or calibrate altitude manually at a known location.

## Power Model Calibration

Recommended parameters:

| Parameter | Meaning |
|---|---|
| Total mass | rider + bicycle + equipment |
| Crr | rolling resistance coefficient |
| CdA | aerodynamic drag area |
| Air density | can be approximated from weather |
| Human efficiency | used for kcal estimation |

## Field Validation

During first rides, compare:

- speed with GPS,
- distance with known route,
- altitude with map data,
- grade with known climbs,
- power with realistic effort level.
