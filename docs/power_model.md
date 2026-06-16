# Cycling Power Model

## Purpose

The power model estimates the rider's mechanical power based on speed, slope and simplified physical assumptions.

This is not a replacement for a calibrated cycling power meter. It is intended as an engineering approximation useful for learning, diagnostics and trend observation.

## Model

The total estimated power is calculated as the sum of simplified components:

- climbing power,
- rolling resistance power,
- aerodynamic drag power.

## Components

### Climbing Power

Climbing power depends on total mass, gravitational acceleration and vertical speed.

### Rolling Resistance

Rolling resistance depends on rolling resistance coefficient, total mass, gravitational acceleration and bicycle speed.

### Aerodynamic Drag

Aerodynamic drag depends on air density, drag area and speed. This component grows very quickly with speed.

## Limitations

The model does not currently include:

- wind speed,
- drivetrain losses,
- road surface changes,
- braking,
- acceleration power,
- exact rider position,
- exact tire pressure and tire model.

## Calibration Parameters

| Parameter | Meaning |
|---|---|
| Total mass | rider + bicycle + luggage |
| Crr | rolling resistance coefficient |
| CdA | aerodynamic drag area |
| Air density | can be approximated from weather |
| Human efficiency | used for kcal estimation |

## Recommended Future Improvements

- Add wind speed compensation.
- Add acceleration component.
- Add configurable rider profiles.
- Add persistent calibration settings.
