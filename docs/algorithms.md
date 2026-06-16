# Algorithms

## Speed Calculation

The Hall sensor produces pulses related to wheel revolutions.

```text
speed_mps = wheel_circumference_m / pulse_period_s
speed_kmh = speed_mps * 3.6
distance_m = wheel_revolutions * wheel_circumference_m
```

## Debounce

False pulses can occur due to noise. A pulse should be ignored if it appears too soon after the previous valid pulse.

```text
if now_ms - last_pulse_ms < debounce_ms:
    ignore pulse
else:
    accept pulse
```

## Stop Detection

If no valid pulse is detected for a configured time, speed should be set to zero.

## EMA Filtering

Speed can be filtered using exponential moving average.

```text
filtered = alpha * new_value + (1 - alpha) * previous_filtered
```

This reduces jitter while preserving reasonable response time.

## Altitude Estimation

Altitude can be estimated from pressure.

```text
altitude_m = 44330 * (1 - (pressure_hpa / sea_level_pressure_hpa)^0.1903)
```

For accurate results, sea level reference pressure should be calibrated.

## Grade

```text
grade_percent = delta_altitude_m / delta_distance_m * 100
```

Grade should be calculated over a window to reduce noise.

## VAM

```text
vam_mh = delta_altitude_m / delta_time_s * 3600
```

VAM is useful for climbing analysis.

## Power Model

```text
P_total = P_climb + P_roll + P_aero
P_climb = mass * g * vertical_speed
P_roll  = Crr * mass * g * speed
P_aero  = 0.5 * rho * CdA * speed^3
```

The result is an estimate and depends heavily on calibration parameters.
