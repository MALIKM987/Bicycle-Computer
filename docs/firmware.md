# Firmware Design

## Firmware Responsibilities

The firmware is responsible for initialization, sensor acquisition, interrupt handling, speed and distance calculation, environmental data processing, IMU processing, ride statistics, OLED rendering, UART diagnostics and CSV logging.

## Recommended Module Boundaries

Recommended firmware modules:

- `app` – high-level application coordinator,
- `speed_sensor` – Hall pulse processing, speed and distance,
- `ride_stats` – trip distance, moving time, average and maximum speed,
- `env_sensor` – BME280 measurements and environmental values,
- `imu_sensor` – roll, pitch and vibration indicators,
- `climb_metrics` – altitude, grade, VAM and ascent,
- `power_model` – estimated cycling power and energy,
- `display_ui` – OLED pages and UI navigation,
- `uart_logger` – diagnostics and CSV export,
- `system_time` – timing helpers.

## Main Loop Concept

The main loop should remain non-blocking. Time-based tasks should be executed periodically using timestamps instead of long blocking delays.

## Interrupts

The Hall sensor should be handled through EXTI. The interrupt should only capture timing and set flags. Heavy calculations should be moved to the main loop.

## Error Handling

Recommended error handling:

- sensor initialization status,
- I2C read/write failures,
- invalid sensor values,
- OLED initialization failure,
- UART buffer overflow,
- Hall pulse timeout.

## Coding Guidelines

- Keep `main.c` minimal.
- Keep generated CubeMX sections intact.
- Put user code into separate modules.
- Avoid blocking delays after initialization.
- Use clear units in variable names, for example `_ms`, `_kmh`, `_m`, `_deg`, `_pa`.
- Keep constants in one configuration header.
