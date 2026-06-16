# System Architecture

## Purpose

The system is designed as a modular bicycle computer running on an STM32 microcontroller. The firmware collects data from multiple sensors, processes it in real time and presents the results through an OLED user interface and UART logging.

## High-Level Architecture

```text
Sensors                    Processing                  Outputs
---------------------------------------------------------------------------
Hall sensor  ----------->  speed/distance module  ---> OLED / UART / stats
BME280       ----------->  env/altitude module    ---> OLED / UART / climb
IMU          ----------->  motion module          ---> OLED / UART / diagnostics
Button       ----------->  UI controller          ---> OLED page switching
Timer        ----------->  scheduler              ---> periodic tasks
```

## Data Flow

1. The Hall sensor generates pulses when the wheel rotates.
2. The EXTI interrupt registers pulses and timestamps them.
3. The speed module calculates instant and filtered speed.
4. Periodic tasks read BME280 and IMU data.
5. Derived metrics are calculated: altitude, grade, VAM, estimated power and ride statistics.
6. The OLED display is refreshed periodically.
7. UART outputs diagnostics or CSV logs.

## Suggested Task Timing

| Task | Suggested Period |
|---|---:|
| Hall pulse interrupt | event-driven |
| Speed update | 5–20 Hz |
| OLED refresh | 2–5 Hz |
| IMU readout | 5 Hz |
| BME280 readout | 1 Hz |
| UART CSV output | 1 Hz |
| Ride statistics update | 1–5 Hz |

## Design Goals

- Keep sensor acquisition separated from calculations.
- Keep display logic separated from measurement logic.
- Avoid blocking delays in the main loop.
- Make every module testable independently.
- Use UART logs to validate real-world behavior.
