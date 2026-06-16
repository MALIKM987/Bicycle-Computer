# UART and CSV Protocol

## Purpose

UART output is used for debugging, validation during development, data logging and plotting ride parameters on a PC.

## UART Settings

Recommended settings:

- baud rate: 115200,
- data bits: 8,
- parity: none,
- stop bits: 1,
- flow control: none.

## Diagnostic Mode

Diagnostic mode should print human-readable messages, for example sensor initialization status, current speed, altitude, power estimate and system errors.

## CSV Mode

CSV mode should print one header line followed by numeric records.

Recommended CSV fields:

- time in milliseconds,
- speed in km/h,
- trip distance in meters,
- moving time in seconds,
- average speed,
- maximum speed,
- temperature,
- pressure,
- humidity,
- altitude,
- grade,
- VAM,
- roll,
- pitch,
- estimated power,
- estimated energy.

## Recommended Logging Frequency

| Mode | Frequency |
|---|---:|
| Diagnostic text | 1 Hz |
| CSV row | 1 Hz |
| High-rate IMU debug | optional, 5–20 Hz |

## Future Extensions

- Add UART command parser.
- Add commands for calibration.
- Add commands for page switching.
- Add compact logging mode.
