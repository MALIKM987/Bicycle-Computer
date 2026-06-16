# Bicycle Computer – STM32 Embedded Bike Computer

[![Platform](https://img.shields.io/badge/platform-STM32-blue)](#)
[![Language](https://img.shields.io/badge/language-C-blue)](#)
[![MCU](https://img.shields.io/badge/MCU-STM32L432KC-blue)](#)
[![Status](https://img.shields.io/badge/status-prototype-orange)](#)
[![Project](https://img.shields.io/badge/project-embedded%20systems-green)](#)

## Overview

**Bicycle Computer** is an embedded system project developed in C for an STM32 microcontroller.  
The goal of the project is to build a modular bicycle computer capable of measuring, processing and displaying ride, environmental and motion-related data in real time.

The project is designed as a practical embedded engineering prototype, not only as a simple speedometer. It combines external sensors, interrupt-based pulse counting, periodic measurements, filtering, derived ride metrics, OLED user interface pages and UART diagnostic output.

## Key Features

- Speed and distance measurement using a Hall effect wheel sensor.
- Ride statistics: trip distance, moving time, average speed and maximum speed.
- Environmental measurements using a BME280 sensor:
  - temperature,
  - pressure,
  - relative humidity,
  - estimated altitude,
  - dew point.
- Motion and orientation data using an IMU:
  - roll,
  - pitch,
  - vibration/RMS indicator.
- Climbing-related metrics:
  - grade,
  - VAM,
  - accumulated ascent.
- Estimated cycling power model:
  - climbing power,
  - rolling resistance,
  - aerodynamic drag,
  - total estimated power,
  - estimated energy expenditure.
- OLED interface with multiple pages.
- UART diagnostic and CSV output for logging and analysis.
- Modular firmware structure suitable for further extension.

## Hardware Platform

The current prototype is based on:

| Component | Role |
|---|---|
| STM32L432KC / NUCLEO-L432KC | Main microcontroller platform |
| Hall effect sensor | Wheel pulse detection for speed and distance |
| BME280 | Temperature, pressure and humidity measurements |
| BMI160 or compatible IMU | Acceleration, orientation and vibration-related data |
| SSD1306 OLED 128x64 | Local user interface |
| UART interface | Diagnostics and CSV logging |

> The hardware layer is intended to be portable. The current version targets a development board, while a future revision may use a custom PCB.

## System Architecture

```text
+-------------------+
|  Hall Sensor       |
|  wheel pulses      |
+---------+---------+
          |
          v
+-------------------+       +-------------------+
|  EXTI / Timer      |       |  BME280 / IMU      |
|  pulse processing  |       |  I2C sensors       |
+---------+---------+       +---------+---------+
          |                           |
          v                           v
+------------------------------------------------+
|                  STM32 Firmware                 |
|------------------------------------------------|
| speed/distance | filtering | altitude | grade  |
| VAM            | power model | ride statistics  |
+---------------------+--------------------------+
                      |
        +-------------+-------------+
        |                           |
        v                           v
+-------------------+       +-------------------+
| SSD1306 OLED       |       | UART CSV / DIAG    |
| user interface     |       | external logging   |
+-------------------+       +-------------------+
```

More details are available in [`docs/architecture.md`](docs/architecture.md).

## Firmware Modules

Recommended high-level module split:

| Module | Responsibility |
|---|---|
| `speed` | Hall pulse processing, speed, distance, stop timeout |
| `env` | BME280 measurements, pressure, altitude, dew point |
| `imu` | Roll, pitch and vibration indicators |
| `ride_stats` | Trip, moving time, average and maximum speed |
| `climb` | Grade, VAM, ascent |
| `power_model` | Estimated cycling power and energy |
| `display` | OLED pages and UI navigation |
| `logger` | UART diagnostics and CSV export |

## Measurement Logic

### Speed and Distance

The wheel sensor generates one or more pulses per wheel revolution.  
The firmware measures pulse timing, filters speed using EMA and detects a stop condition after a configurable timeout.

```text
distance = wheel_revolutions * wheel_circumference

instant_speed = wheel_circumference / pulse_period

filtered_speed = EMA(instant_speed)
```

### Grade and VAM

Altitude is estimated from pressure. Grade is calculated from altitude change over distance or time window.  
VAM estimates vertical ascent speed in meters per hour.

```text
grade [%] = delta_altitude / delta_distance * 100

VAM [m/h] = delta_altitude / delta_time * 3600
```

### Power Estimation

The estimated total power is the sum of simplified components:

```text
P_total = P_climb + P_rolling + P_aero
```

The model is simplified and should be treated as an estimation, not as a replacement for a calibrated power meter.

More details: [`docs/power_model.md`](docs/power_model.md).

## OLED User Interface

The display is organized into several pages, for example:

| Page | Content |
|---|---|
| RIDE | Speed, trip distance, moving time |
| CLIMB | Altitude, grade, VAM, ascent |
| ENV | Temperature, pressure, humidity, dew point |
| IMU | Roll, pitch, vibration |
| STATS | Average speed, max speed, power, energy |

More details: [`docs/ui_oled.md`](docs/ui_oled.md).

## UART / CSV Output

The project supports UART diagnostics and structured CSV logging.  
This allows ride data to be captured on a PC and analyzed in external tools.

Example CSV fields:

```text
time_ms,speed_kmh,trip_m,moving_time_s,avg_kmh,max_kmh,temp_c,pressure_hpa,humidity_pct,altitude_m,grade_pct,vam_mh,roll_deg,pitch_deg,power_w,kcal
```

More details: [`docs/uart_csv_protocol.md`](docs/uart_csv_protocol.md).

## Repository Structure

Recommended structure:

```text
Bicycle-Computer/
├── Core/
│   ├── Inc/
│   └── Src/
├── Drivers/
├── docs/
│   ├── architecture.md
│   ├── hardware.md
│   ├── firmware.md
│   ├── algorithms.md
│   ├── calibration.md
│   ├── power_model.md
│   ├── testing.md
│   ├── uart_csv_protocol.md
│   ├── ui_oled.md
│   └── roadmap.md
├── hardware/
│   └── pinout.md
├── .github/
│   ├── PULL_REQUEST_TEMPLATE.md
│   └── ISSUE_TEMPLATE/
├── README.md
├── CHANGELOG.md
└── LICENSE
```

## Getting Started

### Requirements

- STM32CubeIDE
- STM32CubeMX
- STM32L4 firmware package
- NUCLEO-L432KC or compatible STM32L4 board
- USB-UART connection or ST-LINK virtual COM port
- I2C OLED and sensors connected according to the pinout

### Clone Repository

```bash
git clone https://github.com/MALIKM987/Bicycle-Computer.git
cd Bicycle-Computer
```

### Build and Flash

1. Open the project in STM32CubeIDE.
2. Verify the `.ioc` configuration.
3. Build the project.
4. Connect the STM32 board using ST-LINK.
5. Flash the firmware.
6. Open serial terminal at `115200 8N1`.

## Calibration

Before real-world testing, the following values should be configured:

| Parameter | Meaning |
|---|---|
| Wheel circumference | Required for speed and distance |
| Hall debounce time | Prevents false pulses |
| Stop timeout | Determines when the bicycle is considered stopped |
| BME280 reference pressure | Required for more reliable altitude |
| Rider + bike mass | Required for power estimation |
| Rolling resistance coefficient | Used in power model |
| CdA estimate | Used in aerodynamic power estimation |

More details: [`docs/calibration.md`](docs/calibration.md).

## Current Status

The project is a functional embedded prototype intended for further development and field validation.

Current development priorities:

- Verify measurements during real rides.
- Improve calibration workflow.
- Add persistent ride data storage.
- Improve low-power operation.
- Prepare custom PCB version.
- Add screenshots, wiring photos and example CSV logs.

## Why This Project Matters

This project demonstrates practical skills in:

- STM32 firmware development,
- C programming for embedded systems,
- interrupt-based measurement,
- sensor integration over I2C,
- real-time data processing,
- embedded UI design,
- UART diagnostics,
- signal filtering,
- basic physical modeling,
- hardware/software integration.

## Author

**Maciej Molik**  
Embedded systems / electronics / STM32 / sensor systems
