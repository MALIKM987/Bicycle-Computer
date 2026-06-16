# Hardware Description

## Prototype Platform

The current prototype targets an STM32L432KC / NUCLEO-L432KC board.

## Main Components

| Component | Interface | Purpose |
|---|---|---|
| Hall effect sensor | GPIO / EXTI | Wheel rotation detection |
| BME280 | I2C | Temperature, pressure, humidity |
| BMI160 or compatible IMU | I2C | Acceleration, orientation, vibration |
| SSD1306 OLED 128x64 | I2C | Local display |
| Button | GPIO | User interface navigation |
| UART | USART | Diagnostics and CSV logging |

## Sensor Notes

### Hall Sensor

The Hall sensor is used for speed and distance measurement. Each valid pulse corresponds to wheel rotation. A debounce mechanism is required to avoid false triggering caused by noise or mechanical vibration.

### BME280

The BME280 provides temperature, pressure and humidity. Pressure is also used to estimate altitude, but absolute altitude requires calibration against local reference pressure.

### IMU

The IMU provides acceleration data that can be used to estimate roll, pitch and vibration indicators.

### OLED

The SSD1306 OLED provides a compact low-power interface for live ride data.

## Future Hardware Revision

A future custom PCB may include:

- STM32L4 microcontroller,
- regulated low-power supply,
- battery input,
- charging/protection section,
- I2C sensor connector,
- Hall sensor connector,
- OLED connector,
- SWD programming header,
- optional external flash or EEPROM,
- weather-resistant enclosure support.
