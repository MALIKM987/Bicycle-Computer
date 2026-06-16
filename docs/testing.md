# Testing and Validation

## Goals

Testing should verify that the firmware behaves correctly both on the bench and during real rides.

## Bench Tests

| Test | Expected Result |
|---|---|
| Power-on test | Firmware starts without hard fault |
| OLED init | Display shows startup screen |
| BME280 init | Temperature, pressure and humidity are valid |
| IMU init | Roll/pitch values change when board is moved |
| UART output | Serial terminal receives readable data |
| Button test | OLED page changes after button press |
| Hall pulse test | Speed changes after simulated pulses |

## Hall Sensor Simulation

A Hall pulse can be simulated manually or with a signal generator.

Recommended checks:

- no false pulses when idle,
- correct debounce behavior,
- speed returns to zero after stop timeout,
- distance increases by wheel circumference per valid pulse.

## Real Ride Tests

| Metric | Validation Method |
|---|---|
| Speed | Compare with GPS or commercial bike computer |
| Distance | Compare with known route |
| Altitude | Compare with map/phone/barometer |
| Grade | Compare on known climbs |
| UART CSV | Verify row consistency and missing values |
| OLED UI | Check readability outdoors |

## Regression Testing Ideas

For future development, logic-heavy modules can be extracted and tested on PC:

- speed calculation,
- EMA filter,
- grade calculation,
- VAM calculation,
- power estimation,
- CSV formatting.
