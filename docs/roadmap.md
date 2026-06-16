# Roadmap

## Current Stage

The project is currently an STM32-based prototype focused on sensor integration, ride metrics and OLED/UART presentation.

## Milestone 1 – Repository Quality

- [ ] Replace README with full technical documentation.
- [ ] Add architecture documentation.
- [ ] Add hardware pinout.
- [ ] Add calibration guide.
- [ ] Add testing plan.
- [ ] Add screenshots and wiring photos.
- [ ] Add sample UART CSV logs.

## Milestone 2 – Firmware Cleanup

- [ ] Reduce logic inside `main.c`.
- [ ] Split code into modules.
- [ ] Add configuration header.
- [ ] Add error handling layer.
- [ ] Add consistent units in variable names.
- [ ] Add comments only where they explain non-obvious logic.

## Milestone 3 – Measurement Validation

- [ ] Validate speed against GPS.
- [ ] Validate distance on known route.
- [ ] Validate altitude and grade.
- [ ] Tune EMA filter.
- [ ] Tune Hall debounce.
- [ ] Tune stop timeout.

## Milestone 4 – Data Logging

- [ ] Stable CSV output.
- [ ] Example CSV files.
- [ ] Python plotting script.
- [ ] Ride summary generation.

## Milestone 5 – Low Power

- [ ] Analyze power consumption.
- [ ] Add sleep/stop mode concept.
- [ ] Reduce OLED refresh rate when stationary.
- [ ] Reduce sensor polling when stopped.

## Milestone 6 – Custom Hardware

- [ ] Prepare schematic.
- [ ] Prepare PCB.
- [ ] Add SWD connector.
- [ ] Add sensor connectors.
- [ ] Add power protection.
- [ ] Add enclosure concept.
