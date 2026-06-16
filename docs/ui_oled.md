# OLED User Interface

## Display

The project uses a 128x64 SSD1306 OLED display.

## UI Philosophy

The display should show only the most relevant information on each page. A bicycle computer must be readable quickly, so each screen should avoid unnecessary clutter.

## Suggested Pages

| Page | Content |
|---|---|
| RIDE | Speed, trip distance, moving time |
| CLIMB | Altitude, grade, VAM, ascent |
| ENV | Temperature, pressure, humidity, dew point |
| IMU | Roll, pitch, vibration |
| STATS | Average speed, max speed, power, energy |

## Button Handling

A single button can cycle through pages:

```text
RIDE -> CLIMB -> ENV -> IMU -> STATS -> RIDE
```

Recommended logic:

- short press: next page,
- long press: reset trip or open settings,
- debounce input in software.

## Future UI Improvements

- Add icons.
- Add low-battery indicator.
- Add ride recording indicator.
- Add settings screen.
- Add error/status page.
