# ESP-IDF: ESP32 DevKit V1 (WROVER) Battery Voltage + Current Monitor

This project measures:
- **Battery voltage** in the range **0–15V DC** (through a resistor divider), and
- **Current** using an **ACS712 10A** Hall current sensor.

It is written for **ESP-IDF** on **ESP32 DevKit V1 (WROVER, 3.3V logic)**.

## Hardware connections

> ⚠️ ESP32 ADC pins are **not 5V tolerant**.

### 1) ACS712-10A connection (current)
- `ACS712 VCC` -> `5V`
- `ACS712 GND` -> `GND` (common with ESP32 and battery negative)
- `ACS712 OUT` -> `GPIO34` (ADC1_CH6)

### 2) Battery voltage 0–15V to ADC (voltage)
Use divider:
- `R_TOP = 100k` from `BAT+` to ADC node
- `R_BOTTOM = 27k` from ADC node to `GND`
- ADC node -> `GPIO35` (ADC1_CH7)

With this divider,
`Vadc = Vbat * (27k / (100k + 27k))`
So at 15V battery, ADC sees ~3.19V (close to limit; for extra margin use 120k/27k).

### 3) Grounding
Must share common ground:
- battery negative
- ACS712 GND
- ESP32 GND

## Build and flash

```bash
idf.py set-target esp32
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Notes on accuracy
- ACS712 has noise and offset drift; startup auto-calibration is implemented.
- Use thicker traces/wires for current path through ACS712.
- For better voltage accuracy, use precision resistors (1% or better) and optionally measure/adjust resistor values in code.
- For best ADC stability on ESP32, keep analog wiring short and add small RC filtering if needed.
