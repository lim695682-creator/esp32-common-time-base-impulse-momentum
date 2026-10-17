# Wiring notes for the ESP32 impulse–momentum apparatus

This note records the reference wiring used by the S1 reproduction guide. Treat it as a pin and verification reference, not as a substitute for the specifications and measurements of the physical modules.

## Hardware

- ESP32-D0WDQ6-V3 development board with CH340 USB-UART
- LM2596 adjustable regulator, set to 3.3 V
- NAU7802 24-bit bridge ADC
- Two 49E linear Hall modules
- S-type four-wire bridge load cell
- Two Hall gates, one magnet, common 3.3 V rail, and common ground

## ESP32 pin map

| Signal | Connection |
| --- | --- |
| Hall A analogue output `AO_A` | ESP32 GPIO34 |
| Hall B analogue output `AO_B` | ESP32 GPIO35 |
| NAU7802 SDA | ESP32 GPIO21 |
| NAU7802 SCL | ESP32 GPIO22 |

The current firmware calls `Wire.begin(21, 22)`, configures GPIO34 and GPIO35 as 12-bit ADC inputs with 11 dB attenuation, and reads Hall analogue outputs (`AO`). Hall digital outputs (`DO`) are not used and should remain unconnected. Older notes assigning `DO` to GPIO25/26 do not apply to the current `esp32_impulse.ino`.

## Power and ground

1. Adjust LM2596 OUT+–OUT− to 3.3 V with a multimeter before wiring the load.
2. Connect the regulated 3.3 V rail to the ESP32-compatible 3.3 V input, NAU7802 VCC, and both Hall-module VCC pins.
3. Connect LM2596 OUT−, ESP32 GND, NAU7802 GND, and both Hall-module GND pins to one common ground.
4. Do not connect USB 5 V directly to a 3.3 V pin. If USB and the external regulator are used together, check the board power path for reverse powering.
5. Do not identify LM2596 IN+/IN− or OUT+/OUT− by wire colour alone; measure the terminals.

## Hall modules

For each 49E module:

- VCC → regulated 3.3 V
- GND → common GND
- AO → ADC input (`AO_A → GPIO34`, `AO_B → GPIO35`)
- DO → leave unconnected

Place Hall A before Hall B in the direction of motion. Fix the magnet and sensors, document their orientation and height, and use the firmware `scan` command to measure the effective trigger spacing. The nominal firmware value is `HALL_DISTANCE_M = 0.0560`; record the actual spacing for each build.

## NAU7802 and load cell

- SDA → GPIO21
- SCL → GPIO22
- VCC → regulated 3.3 V
- GND → common GND
- Firmware data rate request: `NAU7802_SPS_320`

Connect the S-type bridge by function, not by insulation colour:

| Load-cell function | NAU7802 terminal |
| --- | --- |
| Signal positive `S+` | `A+` / `AIN+` |
| Signal negative `S−` | `A−` / `AIN−` |
| Excitation positive `E+` | `E+` |
| Excitation negative `E−` | `E−` |

Use the load-cell data sheet or a continuity/resistance test to identify the four functions. If force polarity is reversed, power down and recheck the bridge mapping before changing software signs.

## I2C pull-ups and capacitors

The reference drawing shows 4.7 kΩ pull-ups from SDA and SCL to 3.3 V. Check whether the installed NAU7802 or ESP32 module already includes pull-ups and decoupling capacitors before adding parallel components. Reproduce capacitors and regulator details only when they match the installed module specification.

## Pre-power verification

- Confirm the exact ESP32 board and ADC-capable GPIO34/GPIO35.
- Confirm Hall analogue outputs never exceed the ESP32 ADC input range.
- Confirm NAU7802 logic voltage and I2C pull-ups are ESP32-compatible.
- Measure 3.3 V, verify common ground, and check for shorts.
- Confirm the load-cell bridge mapping from documentation or resistance tests.
- Fix the magnet and Hall sensors and verify the measured effective spacing.
- Check that simultaneous USB and external power cannot create a reverse-power path.

The firmware source confirms software pin assignments and acquisition protocol; it cannot prove that a particular physical assembly is wired correctly. Record any deviations from this note with the board and sensor part numbers, photographs, measurements, and calibration record.
