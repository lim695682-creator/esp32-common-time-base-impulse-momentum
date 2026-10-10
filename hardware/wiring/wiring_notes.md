# Wiring tutorial

This page translates the pin assignments in `firmware/esp32_impulse.ino` and the supplied schematic into a connection checklist. The current hardware identifiers supplied for this apparatus are an ESP32-D0WDQ6-V3 board with CH340 USB-UART, an LM2596 regulator adjusted to 3.3 V, a NAU7802 24-bit ADC breakout, two 49E linear Hall modules, and an S-type four-wire bridge load cell. Practical operation has confirmed that the main supply voltages show no noticeable deviation under the tested setup; exact meter readings and load-dependent measurements have not been archived. It is a build aid, not a replacement for the datasheet of the exact boards or sensors.

Reference files: [PNG preview](assets/momentum_lab_complete_circuit_reference.png) and [editable SVG](assets/momentum_lab_complete_circuit_reference.svg). The drawing was found in the project workspace and is retained as a reference. Before treating it as the final schematic, replace its generic `ESP32 DevKit`/`U5 LDO 3.3V` labels with the actual CH340 board and LM2596 wiring, and verify every rail on the assembled unit.

## What the sketch confirms

| Signal | Connect to | Evidence in the sketch |
| --- | --- | --- |
| Hall A analog output (`AO_A`) | ESP32 GPIO 34 | `HALL_A_PIN = 34` |
| Hall B analog output (`AO_B`) | ESP32 GPIO 35 | `HALL_B_PIN = 35` |
| NAU7802 SDA | ESP32 GPIO 21 | `Wire.begin(21, 22)` |
| NAU7802 SCL | ESP32 GPIO 22 | `Wire.begin(21, 22)` |
| Hall and NAU7802 power/ground | A compatible 3.3 V rail and common GND | Required by the signal reference and shown in the supplied schematic |

The ESP32-D0WDQ6-V3 label identifies the ESP32 module/chip family; retain a photo or product link for the complete CH340 development-board variant used in the build.

The firmware configures GPIO 34 and 35 as 12-bit, 11 dB attenuated analog inputs. The Hall modules' digital outputs (`DO`) are not read and should remain unconnected unless the firmware is deliberately changed.

## Connection steps

### 1. Make the common rails

1. The tested setup used an LM2596 adjusted to 3.3 V, and practical operation found no noticeable supply-voltage deviation. Before reproducing the setup, still verify `OUT+` to `OUT-` with a multimeter and record the reading. Identify the regulator's `IN+ / IN-` and `OUT+ / OUT-`; do not rely on wire colour.
2. Connect the regulated 3.3 V output to the ESP32 board's compatible 3.3 V supply input and to the NAU7802 `VCC`. Connect it to each Hall module's `VCC`.
3. Connect the regulator negative output and the ESP32 `GND` to the NAU7802 `GND` and to the `GND` pin of both Hall modules.
4. Keep the grounds common. A Hall output or I2C signal must not be referenced to a separate, floating ground.

The CH340 is the USB-to-serial interface on the development board; it is not a sensor input and requires no separate connection in the apparatus wiring. If USB is connected at the same time as the LM2596 supply, confirm the board's power-path design first to avoid back-feeding two supplies.

The supplied schematic also shows local decoupling capacitors and a 3.3 V regulator. The capacitor values and regulator arrangement must be checked against the actual breakout boards; do not add a regulator or connect USB 5 V directly to a 3.3 V pin without confirming the board's power design.

### 2. Connect the two Hall modules

| Hall module terminal | ESP32 / rail | Purpose |
| --- | --- | --- |
| `VCC` | Compatible 3.3 V rail | Sensor supply |
| `GND` | Common GND | Signal reference |
| `AO_A` on Hall A | GPIO 34 | Analog gate A |
| `AO_B` on Hall B | GPIO 35 | Analog gate B |
| `DO` on either module | Leave unconnected | Not used by this firmware |

Place the two sensors along the track in the order used by the experiment. The current firmware setting is `HALL_DISTANCE_M = 0.0560`, but the effective magnetic trigger spacing must be checked on the assembled apparatus with the `scan` command. The magnet must pass each sensor consistently; sensor orientation and mechanical height are installation-specific.

### 3. Connect the NAU7802 and load cell

| NAU7802 terminal | ESP32 / load cell | Notes |
| --- | --- | --- |
| `SDA` | GPIO 21 | I2C data |
| `SCL` | GPIO 22 | I2C clock |
| `VCC` | Regulated 3.3 V rail | Use the label and voltage limits of the breakout |
| `GND` | Common GND | Required for I2C and ADC reference |
| `A+` / `AIN+` | Load-cell `S+` signal lead | Exact colour is not universal |
| `A-` / `AIN-` | Load-cell `S-` signal lead | Exact colour is not universal |
| `E+` | Load-cell `E+` excitation lead | Exact colour is not universal |
| `E-` | Load-cell `E-` excitation lead | Exact colour is not universal |

The four load-cell conductors must be identified from the load-cell datasheet or a continuity/resistance check. Do not infer `A+`, `A-`, `E+`, and `E-` from wire colour alone. If the measured force has the wrong sign, power down and verify the bridge assignment before changing software signs.

The firmware starts I2C with `Wire.begin(21, 22)` and requests `NAU7802_SPS_320`. It does not prove that a particular breakout has the same silkscreen, pull-up resistors, connector order, or power input as the schematic.

### 4. Connect the 49E Hall modules

Each module exposes `VCC`, `GND`, `AO`, and `DO`:

| 49E terminal | Connection |
| --- | --- |
| `VCC` | Regulated 3.3 V rail |
| `GND` | Common GND |
| Hall A `AO` | ESP32 GPIO34 |
| Hall B `AO` | ESP32 GPIO35 |
| `DO` on either module | Leave unconnected in the current firmware |

The earlier project guide that connects `DO` to GPIO25/26 belongs to an earlier digital Hall prototype. It does not match `firmware/esp32_impulse.ino`, which samples the 49E analog outputs and detects pulse peaks. Mount Hall A before Hall B along the cart path, and verify the effective spacing rather than assuming the nominal mechanical distance.

### 5. I2C pull-ups and optional capacitors

The supplied diagram depicts 4.7 kOhm pull-ups on SDA and SCL to 3.3 V and local capacitors. Many NAU7802 and ESP32 breakout boards already include some of these parts. Check for duplicate pull-ups and follow the breakout documentation; do not blindly parallel components from the diagram.

## Pre-power checklist

- Confirm the exact ESP32 board model and that GPIO 34/35 are input-capable ADC pins.
- Confirm every Hall analog output stays within the ESP32 ADC input range; never apply 5 V to GPIO 34 or 35.
- Confirm the NAU7802 logic supply and I2C pull-up voltage are compatible with the ESP32.
- Confirm there is one common ground and no accidental short between `3V3` and `GND`.
- Confirm the load-cell bridge mapping from the cell datasheet or a documented resistance test.
- Keep the magnet and Hall sensors mechanically fixed, and measure the effective spacing.

## First boot and calibration

1. Upload the firmware and open Serial Monitor at `115200` baud.
2. Check the boot diagnostics for the pin assignment and sensor status.
3. With no load on the cell, issue `tare`/`t` and keep the platform still.
4. Use `scan` to record Hall raw levels and verify two distinct pulses in the expected order.
5. Use `cal` only as a diagnostic stream; record the calibration method and constants with the trial.
6. Run a controlled test and retain the raw Serial/BLE log together with the exported CSV.

## Scope and limitations

The ino file is sufficient to identify the software pins and acquisition protocol, but it is not sufficient to certify the physical wiring. The exact ESP32 board, Hall part number, power circuit, load-cell wire colours, mechanical dimensions, and calibration record still need to be confirmed on the real apparatus. The supplied schematic should therefore be treated as a reference drawing until those checks are documented.
