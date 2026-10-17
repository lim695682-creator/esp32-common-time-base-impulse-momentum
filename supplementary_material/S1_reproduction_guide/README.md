# S1. Reproduction guide

This guide is the operational companion to *An ESP32-Based Apparatus with a Common Time Base for Impulse–Momentum Measurement*.

## 1. Hardware inventory and build record

The apparatus consists of an inclined track, cart, interchangeable collision head, S-type four-wire load cell, NAU7802 24-bit bridge ADC, ESP32-D0WDQ6-V3 development board (with CH340 USB-UART), two 49E linear Hall modules, two Hall gates, a magnet, and an LM2596 adjustable regulator. The nominal Hall-gate centre-to-centre distance used by the firmware is `HALL_DISTANCE_M = 0.0560` m. Measure and record the effective triggering distance on the assembled apparatus; the nominal value is not a substitute for this measurement.

Keep the hardware inventory, board photographs, module part numbers, and wiring record with each reproduction. The detailed wiring note is stored with this guide at [`wiring/wiring_notes.md`](wiring/wiring_notes.md).

## 2. Power rails and common ground

1. Before connecting the circuit, adjust the LM2596 output to **3.3 V** with a multimeter. Measure and record OUT+–OUT− at the terminals actually used in the build. Do not identify IN+/IN− or OUT+/OUT− by wire colour alone.
2. Connect the regulated 3.3 V rail to the ESP32-compatible 3.3 V input, NAU7802 VCC, and VCC of both Hall modules.
3. Connect LM2596 OUT−, ESP32 GND, NAU7802 GND, and the two Hall-module GND pins to one common ground. Check continuity and check for shorts before applying power.
4. Never connect USB 5 V directly to a 3.3 V pin. If USB and the external regulator are used at the same time, verify the development-board power path so that the supplies cannot drive one another through a reverse-power path.

The regulator setting and common-ground measurement are part of the reproduction record. A schematic alone cannot establish that the installed module has the same power path.

## 3. ESP32 pin map and firmware correspondence

| Function | Module signal | ESP32 pin / setting | Status |
| --- | --- | --- | --- |
| Hall A analogue signal | `AO_A` | GPIO34, 12-bit ADC, 11 dB attenuation | Used by current firmware |
| Hall B analogue signal | `AO_B` | GPIO35, 12-bit ADC, 11 dB attenuation | Used by current firmware |
| NAU7802 I2C data | SDA | GPIO21 | `Wire.begin(21, 22)` |
| NAU7802 I2C clock | SCL | GPIO22 | `Wire.begin(21, 22)` |

The current firmware reads the Hall analogue outputs (`AO`), not the digital outputs (`DO`). Earlier notes that assign Hall `DO` to GPIO25/26 do not apply to the current `esp32_impulse.ino`; leave `DO` unconnected unless a future firmware revision explicitly uses it. Confirm that the selected ESP32 board exposes GPIO34 and GPIO35 as ADC inputs and that no Hall output can exceed the ESP32 ADC input range.

## 4. Hall A/B wiring and mechanical placement

For each 49E Hall module, wire **VCC → regulated 3.3 V**, **GND → common GND**, and **AO → the designated ESP32 ADC input**: Hall A `AO_A → GPIO34`; Hall B `AO_B → GPIO35`. Leave `DO` floating/unconnected for the current firmware. Do not apply 5 V to GPIO34 or GPIO35.

Place Hall A before Hall B in the cart's direction of motion so that the firmware observes the intended incident A→B sequence. Fix the magnet and both sensors mechanically, keep their orientation and height documented, and verify that the magnet produces two clean analogue transitions. Use the firmware `scan` command to inspect the actual trigger interval and record the measured effective distance. The 0.0560 m nominal value, sensor spacing, magnet height, and sensor orientation must all be checked on the physical apparatus.

## 5. NAU7802 and load-cell wiring

Wire the NAU7802 as follows: **SDA → GPIO21**, **SCL → GPIO22**, **VCC → regulated 3.3 V**, and **GND → common GND**. The firmware requests `NAU7802_SPS_320`; record the library and firmware revision used for each dataset.

Connect the four wires of the S-type bridge load cell to the NAU7802 bridge terminals by function:

| Load-cell function | NAU7802 terminal |
| --- | --- |
| Signal positive (`S+`) | `A+` / `AIN+` |
| Signal negative (`S−`) | `A−` / `AIN−` |
| Excitation positive (`E+`) | `E+` |
| Excitation negative (`E−`) | `E−` |

Do not infer `S+`, `S−`, `E+`, and `E−` from insulation colour. Use the load-cell data sheet or identify the bridge pairs with a continuity/resistance test. If the measured force sign is reversed, power down and recheck the bridge mapping before applying a software sign change. Record the final mapping and the test method.

## 6. I2C pull-ups, capacitors, and module-specific details

The wiring drawing shows 4.7 kΩ pull-ups from SDA and SCL to 3.3 V. Some NAU7802 and ESP32 modules already include pull-ups and decoupling capacitors. Inspect the actual module documentation and board markings before adding components: duplicated parallel pull-ups can make the effective resistance too low, while a missing pull-up can prevent I2C communication. Treat capacitors and regulator details shown in the drawing as reference values; reproduce them only when they match the installed module's specification.

## 7. Pre-power checklist

Before the first power-up, document the following checks:

1. The exact ESP32 board type is identified, and GPIO34/GPIO35 are available as ADC inputs.
2. Hall `AO_A` and `AO_B` are within the ESP32 ADC input range; no 5 V signal reaches GPIO34/GPIO35.
3. NAU7802 logic voltage and I2C pull-ups are compatible with the ESP32.
4. Regulated output is 3.3 V, all grounds are common, and no rail-to-ground short is present.
5. The load-cell bridge mapping was established from documentation or resistance testing.
6. The magnet and Hall sensors are fixed, correctly oriented, and separated by the recorded effective trigger distance.
7. Simultaneous USB and external-power use cannot create a reverse-power path.

The `ino` source confirms the software pin assignments and acquisition protocol, but it cannot prove that a particular physical assembly is wired correctly. Board type, Hall model, regulator, load-cell lead assignment, mechanical dimensions, and calibration constants must be verified on the actual device.

## 8. Firmware and acquisition

Install the ESP32 Arduino core and the NAU7802 library. Upload the firmware from the repository's `firmware/` directory, open the serial monitor at 115200 baud, and confirm that force samples and Hall-gate events use the same ESP32 `micros()` time base. Start the browser interface, connect to the ESP32 device, tare the unloaded load cell, enter the cart mass, and retain both the exported CSV and the original serial/BLE log.

## 9. First boot and calibration

Upload the firmware, open the serial monitor at 115200 baud, and inspect the startup diagnostics for the expected pins and sensor status. With the load cell unloaded and stationary, run `tare` (or `t`) and save the resulting zero information. Run `scan` to inspect Hall raw levels and confirm the two pulses occur in the intended order. Use `cal` only within the documented diagnostic/calibration procedure; retain the calibration method, reference load, conversion constant, and firmware revision.

Record cart mass, measured effective gate spacing, track inclination, load-cell zero, and force conversion factor before collecting trials. Report the values and units with every dataset. Repeat the zero check after the trial series and flag any saturated or incomplete record. Preserve the original serial/BLE log as well as the exported CSV.

## 10. Trial procedure

For each collision head, run a controlled release that produces a clear A→B incident passage, collision, and B→A rebound passage. Keep the collision head and release condition in the trial notes. Export one CSV per valid trial. Exclude records with missing gate events, force saturation, or lost data and preserve the exclusion reason.

## 11. Analysis

Use the common timestamps to identify contact start/end, integrate the baseline-corrected force by the trapezoidal rule, calculate incident and rebound velocities from the gate spacing, and apply the gate-to-collision external-impulse correction described in Eqs. (1)–(4) of the manuscript. The raw CSV files in S2 are the source data; all summary tables and figures are derived products.

## 12. Reproducibility record and traceability

For each run, archive the firmware revision, browser/interface revision, board and sensor identifiers, wiring photograph or diagram, regulator measurement, calibration values, raw log, exported CSV, trial notes, and analysis script or notebook. Do not overwrite a raw export after collection. If a wiring detail differs from the reference drawing, describe the difference and explain why the firmware and analysis remain compatible.
