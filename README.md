# ESP32 Impulse-Momentum Lab

An ESP32-based apparatus and browser interface for studying impulse and momentum during cart collisions. The ESP32 timestamps analog Hall-gate pulses and load-cell samples with the same `micros()` clock. The browser displays the force trace and calculated results, and exports a trial CSV.

This repository contains source code and setup notes. It does not currently include a verified wiring diagram, independent example dataset, or complete calibration record. The manuscript in `manuscript/` is background material, not an installation prerequisite; confirm redistribution rights before publishing it.

## Contents

| Path | Contents |
| --- | --- |
| `firmware/` | ESP32 Arduino sketch and protocol notes. |
| `web/frontend/` | React/Vite interface using Web Bluetooth; no server is needed for acquisition. |
| `hardware/` | Component list, firmware pin assignments, and apparatus notes. |
| `docs/` | Setup, calibration, data format, project map, and release checks. |
| `data/` | Data provenance guidance; no trial files are included. |
| `analysis/` | Scope of the current calculations; no standalone scripts are included. |
| `manuscript/` | Author-supplied manuscript files. Redistribution status must be confirmed before public release. |
| `docs/assets/videos/` | Demonstration videos supplied for the apparatus and browser interface. |

## Requirements

- ESP32 board, NAU7802 load-cell ADC, load cell, two analog-output Hall sensors, magnet, cart, and track. Exact models still need confirmation; see [`hardware/bill_of_materials.md`](hardware/bill_of_materials.md).
- Arduino IDE or Arduino CLI with the ESP32 Arduino core and SparkFun Qwiic Scale NAU7802 library.
- Node.js and npm compatible with `web/frontend/package-lock.json`.
- A Chromium-based browser with Web Bluetooth support. Live pairing requires a secure context; local `localhost` works for development.

The firmware uses Hall A on GPIO 34, Hall B on GPIO 35, I2C SDA on GPIO 21, and SCL on GPIO 22. Hall modules' digital outputs are unused. These are software assignments, not a verified physical wiring diagram. Check voltage levels, power, and load-cell connections against the actual components before powering the apparatus. See [`hardware/wiring/wiring_notes.md`](hardware/wiring/wiring_notes.md).

## Quick start

1. Assemble and check the hardware. Review firmware pin assignments and calibration constants before recording measurements.
2. Install the ESP32 Arduino core and SparkFun NAU7802 library. Open `firmware/esp32_impulse.ino`, select the actual board and port, then compile and upload. Inspect boot diagnostics in Serial Monitor at `115200` baud.
3. From `web/frontend/`, run `npm ci` followed by `npm run dev`, then open the URL printed by Vite.
4. Connect to the `ESP32-MomentumLab` device. With the load cell unloaded, tare it and enter the cart mass.
5. Run a controlled collision, inspect warnings and results, then export the CSV. Keep the raw Serial/BLE log and trial notes with the export.

Detailed steps are in [`docs/setup.md`](docs/setup.md), with interface details in [`web/README.md`](web/README.md). The page also supports pasted-log replay without a BLE device.

The [`project map`](docs/project_map.md) links each component to its detailed documentation.

## Measurement and data

The sketch polls Hall analog inputs, estimates pulse-center times, and samples the NAU7802 at its configured 320 SPS setting. Force and event records share the ESP32 clock; the browser is not the measurement clock. The current Hall spacing (`0.0560 m`) and force conversion factor (`0.00000705228 N/count`) are firmware settings, not independently verified calibration for another build. See [`docs/calibration.md`](docs/calibration.md).

The device emits newline-delimited `STATUS`, `EVENT`, `PEAK`, `FORCE`, `RESULT`, `INFO`, and `WARN` records over Serial and BLE. The browser CSV combines metadata, results, timestamps, correction quantities where available, and force samples. It is derived data; retain the original log. Field definitions are in [`docs/data_format.md`](docs/data_format.md).

## Demonstrations and manuscript

- [Collision apparatus demonstration](docs/assets/videos/collision_apparatus_demo.mp4)
- [Web interface demonstration](docs/assets/videos/web_interface_demo.mp4)
- [Restructured author manuscript](manuscript/An_ESP32_Based_Apparatus_restructured.docx)

The videos and manuscript are supplied files whose public redistribution should be confirmed by the rights holder before publishing the repository. The videos are demonstrations, not calibration records or evidence of a repeatable public build.

## Current limits

- Physical wiring, part numbers, actual board model, mechanical dimensions, and calibration records are not verified for a public build guide.
- No trial-level data are included, so paper-level results cannot be reproduced from this repository alone.
- BLE notifications can be lost; retain the Serial log when complete raw acquisition is important and inspect warnings before using a trial.
- The browser needs Web Bluetooth for live acquisition. There is no experiment-specific backend or cloud storage.

## Questions and contributions

For setup or data-format problems, open a GitHub issue with the board and library versions, relevant device status or warning lines, browser version, and reproduction steps. Remove device identifiers or private trial information before posting logs. Changes to calibration or analysis should state the source of the values and how they were checked. Public contribution instructions and issue tracking can be finalized once the repository URL and maintainers are confirmed.

## Citation and licensing

The associated manuscript is titled *An ESP32-Based Apparatus with a Common Time Base for Impulse-Momentum Measurement*. Publication details and DOI are not yet confirmed. Citation metadata will be added after author and repository details are supplied.

No license grant is asserted for this package yet. Before public distribution, rights holders must select a source-code license, decide separately how documentation and manuscript files may be distributed, and supply attribution details. Until then, do not assume permission to reuse or redistribute these files. See [`docs/release_checklist.md`](docs/release_checklist.md).
