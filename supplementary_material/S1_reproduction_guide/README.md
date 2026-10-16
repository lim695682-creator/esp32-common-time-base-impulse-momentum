# S1. Reproduction guide

This guide is the operational companion to *An ESP32-Based Apparatus with a Common Time Base for Impulse–Momentum Measurement*.

## 1. Hardware

Assemble the inclined track, cart, interchangeable collision head, S-type load cell, NAU7802 bridge ADC, ESP32 controller, two Hall-effect gates, magnet, and 3.3 V regulator. Set the Hall-gate centre-to-centre spacing to 0.0560 m and record the actual spacing for the build. Verify supply voltage, sensor polarity, and load-cell wiring before powering the apparatus.

## 2. Firmware and acquisition

Install the ESP32 Arduino core and the NAU7802 library. Upload the firmware from the repository's `firmware/` directory, open the serial monitor at 115200 baud, and confirm that force samples and Hall-gate events use the same ESP32 `micros()` time base. Start the browser interface, connect to the ESP32 device, tare the unloaded load cell, enter the cart mass, and retain both the exported CSV and the original serial/BLE log.

## 3. Calibration

Record cart mass, gate spacing, track inclination, load-cell zero, and force conversion factor before collecting trials. Report the values and units with every dataset. Repeat the zero check after the trial series and flag any saturated or incomplete record.

## 4. Trial procedure

For each collision head, run a controlled release that produces a clear A→B incident passage, collision, and B→A rebound passage. Keep the collision head and release condition in the trial notes. Export one CSV per valid trial. Exclude records with missing gate events, force saturation, or lost data and preserve the exclusion reason.

## 5. Analysis

Use the common timestamps to identify contact start/end, integrate the baseline-corrected force by the trapezoidal rule, calculate incident and rebound velocities from the gate spacing, and apply the gate-to-collision external-impulse correction described in Eqs. (1)–(4) of the manuscript. The raw CSV files in S2 are the source data; all summary tables and figures are derived products.

## 6. Reproducibility record

For each run, archive the firmware revision, browser/interface revision, calibration values, raw log, exported CSV, trial notes, and analysis script or notebook. Do not overwrite a raw export after collection.
