  # ESP32 Common-Time-Base Impulse–Momentum Apparatus

This repository accompanies the paper “An ESP32-Based Apparatus with a Common Time Base for Impulse–Momentum Measurement.” The apparatus uses a single ESP32 clock to timestamp force samples and Hall-gate events, placing collision force and cart motion on a common time base. It also enables estimation of the external impulse accumulated between velocity measurement and collision, allowing correction of the gate-based momentum change.

The repository provides the firmware, construction and calibration instructions, demonstration materials, and supplementary data for reproducing and examining the experiment.

## Supplementary materials

- [S1 — Reproduction guide](supplementary_material/S1_reproduction_guide/README.md): hardware assembly, wiring, firmware, calibration, acquisition, and analysis workflow.
- [S2 — Data and provenance](supplementary_material/S2_data_and_provenance/README.md): trial-level CSV exports grouped by collision head, with provenance notes and a SHA-256 manifest.
- [S3 — Figures and videos](supplementary_material/S3_figures_and_videos/README.md): apparatus diagrams, photographs, result figures, and demonstration video.

The paper cites S1 for the reproducible operating procedure, S2 for raw data and provenance, and S3 for visual documentation. Derived results should be regenerated from S2 using the method specified in S1.
