# Data Analysis for Future Transportation Systems

MATLAB-based signal processing pipeline applied to vehicle telemetry from a blind-challenge driving session. Processes synchronised IMU and GPS data to extract key vehicle dynamics metrics.

## Analysis Performed

- Speed profiles from IMU integration and GPS (Haversine formula), with cross-correlation alignment
- Braking onset detection using dual-threshold conditions on acceleration and speed gradient
- Steering correction identification via bandpass-filtered yaw rate and moving-energy thresholding
- Peak lateral acceleration extraction with Butterworth filtering and RMS smoothing
- Roll and pitch validation by comparing gyroscope integration against measured attitude angles
- Spatial lag quantification between test runs using speed-profile cross-correlation

## Files

- `DAFTS_ProjectCode.m` — main analysis script
- `DA4FTS_Project#03_BlindChallenge#01_Test*.mat` — telemetry datasets (Tests 1–4)
- `DAFTS_Project_Final.pptx` — results presentation

Run `DAFTS_ProjectCode.m` in MATLAB. A file dialog prompts for the test to analyse. Requires the Signal Processing Toolbox.
