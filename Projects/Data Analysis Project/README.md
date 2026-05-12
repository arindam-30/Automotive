# Data Analysis for Future Transportation Systems

A MATLAB-based signal processing and data analysis pipeline applied to vehicle telemetry data collected during a blind challenge. The project processes IMU and GPS measurements to extract key vehicle dynamics metrics including speed profiles, braking onset, lateral acceleration, steering corrections, and roll/pitch behaviour.

## Dataset

Four `.mat` files from a blind-challenge driving session, each containing synchronised time-series measurements from:
- 3-axis accelerometer (`Accelerometer_X/Y/Z`)
- 3-axis gyroscope (`Gyroscope_X/Y/Z`)
- GPS (`Latitude`, `Longitude`)
- Inertial attitude (`Roll`, `Pitch`)

Orientation corrections are applied to Tests 1 and 3 to align sensor axes with the vehicle frame.

## Analysis Pipeline

### 1. Longitudinal Acceleration Filtering
Raw accelerometer X data is zero-mean corrected and filtered using a 4th-order Butterworth low-pass filter (cutoff: 1.5 Hz, zero-phase via `filtfilt`).

### 2. Speed Profiles
Two independent speed estimates are computed and compared:
- **IMU-based**: numerically integrated from filtered longitudinal acceleration using `cumtrapz`
- **GPS-based**: derived from successive coordinate differences using the Haversine formula, with spike removal via moving-median outlier rejection and low-pass filtering (Butterworth, 2 Hz)

Cross-correlation between the two speed signals is used to assess temporal alignment.

### 3. Braking Detection
Braking onset is identified using a dual-threshold condition requiring both filtered longitudinal acceleration below -0.3 m/s² and speed gradient below -0.2 km/h/s, sustained over a 0.3 s window. Detection is restricted to post-peak-speed segments.

### 4. Steering Correction Analysis
Yaw-rate (`Gyroscope_Z`) is bandpass filtered (1–4 Hz, 4th-order Butterworth) to isolate steering correction frequency content. Corrections are detected using a moving-energy envelope threshold (2 × RMS). Frequency content is characterised via FFT and Welch power spectral density.

### 5. Lateral Acceleration
Peak lateral acceleration is extracted from `Accelerometer_Y` using two methods: moving-RMS smoothing (300 ms window) and direct low-pass filtering (2 Hz Butterworth). Cross-correlation between the two estimates is computed for consistency checking.

### 6. Roll and Pitch Validation
Gyroscope integration (`cumtrapz` over `Gyroscope_X/Y`) is compared against directly measured roll and pitch angles. Both signals are low-pass filtered at 5 Hz before comparison.

### 7. Multi-Test Comparison and Spatial Lag
GPS-derived speed profiles for all four tests are plotted on a common time axis. Spatial lag between a reference test and a chosen test is quantified near peak speed using cross-correlation in the spatial domain.

### 8. GPS Trajectory Visualisation
All key events (steering corrections, max lateral acceleration, peak speed, braking onset) are overlaid on the GPS trajectory plot.

## Files

| File | Description |
|------|-------------|
| `DAFTS_ProjectCode.m` | Main MATLAB script — runs the full analysis pipeline |
| `DA4FTS_Project#03_BlindChallenge#01_Test1.mat` | Telemetry data, Test 1 |
| `DA4FTS_Project#03_BlindChallenge#01_Test2.mat` | Telemetry data, Test 2 |
| `DA4FTS_Project#03_BlindChallenge#01_Test3.mat` | Telemetry data, Test 3 |
| `DA4FTS_Project#03_BlindChallenge#01_Test4.mat` | Telemetry data, Test 4 |
| `DAFTS_Project_Final.pptx` | Presentation summarising key results |

## How to Run

1. Open MATLAB and navigate to this folder.
2. Run `DAFTS_ProjectCode.m`.
3. A file dialog will prompt you to select one of the four `.mat` files for individual analysis.
4. All plots are generated automatically. The spatial lag section at the end of the script operates on all four tests simultaneously and does not require user input.

Required MATLAB toolboxes: Signal Processing Toolbox.
