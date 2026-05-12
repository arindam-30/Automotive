# Quarter Car Model — MATLAB

A 2-DOF quarter car suspension model built from first principles in MATLAB. Simulates a wheel passing over a haversine bump and visualises the response through an interactive animated figure.

## Model

State-space formulation of sprung mass (320 kg) and unsprung mass (45 kg) connected by a suspension spring-damper (Ks = 22,000 N/m, Cs = 1,500 N·s/m) on a tyre (Kt = 190,000 N/m). Integrated with 4th-order Runge-Kutta (dt = 1 ms, 3 s window) at 30 km/h over an 8 cm haversine bump.

The GUI includes body and wheel displacement plots, suspension travel, body acceleration, and playback controls (play/pause, scrubber, speed scaling).

## Files

- `quarter_car_model.m` — self-contained script with model, RK4 integration, and interactive GUI

Run `quarter_car_model` in MATLAB (R2020b or later). No additional toolboxes required.
