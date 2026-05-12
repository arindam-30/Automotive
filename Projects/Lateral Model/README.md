# Lateral Vehicle Dynamics Model

MATLAB and Simulink implementation of a nonlinear lateral vehicle dynamics model, capturing coupled lateral and yaw motion with four-wheel tire forces, wheel rotational dynamics, aerodynamic loads, and torque distribution.

## Model

Simulates standard handling manoeuvres including step steer, sine dwell, and steering pad (default: 1°/s ramp at 130 km/h). Outputs include lateral acceleration, sideslip angle, yaw rate, global trajectory, slip angles, and contact forces per wheel.

## Files

- `main_lateral.m` — entry point: loads data, configures inputs, runs simulation, plots results
- `lateral_model.slx` — Simulink model
- `data_vehicle.m` — vehicle geometry, inertia, and suspension parameters
- `data_tire.m` — tire model parameters
- `data_motor.m` — motor torque map
- `data_brake.m` — brake system parameters
- `data_aero.m` — aerodynamic parameters
- `motor_working_points.m` — constant-speed operating point solution
- `brake_distribution.m` — front/rear brake force distribution
- `control_struct.m` — cruise controller structure

Run `main_lateral.m` in MATLAB with Simulink (R2020b or later).
