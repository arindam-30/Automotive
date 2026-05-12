# Longitudinal Vehicle Dynamics Model

MATLAB and Simulink implementation of a 3-DOF longitudinal vehicle dynamics model, incorporating vehicle body motion, front and rear wheel rotational dynamics, and drivetrain interactions.

## Model

Supports three simulation modes: straight-line acceleration, braking, and closed-loop cruise control (default: 50 → 110 → 70 km/h profile). Outputs include longitudinal velocity, acceleration, wheel peripheral velocities, and front/rear contact forces.

## Files

- `longitudinal_sim_file.m` — entry point: configures inputs, runs simulation, plots results
- `longitudinal_model.slx` — Simulink model
- `data_vehicle.m` — vehicle geometry, mass, and inertia
- `data_tire.m` — tire parameters including effective rolling radius
- `data_motor.m` — motor torque map
- `data_brake.m` — brake system parameters
- `data_aero.m` — aerodynamic drag parameters
- `motor_working_points.m` / `motor_working_points_a.m` — operating point computations
- `brake_distribution.m` — front/rear brake force distribution
- `control_struct.m` — cruise controller gains and structure

Run `longitudinal_sim_file.m` in MATLAB with Simulink (R2020b or later).
