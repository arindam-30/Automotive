# Longitudinal Vehicle Dynamics Model

A MATLAB and Simulink implementation of a 3-DOF longitudinal vehicle dynamics model. The model captures vehicle body motion along the longitudinal axis, individual wheel rotational dynamics, and drivetrain interactions. It supports straight-line acceleration, braking, and closed-loop cruise control simulations.

## Degrees of Freedom

| DOF | State variable | Description |
|-----|---------------|-------------|
| 1 | `omega_F` | Front wheel angular velocity [rad/s] |
| 2 | `omega_R` | Rear wheel angular velocity [rad/s] |
| 3 | `vx` | Longitudinal vehicle velocity [m/s] |

Two additional states track auxiliary controller variables. Initial conditions are set in `longitudinal_sim_file.m` based on a defined initial speed `v0`.

## Model Architecture

The Simulink model (`longitudinal_model.slx`) integrates:

- **Vehicle body dynamics**: longitudinal acceleration and velocity from net traction/braking forces and aerodynamic drag
- **Wheel rotational dynamics**: individual front and rear wheel angular velocities driven by motor torque and braking pressure, coupled to the vehicle through tire longitudinal slip forces
- **Tire model**: longitudinal slip ratio (`kappa`) and force (`Fx`) per axle, with vertical load transfer (`Fz`) under acceleration and braking
- **Motor model**: torque maps and operating-point lookup defined in `data_motor.m` and `motor_working_points.m`
- **Brake distribution**: front/rear brake force split from `brake_distribution.m`
- **Cruise controller**: PI-based speed controller defined in `control_struct.m`, tracking a piecewise-constant reference velocity profile

## Simulation Modes

Three input configurations are available in `longitudinal_sim_file.m` (one active, others commented out):

### Cruise Control
Tracks a piecewise speed profile: 50 km/h → 110 km/h → 70 km/h over 55 s. The controller adjusts motor torque to follow the reference.

### Straight-Line Acceleration
Full motor torque applied from rest. Torque ramps from the resistance-equilibrium value at `v0` to maximum motor torque over 1 s, then holds. No braking pressure applied.

### Braking
Zero motor torque with a ramp braking pressure input (`p_ref = 30 bar`).

## Simulation Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `simu.v0` | 50 km/h (default) | Initial vehicle speed |
| `simu.t_end` | 100 s | Simulation duration |
| `simu.dt` | 1×10⁻⁴ s | Integration time step |
| `simu.dt_save` | 1×10⁻² s | Output save interval |
| `simu.mu` | 1.0 | Road friction coefficient |
| `simu.alpha` | 0° | Road grade angle |

## Outputs

| Signal | Description |
|--------|-------------|
| `vx` | Longitudinal vehicle velocity [m/s] |
| `ax` | Longitudinal acceleration [m/s²] |
| `omega` | Front and rear wheel angular velocities [rad/s] |
| `Fx` | Front and rear longitudinal contact forces [N] |
| `Fz` | Front and rear vertical contact forces [N] |

Post-processing computes front and rear wheel peripheral velocities (`vF = omega_F * Re`, `vR = omega_R * Re`) for slip analysis.

## Files

| File | Description |
|------|-------------|
| `longitudinal_sim_file.m` | Entry point — configures inputs, runs simulation, plots results |
| `longitudinal_model.slx` | Simulink model |
| `data_vehicle.m` | Vehicle geometry, mass, and inertia parameters |
| `data_tire.m` | Tire model parameters including effective rolling radius `Re` |
| `data_motor.m` | Motor torque map and parameters |
| `data_brake.m` | Brake system parameters |
| `data_aero.m` | Aerodynamic drag parameters |
| `motor_working_points.m` | Graphical solution for constant-speed motor operating points |
| `motor_working_points_a.m` | Alternative motor working point computation |
| `brake_distribution.m` | Front/rear brake force distribution computation |
| `control_struct.m` | Cruise controller structure and gains |

## How to Run

1. Open MATLAB (R2020b or later with Simulink).
2. Navigate to this folder.
3. Run `longitudinal_sim_file.m` from the Command Window.

The script loads all data files, computes motor operating points and brake distribution, configures the chosen input mode, runs `longitudinal_model.slx`, and generates four output plots: velocity profiles, acceleration, longitudinal contact forces, and vertical contact forces.
