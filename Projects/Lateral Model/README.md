# Lateral Vehicle Dynamics Model

A MATLAB and Simulink implementation of a full nonlinear lateral vehicle dynamics model. The model captures coupled lateral and yaw motion, four-wheel tire forces, wheel rotational dynamics, drivetrain torque distribution, and aerodynamic loads. It is designed for simulating standard handling manoeuvres and evaluating vehicle stability and handling performance.

## Vehicle Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `veh.m` | 1360 kg | Total vehicle mass |
| `veh.l` | 2740 mm | Wheelbase |
| `veh.lF / lR` | 1520 / 1220 mm | Front/rear axle distance from CoM |
| `veh.hG` | 486 mm | Centre of mass height |
| `veh.Jz` | 1690 kg·m² | Yaw moment of inertia |
| `veh.tau_delta` | 14.5 | Steering ratio |
| `veh.krol` | 0.5 | Front roll stiffness fraction |

Suspension dynamics are modelled as second-order roll and pitch sub-systems with natural frequencies and damping ratios defined in `data_vehicle.m`.

## Model Architecture

The Simulink model (`lateral_model.slx`) integrates:

- **Vehicle body dynamics**: lateral velocity, yaw rate, sideslip angle, global position
- **Wheel rotational dynamics**: individual wheel angular velocities (`omega`) and accelerations (`omegad`) per corner
- **Tire model**: slip-angle-based lateral force generation (`Fy`), longitudinal slip and force (`Fx`, `kappa`)
- **Vertical load transfer**: static and dynamic axle loads (`Fz`) accounting for longitudinal and lateral acceleration
- **Aerodynamic loads**: defined in `data_aero.m`
- **Motor model**: torque maps and working-point lookup (`data_motor.m`, `motor_working_points.m`)
- **Brake distribution**: front/rear brake bias computation (`brake_distribution.m`)
- **Cruise/torque controller**: closed-loop longitudinal speed control (`control_struct.m`)

## Simulation Setup

The entry point `main_lateral.m` configures and runs the simulation:

- **Default manoeuvre**: steering pad with a 1°/s linear ramp input at 130 km/h constant speed
- **Simulation duration**: 45 s, time step 1×10⁻⁴ s, save interval 1×10⁻² s
- **Initial conditions**: vehicle at 130 km/h, all wheel speeds matched to vehicle speed

Alternative input profiles (step steer, sine dwell) are included as commented-out code blocks in `main_lateral.m`.

## Outputs

The simulation returns and plots the following signals:

| Signal | Description |
|--------|-------------|
| `ay` | Lateral acceleration [m/s²] |
| `beta` | Sideslip angle [rad] |
| `psid` | Yaw rate [rad/s] |
| `psidd` | Yaw angular acceleration [rad/s²] |
| `psi` | Yaw angle [rad] |
| `v` | Vehicle speed magnitude [m/s] |
| `xG, yG` | Global trajectory [m] |
| `alpha` | Tire slip angles [rad] |
| `Fy` | Lateral contact forces [N] |
| `Fx` | Longitudinal contact forces [N] |
| `Fz` | Vertical contact forces [N] |
| `omega` | Wheel angular velocities [rad/s] |
| `T` | Applied torques [N·m] |
| `delta` | Steering angle [rad] |

## Files

| File | Description |
|------|-------------|
| `main_lateral.m` | Entry point — configures inputs, runs simulation, plots results |
| `lateral_model.slx` | Simulink model |
| `data_vehicle.m` | Vehicle geometry, inertia, and suspension parameters |
| `data_tire.m` | Tire model parameters |
| `data_motor.m` | Motor torque map and parameters |
| `data_brake.m` | Brake system parameters |
| `data_aero.m` | Aerodynamic drag and downforce parameters |
| `motor_working_points.m` | Graphical solution for constant-speed motor operating points |
| `brake_distribution.m` | Front/rear brake force distribution computation |
| `control_struct.m` | Cruise controller structure definition |

## How to Run

1. Open MATLAB (R2020b or later with Simulink).
2. Navigate to this folder.
3. Run `main_lateral.m` from the Command Window.

The script loads all data files, computes operating points, configures the simulation, runs `lateral_model.slx`, and generates trajectory and dynamics plots automatically.
