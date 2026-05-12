# Quarter Car Model — MATLAB

A two-degree-of-freedom (2-DOF) quarter car suspension model built from first principles in MATLAB. Simulates a vehicle wheel passing over a single haversine bump and visualises the transient response through an interactive animated figure.

## Physical Model

```
      [ ms ]   <- Sprung mass (car body)
       |  |
      Ks  Cs   <- Suspension spring and damper
       |  |
      [ mu ]   <- Unsprung mass (wheel + hub)
       |  |
      Kt  Ct   <- Tyre stiffness and structural damping
       |  |
      ======   <- Road profile z_r(t)
```

### Equations of Motion

Derived from Newton's Second Law applied to each free body:

```
ms * z_s'' = -Ks*(zs - zu) - Cs*(zs' - zu')
mu * z_u'' =  Ks*(zs - zu) + Cs*(zs' - zu') - Kt*(zu - zr) - Ct*(zu' - zr')
```

Reformulated as a state-space system (`x = [zs, zs', zu, zu']`, `u = [zr, zr']`):

```
x' = A*x + B*u
```

### Parameters

| Symbol | Value | Description |
|--------|-------|-------------|
| `ms` | 320 kg | Sprung mass (1/4 of 1280 kg body) |
| `mu` | 45 kg | Unsprung mass (wheel + hub + brake) |
| `Ks` | 22,000 N/m | Suspension spring stiffness |
| `Cs` | 1,500 N·s/m | Suspension damper coefficient |
| `Kt` | 190,000 N/m | Tyre radial stiffness |
| `Ct` | 50 N·s/m | Tyre structural damping |
| `v` | 30 km/h | Vehicle forward speed |
| `H_bump` | 8 cm | Bump height (haversine profile) |
| `L_bump` | 10 cm | Bump half-width |

## Numerical Method

Integration uses a 4th-order Runge-Kutta (RK4) scheme with a fixed time step of 1 ms over a 3-second simulation window. All data is pre-computed before the GUI launches.

## Interactive GUI

Running the script opens a MATLAB figure with four panels:

| Panel | Content |
|-------|---------|
| Left | Animated car body and wheel passing over the bump, with live spring and damper visualisation |
| Top-right | Body displacement, wheel displacement, and road profile vs time |
| Mid-right | Suspension travel vs time |
| Bottom-right | Body vertical acceleration (g) vs time |

### Controls

| Control | Action |
|---------|--------|
| Time scrubber (slider) | Jump to any moment in the simulation |
| Play / Pause | Start or freeze the animation |
| Reset | Return to t = 0 |
| Step buttons | Step +/- 50 frames (~50 ms) |
| Speed slider | Scale playback from 0.25x to 4x |
| Loop toggle | Auto-rewind at end of playback |

## Sample Results

| Metric | Value |
|--------|-------|
| Peak body displacement | ~0.007 m |
| Peak wheel displacement | ~0.085 m |
| Peak suspension travel | ~0.079 m |
| Peak body acceleration | ~0.26 g |

## Files

| File | Description |
|------|-------------|
| `quarter_car_model.m` | Self-contained MATLAB script — physical model, RK4 integration, and interactive GUI |

## How to Run

1. Open MATLAB (R2020b or later).
2. Navigate to this folder.
3. Run in the Command Window:

```matlab
quarter_car_model
```

No additional toolboxes are required.
