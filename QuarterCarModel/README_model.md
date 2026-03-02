# 🚗 Quarter Car Model — MATLAB

A two-degree-of-freedom (2-DOF) **quarter car model** built entirely from first principles in MATLAB.  
Simulates a vehicle wheel passing over a single haversine bump and visualises the response with a fully interactive animated figure.

---

## 📐 Physical Model

```
      [ ms ]   ← Sprung mass (car body, 320 kg)
       |  |
      Ks  Cs   ← Suspension spring (22 000 N/m) & damper (1 500 N·s/m)
       |  |
      [ mu ]   ← Unsprung mass (wheel + hub, 45 kg)
       |  |
      Kt  Ct   ← Tyre stiffness (190 000 N/m) & damping (50 N·s/m)
       |  |
      ======   ← Road profile  z_r(t)
```

### Equations of Motion
Derived from Newton's Second Law applied to each free body:

```
ms · z̈s = −Ks(zs − zu) − Cs(żs − żu)
mu · z̈u =  Ks(zs − zu) + Cs(żs − żu) − Kt(zu − zr) − Ct(żu − żṙ)
```

Reformulated as a **state-space system**:

```
ẋ = A·x + B·u

State : x = [zs, żs, zu, żu]ᵀ
Input : u = [zr, żṙ]ᵀ
```

---

## ⚙️ Parameters

| Symbol | Value | Description |
|--------|-------|-------------|
| `ms` | 320 kg | Sprung mass (¼ of 1 280 kg car body) |
| `mu` | 45 kg | Unsprung mass (wheel + hub + brake) |
| `Ks` | 22 000 N/m | Suspension spring stiffness |
| `Cs` | 1 500 N·s/m | Suspension damper coefficient |
| `Kt` | 190 000 N/m | Tyre radial stiffness |
| `Ct` | 50 N·s/m | Tyre structural damping |
| `v` | 30 km/h | Vehicle forward speed |
| `H_bump` | 8 cm | Bump height (haversine profile) |
| `L_bump` | 10 cm | Bump half-width |

---

## 🔢 Numerical Method

Integration is performed using a **4th-order Runge–Kutta (RK4)** scheme with a fixed time step of `dt = 1 ms` over a 3-second simulation window.  
All data is pre-computed before the GUI launches, so scrubbing and playback are instant.

---

## 🖥️ Interactive GUI

Running the script opens a MATLAB figure with:

| Panel | Content |
|-------|---------|
| **Left** | Animated car body + wheel bouncing over the bump, with live spring and damper visualisation |
| **Top-right** | Body displacement, wheel displacement, and road profile vs time |
| **Mid-right** | Suspension travel vs time |
| **Bottom-right** | Body vertical acceleration (in g) vs time |

### Controls

| Control | Action |
|---------|--------|
| **Time scrubber (slider)** | Drag to jump to any moment in the simulation |
| **▶ Play / ⏸ Pause** | Start or freeze the animation |
| **↺ Reset** | Return to `t = 0` |
| **`\|<` / `>\|`** | Step ±50 frames (~50 ms) |
| **Speed slider** | Scale playback from **0.25×** to **4×** |
| **Loop toggle** | Auto-rewind when playback reaches the end |

---

## ▶️ How to Run

1. Open MATLAB (R2020b or later).
2. Navigate to this folder.
3. Type in the Command Window:
   ```matlab
   quarter_car_model
   ```
4. The interactive figure will open immediately.

No additional toolboxes are required.

---

## 📊 Sample Results

| Metric | Value |
|--------|-------|
| Peak body displacement | ~0.007 m |
| Peak wheel displacement | ~0.085 m |
| Peak suspension travel | ~0.079 m |
| Peak body acceleration | ~0.26 g |

---

## 📄 Licence
MIT — free to use and adapt with attribution.
