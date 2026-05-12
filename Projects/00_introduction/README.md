# Deep Learning Surrogate Model for Blast-Loaded Plates

A PyTorch-based surrogate model that predicts the peak midpoint deflection of a simply-supported square steel plate subjected to a close-in detonation, without running finite element or MDOF simulations. Developed as part of a deep learning for structural engineering course at Politecnico di Milano.

## Physical Problem

A simply-supported square steel plate (side `a`, thickness `h`) is loaded by a near-field blast at stand-off distance `R`, producing a nonuniform impulsive pressure. The plate is reduced to an equivalent single-degree-of-freedom (SDOF) system following the method of Grisaro (2025), which accounts for nonuniformity through load-dependent transformation factors derived from Navier's double Fourier sine series.

The key dimensionless parameter is `eta = R/a`: small `eta` produces a concentrated near-field load, large `eta` approaches a quasi-uniform far-field load.

### Governing Equation

Peak midpoint deflection (Grisaro 2025, Eq. 29):

```
w_max = I(eta) / sqrt(kLM(eta) * M * K(eta))

where:
  I(eta) = i0 * gamma(eta) * a^2        (total impulse)
  M      = rho * h * a^2                (plate mass)
  K      = K_hat(eta) * D / a^2         (equivalent stiffness)
  D      = E*h^3 / (12*(1 - nu^2))      (flexural rigidity)
```

Transformation factors `kLM`, `K_hat`, and `gamma` are functions of `eta`, precomputed from Navier's static solution on a dense lookup table.

## Model Inputs and Output

| Symbol | Description            | Range       | Unit  |
|--------|------------------------|-------------|-------|
| `h`    | Plate thickness        | 3 – 15      | mm    |
| `a`    | Plate side (square)    | 0.4 – 2.0   | m     |
| `R`    | Stand-off distance     | 0.04 – 1.5  | m     |
| `i0`   | Peak specific impulse  | 100 – 2000  | Pa·s  |

Output: `w_max` — peak midpoint deflection in mm.

## Dataset

2,500 samples generated analytically from the Grisaro (2025) SDOF model. Samples are filtered to the linear-elastic, bending-dominated regime (`w_max/h < 3`, `w_max/a < 0.05`). Multiplicative Gaussian noise (5%) is added to simulate FEM mesh variability. Split: 70% train / 15% val / 15% test.

## Network Architecture

Fully-connected multilayer perceptron (MLP) implemented in PyTorch:

```
Input (4) -> Linear(64) -> ReLU -> Dropout(0.1)
          -> Linear(64) -> ReLU -> Dropout(0.1)
          -> Linear(32) -> ReLU -> Dropout(0.1)
          -> Linear(1)
```

Total trainable parameters: 6,593.

The output is trained in `log10(w_max)` space to handle three orders of magnitude in the target. Both inputs and output are z-score standardised using training set statistics only.

## Training

- Optimiser: AdamW (lr=1e-3, weight_decay=1e-4)
- Scheduler: Cosine annealing (T_max=400, eta_min=1e-5)
- Loss: Mean squared error on standardised log-space predictions
- Epochs: 400 with best-val-loss checkpoint

## Results

| Split | RMSE (mm) | MAE (mm) | MAPE (%) | R²     |
|-------|-----------|----------|----------|--------|
| Train | ~1.17     | ~0.76    | ~5.0     | ~0.986 |
| Val   | ~1.43     | ~0.96    | ~6.3     | ~0.980 |
| Test  | ~1.34     | ~0.88    | ~6.3     | ~0.982 |

## Files

| File | Description |
|------|-------------|
| `02_dl/DL_surrogate_blast_plate.ipynb` | Main notebook: dataset generation, EDA, MLP training, diagnostics, and four experiments |

## How to Run

Open the notebook in Jupyter or Google Colab:

```bash
jupyter notebook 02_dl/DL_surrogate_blast_plate.ipynb
```

Required packages: `numpy`, `matplotlib`, `torch`.

## Reference

H.Y. Grisaro, *Simplified equivalent SDOF system for predicting complex dynamic response of 1D and 2D elements under nonuniform dynamic load*, Journal of Sound and Vibration **609** (2025) 119090.
