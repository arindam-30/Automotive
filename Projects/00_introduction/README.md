# Deep Learning Surrogate Model — Blast-Loaded Plate

A PyTorch MLP surrogate that predicts peak midpoint deflection of a simply-supported square steel plate under a close-in detonation, replacing FEM/MDOF simulations.

## Method

Dataset of 2,500 samples generated analytically from the Grisaro (2025) equivalent-SDOF model, using Navier's double Fourier series to compute nonuniformity-dependent transformation factors. Network architecture: 4 → 64 → 64 → 32 → 1, trained on log-transformed, z-score normalised data. Optimiser: AdamW with cosine annealing over 400 epochs.

## Inputs / Output

| Input | Range |
|-------|-------|
| Plate thickness `h` | 3 – 15 mm |
| Plate side `a` | 0.4 – 2.0 m |
| Stand-off distance `R` | 0.04 – 1.5 m |
| Peak specific impulse `i0` | 100 – 2000 Pa·s |

Output: peak midpoint deflection `w_max` — test R² ≈ 0.982, MAPE ≈ 6.3%.

## Files

- `02_dl/DL_surrogate_blast_plate.ipynb` — dataset generation, model training, diagnostics, and experiments
