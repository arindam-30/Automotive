# Track Grip Optimisation Prototype

A machine learning system for predicting track grip coefficients in motorsports from vehicle telemetry and environmental data.

## Method

Synthetic telemetry data (tire temperatures/pressures/wear, vehicle dynamics, driver inputs, environmental conditions) is used to train an XGBoost + LSTM ensemble. Feature engineering extracts 40+ derived features from raw inputs. The trained ensemble is served via a FastAPI inference server with sub-100 ms prediction latency.

**Target:** `grip_coefficient` (0.6 – 1.6). Achieved RMSE ~0.08, R² ~0.87.

## Files

- `run_pipeline.py` — full pipeline: data generation, training, and evaluation
- `run_from_data.py` — training and evaluation only (skips data generation)
- `requirements.txt` — Python dependencies
- `USAGE.md` — detailed usage guide

## How to Run

```bash
pip install -r requirements.txt
python run_pipeline.py --quick   # fast test run
python run_pipeline.py           # full run (100 sessions, 100 epochs)
```
