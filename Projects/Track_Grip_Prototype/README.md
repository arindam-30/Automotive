# Track Grip Optimisation Prototype

A machine learning system for predicting track grip coefficients in motorsports from telemetry and environmental data. The system uses an XGBoost + LSTM ensemble trained on synthetic racing session data to deliver sub-100 ms grip predictions via a FastAPI inference server.

## Technical Overview

### Data Pipeline

Synthetic telemetry data is generated per racing session, covering:
- Environmental conditions: track temperature, air temperature, humidity, wind
- Tire state (4 corners): temperature, pressure, wear estimate, compound
- Vehicle dynamics: speed, lateral/longitudinal G-forces, slip angles, ride height
- Driver inputs: steering angle, throttle position, brake pressure
- Track sector encoding

Feature engineering extracts 40+ derived features including tire temperature gradients and balance, pressure distribution, and driver input patterns. Sequences of 30 timesteps are used as input windows for the LSTM model.

Target variable: `grip_coefficient` in the range 0.6 – 1.6.

### Models

**XGBoost baseline**: gradient-boosted trees on the full engineered feature set. Operates on individual timesteps.

**LSTM temporal model**: PyTorch recurrent network trained on 30-step input sequences to capture temporal grip evolution.

**Ensemble**: weighted combination of XGBoost and LSTM predictions. The ensemble outperforms either model individually.

### Inference Server

A FastAPI server exposes the trained ensemble via REST API with sub-100 ms prediction latency.

## Performance

| Metric | Target | Achieved |
|--------|--------|----------|
| RMSE | < 0.10 | ~0.08 |
| MAE | < 0.07 | ~0.06 |
| R² | > 0.80 | ~0.87 |
| Direction Accuracy | > 80% | ~85% |
| Inference Latency | < 100 ms | ~50 ms |

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/api/v1/grip/predict` | POST | Single-sample grip prediction |
| `/api/v1/grip/forecast` | GET | 30-minute grip forecast |
| `/api/v1/grip/sector-map` | GET | Per-sector grip map |
| `/api/v1/health` | GET | Health check |
| `/docs` | GET | Swagger UI |

## Files

| File | Description |
|------|-------------|
| `run_pipeline.py` | Full pipeline runner: data generation, training, and evaluation in one command |
| `run_from_data.py` | Training and evaluation only, skipping data generation (for existing datasets) |
| `requirements.txt` | Python dependencies |
| `USAGE.md` | Detailed usage guide for each pipeline stage |

## Project Structure

```
Track_Grip_Prototype/
├── src/
│   ├── data/
│   │   ├── generate_data.py        # Synthetic telemetry generator
│   │   └── feature_engineering.py  # Feature extraction and scaling
│   ├── models/
│   │   ├── xgboost_model.py        # XGBoost baseline
│   │   ├── lstm_model.py           # PyTorch LSTM model
│   │   ├── ensemble.py             # Ensemble combination
│   │   └── train_models.py         # Training orchestrator
│   ├── serving/
│   │   └── inference_server.py     # FastAPI inference server
│   ├── visualization/
│   │   └── dashboard.py            # Plotly interactive dashboard
│   └── evaluation/
│       └── evaluate.py             # Evaluation metrics and plots
├── notebooks/
│   └── 01_quick_start.ipynb        # Interactive tutorial
├── requirements.txt
├── run_pipeline.py
└── run_from_data.py
```

## Dependencies

```
numpy, pandas, scikit-learn, xgboost, torch, scipy,
matplotlib, seaborn, plotly, fastapi, uvicorn, pydantic,
pyyaml, joblib, tqdm, jupyter
```

Install with:

```bash
pip install -r requirements.txt
```

## How to Run

### Full pipeline (data generation + training + evaluation)

```bash
python run_pipeline.py           # 100 sessions, 100 LSTM epochs
python run_pipeline.py --quick   # 50 sessions, 30 epochs (faster)
```

### Training only (existing data)

```bash
python run_from_data.py --data data/raw/all_sessions.csv --epochs 100
```

### Inference server

```bash
python src/serving/inference_server.py --models output/models/ --port 8000
```

Open `http://localhost:8000/docs` for the Swagger UI.

### Dashboard

```bash
python src/visualization/dashboard.py
```

Output saved to `output/dashboard_demo.html`.

All output artefacts (data, models, evaluation reports, plots) are written to the `output/` directory by default.
