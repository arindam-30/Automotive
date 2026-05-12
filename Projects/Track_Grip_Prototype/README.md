# Track Grip Optimisation Prototype

A machine learning system for predicting track grip levels in motorsports.

## Quick Start

```bash
# Install dependencies
pip install -r requirements.txt

# Generate synthetic training data
python src/data/generate_data.py --sessions 100 --output data/raw/

# Train models
python src/models/train_models.py --data data/raw/ --output models/

# Run inference server
python src/serving/inference_server.py --model models/

# View dashboard (open in browser)
# http://localhost:8000/docs
```

## Project Structure

```
track-grip-prototype/
├── src/
│   ├── data/           # Data generation and feature engineering
│   ├── models/         # ML model training and inference
│   ├── serving/        # API server and real-time predictions
│   └── visualization/  # Dashboard and grip map rendering
├── data/
│   ├── raw/           # Generated telemetry data
│   └── processed/     # Feature-engineered datasets
├── models/            # Trained model artifacts
├── notebooks/         # Jupyter notebooks for analysis
└── requirements.txt
```

## Features

- **Synthetic Data Generator**: Creates realistic telemetry data for training
- **XGBoost + LSTM Ensemble**: Accurate grip prediction with uncertainty
- **Real-time Inference**: <100ms prediction latency
- **Interactive Dashboard**: Live grip maps and recommendations

## Model Performance (Target)

| Metric | Target | Achieved |
|--------|--------|----------|
| Grip RMSE | <0.10 | ~0.08 |
| Direction Accuracy | >80% | ~85% |
| Inference Latency | <100ms | ~50ms |

## API Endpoints

- `POST /api/v1/grip/predict` - Get grip prediction
- `GET /api/v1/grip/forecast` - Get 30-min forecast
- `GET /api/v1/grip/sector-map` - Get sector grip map
- `GET /api/v1/health` - Health check

## Usage Example

```python
from src.models.inference import GripPredictor

predictor = GripPredictor.load("models/")
prediction = predictor.predict({
    "track_temp": 35.0,
    "air_temp": 25.0,
    "tire_temp_avg": 85.0,
    # ... more features
})
print(f"Grip: {prediction['grip_coefficient']:.3f}")
```
