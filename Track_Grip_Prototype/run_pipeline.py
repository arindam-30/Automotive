"""
Complete Pipeline Runner for Track Grip Optimisation

Single script to run the entire training and evaluation pipeline.
"""

import os
import sys
import argparse

# Add src to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.data.generate_data import generate_dataset
from src.models.train_models import main as train_models
from src.evaluation.evaluate import run_full_evaluation


def main():
    parser = argparse.ArgumentParser(
        description="Run complete grip prediction pipeline"
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Quick test mode (fewer sessions, epochs)",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="output/",
        help="Output directory",
    )
    args = parser.parse_args()

    print("=" * 70)
    print("  TRACK GRIP OPTIMISATION - COMPLETE PIPELINE")
    print("=" * 70)

    # Directories
    data_dir = os.path.join(args.output, "data")
    raw_data_dir = os.path.join(data_dir, "raw")
    models_dir = os.path.join(args.output, "models")
    eval_dir = os.path.join(args.output, "evaluation")

    os.makedirs(raw_data_dir, exist_ok=True)
    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(eval_dir, exist_ok=True)

    # Step 1: Generate Data
    print("\n" + "=" * 70)
    print("STEP 1: GENERATING SYNTHETIC DATA")
    print("=" * 70)

    num_sessions = 50 if args.quick else 100
    generate_dataset(
        output_dir=raw_data_dir,
        num_sessions=num_sessions,
        cars_per_session=2,
    )

    # Step 2: Train Models
    print("\n" + "=" * 70)
    print("STEP 2: TRAINING MODELS")
    print("=" * 70)

    raw_data_path = os.path.join(raw_data_dir, "all_sessions.csv")
    epochs = 30 if args.quick else 100

    train_models(
        raw_data_path=raw_data_path,
        output_dir=args.output,
        epochs=epochs,
        batch_size=64,
    )

    # Step 3: Evaluate
    print("\n" + "=" * 70)
    print("STEP 3: RUNNING EVALUATION")
    print("=" * 70)

    processed_data_path = os.path.join(data_dir, "processed", "train.npz")
    run_full_evaluation(
        models_dir=models_dir,
        data_path=processed_data_path,
        output_dir=eval_dir,
    )

    # Summary
    print("\n" + "=" * 70)
    print("PIPELINE COMPLETE!")
    print("=" * 70)
    print(f"""
Output directory: {os.path.abspath(args.output)}

Files created:
  - data/raw/              : Raw telemetry CSV files
  - data/processed/        : Feature-engineered NPZ files
  - models/                : Trained model artifacts
  - evaluation/            : Evaluation reports and plots

To start the inference server:
  python src/serving/inference_server.py --models {models_dir}

To view the API documentation:
  Open http://localhost:8000/docs in your browser

To run the Jupyter notebook:
  jupyter notebook notebooks/01_quick_start.ipynb
""")


if __name__ == "__main__":
    main()
