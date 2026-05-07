"""
Run Training and Evaluation Pipeline (Skip Data Generation)

Use this when synthetic data has already been generated.
"""

import os
import sys
import argparse

# Add src to path
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.models.train_models import main as train_models
from src.evaluation.evaluate import run_full_evaluation


def main():
    parser = argparse.ArgumentParser(
        description="Run training and evaluation (data already exists)"
    )
    parser.add_argument(
        "--data",
        type=str,
        default="data/raw/all_sessions.csv",
        help="Path to existing raw data",
    )
    parser.add_argument(
        "--output",
        type=str,
        default="output/",
        help="Output directory",
    )
    parser.add_argument(
        "--epochs",
        type=int,
        default=100,
        help="Number of LSTM training epochs",
    )
    parser.add_argument(
        "--quick",
        action="store_true",
        help="Quick mode (30 epochs)",
    )
    args = parser.parse_args()

    print("=" * 70)
    print("  TRACK GRIP OPTIMISATION - TRAINING & EVALUATION")
    print("=" * 70)

    # Verify data exists
    if not os.path.exists(args.data):
        print(f"\nERROR: Data file not found: {args.data}")
        print("Run data generation first:")
        print("  python src/data/generate_data.py --sessions 50")
        sys.exit(1)

    print(f"\nUsing existing data: {args.data}")

    # Directories
    models_dir = os.path.join(args.output, "models")
    eval_dir = os.path.join(args.output, "evaluation")

    os.makedirs(models_dir, exist_ok=True)
    os.makedirs(eval_dir, exist_ok=True)

    # Step 1: Train Models
    print("\n" + "=" * 70)
    print("STEP 1: TRAINING MODELS")
    print("=" * 70)

    epochs = 30 if args.quick else args.epochs

    train_models(
        raw_data_path=args.data,
        output_dir=args.output,
        epochs=epochs,
        batch_size=64,
    )

    # Step 2: Evaluate
    print("\n" + "=" * 70)
    print("STEP 2: RUNNING EVALUATION")
    print("=" * 70)

    processed_data_path = os.path.join(
        os.path.dirname(args.data).replace("raw", "processed"),
        "train.npz"
    )

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
  - data/processed/        : Feature-engineered NPZ files
  - models/                : Trained model artifacts
  - evaluation/            : Evaluation reports and plots

To start the inference server:
  python src/serving/inference_server.py --models {models_dir}

To view the API documentation:
  Open http://localhost:8000/docs in your browser
""")


if __name__ == "__main__":
    main()
