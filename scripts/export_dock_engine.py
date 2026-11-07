#!/usr/bin/env python3
"""Build a local FP16 TensorRT engine with the Jetson CUDA Python environment."""
import argparse
from pathlib import Path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--weights', type=Path, default=(
        Path(__file__).resolve().parents[1] / 'src/kaboat_perception/models/best.pt'))
    args = parser.parse_args()
    import torch
    from ultralytics import YOLO
    if not torch.cuda.is_available():
        raise RuntimeError('Jetson CUDA-enabled PyTorch is required')
    torch.set_num_threads(2)
    YOLO(str(args.weights)).export(
        format='engine', quantize=16, imgsz=640, workspace=1,
        device=0, batch=1, dynamic=False, simplify=True)


if __name__ == '__main__':
    main()
