"""Train the first class-agnostic cube instance segmentation checkpoint.

Dataset YAML must have train and val image splits. Polygon labels use a single
"object" class; identity is inferred separately from tag/color/DINO evidence.
"""
import argparse
from pathlib import Path

import yaml


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--data", type=Path, required=True)
    parser.add_argument("--base-weights", default="yolo11n-seg.pt")
    parser.add_argument("--epochs", type=int, default=100)
    parser.add_argument("--device", default="auto")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args(argv)
    if not args.data.is_file():
        parser.error("dataset YAML missing")
    with args.data.open(encoding="utf-8") as stream:
        dataset = yaml.safe_load(stream)
    if not all(key in dataset for key in ("train", "val")):
        parser.error("dataset requires train and val splits")
    names = dataset.get("names")
    if names not in ({0: "object"}, ["object"], {"0": "object"}):
        parser.error("segmentation dataset must contain one 'object' class")
    if args.epochs <= 0:
        parser.error("epochs must be positive")
    from ultralytics import YOLO
    model = YOLO(args.base_weights)
    options = {"data": str(args.data.resolve()), "imgsz": 640,
               "epochs": args.epochs, "project": str(args.output_dir.resolve()),
               "name": "cube_instances"}
    if args.device != "auto":
        options["device"] = args.device
    result = model.train(**options)
    print(result.save_dir)


if __name__ == "__main__":
    main()
