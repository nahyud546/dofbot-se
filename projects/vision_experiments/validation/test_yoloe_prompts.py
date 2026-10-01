#!/usr/bin/env python3
"""Compare YOLOE text prompts + real-cube visual prompt for segment step.

Usage:
  .venv/bin/python projects/vision_experiments/validation/test_yoloe_prompts.py \
      --images 'workspaces/dofbot_robot_arm_6dof/sim_images/pose_*.png' \
      --out-dir /tmp/yoloe_test
  # with a real cube enrollment (snapshot from /dev/video2 + xyxy box):
  ... --enroll-image /tmp/robot_cam.png --enroll-bbox 414,268,676,465

Writes annotated overlays + report.json in out-dir.
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np

PROMPT_SETS = {
    "cube": ["cube"],
    "toy_block": ["toy block"],
    "colored_cube": ["colored cube"],
    "all_three": ["cube", "toy block", "colored cube"],
}


def annotate(img, boxes, color=(0, 255, 0)):
    out = img.copy()
    for x1, y1, x2, y2 in boxes:
        cv2.rectangle(out, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
    return out


def run_text(weights, img, prompts, conf):
    from ultralytics import YOLOE

    model = YOLOE(weights)
    model.set_classes(prompts, model.get_text_pe(prompts))
    result = model.predict(img, conf=conf, verbose=False)[0]
    boxes, confs = [], []
    if result.boxes is not None and result.masks is not None:
        boxes = result.boxes.xyxy.cpu().numpy().tolist()
        confs = result.boxes.conf.cpu().numpy().tolist()
    return boxes, confs


def run_visual(weights, img, enrollment, conf):
    import sys

    sys.path.insert(0, str(Path("workspaces/dofbot_robot_arm_6dof/src/cap_vision")))
    from cap_vision.yoloe_segmenter import YOLOESegmenter

    seg = YOLOESegmenter(weights, conf=conf)
    seg.visual_prompts = {
        "bboxes": np.asarray(enrollment["bboxes"], dtype=np.float32),
        "cls": np.asarray(enrollment["cls"], dtype=int),
    }
    instances = seg.segment(img)
    return [list(i.bbox) for i in instances], [i.confidence for i in instances]


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--images", required=True)
    ap.add_argument("--weights", default="workspaces/dofbot_robot_arm_6dof/src/cap_vision/config/yoloe-11s-seg.pt")
    ap.add_argument("--out-dir", default="/tmp/yoloe_test")
    ap.add_argument("--conf", type=float, default=0.3)
    ap.add_argument("--enroll-image", default="")
    ap.add_argument("--enroll-bbox", default="")
    args = ap.parse_args(argv)

    out = Path(args.out_dir)
    out.mkdir(parents=True, exist_ok=True)
    paths = sorted(glob.glob(args.images))
    if not paths:
        raise SystemExit(f"no images match {args.images}")

    enrollment = None
    if args.enroll_image and args.enroll_bbox:
        ref = cv2.imread(args.enroll_image)
        if ref is None:
            raise SystemExit(f"cannot read {args.enroll_image}")
        box = [float(v) for v in args.enroll_bbox.split(",")]
        enrollment = {"bboxes": [box], "cls": [0]}
        cv2.imwrite(str(out / "enroll_refer.png"), annotate(ref, [box], (255, 0, 0)))

    report = {"images": [], "prompt_sets": list(PROMPT_SETS) + (["visual_real_cube"] if enrollment else [])}
    for path in paths:
        img = cv2.imread(path)
        if img is None:
            continue
        entry = {"image": Path(path).name, "shape": list(img.shape[:2]), "results": {}}
        for key, prompts in PROMPT_SETS.items():
            boxes, confs = run_text(args.weights, img, prompts, args.conf)
            entry["results"][key] = {"prompts": prompts, "n_masks": len(boxes),
                                     "boxes": boxes, "confs": [round(c, 3) for c in confs]}
            cv2.imwrite(str(out / f"{Path(path).stem}_{key}.png"), annotate(img, boxes))
        if enrollment:
            boxes, confs = run_visual(args.weights, img, enrollment, args.conf)
            entry["results"]["visual_real_cube"] = {
                "prompts": "visual(bbox from real cube frame)", "n_masks": len(boxes),
                "boxes": boxes, "confs": [round(c, 3) for c in confs]}
            cv2.imwrite(str(out / f"{Path(path).stem}_visual.png"), annotate(img, boxes, (255, 0, 255)))
        report["images"].append(entry)

    (out / "report.json").write_text(json.dumps(report, indent=2), encoding="utf-8")
    print(f"{'image':22s} {'cube':>4s} {'toy':>4s} {'colored':>7s} {'all3':>4s}" +
          (" {'visual':>6s}" if enrollment else ""))
    for e in report["images"]:
        r = e["results"]
        line = (f"{e['image']:22s} {r['cube']['n_masks']:>4d} {r['toy_block']['n_masks']:>4d} "
                f"{r['colored_cube']['n_masks']:>7d} {r['all_three']['n_masks']:>4d}")
        if enrollment:
            line += f" {r['visual_real_cube']['n_masks']:>6d}"
        print(line)
    print(f"wrote {out}/report.json + overlays")


if __name__ == "__main__":
    main()
