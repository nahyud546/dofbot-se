import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np
import yaml
from cap_vision.object_pipeline import AprilTagPoseEstimator, ObjectInstance, rigid
from cap_vision.object_perception_node import load_models
from dt_apriltags import Detector
from scipy.spatial.transform import Rotation

def calculate_errors(pose_gt, pose_pred):
    T_gt = pose_gt.camera_T_object
    T_pred = pose_pred.camera_T_object
    
    t_err = np.linalg.norm(T_gt[:3, 3] - T_pred[:3, 3])
    
    R_gt = Rotation.from_matrix(T_gt[:3, :3])
    R_pred = Rotation.from_matrix(T_pred[:3, :3])
    r_err = np.linalg.norm((R_gt.inv() * R_pred).as_rotvec())
    
    return t_err, math.degrees(r_err)


def main():
    parser = argparse.ArgumentParser(description="Evaluate RGB pose vs AprilTag ground truth.")
    parser.add_argument("--images_dir", type=Path, required=True, help="Directory containing RGB images")
    parser.add_argument("--config", type=Path, required=True, help="Camera config YAML")
    parser.add_argument("--models", type=Path, required=True, help="Object models YAML")
    args = parser.parse_args()

    models = load_models(args.models)
    with open(args.config) as f:
        config = yaml.safe_load(f)
    
    K = np.asarray(config["camera"]["K"], float).reshape(3, 3)
    D = np.asarray(config["camera"]["distortion"], float)
    
    tag_estimator = AprilTagPoseEstimator(Detector(families="tag36h11"))
    
    # Placeholder cho viec load RGB pose estimator model
    # rgb_estimator = RGBPoseEstimator(weights_path)

    results = []
    
    for img_path in args.images_dir.glob("*.png"):
        bgr = cv2.imread(str(img_path))
        if bgr is None: continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        
        # Gia su ta co instance mask tu segmenter cho image nay, hoac the hien bang toan bo anh (de tag detector tu tim)
        mask = np.ones(rgb.shape[:2], dtype=bool)
        
        for cube_id, model in models.items():
            # Get Ground Truth tu AprilTag
            pose_gt = tag_estimator.estimate(rgb, mask, model, {"K": K, "D": D})
            if not pose_gt:
                continue
                
            # Get Prediction tu RGB pose estimator (mocking bang chinh GT de demo hoac pending tich hop)
            # pose_pred = rgb_estimator.estimate(rgb, mask, model, {"K": K, "D": D})
            pose_pred = None
            
            if pose_pred:
                t_err, r_err = calculate_errors(pose_gt, pose_pred)
                results.append({"image": img_path.name, "cube_id": cube_id, "t_err_m": t_err, "r_err_deg": r_err, "reproj_err_px": pose_pred.reprojection_error_px})
                
    if results:
        print(f"Evaluated {len(results)} samples.")
        print(f"Mean Translation Error: {np.mean([r['t_err_m'] for r in results])*1000:.2f} mm")
        print(f"Mean Rotation Error: {np.mean([r['r_err_deg'] for r in results]):.2f} deg")
    else:
        print("No paired GT and Prediction found to evaluate. Check RGB Pose Estimator integration.")

if __name__ == "__main__":
    main()
