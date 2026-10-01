# Cube 6D with the joint-4 eye-in-hand camera

`cube_4x6_face_geometry.yaml` defines four 30 mm cubes. Its `+Z` face is the
20 mm AprilTag; `-Z` is the colour face. The four trash labels form the side
ring. The geometry loader checks this against `object_models.yaml` at startup.
The YAML tag corners list the same physical square as the AprilTag estimator,
but in a different cyclic order; the estimator keeps its established detector
corner order.

The RGB path needs a cube instance mask and **two distinct adjacent physical
faces** with reliable labels. It detects face quadrilaterals inside the mask,
rectifies them, classifies the printed face with DINO or the mostly uniform
colour face with HSV, enumerates cube corner correspondences, and accepts a
metric PnP pose only if reprojection is at most 2 px and the best solution is
unambiguous. One plain face cannot establish the cube's rotation. With too few
faces, the existing position-only `rgb_geometry` path remains available.
AprilTag pose takes precedence over position-only RGB evidence.

`cube_4x6_face_geometry.yaml` is the declared physical face mapping and is
therefore enabled by default (`face_mapping_verified:=true`). Accepted
tag-hidden poses use `rgb_faces_pnp` and participate in the full-6D gates.
Use `face_mapping_verified:=false` only while experimenting with a different
cube build or a deliberately unverified face ordering; that diagnostic mode
draws an orange `rgb_faces_provisional` wireframe and never marks it
graspable.

The checked-in `red_scene.yaml` has `camera.calibrated: false` and no eye-in-
hand transform. This prevents the general `base_T_cube` output, but does not
prevent camera-frame 6D pose estimation or the fixed observation-pose grasp
path. `cube_sort_3d.py` returns the arm to the same `READY_POSE` as
`cube_sort_stage1`, projects the visible face centre from the camera-frame
6D pose, then uses the existing stage1 pixel-to-KDL map. This path requires
the same 640x480 camera and the arm to remain at that observation pose while
perception selects a cube. `calibrate_tag_hand_eye` remains available for a
future mode that needs base-frame poses while the camera moves through
different arm poses; it is not required for fixed-pose sorting.

The cube perception node loads color bounds from `config/cube_color_hsv.yaml`;
these match the tuned ranges in `dofbot_color_sorting/HSV_config.txt`. A
recognised colour face can seed a cube instance without YOLOE. Trash artwork
is only an anchor: the node requires a separate closed outer-face contour and
a white ring before marking that proposal ready for grasp. A partial artwork
contour may still be tracked but cannot authorize a pick. RGB poses are
smoothed per track; single-face pose is a top-grasp estimate, not semantic 6D.
`ObjectState.camera_pose_valid` means a camera-frame pose was measured;
`top_grasp_ready` is the separate camera-frame grasp gate. `pose_valid` still
requires calibrated base-frame full 6D. The bridge requires two consistent,
fresh top-face TCP centres and square-symmetric yaw angles.
The sort bridge defaults to `--pick-x-offset-mm 15` because positive KDL X
moves the TCP toward the robot base/foot; use `0` to disable the correction.

## Optional hand-eye collection for arbitrary camera poses

This is optional for the current fixed-pose sort flow. Use it only when a
consumer needs base-frame cube poses while the camera moves through different
arm poses. Start the normal real-joint/TF and camera system first. Fix **one** AprilTag
rigidly to the table for the entire collection. A loose cube must not be used;
if a 30 mm cube is deliberately held fixed with its `+Z` tag uppermost, set
`--tag-to-table-m 0.030`. A flat printed tag on the table uses `0.0`.

`collect_tag_hand_eye` never sends a servo command. Move the robot with the
normal safe controller through 14 clearly different views of the fixed tag.
Use pitch and roll changes as well as yaw/translation; keep the tag well
inside the image and press Space only after the arm stops. The last four
captures are held out for validation.

```bash
ros2 run cap_vision collect_tag_hand_eye -- \
  --output /home/jloy/Desktop/robot-arm/config/calibration/tag_hand_eye_v1 \
  --tag-id 4 --tag-size-m 0.020 --tag-to-table-m 0.0

ros2 run cap_vision calibrate_tag_hand_eye -- \
  --dataset /home/jloy/Desktop/robot-arm/config/calibration/tag_hand_eye_v1/dataset.yaml \
  --config /home/jloy/Desktop/robot-arm/workspaces/dofbot_robot_arm_6dof/src/cap_vision/config/red_scene.yaml \
  --output /home/jloy/Desktop/robot-arm/config/calibration/red_scene_calibrated.yaml
```

The launch must receive the resulting `--output` config explicitly; otherwise
it keeps using the uncalibrated default `red_scene.yaml`.

The 16 photos in `ai/datasets/trash-images` show cubes on backgrounds; they
are references for face identity, not independent mask or pose ground truth.
The existing `collect_instance_dataset` collects tag-anchored masks, and
`train_object_segmenter` trains a one-class cube segmentation model. Include
manually labelled tag-hidden frames, varied backgrounds, table and in-gripper
frames, and separate camera sessions for validation. Pass its trained `.pt`
path as `segmentation_weights:=...` to `object_pipeline.launch.py`; the bundled
YOLOE remains a commissioning fallback until a cube-specific checkpoint is
available.

Run `object_pipeline.launch.py` with the default geometry file to obtain the
camera-frame 6D wireframe. The launch defaults `face_geometry` to
`<repo_root>/cube_4x6_face_geometry.yaml` and enables the declared face map.
For a base-frame grasp pose, consumers should require `pose_valid` and inspect
`pose_method`. The fixed-READY_POSE bridge can instead use
`top_grasp_ready` with `rgb_single_face_cube`; two adjacent faces remain
necessary for semantic full 6D without a tag. This node never moves the arm
automatically. Bridge `--dry-run` now prevents motor calls even after Space.
