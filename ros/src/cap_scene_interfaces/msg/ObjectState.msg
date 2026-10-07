# Both poses describe the object origin; their headers name the respective frames.
std_msgs/Header header
string track_id
string object_type
int32 object_id
int32 geometry_model_id
float32 identity_confidence
float32[4] identity_probabilities
int32[4] bbox_xyxy
sensor_msgs/Image mask
geometry_msgs/PoseStamped camera_pose
geometry_msgs/PoseStamped base_pose
float32 pose_confidence
float32 reprojection_error_px
string pose_method
string[] visible_surfaces
string state
# camera_pose can be useful for calibration/debug even before hand-eye exists.
bool camera_pose_valid
bool base_pose_valid
# Backward-compatible grasp gate: true only when base_pose_valid is true.
bool pose_valid
# Camera-frame pose is fresh and its proposal is safe for a top grasp.
bool top_grasp_ready
string reason
