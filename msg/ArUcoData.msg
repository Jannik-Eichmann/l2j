# boolean to declare, if an aruco marker is detected by the camera
bool marker_detected

# camera intrinsics
float32 intrinsics_fx
float32 intrinsics_fy
float32 intrinsics_ppx
float32 intrinsics_ppy

# aruco vertex coordinates in the image coordinate system
geometry_msgs/Pose2D corner_0
geometry_msgs/Pose2D corner_1
geometry_msgs/Pose2D corner_2
geometry_msgs/Pose2D corner_3