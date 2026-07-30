std_msgs/Header header
int32 uav_id
float64 dt
uint32 steps
# Per step: [x, y, z, yaw, vx, vy, vz, yaw_rate].
# Pose and linear velocity are expressed in header.frame_id.
float64[] states
