std_msgs/Header header
uint32 trajectory_id
uint32 steps
# Per waypoint: [x, y, z, vx, vy, vz].
# Position and linear velocity are expressed in header.frame_id.
float64[] waypoints
