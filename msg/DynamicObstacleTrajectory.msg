std_msgs/Header header
int32 obstacle_id
uint8 shape                 # 0=球体, 1=圆柱
float64 radius              # 障碍物半径 (m)，默认 0.3
float64 height              # 圆柱高度 (m)，球体填 0
float64 dt
uint32 steps
# Per step: [x, y, z, yaw, vx, vy, vz, yaw_rate].
# 位姿与速度均为 header.frame_id (world) 系；z 为形状中心高度
# （圆柱 z=H/2 即坐在地面，球体 z 为球心）。
float64[] states
