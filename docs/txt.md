codex resume 019f89af-26ae-7f82-9cf3-ba8a3e52f8a7 57倍问题（角速度）

txt文件以及空目录不要动，另外再根据情况修改.gitignore文件

背景：我的主要开发环境是 Ubuntu 20.04、ROS Noetic、PX4以及gazebo，现在要研究一个无人机编队导航算法：用kinodynamicA*进行全局轨迹规划，然后mpc再进行局部规划和编队跟踪。局部规划考虑动态和静态避障，采用分布式规划；全局规划采用集中式（考虑到分布式规划会出现路径交叉的问题）；全局规划的作用是提供全局路径引导避免局部最优以及为mpc提供较好的优化初始值，全局+局部跑通之后再加入rl动态调节mpc代价函数的权重矩阵以及预测时域，整个算法仿真做好后再进行实机实验，然后写一篇论文。因为之前复现过一篇利用lmpc（mpc的基础上增加利用反步法求解出的辅助控制器约束保证系统李雅普诺夫稳定性）进行编队追踪包围的论文，所以现在沿用之前的算法，也使用lmpc并采用casadi的ipopt求解器求解最优（若没有更适合的求解器）。但是原论文直接将障碍物的中心位置给出并仅考虑与中心位置的距离，而我的算法框架加入了感知模块，要处理障碍物点云。一开始处理为esdf地图，然后利用三线插值的方法使其代价函数的障碍物距离惩罚项可以被求导从而进行求解，但是发现出现了两个致命问题：求解器与插值器难以兼容且计算量巨大。所以打算参考～/catkin_ws/src/form_nav/Mapless Collision-Free Flight via MPC using Dual KD-Trees in Cluttered.pdf所示的论文提出的方法，利用当前状态->建立 obatacle KD-tree->根据当前参考点查询最近障碍点->构造 MPC
cost->IPOPT 优化这种方式来处理障碍物距离惩罚。参考的论文算法代码路径为/home/raner/catkin_ws/src/Avoid-MPC/roswrapper/ros/src/avoid_mpc。按“1.单无人机mpc仅跟踪全局路径->2.编队跟踪->3.加入kdtree，mpc加入静态避障->4.加入动态避障->5.加入稳定性约束->6.加入主动提供较好mpc优化初始值"的步骤逐步完善算法编写。

cd /home/raner/catkin_ws/src/form_nav/bag
rosbag record --lz4 \
  /uav0/mavros/local_position/pose \
  /uav0/mavros/local_position/velocity_local \
  /uav0/mavros/local_position/odom_world \
  /uav0/mavros/setpoint_velocity/cmd_vel \
  /uav0/mavros/setpoint_position/local \
  /uav0/global_trajectory \
  /uav0/predicted_path_share \
  /uav0/traveled_path \
  /uav0/mpc_sub_goal_marker \
  /uav0/mpc_predicted_path_marker \
  /uav0/mpc_queried_obstacles \
  /move_base_simple/goal \
  /uav0/obstacle_cloud \
  /map_generator/global_cloud \
  /tf /tf_static

cd /home/raner/catkin_ws/src/form_nav/bag
rosbag record --lz4 -e "/uav[0-5]+/(mavros/local_position/(pose|velocity_local|odom_world)|mavros/setpoint_velocity/cmd_vel|mavros/setpoint_position/local|global_trajectory|predicted_path_share|traveled_path|mpc_sub_goal_marker|mpc_predicted_path_marker|mpc_queried_obstacles|obstacle_cloud)"