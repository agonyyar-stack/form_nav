# 固定复杂障碍物场景（定量对比实验）

## 1. 场景说明

固定场景由 `src/fixed_obs.cpp` 生成（参考 `src/random_obs.cpp` 的发布方式，
话题输出保持一致）：

- 发布话题：`/map_generator/global_cloud`、`/map_generator/local_cloud`
  （`sensor_msgs/PointCloud2`，`world` 坐标系），频率默认 10 Hz
- 场景范围：20 m × 20 m × 3 m，左边缘中心位于 `(x, y) = (-20, 0)`，即
  `x ∈ [-20, 0]`，`y ∈ [-10, 10]`，`z ∈ [0, 3]`
- 场景内仅包含立方体障碍物（无连续墙体）：高度 0.5~3 m，
  底面长宽 0.5~3 m 且长宽可不相同
- 共 30 个障碍物，散乱分布（不按网格/行列对齐）；大尺寸
  （底面 ≥2 m 或高度 ≥2.5 m）约 6 个，其余为中/小型
- 场景左侧（起飞区 `x < -20`）与右侧（目标区 `x > 0`）保持空旷
- 完全确定性：不使用随机数，每次运行生成的点云完全一致，可直接横向对比不同算法策略

### 场景结构（俯视粗略分布，每格 2 m，`▓` 表示该格有障碍物）

```
    x=-20    -18    -16    -14    -12    -10     -8     -6     -4     -2     0
y=  +9  |·|·|·|·|·|▓|·|▓|·|·|
y=  +7  |·|·|·|·|▓|·|▓|·|·|▓|
y=  +5  |·|·|·|·|·|·|·|·|·|▓|
y=  +3  |·|·|·|·|▓|·|·|·|·|·|
y=  +1  |·|·|▓|·|·|·|·|·|·|·|
y=  -1  |·|·|·|▓|·|·|·|·|·|·|
y=  -3  |·|·|▓|·|·|·|▓|▓|·|·|
y=  -5  |·|·|·|·|▓|·|·|·|▓|·|
y=  -7  |▓|·|·|·|·|▓|·|·|·|·|
y=  -9  |·|▓|·|▓|·|·|·|·|·|·|
      入口 (x≈-19, y≈0)                   出口 (x≈-2, y≈0) -> 目标区 x>0
```

障碍物位置刻意散乱，存在多条蜿蜒可行路径（主干通道间隙约 1.6~2.5 m，
个别旁侧间隙约 0.7~1.0 m，互不重叠）；入口、出口附近留空 ≥1.6 m。
低矮立方体（高度 <1.5 m）也可选择从上方越过。

全部 30 个立方体障碍物（中心位置为世界坐标，`ox=-20`，`oy=0`）：

| 序号 | 中心 (x, y) | 底面长 × 宽 (m) | 高度 (m) |
| --- | --- | --- | --- |
| 1 | (-19.2, -7.0) | 1.3 × 1.6 | 1.6 |
| 2 | (-18.4, -4.8) | 1.1 × 1.3 | 0.9 |
| 3 | (-17.8, 4.2) | 1.6 × 1.2 | 2.4 |
| 4 | (-18.2, 7.6) | 1.3 × 1.6 | 1.2 |
| 5 | (-16.4, -9.0) | 1.6 × 1.3 | 2.0 |
| 6 | (-15.8, -6.2) | 2.1 × 1.5 | 2.8 |
| 7 | (-15.2, -2.4) | 1.2 × 1.6 | 1.8 |
| 8 | (-14.5, 1.5) | 1.5 × 1.4 | 3.0 |
| 9 | (-15.4, 5.8) | 1.3 × 1.5 | 1.0 |
| 10 | (-14.0, 8.2) | 1.8 × 1.2 | 2.2 |
| 11 | (-12.4, -8.2) | 1.5 × 1.7 | 1.4 |
| 12 | (-11.7, -5.0) | 2.2 × 1.4 | 2.6 |
| 13 | (-12.8, -1.0) | 1.2 × 1.5 | 2.0 |
| 14 | (-11.0, 2.4) | 1.6 × 1.3 | 1.2 |
| 15 | (-11.4, 6.4) | 1.4 × 1.8 | 2.4 |
| 16 | (-9.8, 8.8) | 1.8 × 1.5 | 1.0 |
| 17 | (-8.5, -7.0) | 1.3 × 1.6 | 2.2 |
| 18 | (-7.4, -3.6) | 2.0 × 1.3 | 1.5 |
| 19 | (-6.4, -7.8) | 1.4 × 1.3 | 2.8 |
| 20 | (-8.2, 3.6) | 1.2 × 1.6 | 0.8 |
| 21 | (-6.2, 6.8) | 1.7 × 1.3 | 1.9 |
| 22 | (-5.4, 9.0) | 1.4 × 1.2 | 2.3 |
| 23 | (-5.8, 2.6) | 1.3 × 1.4 | 2.0 |
| 24 | (-4.2, -8.6) | 1.6 × 1.8 | 1.3 |
| 25 | (-3.4, -5.4) | 1.3 × 1.5 | 2.4 |
| 26 | (-4.6, -2.6) | 1.8 × 1.2 | 1.7 |
| 27 | (-3.8, 5.2) | 1.5 × 1.3 | 1.1 |
| 28 | (-1.6, 7.4) | 1.6 × 1.4 | 2.6 |
| 29 | (-0.8, 4.6) | 1.2 × 1.5 | 0.8 |
| 30 | (-1.0, -5.8) | 1.3 × 1.2 | 1.6 |

所有障碍物互不重叠，相邻最小间隙约 0.7 m，主干可行通道间隙约 1.6~2.5 m；
入口、出口附近留空 ≥1.6 m，起飞区（x < -20）与目标区（x > 0）完全空旷。

## 2. 启动方式

单独查看场景（不启动规划栈）：

```bash
roslaunch form_nav fixed_obs.launch rviz:=true
```

完整定量对比实验（Gazebo + 6 机编队 + kino A* + MPC，使用固定场景）：

```bash
roslaunch form_nav formation_planning.launch scene:=fixed
```

单机快速验证（Gazebo + uav0 + kino A* + MPC，使用固定场景）：

```bash
roslaunch form_nav test.launch scene:=fixed
```

`scene` 参数默认 `random`（保持原有随机障碍物行为不变），传 `fixed` 时自动用
`fixed_obs` 替换 `random_obs`，其余节点与话题不变；该参数在
`formation_planning.launch` 与 `test.launch` 中均已支持。

## 3. 发布固定目标

所有无人机订阅同一个 `/move_base_simple/goal`（`geometry_msgs/PoseStamped`，
`world` 坐标系），每架无人机的终点 = 目标点 + 本机编队偏移
（`config/formations.yaml`，z 在 `kino_astar_node` 中固定为 1.0 + 偏移）。

```bash
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'world', stamp: now}
pose:
  position: {x: 15.0, y: 0.0, z: 1.0}
  orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}"
```

目标点 `(15, 0)` 位于场景右侧空旷区，对应编队目标范围约
`x ∈ [14, 16]`、`y ∈ [-2, 2]`。若某架机未收到目标（例如规划节点启动晚于发布），
可用 `rostopic pub -r 1 /move_base_simple/goal ...` 连续发布几秒后再停。

## 4. 录包命令

```bash
rosbag record -O bench_fixed_scene \
  /map_generator/global_cloud /map_generator/local_cloud \
  /move_base_simple/goal /tf /tf_static \
  /uav{0..5}/mavros/local_position/odom \
  /uav{0..5}/mavros/local_position/odom_world \
  /uav{0..5}/kino_astar/trajectory \
  /uav{0..5}/predicted_path_share \
  /uav{0..5}/traveled_path \
  /uav{0..5}/mavros/setpoint_velocity/cmd_vel \
  /uav{0..5}/sdf_map/occupancy_inflate \
  /uav{0..5}/grid_map/occupancy_inflate
```

说明：

- `/uavX/mavros/local_position/odom_world` 是世界系里程计（规划器实际使用的输入），
  `/odom` 为 Gazebo/MAVROS 原始里程计，两者都录便于核对坐标系
- `/uavX/kino_astar/trajectory` 是全局规划轨迹，`/uavX/predicted_path_share`
  是 MPC 预测轨迹，`/uavX/traveled_path` 是实际飞行路径
- `/uavX/mavros/setpoint_velocity/cmd_vel` 是 MPC 速度指令，用于分析控制品质
- `/uavX/sdf_map/occupancy_inflate`、`/uavX/grid_map/occupancy_inflate`
  是各机维护的局部占用地图（膨胀后），用于核对感知一致性与避障裕度
- 需要时长限制时可加 `--duration=300`（秒）或 `--split`；建议在发布目标前开始录制，
  以便记录完整的"起飞 → 过门 → 到达"过程

## 5. 定量对比建议

同一固定场景下分别运行不同算法策略（如 A*、kinodynamic A*、DWA、MPC，
或同一算法不同参数），每次使用相同的目标点与录包命令，然后对比：

| 指标 | 数据来源 | 建议统计量 |
| --- | --- | --- |
| 到达时间 | `/move_base_simple/goal` 时间戳 → `/uavX/mavros/local_position/odom_world` 首次进入终点 0.5 m | 均值 / 最大 / 标准差 |
| 路径长度 | `/uavX/traveled_path` 逐点累计距离 | 与理论最短路径之比 |
| 路径平滑度 | `/uavX/traveled_path` 航向变化率、`cmd_vel` 抖动 | 转角累积、速度方差 |
| 最近障碍距离 | `/uavX/mavros/local_position/odom_world` 与 `/map_generator/global_cloud` 最近邻距离 | 最小值 / 5% 分位 |
| 规划/控制频率 | `/uavX/kino_astar/trajectory`、`/uavX/predicted_path_share` 消息间隔 | 平均间隔、缺失率 |

由于场景完全固定，不同策略之间的差异只来自算法本身，可直接归因对比。
