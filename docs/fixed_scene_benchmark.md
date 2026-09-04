# 固定复杂障碍物场景（定量对比实验）

## 1. 场景说明

固定场景由 `src/obs_generator.cpp` 按 `config/scenes/fixed_scene.yaml` 生成
（参考 `src/random_obs.cpp` 的发布方式，话题输出保持一致）：

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

### 动态障碍物（2026-08-31 新增）

固定场景默认带 **2 个**动态障碍物（1 圆柱 + 1 球体），用于验证 MPC 动态避障：
均为直线往复运动，运动中心位于世界坐标 x 轴（y=0 主线走廊）附近，扫掠范围
覆盖无人机主线飞行通道。动态障碍物的位置、形状、尺寸、轨迹方向/长度、
最大速度均在 `config/scenes/fixed_scene.yaml` 中定义（用编辑器修改后保存即可）：

| id | 形状 | 运动中心 (x, y) | 方向 | 轨迹长度 | 最大速度 |
| --- | --- | --- | --- | --- | --- |
| 0 | 圆柱 (r=0.3, h=2.0) | (-5.0, 0) | 0°（+x） | 3.0 m | 1.5 m/s |
| 1 | 球体 (r=0.3) | (-8.0, 0) | 90°（+y） | 1.0 m | 1.5 m/s |

运动规律（完全确定性，每次运行一致）：

```
p(t) = center + A·sin(ωt + φ)·dir，A = length/2，ω = v_max / A
```

轨迹采样 `dyn_obs/traj_dt` = 0.05 s、`dyn_obs/traj_steps` = 21（与 MPC
`N=20` 网格对齐，修改 MPC 预测时域时需同步调整）。

发布方式与 `random_obs` 保持一致：

- `/map_generator/global_cloud`、`/map_generator/local_cloud` —— 仍仅含静态
  立方体（A*/静态 KD-tree 使用，动态障碍物不进入，避免污染全局规划）
- `/map_generator/dynamic_cloud` —— 动态障碍物点云（仅可视化）
- `/dynamic_obstacles/trajectory` —— 动态障碍物预测轨迹（MPC 动态避障输入，
  `form_nav::DynamicObstacleTrajectory`，world 系）

关闭动态障碍物（静态基准对比）：`dyn_obs_num:=0`。

## 2. 启动方式

单独查看场景（不启动规划栈）：

```bash
roslaunch form_nav obs_generator.launch rviz:=true
```

完整定量对比实验（Gazebo + 6 机编队 + kino A* + MPC，使用固定场景）：

```bash
roslaunch form_nav formation_test.launch scene:=1   # 1=fixed（默认），2=random
```

单机快速验证（Gazebo + uav0 + kino A* + MPC，使用固定场景）：

```bash
roslaunch form_nav single_test.launch scene:=1
```

`scene:=1`（默认）时自动用 `obs_generator`（YAML 场景）替换 `random_obs`，
其余节点与话题不变；`scene:=2` 保持原有随机障碍物行为。该参数在
`formation_test.launch` 与 `single_test.launch` 中均已支持。

固定场景默认带上述 1 个动态圆柱；如需纯静态基准，可在启动时传
`dyn_obs_num:=0`（作用于 `obs_generator.launch`，例如
`roslaunch form_nav obs_generator.launch dyn_obs_num:=0`）；
自定义场景文件可传 `scene_file:=path/to/my_scene.yaml`。

## 2.5 可视化场景编辑器

`scripts/scene_editor.py` 是基于 PyQt5 的 2D 俯视编辑器，用于手动自定义
障碍物布局：

```bash
python3 scripts/scene_editor.py                          # 打开默认固定场景
python3 scripts/scene_editor.py --scene fixed_scene.yaml    # 打开指定场景
python3 scripts/scene_editor.py --publish                # 编辑 + 实时发布到 ROS
```

功能：

- 静态障碍物：立方体（`cuboid`，`size=[lx, ly, h]`）或圆柱（`cylinder`，
  `radius`/`height`），可编辑位置与尺寸
- 动态障碍物：圆柱或球体，可编辑位置、尺寸、轨迹方向（`direction_deg`，
  `0=+x, 90=+y`）与轨迹长度（`length`，峰峰长度，振幅 = length/2）；
  最大速度默认 1.5 m/s（可编辑），仅支持直线往复运动
- 画布上直接用鼠标编辑：拖动障碍物移动位置，拖动选中障碍物四角手柄调整
  xy 尺寸（立方体调长宽、圆柱/球体调半径），右侧属性面板精确编辑；
  编辑按钮位于右侧属性面板下方，底部时间滑条 + 播放按钮可预览动态障碍物
  运动与扫掠区域
- 多选与组编辑：画布空白处拖拽框选、按住 Ctrl 单击逐个多选（Ctrl/Shift
  + 框选为追加选择）；右侧障碍物列表支持 Shift 范围多选 / Ctrl 逐个多选，
  画布与列表选择双向同步。多选后可整体拖动同时移动，也可在右侧属性面板
  组编辑共有参数（某参数各障碍物取值不一致时显示"多值"占位，输入新值按
  Enter 统一赋值给所有选中障碍物）
- 数值输入与界面状态记忆：数值输入框输入完成后按 Enter（或失焦）才应用，
  不随按键实时修改；关闭时自动记住窗口大小、左右区域占比与最近打开的
  文件，下次启动自动恢复（命令行 `--scene` 优先级更高，上次文件不存在时
  回退默认场景）
- 冲突检查：静态-静态重叠、动态扫掠与静态相交、动态-动态扫掠相交会红色
  高亮并在状态栏提示
- 保存/加载 YAML 场景文件（与 `obs_generator` 节点共用格式），可用
  `scene_file:=` 传给 `obs_generator.launch` 直接运行

`--publish` 模式会把当前场景实时发布到
`/map_generator/global_cloud`、`local_cloud`、`dynamic_cloud` 与
`/dynamic_obstacles/trajectory`（需先 `source devel/setup.bash`），可在 RViz
中直接看到编辑效果。

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
