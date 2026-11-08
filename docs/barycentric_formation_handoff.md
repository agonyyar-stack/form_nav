# 重心编队实施交接文档

> 日期：2026-11-08
>
> 仓库：`/home/raner/catkin_ws/src/form_nav`
>
> 分支：`develop/barycentric-formation`
>
> 当前提交：`ab91071 实现领导者-跟随者重心编队体系`
>
> 工作树状态：干净，无未提交修改

## 1. 新对话应先读什么

请按以下顺序阅读：

1. `docs/barycentric_formation_handoff.md`，即本文档。
2. `docs/barycentric_formation_code_changes.md`，重点只看第 7–10 步。
3. `docs/context.md`，了解现有 MPC、A*、通信和避障结构。
4. `config/formations.yaml` 和 `config/multi_uav_frames.yaml`，确认当前队形。

不要把已经完成的第 1–6 步重新实现。后续工作从“编队失效与退化处理”开始。

---

## 2. 当前实现目标

采用领导者—跟随者重心编队：

- 领导者：`uav1、uav3、uav5`
- 跟随者：`uav0、uav2、uav4`
- 只有领导者运行全局 A*
- 跟随者不使用独立全局路径，只跟踪领导者预测轨迹的重心组合
- 所有无人机继续运行 MPC、静态避障、动态避障和机间避碰
- 所有无人机继续广播预测轨迹

核心关系：

$$
p_i^*(k)=\sum_{l\in\{1,3,5\}}\mu_{il}p_l(k)
$$

---

## 3. 当前编队与权重

### 3.1 标称队形

| 无人机 | 角色 | 标称位置 `(x, y)` / m |
| --- | --- | --- |
| uav1 | 领导者 | `(-1.2, -2.0)` |
| uav3 | 领导者 | `(1.2, 0.0)` |
| uav5 | 领导者 | `(-1.2, 2.0)` |
| uav0 | 跟随者 | `(-0.4, -1.0)` |
| uav2 | 跟随者 | `(0.4, 0.0)` |
| uav4 | 跟随者 | `(-0.4, 1.0)` |

领导者三角形面积：`4.8 m²`。

### 3.2 重心权重

领导者顺序为 `[uav1, uav3, uav5]`。

| 跟随者 | uav1 | uav3 | uav5 |
| --- | ---: | ---: | ---: |
| uav0 | `7/12` | `1/3` | `1/12` |
| uav2 | `1/6` | `2/3` | `1/6` |
| uav4 | `1/12` | `1/3` | `7/12` |

`formations.yaml` 中使用十二位小数，避免严格权重和校验失败。

### 3.3 Gazebo 初始位置与 RViz 静态 TF

以世界坐标 `(-23.0, -1.5, 0.0)` 为编队基准：

| 无人机 | Gazebo spawn / RViz TF 原点 |
| --- | --- |
| uav0 | `(-23.4, -2.5, 0.0)` |
| uav1 | `(-24.2, -3.5, 0.0)` |
| uav2 | `(-22.6, -1.5, 0.0)` |
| uav3 | `(-21.8, -1.5, 0.0)` |
| uav4 | `(-23.4, -0.5, 0.0)` |
| uav5 | `(-24.2, 0.5, 0.0)` |

以下三处必须保持一致，修改队形时需要同时检查：

- `config/formations.yaml`
- `config/multi_uav_frames.yaml`
- `launch/multi_uav_gazebo.launch`

---

## 4. 已完成代码修改

### 第 1 步：配置结构与几何校验

- `include/form_nav/formation_config.h`
- `src/formation_config.cpp`
- `config/formations.yaml`
- `scripts/check_barycentric_formation.py`

已实现：

- `formation_mode`
- `leaders`
- `weights`
- `isLeader()`、`isFollower()`、`hasWeights()`
- 领导者数量、权重完整性、非负性、权重和校验
- 权重重构标称位置校验
- 领导者三角形退化校验

### 第 2 步：MPC 重心目标函数

- `include/form_nav/mpc.h`
- `src/mpc.cpp`

已实现：

- `Mpc::setBarycentricFormation()`
- `Mpc::barycentricTarget()`
- 缺少任一领导者时不静默归一化，返回失败

### 第 3 步：follower 重心编队代价

`Mpc::formationError()` 已支持重心模式：

```text
e = p_i - sum_l(w_l * p_l)
S_f = [e.x, e.y, e.z, 0]
```

领导者编队权重为 0；follower 使用 XYZ 等权、yaw 为 0 的 `Q_f_`。

### 第 4 步：分离编队订阅与避碰邻居

已实现：

- `Mpc::setCollisionNeighbors()`
- `collisionAvoidance()` 只处理避碰邻居白名单
- follower 为计算 `p*` 额外订阅的领导者不会自动进入 `J_ca`

订阅集合为：

```text
required_neighbor_ids =
    union(formation_leaders, collision_neighbors)
```

### 第 5 步：follower 脱离全局路径

已实现：

- `Mpc::setFormationRole()`
- leader 继续使用全局路径和链式逐点参考
- follower 不要求 `has_global_traj_`
- follower 不调用 `projectToPath()` 或 `generateStepTargets()`
- follower 使用领导者预测轨迹生成逐点重心参考
- follower 的制动和到达目标为 `mission_goal + own_offset`
- follower 到达后还要求三个领导者速度接近零
- 所有 MPC 节点订阅 `/move_base_simple/goal`
- follower 只为末端目标订阅，不据此生成路径

### 第 6 步：只给领导者启动全局规划器

已实现：

- `launch/astar_grid.launch` 增加 `enable_global_planner`
- `launch/kino_astar_grid.launch` 增加同名开关
- `formation_test.launch` 中：
  - `uav1/uav3/uav5`：启用全局规划器
  - `uav0/uav2/uav4`：禁用全局规划器
- follower 仍保留 `uavX_odom_to_world`
- 固定场景文件可通过 `scene_file` 透传

---

## 5. 当前数据流

1. 用户发布 `/move_base_simple/goal`。
2. `uav1/uav3/uav5` 分别运行 A* 或 kinoA*。
3. 三个领导者 MPC 跟踪各自全局路径。
4. 三个领导者广播 `/uavX/predicted_path_share`。
5. follower 将三个领导者预测轨迹对齐到本机求解时刻。
6. follower 计算每个预测步的：

   ```text
   p_i*(k) = sum_l(mu_il * p_l(k))
   ```

7. follower MPC 主要跟踪 `p_i*(k)`。
8. 所有无人机广播自身预测轨迹用于机间避碰。
9. 领导者到达自己的路径末端后发布静止预测。
10. follower 到达自身最终槽位且三个领导者已停稳后锁定悬停。

---

## 6. 当前通信与避碰拓扑

`config/formations.yaml` 当前避碰邻居：

```yaml
uav0: [1, 2, 3, 4, 5]
uav1: [3, 5]
uav2: [1, 3, 5]
uav3: []
uav4: [1, 2, 3, 5]
uav5: [3]
```

实际必需订阅还要加入 follower 的编队领导者：

```text
uav0: [1, 2, 3, 4, 5]
uav1: [3, 5]
uav2: [1, 3, 5]
uav3: []
uav4: [1, 2, 3, 5]
uav5: [3]
```

当前依赖图是无环的，`uav3` 是启动根节点。

### 仍然存在的风险

`controlLoop()` 等待邻居时仍只执行：

```text
publishHoldCommand()
return
```

不会发布静止预测轨迹。如果某个关键上游节点在第一次求解前没有发布预测轨迹，依赖它的节点仍会持续等待。

建议在后续第 7 步优先加入：

```cpp
publishStationaryPrediction(solve_stamp);
```

让所有节点至少能通过静止轨迹完成第一次求解，再进入正常的编队状态机。

---

## 7. 已完成验证

已执行并通过：

```bash
catkin_make --pkg form_nav -j2
./scripts/check_barycentric_formation.py config/formations.yaml
xmllint --noout launch/multi_uav_gazebo.launch
xmllint --noout launch/formation_test.launch
git diff --check
```

检查结果：

- 领导者三角形面积：`4.8 m²`
- 权重重构位置最大误差：约 `4e-13 m`
- Gazebo spawn、`formations.yaml` offsets、RViz 静态 TF 三者一致
- A* launch 节点展开结果：
  - `uav0/uav2/uav4` 只有 `odom_to_world`
  - `uav1/uav3/uav5` 有 `odom_to_world + astar_node`
- kinoA* 的角色开关行为相同

尚未完成：

- 完整的多机 Gazebo 障碍环境闭环验证
- 编队误差、净空、求解耗时、求解失败率统计
- 失效和退化状态验证

当前终端曾因找不到 ROS 包 `px4` 而无法展开完整 `formation_test.launch`。新环境需要先正确加载 PX4 的 ROS 包路径。

---

## 8. 下一步实施重点

后续直接执行 `docs/barycentric_formation_code_changes.md` 的第 7–10 步。

### 第 7 步：编队失效与退化处理

增加显式状态：

```cpp
enum class FormationState {
    WAIT_LEADERS,
    ACTIVE,
    FROZEN,
    DEGRADED,
    ARRIVED
};
```

需要处理：

- 三个领导者是否齐全
- 轨迹是否超时
- 轨迹是否非法或含 NaN/Inf
- 三角形有符号面积是否低于阈值
- 三角形方向是否翻转
- 领导者边长方是否过短
- MPC 连续求解失败
- 短时冻结最近一次有效 `p*`
- 长时间失效后安全悬停
- 连续有效后恢复 `ACTIVE`

### 第 8 步：MPC 耗时和失败率

在 `mpc_ptr->solve(...)` 前后使用 `std::chrono::steady_clock` 计时。

至少统计：

```text
solve_ms 当前值、均值、P50、P95、最大值
solve_attempts_total
solve_failure_total
failure_rate_total
failure_rate_10s
solve_availability
consecutive_failures
max_consecutive_failures
```

建议发布：

```text
/uavX/mpc_diagnostics
```

使用 `std_msgs/Float64MultiArray`，字段定义见实施计划第 8 步。

### 第 9 步：可视化和指标

建议话题：

```text
/formation/triangle
/uavX/formation_target
/uavX/formation_error
/uavX/formation_barycentric
/formation/area
```

离线指标至少包含：

- `||p_i - p_i*||`
- 实际重心坐标
- 领导者三角形面积和方向
- 最小净空
- 最小机间距离
- MPC 耗时和失败率
- 退化次数与持续时间

### 第 10 步：直接进行障碍环境验证

不再分多个无障碍、悬停、静态障碍阶段。直接在固定障碍场景中验证完整导航。

建议先使用：

```text
config/scenes/simple.yaml
```

再使用：

```text
config/scenes/fixed_scene.yaml
```

核心验收：

- 六机均进入 MPC 求解
- 只有 `uav1/uav3/uav5` 有全局轨迹
- follower 的 `p*` 可由三个领导者实时重构
- 无碰撞
- MPC 求解 P95 小于 `50 ms`
- 求解失败率不超过 `1%`
- 有效求解可用率不低于 `95%`
- 退化事件均被记录

---

## 9. 推荐新对话开场提示

可直接复制：

```text
项目：/home/raner/catkin_ws/src/form_nav
分支：develop/barycentric-formation
当前提交：ab91071

请先完整阅读：
1. docs/barycentric_formation_handoff.md
2. docs/barycentric_formation_code_changes.md 的第 7-10 步
3. docs/context.md

第 1-6 步已经完成并提交，不要重复修改。当前队形领导者是
uav1/uav3/uav5，跟随者是 uav0/uav2/uav4。

本轮从第 7 步开始：先增加编队状态机、领导者有效性检查、
三角形退化检查和安全退化；同时修复等待邻居时不发布静止预测轨迹的问题。
不要提前实现 CBF 或指定时间恢复。
```

---

## 10. 启动与录包命令

### 10.1 编译

```bash
cd /home/raner/catkin_ws
catkin_make --pkg form_nav -j2
source devel/setup.bash
```

### 10.2 检查编队配置

```bash
cd /home/raner/catkin_ws/src/form_nav
./scripts/check_barycentric_formation.py config/formations.yaml
```

### 10.3 启动当前固定场景

```bash
roslaunch form_nav formation_test.launch \
  scene:=1 \
  scene_file:=/home/raner/catkin_ws/src/form_nav/config/scenes/simple.yaml
```

### 10.4 录制调试 bag

```bash
mkdir -p /home/raner/catkin_ws/src/form_nav/bag

rosbag record \
  -O /home/raner/catkin_ws/src/form_nav/bag/barycentric_debug_$(date +%Y%m%d_%H%M%S).bag \
  /rosout /tf /tf_static /clock \
  /move_base_simple/goal \
  /gazebo/model_states \
  /uav{0..5}/mavros/state \
  /uav{0..5}/mavros/local_position/odom \
  /uav{0..5}/mavros/local_position/odom_world \
  /uav{0..5}/mavros/setpoint_velocity/cmd_vel \
  /uav{0..5}/global_trajectory \
  /uav{0..5}/predicted_path_share \
  /uav{0..5}/mpc_predicted_path_marker \
  /uav{0..5}/mpc_sub_goal_marker \
  /dynamic_obstacles/trajectory \
  /map_generator/global_cloud
```

相关主要文件：

- `config/formations.yaml`
- `config/multi_uav_frames.yaml`
- `config/scenes/simple.yaml`
- `include/form_nav/formation_config.h`
- `include/form_nav/mpc.h`
- `src/formation_config.cpp`
- `src/mpc.cpp`
- `src/mpc_node.cpp`
- `launch/astar_grid.launch`
- `launch/kino_astar_grid.launch`
- `launch/formation_test.launch`
- `launch/multi_uav_gazebo.launch`
- `scripts/check_barycentric_formation.py`
