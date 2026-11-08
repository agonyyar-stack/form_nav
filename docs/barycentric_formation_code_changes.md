# 重心编队代码修改实施计划

> 范围：仅描述从现有代码到“领导者规划、跟随者重心编队”的代码修改步骤。
>
> 原则：每一步尽量只改一层逻辑，完成一个可编译、可回退的代码提交后再进入下一步。不要把配置解析、MPC 代价、角色切换、失效处理和诊断统计一次性合入。

## 0. 当前实现基线

沿用当前已有的三角形队形：

- 领导者：`uav0、uav1、uav2`
- 跟随者：`uav3、uav4、uav5`
- 领导者负责全局 A*、路径跟踪和预测轨迹广播
- 跟随者不运行独立 A*，只跟踪领导者预测轨迹的重心组合
- 所有无人机继续运行 MPC、静态避障、动态避障和机间避碰
- 所有无人机继续广播 `PredictedTrajectory`

领导者标称位置：

| 领导者 | 标称位置（x, y） |
| --- | --- |
| uav0 | `(0.0, 0.0)` |
| uav1 | `(0.0, 3.0)` |
| uav2 | `(2.0, 1.5)` |

跟随者期望位置：

$$
p_i^*(k)=\sum_{l\in\{0,1,2\}}\mu_{il}p_l(k)
$$

当前三角形对应的重心权重：

| 跟随者 | uav0 | uav1 | uav2 |
| --- | ---: | ---: | ---: |
| uav3 | `0.475000` | `0.475000` | `0.050000` |
| uav4 | `0.016667` | `0.483333` | `0.500000` |
| uav5 | `0.483333` | `0.016667` | `0.500000` |

---

## 大步骤 1：增加配置结构，但暂不接入 MPC

这一步只建立角色、领导者和权重配置，不改变任何现有控制行为。

### 1.1 修改 `include/form_nav/formation_config.h`

在 `FormationConfig` 中增加：

```cpp
std::string formation_mode = "offset";
std::vector<int> leader_ids;
std::map<int, std::map<int, double>> weights;

bool isLeader(int id) const;
bool isFollower(int id) const;
bool hasWeights(int id) const;
```

保留现有：

```cpp
std::map<int, std::vector<double>> offsets;
std::vector<int> neighbor_ids;
```

此时 `neighbor_ids` 继续表示机间避碰拓扑，不用于计算重心编队。

### 1.2 修改 `src/formation_config.cpp`

增加以下参数解析：

```text
/formation_config/formation_mode
/formation_config/leaders
/formation_config/weights
```

增加基础校验：

- `formation_mode` 只能是 `offset` 或 `barycentric`
- 领导者 ID 不重复
- 每个领导者都存在于 `offsets`
- 每个 follower 的权重键存在
- 权重非负
- 每个 follower 的权重和接近 1

这一步只解析配置并保存成员，不修改 `Mpc`。

### 1.3 修改 `config/formations.yaml`

增加：

```yaml
formation_mode: barycentric
leaders: [0, 1, 2]

weights:
  uav3: {uav0: 0.475000, uav1: 0.475000, uav2: 0.050000}
  uav4: {uav0: 0.016667, uav1: 0.483333, uav2: 0.500000}
  uav5: {uav0: 0.483333, uav1: 0.016667, uav2: 0.500000}
```

保持现有 `offsets` 和 `agents` 不变。

### 1.4 增加离线几何检查脚本

新增脚本：

```text
scripts/check_barycentric_formation.py
```

脚本只做以下检查并输出结果：

```text
领导者是否共线
每个 follower 的重心权重
权重和
权重是否非负
权重重构位置与 offsets 的误差
```

代码修改点到此为止，不接入 MPC。

---

## 大步骤 2：在 MPC 内部增加重心目标计算

这一步只增加符号表达式和函数，不修改优化问题的代价结构。

### 2.1 修改 `include/form_nav/mpc.h`

增加成员：

```cpp
bool use_barycentric_ = false;
std::vector<int> leader_ids_;
std::map<int, double> bary_weights_;
```

增加接口：

```cpp
void setBarycentricFormation(
    const std::vector<int>& leader_ids,
    const std::map<int, double>& weights);
```

增加函数：

```cpp
MX barycentricTarget(
    int k_step,
    const std::map<int, MX>& neighbor_params) const;
```

### 2.2 修改 `src/mpc.cpp`

实现 `Mpc::barycentricTarget()`：

```text
只遍历 bary_weights_ 中的领导者
不遍历全部 neighbors
不使用多机邻居平均
不重新归一化缺失领导者后的剩余权重
```

如果任一领导者不在 `neighbor_params` 中：

```text
返回空 MX 表达式或通过有效性接口报告失败
```

不要用零向量作为找不到领导者时的兜底目标。

### 2.3 暂不替换 `formationError()`

当前 `formationError()` 和现有 `w_form` 行为保持不变。

这一步结束后，代码应仍然运行原来的邻居共识编队逻辑，只是多了一个尚未被调用的重心目标函数。

---

## 大步骤 3：增加 follower 的重心编队代价

这一步只修改代价函数，不改变领导者规划方式，也不删除 follower 的全局轨迹。

### 3.1 修改 `src/mpc.cpp` 中的 `formationError()`

增加重心模式分支：

```text
if use_barycentric_:
    p_i = X_k.position
    p_star = barycentricTarget(k_step, neighbor_params)
    e = p_i - p_star
    S_f = [e.x, e.y, e.z, 0]
else:
    保持原邻居共识逻辑
```

领导者不启用该分支。

### 3.2 修改编队权重

在 `src/mpc_node.cpp` 中：

```cpp
if (is_leader) {
    mpc_ptr->setFormationErrorWeight(
        DM::diag(DM{0.0, 0.0, 0.0, 0.0}));
} else {
    mpc_ptr->setFormationErrorWeight(
        DM::diag(DM{w_form, w_form, w_form, 0.0}));
}
```

这一步仍保留 follower 原有的路径参考和终端参考，只把编队项切换为重心式，用于隔离“编队目标计算是否正确”。

### 3.3 节点下发重心配置

在 `src/mpc_node.cpp` 初始化段读取 `FormationConfig` 后：

```text
if follower and hasWeights(my_id):
    mpc_ptr->setBarycentricFormation(
        formation.leader_ids,
        formation.weights.at(my_id));
```

领导者不调用该接口。

此阶段先保证 follower 能收到三个领导者的预测轨迹。

可以暂时把 `leader_ids` 合并进订阅列表，但不要删除 follower A*，也不要修改避碰逻辑。

---

## 大步骤 4：拆分“编队订阅”和“避碰邻居”

这一步只解决通信数据来源，不修改 MPC 求解和全局规划。

### 4.1 修改 `src/mpc_node.cpp`

增加两个集合：

```cpp
std::vector<int> formation_leader_ids;
std::vector<int> collision_neighbor_ids;
std::vector<int> required_neighbor_ids;
```

初始化时：

```text
formation_leader_ids = formation.isFollower(my_id)
                     ? formation.leader_ids
                     : empty

collision_neighbor_ids = formation.neighbor_ids

required_neighbor_ids = union(
    formation_leader_ids,
    collision_neighbor_ids)
```

邻居订阅循环继续使用 `required_neighbor_ids`。

### 4.2 修改 `include/form_nav/mpc.h`

增加：

```cpp
std::set<int> collision_neighbor_ids_;

void setCollisionNeighbors(const std::vector<int>& neighbor_ids);
```

### 4.3 修改 `src/mpc.cpp` 中的 `collisionAvoidance()`

遍历 `neighbor_params` 时增加过滤：

```cpp
if (collision_neighbor_ids_.count(j_id) == 0) {
    continue;
}
```

这样 follower 即使额外订阅了三个领导者，也不会因此改变原机间避碰拓扑。

这一步完成后，follower 同时拥有：

```text
三个领导人轨迹：用于未来重心编队
原有避碰邻居轨迹：用于 J_ca
```

---

## 大步骤 5：移除 follower 对独立全局路径的依赖

这一步改动较大，但只处理 follower 的参考来源。

### 5.1 修改 `include/form_nav/mpc.h`

增加角色成员：

```cpp
bool is_leader_ = false;

void setFormationRole(bool is_leader);
```

将 leader 和 follower 的参考准备拆开：

```cpp
bool prepareLeaderReference(...);
bool prepareFollowerReference(...);
```

公共的动力学、输入、避障、机间避碰、热启动和求解流程保持不变。

### 5.2 修改 `src/mpc.cpp`

当前 `Mpc::solve()` 无条件检查 `has_global_traj_`，改为：

```text
if leader:
    必须存在 global trajectory
    projectToPath()
    generateStepTargets()
    使用原路径跟踪代价

if follower:
    不要求 global trajectory
    不调用 projectToPath()
    不调用 generateStepTargets()
    不加入逐点路径参考代价
    使用 barycentricTarget() 生成编队代价
```

跟随者的制动目标使用：

```text
mission_goal + own_formation_offset
```

这时 follower 仍然需要任务目标，但不需要任务目标对应的 A* 路径。

### 5.3 修改 `src/mpc_node.cpp` 的全局轨迹等待条件

当前控制循环依赖：

```cpp
has_global_traj
```

改为：

```text
leader:
    必须收到 global trajectory

follower:
    必须收到全部领导者预测轨迹
    不要求 global trajectory
```

### 5.4 增加任务目标订阅

所有 MPC 节点订阅：

```text
/move_base_simple/goal
```

用途区分：

```text
领导者：
    原有 A* 节点继续使用

跟随者：
    仅保存 mission_goal
    计算 formation_goal = mission_goal + own_offset
    用于制动和到达判断
    不触发 A*，不生成路径
```

### 5.5 修改到达和悬停逻辑

`publishHoldPositionSetpoint()` 当前直接使用 `global_traj.back()`，增加角色分支：

```text
leader:
    使用 global_traj.back()

follower:
    使用 formation_goal
    或最近一次安全悬停位置
```

follower 到达判定使用：

```text
||p_i - formation_goal|| < arrival_distance
||v_i|| < arrival_velocity
三个领导者近期速度接近零
```

---

## 大步骤 6：只给领导者启动 A*

这一步只修改 launch，不改 MPC 算法。

### 6.1 修改 `launch/astar_grid.launch`

增加参数：

```xml
<arg name="enable_global_planner" default="true"/>
```

只给 `astar_node` 增加条件：

```xml
<node if="$(arg enable_global_planner)"
      name="$(arg uav_id)_astar_node"
      pkg="form_nav"
      type="astar_node">
```

`uavX_odom_to_world` 节点仍然无条件启动，否则 follower 会没有 `/odom_world`。

### 6.2 修改 `launch/formation_test.launch`

领导者：

```text
uav0、uav1、uav2
```

启动 A*：

```xml
<arg name="enable_global_planner" value="true"/>
```

跟随者：

```text
uav3、uav4、uav5
```

不启动 A*，但保留 `odom_to_world`：

```xml
<arg name="enable_global_planner" value="false"/>
```

### 6.3 修改固定场景透传

给 `formation_test.launch` 增加：

```xml
<arg name="scene_file"
     default="$(find form_nav)/config/scenes/fixed_scene.yaml"/>
```

并将其传给 `obs_generator.launch`。

这一步只解决“领导者规划、跟随者不规划”的启动结构。

---

## 大步骤 7：增加编队失效与退化处理

这一步单独修改节点状态机，不改变 MPC 的数学形式。

### 7.1 增加状态枚举

```cpp
enum class FormationState {
    WAIT_LEADERS,
    ACTIVE,
    FROZEN,
    DEGRADED,
    ARRIVED
};
```

### 7.2 增加领导者有效性检查

检查：

```text
三个领导者消息是否都存在
消息年龄是否超时
轨迹是否为 N+1 步
轨迹是否包含 NaN/Inf
三个轨迹能否对齐到当前求解时刻
```

任一检查失败时：

```text
不更新 p*
不根据剩余领导者重新归一化权重
短时使用最近一次有效领导者轨迹
超过冻结时间后进入 DEGRADED
```

### 7.3 增加三角形退化检查

使用领导者 xy 位置计算：

```text
signed_area
area_ratio = |signed_area| / nominal_area
edge_lengths
```

建议判据：

```text
area_ratio < formation/min_area_ratio
signed_area 与标称方向相反
任一边长 < formation/min_edge_length
```

任一条件成立时：

```text
短暂保持最近一次有效 p*
超过 formation/freeze_timeout 后进入 DEGRADED
恢复连续有效 formation/recover_hold_time 后回到 ACTIVE
```

### 7.4 增加 MPC 求解失败退化

只有实际调用 `Mpc::solve()` 才统计一次尝试。

等待领导者、等待里程计、已经到达等情况不计为求解失败。

连续失败达到：

```text
formation/max_consecutive_solve_failures
```

后进入 `DEGRADED`，停止更新编队目标并发布静止预测轨迹。

### 7.5 建议参数

```yaml
formation/min_area_ratio: 0.30
formation/min_edge_length: 0.80
formation/freeze_timeout: 0.20
formation/recover_hold_time: 0.50
formation/max_consecutive_solve_failures: 3
```

---

## 大步骤 8：增加 MPC 耗时、失败率和退化统计

这一步只增加计时、计数和诊断发布，不改变控制逻辑。

### 8.1 在 `src/mpc_node.cpp` 中计时

围绕实际求解调用：

```cpp
const auto solve_start = std::chrono::steady_clock::now();
const bool solve_ok =
    mpc_ptr->solve(s0, neighbors_vec, dyn_obs_vec, current_mpc_state);
const auto solve_end = std::chrono::steady_clock::now();

const double solve_ms =
    std::chrono::duration<double, std::milli>(
        solve_end - solve_start).count();
```

### 8.2 增加累计计数

```cpp
uint64_t solve_attempts_total = 0;
uint64_t solve_success_total = 0;
uint64_t solve_failure_total = 0;
int consecutive_solve_failures = 0;
```

### 8.3 增加滑动窗口

建议保留最近 10 秒数据：

```text
solve_ms_window
solve_ok_window
```

统计：

```text
均值
P50
P95
最大值
10 秒失败率
累计失败率
最大连续失败次数
有效求解可用率
```

### 8.4 增加 `Mpc` 失败原因

在 `include/form_nav/mpc.h` 中增加：

```cpp
enum class SolveFailureReason {
    NONE,
    NO_GLOBAL_TRAJECTORY,
    INVALID_STATE,
    OPTI_BUILD_FAILED,
    SOLVER_EXCEPTION,
    INVALID_FOLLOWER_REFERENCE
};

SolveFailureReason lastSolveFailureReason() const;
```

节点发布诊断时区分：

```text
求解失败
等待领导者
三角形退化
已到达
未准备好
```

### 8.5 新增诊断话题

新增：

```text
/uavX/mpc_diagnostics
```

推荐使用 `std_msgs/Float64MultiArray`，字段顺序固定为：

| 下标 | 字段 |
| ---: | --- |
| 0 | `stamp` |
| 1 | `solve_ms` |
| 2 | `solve_ok` |
| 3 | `control_cycle_ms` |
| 4 | `attempts_total` |
| 5 | `failures_total` |
| 6 | `failure_rate_total` |
| 7 | `failure_rate_10s` |
| 8 | `solve_p50_10s` |
| 9 | `solve_p95_10s` |
| 10 | `consecutive_failures` |
| 11 | `formation_state` |
| 12 | `leader_valid` |
| 13 | `triangle_area` |
| 14 | `triangle_area_ratio` |
| 15 | `triangle_sign_valid` |
| 16 | `formation_error` |
| 17 | `actual_barycentric_min` |
| 18 | `degraded_event_count` |
| 19 | `degraded_duration` |
| 20 | `no_solve_reason` |

指标定义：

```text
failure_rate_total = failures_total / attempts_total
failure_rate_10s   = 10 秒窗口失败次数 / 10 秒窗口尝试次数
solve_availability = solve_success_total / control_ticks
```

`solve_availability` 必须单独记录，避免出现“少调用求解器导致失败率看起来很低”的情况。

---

## 大步骤 9：增加编队可视化和离线指标脚本

这一步只增加观察手段，不修改控制逻辑。

### 9.1 可视化

建议发布：

```text
/formation/triangle
/uavX/formation_target
/uavX/formation_error
/uavX/formation_barycentric
/formation/area
/uavX/mpc_diagnostics
```

可使用现有消息类型：

```text
visualization_msgs/Marker
geometry_msgs/PoseStamped
std_msgs/Float64
std_msgs/Float64MultiArray
```

### 9.2 离线指标脚本

新增：

```text
scripts/barycentric_metrics.py
```

从 bag 或 CSV 计算：

```text
||p_i - p_i*|| 的均值、最大值、90% 分位
实际重心坐标最小值
领导者三角形面积
三角形方向是否翻转
MPC solve_ms 的均值、P50、P95、最大值
MPC 累计失败率
MPC 10 秒滑动失败率
有效求解可用率
最大连续失败次数
WAIT_LEADERS/FROZEN/DEGRADED 时间占比
退化事件次数和恢复时间
```

---

## 大步骤 10：切换到最终运行模式

这一步只删除调试期保留路径，不改已实现的重心编队公式。

### 10.1 确认 follower 不再依赖 A*

最终代码中：

```text
followers 不启动 astar_node
followers 不订阅自身 global_trajectory
followers 不调用 projectToPath()
followers 不调用 generateStepTargets()
followers 不加入逐点路径参考代价
```

### 10.2 保留的调试接口

可以保留参数：

```yaml
formation_mode: offset | barycentric
formation/debug_allow_follower_path: false
```

其中 `debug_allow_follower_path` 只用于排查问题，正式固定障碍场景必须保持 `false`。

### 10.3 固定障碍场景参数

最终运行直接使用：

```text
formation_test.launch
fixed_scene.yaml
三个领导者 A*
三个跟随者重心编队
静态和动态障碍同时存在
```

不再另设有障碍、无避障、纯悬停等分阶段仿真。

---

## 建议提交顺序

建议每个提交只完成以下一项：

1. `formation_config` 增加模式、领导者、权重和解析校验
2. `formations.yaml` 增加重心权重
3. `Mpc` 增加 `setBarycentricFormation()` 和 `barycentricTarget()`
4. `formationError()` 切换到重心误差，但暂时保留 follower A*
5. follower 增加领导者订阅，同时保持避碰拓扑不变
6. `Mpc::solve()` 支持 follower 无全局路径
7. `mpc_node` 增加角色分支、final goal 和到达逻辑
8. launch 只给领导者启动 A*
9. 增加编队状态机和退化处理
10. 增加 MPC 耗时、失败率和诊断话题
11. 增加可视化和离线指标脚本
12. 删除 follower 的调试用 A* 依赖

不要把步骤 3、5、6、8 合并在同一次提交中。这样出现仿真异常时，可以按提交点判断问题来自：

```text
权重计算
MPC 编队代价
领导者订阅
follower 参考模式
A* 启动范围
```

