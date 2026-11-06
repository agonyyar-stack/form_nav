# 理论仿真跑通：分步骤实现说明（单人 / 10 天）

> 目标：在**现有 form_nav 代码**上，把论文的 3 个新模块（重心编队、CBF 安全约束、指定时间编队恢复）加进去，
> 跑出一组"能放进论文"的仿真结果。**只求跑通、能出图、结论站得住**，不做工程化重构。
> 环境：Ubuntu 20.04 + ROS Noetic + PX4 SITL + Gazebo；仓库分支 `develop/paper-algorithm`。
> 用法：**每一步开一个新对话**，开场模板见 §0.2。

---

## 0 怎么用这份文档

### 0.1 四步走（每步一个新对话）

| 步骤 | 内容 | 预计 | 产出（能看见的东西） |
| --- | --- | --- | --- |
| **第 0 步** | 准备：建分支、修 2 个挡路的小 bug、加 2 行日志、跑基线 | 0.5–1 天 | 6 机能飞；日志能打印耗时与最小净空；有基线 bag |
| **第 1 步** | 重心编队：新队形 + 权重 + 领导者/跟随者两套代价 | 3 天 | 3 领导者在三角形上、3 跟随者跟着三角形走；编队误差曲线 |
| **第 2 步** | CBF 硬约束：静态障碍的 `n^T(v_{k+1}-v^o)+γh ≥ 0` + 松弛 | 3 天 | 最小净空明显抬高、100 次无碰撞的统计 |
| **第 3 步** | 指定时间恢复：`h_min` 事件触发 + `‖e‖≤ρ(t)` 时变约束 | 2 天 | 避障后编队误差在 T 内回到 ε_f 的曲线 |
| **收尾** | 三个实验的录包、出图、写进论文 | 1 天 | 3 张图 + 1 张参数表 |

**排期建议**（10 天，单人，留 1 天缓冲）：

```
D1      第0步 + 第1步的队形/权重脚本
D2-D4   第1步（D2 配置+几何，D3 MPC 改代价，D4 联调）
D5-D7   第2步（D5 参数+查询，D6 约束+松弛，D7 验证）
D8-D9   第3步
D10     收尾出图
```

### 0.2 每个新对话的开场模板

每一步单独开对话时，第一条消息直接贴下面这段（把 `<...>` 换成实际值）：

```
项目：~/catkin_ws/src/form_nav（分支 <branch>），ROS Noetic + PX4 + Gazebo，C++17。
请先读 docs/sim_steps.md 的第 <N> 步（只看这一步，不要做其他步骤的事），
以及 docs/context.md 了解现有实现。

当前状态：第 <N-1> 步已完成，工作树干净，已按该步"结束状态"验证通过。
本轮目标：完成第 <N> 步，验收标准是文档 §<N>.x 里那几条。
约束：只改第 <N> 步列出的文件；不要重构无关代码；每改完一处先 catkin_make 跑通再继续；
改动前先 git status 确认干净，改动后按文档的验证清单逐条过一遍。
```

**每个新对话的第一件事**：`git status` + `git log --oneline -3`，确认自己接的是上一步的结束状态。

### 0.3 每一步"做完了"的标准

每一步都必须同时满足三条，才允许开下一个对话：

1. `catkin_make` 无错误、无新增 warning（警告可接受，报错不行）；
2. 文档该步的**验证清单逐条通过**（都有可观察的现象，不是"我觉得行"）；
3. `git commit` 一次，信息里写清"第 N 步 + 改了什么 + 怎么验证的"。

---

## 1 现状与总体方案

### 1.1 已经有的（不用动，直接用）

| 能力 | 位置 | 说明 |
| --- | --- | --- |
| 8 维状态 / 4 维控制、ZOH 精确离散 | `src/mpc.cpp`（模型段） | 与理论 §1 完全一致，**一行都不用改** |
| 纯 A* + 去锯齿 | `src/a_star.cpp`、`src/astar_node.cpp` | 每个无人机独立规划到自己的槽位 |
| 静态避障 softplus 势垒 | `Mpc::kdtreeObstacleAvoidance` | 最近 3 点 + `(W/d_scale)·softplus(k(D̄−d_eff))` |
| 动态障碍 softplus 势垒 | `Mpc::dynamicObstacleAvoidance` | 点-折线最短距离 + 形状感知 |
| 机间避碰 softplus 势垒 | `Mpc::collisionAvoidance` | 邻居预测轨迹 → 代价 |
| 邻居预测轨迹广播与时间对齐 | `src/mpc_node.cpp` | 20 Hz，8×(N+1)，world 系，按 age 插值 |
| 逐点参考（梯形速度剖面）+ 终端 + 制动约束 | `Mpc::generateStepTargets`、`buildOpti` | 领导者那套代价已经齐了 |
| 编队误差项（**共识式**，默认关） | `Mpc::formationError` | 第 1 步要把它换成**重心式** |
| 6 机 Gazebo + 固定场景 + 动态障碍 | `launch/formation_test.launch` | 直接用 |

### 1.2 差的三个模块 → 三步

| 模块 | 现状 | 本方案怎么做（最小改动） |
| --- | --- | --- |
| 重心编队 | `formationError` 是"邻居共识"（`(p_j−M_j)−(p_i−M_i)`），且 `w_form=0`；队形是 2×3 网格，三机共线，不满足凸包 | 换队形（三角形 + 3 个内部点）→ 离线算 3 个权重 → `formationError` 改成 `p_i − Σ_j w_j p_j` → follower 打开 `w_form`、领导者保持 0 |
| CBF 硬约束 | 完全没有 | 在 `buildOpti` 里加一组**线性约束**：`n_k^T(α v_k + β u_k) + γ h_k + δ_k ≥ 0`，`n_k,h_k` 由上一轮解冻结成参数；`δ_k` 做松弛 |
| 指定时间恢复 | 完全没有 | `mpc_node` 里 60 行状态机 → 每周期生成 `ρ_k` → MPC 里加一条**二次约束** `‖p_k−p*_k‖² ≤ ρ_k²`（`ρ` 是参数，不触发就设 50） |

**关键简化**（都是可接受的近似，论文里如实写一句即可）：

- CBF 只做**静态障碍**（动态障碍继续用现有 softplus 软势垒）——省掉一半工作量，效果上仍是"硬约束 + 软引导"双层结构；
- CBF 的 `n_k,h_k` 在 `k≥1` 时用**上一轮解冻结**（名义化 CBF），`k=0` 用当前实测（严格成立）；
- 跟随者**保留自己的 A***（只把终端/进度项降权），不删节点——"跟随者不规划"在论文里写成本方法的理想形式，实现上作为"兜底参考"，答辩时解释得通；
- 恢复只作用在 follower 上，且**首次收敛后才武装**（避免起飞阶段误触发）。

### 1.3 明确不做的（防止又跑偏）

- ❌ 不重构代价框架（不建 `CostTerm` 类，直接在 `buildOpti` 里加项）
- ❌ 不做 A* 滚动重规划、不做感知窗口、不改全局层
- ❌ 不重构 launch（6 份 `<group>` 保持原样，只加参数）
- ❌ 不改任何 `.msg` 格式
- ❌ 不做 ESDF、不做 kino A*、不做 DWA
- ❌ 不写自动化评测框架（用 `rosbag` + 20 行 python 算指标就够）
- ❌ 不追求"任意队形/任意机数"，就固定 3 领导者 + 3 跟随者

### 1.4 改动点总览（一眼看清改哪几个文件）

| 文件 | 第 0 步 | 第 1 步 | 第 2 步 | 第 3 步 |
| --- | --- | --- | --- | --- |
| `config/formations.yaml` | | 换 offsets + 加 `leaders`/`weights` | | |
| `include/form_nav/formation_config.h` | | 加字段 | | |
| `src/formation_config.cpp` | | 解析 + 校验 | | |
| `include/form_nav/mpc.h` | | 权重成员 + setter | CBF 成员/参数 | `rho` 参数 |
| `src/mpc.cpp` | | 改 `formationError` | 加 CBF 约束 + 参数填充 | 加 `‖e‖²≤ρ²` |
| `src/mpc_node.cpp` | 失败分支发预测、加日志 | 角色/订阅/权重下发/降权 | 填 CBF 参数、回退 | 状态机 |
| `launch/mpc.launch` | | 参数 | 参数 | 参数 |
| `scripts/formation_weights.py`（新） | | 算权重 | | |

### 1.5 公共约定

- 所有新参数加前缀 `mpc/`（保持和现有风格一致），在 `mpc_node.cpp` 里用 `nh_private.param<T>()` 读，**读到就打一行 ROS_INFO**。
- 所有新代码块前面加注释 `// === 第N步：<名字> ===`，方便 diff 和回退。
- 每个周期都打印的关键量只有三个：`solve_ms`、`h_min`、`formation_err`（多了会淹掉日志）。

---

## 2 第 0 步：准备（0.5–1 天）

### 2.0 本步开始前的状态
当前分支可编译、6 机能飞（`roslaunch form_nav formation_test.launch`）、`w_form=0`。

### 2.1 建分支

```bash
cd ~/catkin_ws/src/form_nav
git checkout develop/paper-algorithm
git pull
git checkout -b sim/thesis        # 后面 4 步都在这个分支上
```

### 2.2 修 B1：求解失败/等邻居时**不发布**本机预测轨迹

**为什么必须修**：第 1 步开始，跟随者要拿领导者的预测轨迹算凸组合。如果某个领导者某一帧求解失败就不发消息，
跟随者 0.3 s 后判定超时 → 也停止求解 → 变成"一个失败、全队停摆"。这是目前**最容易踩的坑**，一定要先修。

**改哪**：`src/mpc_node.cpp` 里三处分支（搜 `WARN` / `hold` / `required neighbor`）：
- MPC 求解失败分支（`solve()` 返回 false 后发零速那段）
- 等待必需邻居的分支（`required_neighbor_ids` 未到齐）
- 邻居超时判定分支

**怎么改**：把这三处统一改成调用已有的 `publishStationaryPrediction()`（到达锁定/悬停分支已经在用这个函数）。
如果找不到合适的"上一轮预测"，就用**静止预测**（速度 0、位置=当前），保证消息不断流。

> 参考：现有代码里"到达锁定""制动预检""近障碍"三个分支已经这么做了，照抄即可。

**验证**：`rosnode kill /uav3/mpc_node`，然后 `rostopic hz /uav0/predicted_path_share` 仍是 ~20 Hz（别的机不受影响）。

### 2.3 加两行日志（后面两步的验收全靠它）

在 `src/mpc_node.cpp` 的定时器回调里，`mpc_ptr->solve(...)` 前后加：

```cpp
auto t0 = std::chrono::steady_clock::now();
bool ok = mpc_ptr->solve(s0, neighbors_vec, dyn_obs_vec, current_mpc_state);
double solve_ms = std::chrono::duration<double, std::milli>(
                      std::chrono::steady_clock::now() - t0).count();
```

然后在成功分支打印（**记得 20 Hz 别刷屏，用 1 Hz 节流**）：

```cpp
ROS_INFO_STREAM_THROTTLE(1.0, "[" << UAV_NAME << "] solve=" << solve_ms
    << "ms h_min=" << h_min_now << " e_form=" << formation_err_now);
```

`h_min` 的计算放在第 2 步（那里本来就要算），本步先打印 `-1` 占位即可。

**验证**：`roslaunch form_nav single_test.launch` 后 `rqt_console` 里能看到 `solve=xx ms`。记一下单机正常值（一般 10–30 ms），**这是第 2 步判断 CBF 开销的基准**。

### 2.4 跑基线、记数字

```bash
# 终端 1
roslaunch form_nav formation_test.launch
# 终端 2：发目标
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'world', stamp: now}
   pose: {position: {x: 15.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}"
# 终端 3：录包
rosbag record -O baseline_$(date +%m%d_%H%M) \
  /uav{0..5}/mavros/local_position/odom_world \
  /uav{0..5}/predicted_path_share \
  /uav{0..5}/global_trajectory \
  /map_generator/global_cloud /tf /tf_static
```

**记下三个数**（第 1、2 步都要拿来对比）：
- 到达时间（发目标 → 每机进入终点 0.5 m）
- 编队误差（用 `p_i − (p_i的offset + 质心)` 粗算，或者肉眼看 RViz 间距）
- 最小净空（后面第 2 步会算，先肉眼看最近障碍距离）

### 2.5 本步结束状态（下一步的前提）

- [ ] 分支 `sim/thesis` 存在，2 个 commit（修 B1、加日志）
- [ ] 6 机可飞，`predicted_path_share` 在任何情况下都不断流
- [ ] 日志能打印 `solve=xx ms`
- [ ] 有基线 bag 一个

---

## 3 第 1 步：重心编队（约 3 天）

### 3.0 本步开始前的状态
第 0 步的 4 条全部满足；`w_form=0`（编队项关闭）；队形还是 2×3 网格。

### 3.1 目标与验收

**目标**：3 架领导者（uav1/3/5）构成三角形、各自飞向自己的槽位；3 架跟随者（uav0/2/4）用
`p*_k = Σ_j w_j · p_{Lj,k}` 作为期望位置（`p_{Lj,k}` 来自领导者的预测轨迹第 k 步），跟随者代价里
`J_form` 为主、终端/进度项降权。

**验收**（缺一不可）：
1. 飞行全程，把每架 follower 的当前位置投影到 3 个领导者当前构成的三角形里，**重心坐标三个分量恒为正**（>0）；
2. 巡航段（速度 >1 m/s 且无避障）编队误差 `‖p_i − p*_i‖ < 0.3 m`；
3. RViz 里把 3 个领导的预测轨迹连成三角形，follower 始终在三角形内部；
4. 改 `nominal` 整体乘 1.2（放大），重跑一次，队形等比放大且 follower 仍在内 → 验证仿射不变性。

### 3.2 队形与权重（**先用这组，已经算好**）

现有 2×3 网格里 uav0/1/2 三点共线，重心坐标无解，**必须换队形**。用下面这组（保持 2 m × 4 m 的包络）：

| 机 | 角色 | 标称位置 (x, y) | 说明 |
| --- | --- | --- | --- |
| uav1 | **领导者** | (−1.2, −2.0) | 三角形顶点 |
| uav3 | **领导者** | ( 1.2, 0.0) | 三角形顶点 |
| uav5 | **领导者** | (−1.2, 2.0) | 三角形顶点 |
| uav0 | 跟随者 | (−0.4, −1.0) | 三角形内部 |
| uav2 | 跟随者 | (−0.4, 1.0) | 三角形内部 |
| uav4 | 跟随者 | ( 0.0, 0.0) | 三角形内部（重心附近） |

对应重心权重（按领导者顺序 **[uav1, uav3, uav5]**，和为 1，全正）：

| follower | w(uav1) | w(uav3) | w(uav5) |
| --- | --- | --- | --- |
| uav0 | 7/12 ≈ 0.583333 | 4/12 ≈ 0.333333 | 1/12 ≈ 0.083333 |
| uav2 | 1/12 ≈ 0.083333 | 4/12 ≈ 0.333333 | 7/12 ≈ 0.583333 |
| uav4 | 3/12 = 0.250000 | 6/12 = 0.500000 | 3/12 = 0.250000 |

**校验**（自己算一遍，几秒钟）：uav4 = 0.25·(−1.2,−2) + 0.5·(1.2,0) + 0.25·(−1.2,2) = (−0.3−0.3+0.6, −0.5+0+0.5) = (0,0) ✓
三角形面积 = 0.5 × 4 × 2.4 = 4.8 m²（非退化）。

> 如果要换自己想要的队形，用 §3.3 的脚本重新算，**不要手推**。

### 3.3 写权重脚本（20 分钟，纯 python，不依赖 ROS）

新建 `scripts/formation_weights.py`：

```python
#!/usr/bin/env python3
"""由标称队形解重心坐标：p_F = sum_j m_j p_Lj,  sum_j m_j = 1."""
import numpy as np, yaml, sys

nominal = {                      # 改成你的标称位置即可
    'uav0': (-0.4, -1.0), 'uav1': (-1.2, -2.0), 'uav2': (-0.4, 1.0),
    'uav3': ( 1.2,  0.0), 'uav4': ( 0.0,  0.0), 'uav5': (-1.2, 2.0),
}
leaders   = ['uav1', 'uav3', 'uav5']
followers = ['uav0', 'uav2', 'uav4']

P = np.array([nominal[l] for l in leaders])            # 3x2
A = np.vstack([np.column_stack([P, np.ones(3)]).T])    # 3x3: [x;y;1] -> m
out = {}
for f in followers:
    b = np.array([*nominal[f], 1.0])
    m = np.linalg.solve(A, b)
    assert np.all(m > 1e-6), f"{f} 不在三角形内部: {m}"
    assert abs(m.sum() - 1) < 1e-9
    out[f] = {l: float(round(v, 6)) for l, v in zip(leaders, m)}
    print(f, dict(zip(leaders, np.round(m, 6))))

# 三角形面积（做退化检查）
area = 0.5 * abs(np.cross(P[1]-P[0], P[2]-P[0]))
print("area =", area); assert area > 0.25
print(yaml.dump({'weights': out}, allow_unicode=True))
```

跑一下，把输出的 `weights` 贴进 `formations.yaml`。**脚本留着**，论文里"队形缩放/旋转实验"要用它重算。

### 3.4 改配置（`config/formations.yaml`）

```yaml
formation_mode: barycentric        # 新增；offset 时走老逻辑

leaders: [1, 3, 5]                 # 新增：领导者 id

offsets:                           # 改成上面的标称位置（终点分配也用它）
  uav0: [-0.4, -1.0, 0.0, 0.0]
  uav1: [-1.2, -2.0, 0.0, 0.0]
  uav2: [-0.4,  1.0, 0.0, 0.0]
  uav3: [ 1.2,  0.0, 0.0, 0.0]
  uav4: [ 0.0,  0.0, 0.0, 0.0]
  uav5: [-1.2,  2.0, 0.0, 0.0]

weights:                           # 新增：由脚本生成，勿手改
  uav0: {uav1: 0.583333, uav3: 0.333333, uav5: 0.083333}
  uav2: {uav1: 0.083333, uav3: 0.333333, uav5: 0.583333}
  uav4: {uav1: 0.250000, uav3: 0.500000, uav5: 0.250000}

agents:                            # followers 必须能收到全部 3 个 leader
  uav0: {neighbors: [1, 2, 3, 4, 5]}
  uav1: {neighbors: [0, 2, 3, 4, 5]}
  uav2: {neighbors: [0, 1, 3, 4, 5]}
  uav3: {neighbors: [0, 1, 2, 4, 5]}
  uav4: {neighbors: [0, 1, 2, 3, 5]}
  uav5: {neighbors: [0, 1, 2, 3, 4]}
```

> `neighbors` 全连是最省事的做法（6 机全连只有 30 条边，J_ca 计算量可接受）。
> 如果飞行中发现机间避碰项太强导致抖动，把 `w_ca` 从 5 降到 2。

### 3.5 改结构体（`include/form_nav/formation_config.h`）

```cpp
struct FormationConfig {
    std::map<int, std::vector<double>> offsets;
    std::vector<int> neighbor_ids;

    // === 第1步：重心编队 ===
    std::string formation_mode = "offset";           // offset | barycentric
    std::vector<int> leader_ids;
    std::map<int, std::map<int, double>> weights;    // follower → {leader → w}

    bool isLeader(int id) const;
    bool hasWeights(int id) const;
    // === end ===

    static FormationConfig load(const ros::NodeHandle& nh,
                                const std::string& config_namespace, int my_id);
};
```

### 3.6 改解析（`src/formation_config.cpp`）

在 `FormationConfig::load` 末尾加（现有代码已经有 `parseAgentId`，直接复用）：

```cpp
// === 第1步：重心编队 ===
nh.param<std::string>(config_namespace + "/formation_mode",
                      cfg.formation_mode, "offset");

if (cfg.formation_mode == "barycentric") {
    XmlRpc::XmlRpcValue lv;
    if (!nh.getParam(config_namespace + "/leaders", lv) || lv.getType() != XmlRpc::XmlRpcValue::TypeArray)
        throw std::runtime_error("barycentric 模式必须有 leaders 列表");
    for (int i = 0; i < lv.size(); ++i)
        cfg.leader_ids.push_back(static_cast<int>(lv[i]));

    XmlRpc::XmlRpcValue wv;
    if (!nh.getParam(config_namespace + "/weights", wv))
        throw std::runtime_error("barycentric 模式必须有 weights");

    for (auto it = wv.begin(); it != wv.end(); ++it) {
        const int fid = parseAgentId(it->first);
        if (it->second.getType() != XmlRpc::XmlRpcValue::TypeStruct)
            throw std::runtime_error("weights 项必须是 struct");
        std::map<int, double> m;
        double sum = 0.0;
        for (auto jt = it->second.begin(); jt != it->second.end(); ++jt) {
            const int lid = parseAgentId(jt->first);
            const double w = static_cast<double>(jt->second);
            if (w < 0.0) throw std::runtime_error("重心权重必须非负");
            m[lid] = w; sum += w;
        }
        if (std::abs(sum - 1.0) > 1e-4) throw std::runtime_error("重心权重和不为 1");
        cfg.weights[fid] = m;
    }
}
// === end ===
```

**编译**：`catkin_make`。**只解析、先不接算法**，跑一次 `roslaunch` 看有没有抛异常。

> 注意：`/formation_config` 是父 launch 用 `rosparam` 一次性载入的，改 YAML 后要**重启 launch**。

### 3.7 改 MPC：把"邻居共识"换成"重心凸组合"

这是本步**唯一需要动算法**的地方，只改一个函数。

**先加成员**（`include/form_nav/mpc.h`，跟 `formation_offsets_` 放一起）：

```cpp
// === 第1步：重心编队 ===
std::map<int, double> bary_weights_;   // my_id 对各个 leader 的权重
bool use_barycentric_ = false;
public:
    void setBarycentricWeights(const std::map<int, double>& w) {
        bary_weights_ = w; use_barycentric_ = !w.empty();
    }
// === end ===
```

**再改 `Mpc::formationError`**（`src/mpc.cpp`，现有函数体里那段 for 循环整体替换）：

```cpp
ErrorTerms ret;

if (use_barycentric_) {
    // === 第1步：p* = Σ w_j p_j，误差 e = p_i − p* ===
    MX p_i = current_states(Slice(0, 3));
    MX p_star = MX::zeros(3, 1);
    double w_sum = 0.0;
    for (auto const& [j_id, j_param] : neighbor_params) {
        auto it = bary_weights_.find(j_id);
        if (it == bary_weights_.end()) continue;
        p_star += it->second * j_param(Slice(0, 3), k_step);
        w_sum  += it->second;
    }
    if (w_sum < 0.5) {          // leader 缺消息：退化为"无编队项"
        ret.S1 = MX::zeros(4, 1);
        ret.S1_dot = MX::zeros(4, 1); ret.S2 = ret.S1_dot;
        ret.sum_term = ret.S1_dot;
        return ret;
    }
    MX e = p_i - p_star / w_sum;      // 归一化，缺某个 leader 也不会把目标拉偏
    ret.S1 = MX::zeros(4, 1);
    ret.S1(Slice(0, 3)) = e;          // 第 4 维(yaw)留 0
} else {
    // …原代码（共识式）保持不动…
}
```

**权重矩阵也要改**（`src/mpc_node.cpp` 里 setFormationErrorWeight 那行）：
原来是 `DM::diag(DM{w_form, w_form, w_form/2.0, 0.0})`（z 通道减半），
改成 `DM::diag(DM{w_form, w_form, w_form, 0.0})` —— 重心误差三个方向要**等权**，否则 z 方向收敛慢。

> 为什么这样改最小：`buildOpti` 里 `cost += S_f.T() * Q_f_ * S_f` 那几行**完全不用动**。

### 3.8 改节点：角色、订阅、代价差异

`src/mpc_node.cpp` 的初始化段（现在是 `FormationConfig::load(...)` 那一段）：

```cpp
// === 第1步：角色与权重下发 ===
const bool is_leader = formation.isLeader(my_id);
if (!is_leader && formation.hasWeights(my_id)) {
    mpc_ptr->setBarycentricWeights(formation.weights.at(my_id));
    ROS_INFO("[%s] role=follower, weights=%zu", UAV_NAME.c_str(),
             formation.weights.at(my_id).size());
} else {
    ROS_INFO("[%s] role=leader", UAV_NAME.c_str());
}

// 跟随者：终端/进度项降权（保留 A* 做兜底参考，但让编队项主导）
double target_scale = is_leader ? 1.0
                    : nh_private.param<double>("mpc/follower_target_scale", 0.2);
w_subgoal     *= target_scale;
w_subgoal_int *= target_scale;

// 编队项：leader 关，follower 开
double w_form = is_leader ? 0.0
              : nh_private.param<double>("mpc/w_form", 400.0);
// === end ===
```

**订阅**：跟随者必须拿到 3 个领导者的预测轨迹。因为 §3.4 已经把 `neighbors` 改成全连，
现有的订阅循环（`for (int nid : neighbor_ids) neighbor_subs.push_back(...)`）**自动就包含了领导者**，不用改代码。

**`launch/mpc.launch` 加参数**：

```xml
<param name="mpc/w_form" value="400.0"/>
<param name="mpc/follower_target_scale" value="0.2"/>
```

> `w_form` 从 400 起调：**太大**（>2000）follower 会被编队项拽着走、绕障变差；**太小**（<100）跟不住队形。
> 先给 400，跑一次看误差曲线再微调。

### 3.9 编译、运行、逐项验证

```bash
catkin_make && source devel/setup.bash
roslaunch form_nav formation_test.launch      # 6 机
# 另开终端发目标（同 §2.4）
```

验证清单：

| # | 检查什么 | 怎么看 | 通过标准 |
| --- | --- | --- | --- |
| 1 | 有没有抛异常（配置解析） | 启动日志 | 无 `FATAL`，打印出 `role=follower, weights=3` |
| 2 | follower 是否在三角形内 | RViz：把 3 个领导的 `predicted_path_share` 用 Marker 连成三角形（或用 `scripts/plot_formation.py` 后处理） | 全程在内，不穿出 |
| 3 | 编队误差 | 日志 `e_form`（本步顺手算：`||p_i − Σ w_j p_j(0)||`，在节点里 3 行） | 巡航段 < 0.3 m，稳态 < 0.15 m |
| 4 | 和基线的差异 | 到达时间 | 与第 0 步基线相差 < 20%（变慢是正常的，别离谱） |
| 5 | 缩放不变性 | 把 `offsets`/标称整体乘 1.2、脚本重算权重、重跑 | 队形等比放大，follower 仍在内部 |

### 3.10 常见坑

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 启动就报"权重和不为 1" | YAML 里权重手写有舍入误差 | 用脚本输出直接粘贴（保留 6 位） |
| follower 一直往一个角飞 | `offsets` 改了但 A* 的目标没同步（用的还是旧 offset），或 `neighbors` 没包含 leader | 检查 YAML 的 `offsets` 与 `weights` 是否同一套标称；检查日志里订阅的 neighbor 数 |
| 编队项开着但没效果 | `w_form` 被 leader 分支覆盖成 0；或 `formationError` 走了 `else` 分支 | 日志打印 `use_barycentric`；确认 follower 的 `w_form>0` |
| 起飞阶段乱飞 | 初始位置与标称位置差太多，编队项和避障项打架 | 正常，10 s 内会收敛；临时把 `w_form` 乘 `min(1, t/10)` 渐入 |
| 两个 follower 撞一起 | 全连后 `w_ca` 相对变小 | `w_ca` 提到 8–10，或把 `d_act_ca` 提到 0.8 |

### 3.11 本步结束状态（下一步的前提）

- [ ] `formations.yaml` 有新队形 + `leaders` + `weights`；`scripts/formation_weights.py` 可复算
- [ ] follower 用重心期望，`w_form>0`、终端/进度项降权；leader 保持原代价
- [ ] 验证清单 5 条全过，有 1 个 bag + 1 条编队误差曲线
- [ ] `git commit`："第1步：重心编队（新队形+权重+两套代价）"

---

## 4 第 2 步：CBF 安全约束（约 3 天）

### 4.0 本步开始前的状态
第 1 步验证全过；follower 用重心期望、`w_form=400`；静态避障只有 softplus 软势垒。

### 4.1 目标与验收

**目标**：给 MPC 加一条**硬约束**：沿障碍法向的接近速度不超过 `γh`，即
`n_k^T(v_{k+1} − v^o_k) + γ h_k ≥ 0`，其中 `v_{k+1}=αv_k+βu_k` 补偿执行器滞后，
`h_k = ‖p_k−o_k‖ − r_drone − d_safe`，`n_k` 为法向单位向量。配松弛变量 `δ_k` 保证可行。

**本步的最小可行范围**（先做这个，别贪）：
- 只对**静态障碍**（每预测步取最近 1 个点）施加 CBF；动态障碍继续用现有 softplus；
- `k=0` 用**当前实测位置**算 `n_0,h_0`（常数，严格成立）；
- `k≥1` 用**上一轮解的预测位置**冻结 `n_k,h_k`（名义化 CBF，论文里写明）；
- 每条约束配 `δ_k ≥ 0`，`δ` 加大权重线性罚。

**验收**：
1. 固定场景连飞 10 次，**无碰撞**（机身与障碍最近距离 > 0）；
2. 最小净空（全程 `min(‖p−o‖−0.3)`）的 **5% 分位**比“关 CBF”那组高 ≥ 30%；
3. 平均单周期 `solve` 耗时增加 ≤ 15 ms；
4. 求解成功率 ≥ 99%（日志统计 `solve=false` 次数）；
5. `cbf/enable=false` 时轨迹与第 1 步完全一致（误差 < 1 cm，说明开关干净）。

### 4.2 为什么这么设计（答辩会被问）

| 问题 | 回答 |
| --- | --- |
| 理论说“对速度指令线性”，真的线性吗 | `n,h` 依赖决策变量 `p_k`。所以 `k=0` 用实测（不依赖决策变量，天然线性、天然严格）；`k≥1` 冻结成参数（序列线性化，等价于 SQP 走一步）。论文写“名义化 CBF / 序列线性化” |
| 为什么要松弛 | 密集障碍或窄通道里 CBF 与制动约束可能冲突，硬约束会让问题不可行 → 求解失败 → 零速悬停，比“稍微越界”更糟 |
| 与 softplus 重复吗 | 不重复而是分工：softplus 在 `d_eff > D_act` 的外层做**提前引导**，CBF 在 `h` 附近做**不许越过**。开 CBF 后把 `w_oa` 从 5 降到 2 |
| `γ=0.65` 是什么 | `α(h)=γh` 的系数，单位 1/s；等价于碰撞时间不小于 `1/γ≈1.54 s` |

### 4.3 先做一个小重构：抽出 `barycentricTarget`

把第 1 步 `formationError` 里算 `p*` 的那段抽成单独函数（10 分钟），第 2、3 步都要用它：

```cpp
// mpc.h
MX barycentricTarget(int k_step, const std::map<int, MX>& neighbor_params) const;

// mpc.cpp
MX Mpc::barycentricTarget(int k_step,
                          const std::map<int, MX>& neighbor_params) const {
    MX p_star = MX::zeros(3, 1);
    double w_sum = 0.0;
    for (auto const& [j_id, j_param] : neighbor_params) {
        auto it = bary_weights_.find(j_id);
        if (it == bary_weights_.end()) continue;
        p_star += it->second * j_param(Slice(0, 3), k_step);
        w_sum  += it->second;
    }
    if (w_sum < 0.5) return MX::zeros(3, 1);   // 缺 leader：调用方自己判断
    return p_star / w_sum;
}
```

然后 `formationError` 里改成 `MX e = p_i - barycentricTarget(k_step, neighbor_params);`。

### 4.4 加成员（`include/form_nav/mpc.h`）

```cpp
// === 第2步：CBF 安全约束 ===
bool   cbf_enable_        = false;
double cbf_gamma_         = 0.65;   // 1/s
double cbf_safe_dist_     = 0.10;   // m
double cbf_slack_weight_  = 1e4;
double cbf_big_h_         = 10.0;   // 无障碍时给的大净空
int    cbf_fail_count_    = 0;

MX cbf_norm_param_;                 // (3, N+1)
MX cbf_h_param_;                    // (1, N+1)
MX cbf_slack_var_;                  // (1, N+1) 松弛变量
bool built_cbf_enable_ = false;     // 参与构建签名
std::vector<double> predicted_clearance_;   // (N+1)

public:
    void setCbfEnabled(bool on)       { cbf_enable_ = on; }
    void setCbfGamma(double g)        { cbf_gamma_ = g; }
    void setCbfSafeDistance(double d) { cbf_safe_dist_ = d; }
    void setCbfSlackWeight(double w)  { cbf_slack_weight_ = w; }
    bool isCbfEnabled() const         { return cbf_enable_; }
    const std::vector<double>& getPredictedClearance() const {
        return predicted_clearance_; }
// === end ===
```

### 4.5 在 `buildOpti` 里加约束

位置：现有“高度硬约束”那段之后、“求解器设置”之前。**注意此时 `cost` 还没有 minimize**。

```cpp
// === 第2步：CBF 约束 ===
if (cbf_enable_) {
    cbf_norm_param_ = opti_->parameter(3, N_ + 1);
    cbf_h_param_    = opti_->parameter(1, N_ + 1);
    cbf_slack_var_  = opti_->variable(1, N_ + 1);

    // v_{k+1} = A_vv * v_k + B_vu * u_k（A_/B_ 就是现有的 ZOH 系数矩阵）
    MX A_vv = A_(Slice(4, 7), Slice(4, 7));
    MX B_vu = B_(Slice(4, 7), Slice(0, 3));

    for (int k = 0; k < N_; ++k) {          // k=N 没有 u_N，故只到 N-1
        MX n      = cbf_norm_param_(Slice(0, 3), k);   // 3x1
        MX h      = cbf_h_param_(k);
        MX v_next = mtimes(A_vv, X(Slice(4, 7), k))
                  + mtimes(B_vu, U(Slice(0, 3), k));
        opti_->subject_to( mtimes(n.T(), v_next) + cbf_gamma_ * h
                           + cbf_slack_var_(k) >= 0 );
        opti_->subject_to( cbf_slack_var_(k) >= 0 );
    }
    cost += cbf_slack_weight_ * sum1(cbf_slack_var_);
}
// === end ===
```

**签名里加一行**（否则开关不触发重建）：

```cpp
if (opti_ && ... && built_cbf_enable_ == cbf_enable_ && ...) return true;  // 复用条件
...
built_cbf_enable_ = cbf_enable_;                                         // 末尾记录
```

> **坑**：忘了加签名，把 `cbf/enable` 从 false 改 true 后不生效（opti 被复用），会排查很久。

### 4.6 在 `solve` 里填参数

找到 `solve()` 里查障碍点的地方（调 `queryObstacleValues` 那段），在**每次查询之后**、
`opti_->set_value(obstacle_param_, ...)` 附近加：

```cpp
// === 第2步：填充 CBF 参数 ===
if (cbf_enable_) {
    const int M = nearest_obs_num_;
    std::vector<double> nvals(3 * (N_ + 1), 0.0);
    std::vector<double> hvals(N_ + 1, cbf_big_h_);
    predicted_clearance_.assign(N_ + 1, cbf_big_h_);

    for (int k = 0; k <= N_; ++k) {
        // 位置：k=0 用实测，k>=1 用上一轮解
        Eigen::Vector3d p = (k == 0 || !has_last_solution_)
                          ? current_states.head<3>()
                          : last_pred_x_.col(k).head<3>();

        // 每步第 0 个最近点（列优先布局：vals[k*3*M + 3*m + c]）
        Eigen::Vector3d o(obstacle_vals[k * 3 * M + 0],
                          obstacle_vals[k * 3 * M + 1],
                          obstacle_vals[k * 3 * M + 2]);

        const double dist = (p - o).norm();
        const double h    = dist - drone_radius_ - cbf_safe_dist_;
        hvals[k] = h;
        predicted_clearance_[k] = h;

        if (dist > 1e-6) {
            nvals[3 * k + 0] = (p - o).x() / dist;
            nvals[3 * k + 1] = (p - o).y() / dist;
            nvals[3 * k + 2] = (p - o).z() / dist;
        } else {
            nvals[3 * k + 2] = 1.0;   // 重合兜底：朝上
        }
    }

    DM Nmat = DM::zeros(3, N_ + 1);
    for (int k = 0; k <= N_; ++k)
        for (int c = 0; c < 3; ++c) Nmat(c, k) = nvals[3 * k + c];
    opti_->set_value(cbf_norm_param_, Nmat);
    opti_->set_value(cbf_h_param_, DM(hvals).T());
}
// === end ===
```

> `obstacle_vals` 就是 `queryObstacleValues(...)` 的返回值，函数已经存在，直接用它的返回值即可，不用再查一次 KD-tree。

### 4.7 松弛不够时：失败回退

在节点调用 `solve` 的地方（第 0 步加的计时代码那里）：

```cpp
// === 第2步：CBF 求解失败自动回退 ===
bool ok = mpc_ptr->solve(...);
if (!ok && cbf_on) {
    mpc_ptr->setCbfEnabled(false);          // 关掉后签名变化 → 下一句重建
    ok = mpc_ptr->solve(...);
    if (ok) {
        ++cbf_fail_count_;
        ROS_WARN("[%s] CBF failed, retried without (n=%d)", UAV_NAME.c_str(), cbf_fail_count_);
        if (cbf_fail_count_ >= 3) { cbf_on = false; ROS_ERROR("CBF disabled this flight"); }
    }
} else if (ok) {
    cbf_fail_count_ = 0;
    if (cbf_on && !mpc_ptr->isCbfEnabled()) mpc_ptr->setCbfEnabled(true);
}
// === end ===
```

### 4.8 参数（`launch/mpc.launch`）

```xml
<param name="cbf/enable"       value="true"/>
<param name="cbf/gamma"        value="0.65"/>
<param name="cbf/safe_dist"    value="0.10"/>
<param name="cbf/slack_weight" value="10000.0"/>
<param name="mpc/w_oa"         value="2.0"/>   <!-- 开 CBF 后软势垒降权 -->
```

节点里读取并打日志：

```cpp
cbf_on = nh_private.param<bool>("cbf/enable", false);
mpc_ptr->setCbfEnabled(cbf_on);
mpc_ptr->setCbfGamma(nh_private.param<double>("cbf/gamma", 0.65));
mpc_ptr->setCbfSafeDistance(nh_private.param<double>("cbf/safe_dist", 0.10));
mpc_ptr->setCbfSlackWeight(nh_private.param<double>("cbf/slack_weight", 1e4));
ROS_INFO("[%s] cbf=%d gamma=%.2f", UAV_NAME.c_str(), (int)cbf_on, ...);
```

### 4.9 验证与指标

**先单机**（`single_test.launch`）：确认能飞、看 `solve` 增加多少毫秒。
**再 6 机**：`formation_test.launch`，跑 10 次攒数据。

**最小净空脚本**（新建 `scripts/min_clearance.py`，思路 20 行）：

1. 读 `/uavX/mavros/local_position/odom_world` → 轨迹点序列；
2. 读 `/map_generator/global_cloud` 第一帧 → 障碍点；
3. 每个轨迹点找最近障碍点，`clearance = dist − 0.3`；
4. 输出 `min / 5% 分位 / 均值`。

同一场景、同一目标点，`cbf=false` 与 `true` 各 10 次，对比 5% 分位。

### 4.10 常见坑

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 改了 `cbf/enable` 没反应 | 构建签名没加 `cbf_enable_` | 见 §4.5 的提醒 |
| 一开 CBF 就求解失败 | 约束太紧 | `safe_dist` 降到 0.05、`γ` 降到 0.4、`slack_weight` 先给 1e3 |
| 轨迹来回抖 | `n_k` 每周期跳变（最近点在两个障碍间切换） | 加迟滞：新点距旧点 < 0.15 m 时沿用旧点 |
| 求解慢到 60 ms+ | 20 条约束 + 20 个松弛变量 | ① 隔一步施加（`for (int k=0;k<N_;k+=2)`）；② `ipopt_max_iter` 降到 60；③ 只对 `k=0..N/2` 施加 |
| 靠近障碍直接悬停 | `δ` 很大 + 制动约束也紧了 | 属于“降级”，确认没撞即可 |

### 4.11 本步结束状态

- [ ] `buildOpti` 有 CBF 约束块；`solve` 填 `n/h`；失败回退可用
- [ ] 6 机 10 次无碰撞；最小净空 5% 分位提升 ≥ 30%；耗时 +≤15 ms
- [ ] 有 `cbf=true/false` 两组 bag 与对比图
- [ ] `git commit`：“第2步：静态障碍 CBF 硬约束 + 松弛”
---

## 5 第 3 步：指定时间编队恢复（约 2 天）

### 5.0 本步开始前的状态
第 2 步验证全过；`getPredictedClearance()` 已经能从 MPC 读出来。

### 5.1 目标与验收

**目标**：避障结束后，编队误差 `‖e(t)‖` 在 `T` 秒内收缩到 `ε_f` 邻域。
做法：给 follower 的 MPC 加一条**时变二次约束** `‖p_k − p*_k‖² ≤ ρ_k²`，
其中 `ρ_k = ε_f + (ρ_0−ε_f)·φ(s)`，`φ(s)=1−3s²+2s³`，`s=(t−t_k)/T`，`t_k` 为触发时刻。

**验收**：
1. 触发后 `‖e‖` 单调下降，`T` 秒内 ≤ `ε_f`（10 次里至少 9 次达标）；
2. 单次飞行**误触发 ≤ 1 次**；
3. 恢复中遇到新障碍 → 立即暂停（`ρ` 不再收紧），不撞；
4. 与“不开恢复”对比：过障后收敛时间缩短 ≥ 2 倍。

### 5.2 三个关键决定

1. **`h_min` 直接取第 2 步的**：`min(getPredictedClearance())`，不再单独查 KD-tree。这样“避障模式判据”与 CBF 完全同义（论文里 `h` 只有一个定义）。
2. **状态机只有两态**：`AVOID`（`h_min < h_enter`）与 `NORMAL`。`AVOID` 退出后若满足“净空已恢复 `T_clear` 且驻留 `T_dwell` 且 `‖e‖>e_th`”，才锁存 `t_k` 开始恢复窗口；窗口靠时间自然结束。
3. **不触发时不重建 Opti**：把 `ρ_k` 填成 50 m（约束存在但等效无效）。若按“有/无约束”切换，每次切换都要重建 Opti，20 Hz 撑不住。

### 5.3 MPC 侧改动

**参数与约束**（`buildOpti` 里，只对 follower 建）：

```cpp
// === 第3步：编队误差时变约束 ===
if (use_barycentric_) {                 // 只有 follower 有 p*
    rho_param_ = opti_->parameter(1, N_ + 1);
    for (int k = 0; k <= N_; ++k) {
        MX e = X(Slice(0, 3), k) - barycentricTarget(k, neighbor_params_);
        opti_->subject_to( sumsqr(e) - rho_param_(k) * rho_param_(k) <= 0 );
    }
}
// === end ===
```

`rho_param_` 要参与构建签名（加一个 `built_rho_enable_`），否则从 leader 切 follower 时不会重建。

**成员与 setter**：

```cpp
MX rho_param_;
std::vector<double> rho_buffer_;        // 长度 N+1
public:
    void setFormationRho(const std::vector<double>& rho) { rho_buffer_ = rho; }
```

```cpp
// solve() 里，参数都设完之后：
if (!rho_param_.is_empty()) {
    std::vector<double> v = rho_buffer_.empty()
        ? std::vector<double>(N_ + 1, 50.0) : rho_buffer_;
    opti_->set_value(rho_param_, DM(v).T());
}
```

> 注意 `rho_buffer_` 每周期都会被覆盖；节点没调用 setter 时（比如 leader）用 50 兜底。

### 5.4 节点侧状态机（`src/mpc_node.cpp`，约 60 行）

放在定时器回调里、`solve` **之前**：

```cpp
// === 第3步：指定时间编队恢复 ===
static bool   in_avoid   = false;
static bool   clear_ok   = false;
static double clear_since = 0.0;
static bool   recovering = false;
static bool   armed      = false;
static double tk = -1e9, last_trigger = -1e9;
static double rho0 = 0.15;

const double now = ros::Time::now().toSec();

// 1) 武装：起飞 10 s 后，且编队误差曾经小于 e_th
if (!armed && (now - t_start_) > 10.0 && formation_err_ < rp.e_th) armed = true;

// 2) 净空滞回
if (h_min < rp.h_enter)      { in_avoid = true;  clear_ok = false; }
else if (h_min > rp.h_exit)  { if (!clear_ok) { clear_ok = true; clear_since = now; } }
else                         { clear_ok = false; }

// 3) 安全优先：避障中一律暂停（时钟语义上等价于清零）
if (in_avoid) recovering = false;

// 4) 触发
if (armed && !in_avoid && clear_ok
    && (now - clear_since)  >= rp.t_clear
    && (now - last_trigger) >= rp.t_dwell
    && formation_err_ > rp.e_th) {
    tk = now; last_trigger = now; recovering = true;
    rho0 = std::min(rp.rho_max, std::max(rp.kappa * formation_err_, rp.e_th));
    ROS_INFO("[%s] RECOVERY e=%.2f rho0=%.2f", UAV_NAME.c_str(), formation_err_, rho0);
}

// 5) 生成 rho_k
std::vector<double> rho(N_ + 1, 50.0);
if (recovering && is_follower) {
    for (int k = 0; k <= N_; ++k) {
        double s = (now + k * dt_ - tk) / rp.big_t;
        s = std::min(1.0, std::max(0.0, s));
        const double phi = 1.0 - 3.0*s*s + 2.0*s*s*s;      // 1 → 0，两端 C1
        rho[k] = rp.eps_f + (rho0 - rp.eps_f) * phi;
    }
    if (now - tk > rp.big_t + 1.0) recovering = false;      // 窗口结束后放宽
}
mpc_ptr->setFormationRho(rho);
// === end ===
```

**`formation_err_` 怎么算**（节点里，10 行）：

```cpp
Eigen::Vector3d p_star = Eigen::Vector3d::Zero();
double wsum = 0.0;
for (const auto& kv : bary_weights) {          // 本机权重表
    auto it = neighbor_states_.find(kv.first); // 已按 age 对齐的 8x(N+1)
    if (it == neighbor_states_.end()) continue;
    p_star += kv.second * it->second.col(0).head<3>();   // 第 0 列 = 当前时刻
    wsum   += kv.second;
}
formation_err_ = (wsum > 0.5) ? (p_self_pos - p_star / wsum).norm() : 0.0;
```

> 如果节点里没有 `neighbor_states_` 缓存，就找现有填 `UavState.predicted_states` 的地方，用那份数据。

### 5.5 参数（`launch/mpc.launch`）

```xml
<param name="recovery/enable"  value="true"/>
<param name="recovery/h_enter" value="0.25"/>
<param name="recovery/h_exit"  value="0.45"/>
<param name="recovery/t_dwell" value="0.5"/>
<param name="recovery/t_clear" value="0.5"/>
<param name="recovery/big_t"   value="3.0"/>
<param name="recovery/eps_f"   value="0.15"/>
<param name="recovery/e_th"    value="0.20"/>
<param name="recovery/kappa"   value="1.1"/>
<param name="recovery/rho_max" value="2.0"/>
```

### 5.6 验证：怎么**稳定地**触发一次

固定场景里“编队穿窄通道 → 形变 → 通道过后恢复”不一定每次都触发。为了能反复测，加一个调试钩子（10 行）：

```cpp
// === 第3步：调试用（正式实验关掉）===
bool force_trigger = false;
ros::Subscriber sub = nh.subscribe<std_msgs::Bool>(
    "/" + UAV_NAME + "/recovery/force_trigger", 1,
    [&](const std_msgs::Bool::ConstPtr& m){ if (m->data) force_trigger = true; });
// 状态机第 4 步里加： if (force_trigger && armed) { /* 走触发分支 */ force_trigger = false; }
```

用法：

```bash
rostopic pub -1 /uav0/recovery/force_trigger std_msgs/Bool "data: true"
```

**实验 A（可控）**：等编队稳定 → 触发 → 从 bag 里量“触发 → `‖e‖ ≤ ε_f`”的时间，做 10 次。
**实验 B（真实）**：`fixed_scene` 里从 x=−20 穿到 x=0，统计自然触发次数与恢复时间。

### 5.7 常见坑

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 一飞就触发 | 起飞阶段误差大 | `armed` 门控（10 s + 曾收敛） |
| 触发后求解失败 | `ρ_0` 比当前误差小 | `ρ_0 = max(κ·e, e_th)` 并 clamp 到 `ρ_max`；`κ` 给 1.1~1.3 |
| 一直触发（Zeno） | `T_dwell/T_clear` 太小 | 都 ≥ 0.5 s；`e_th` 提到 0.25 |
| 恢复拉不动 | `w_form` 太小 | 恢复期间把 `w_form` 临时乘 2 |
| 影响避障 | 恢复约束与 CBF 打架 | `AVOID` 时 `rho=50`（即暂停），让 CBF 优先 |

### 5.8 本步结束状态

- [ ] MPC 有 `rho` 参数与 `‖e‖²≤ρ²` 约束；节点有状态机与 `force_trigger`
- [ ] 10 次可控触发，恢复时间 ≤ 1.1T
- [ ] 有“开/关恢复”的对比曲线
- [ ] `git commit`：“第3步：事件触发指定时间编队恢复”

---

## 6 收尾：出图与写论文（1 天）

### 6.1 三张必需的图

| 图 | 横轴 / 纵轴 | 数据来源 | 说明什么 |
| --- | --- | --- | --- |
| **图 1 编队误差 + ρ 包络** | 时间 / `‖e‖` (m) | 一次“形变→恢复”的 CSV | 证明指定时间收敛：`‖e‖` 被 `ρ(t)` 压着走，T 内进 `ε_f` |
| **图 2 最小净空对比** | 组别（CBF 开/关）/ 净空 (m) 箱线图 | 各 10 次 bag + `min_clearance.py` | 证明 CBF 有效：5% 分位抬高 |
| **图 3 轨迹与队形快照** | xy 平面轨迹 + 若干时刻的三角形 | RViz 截图或 bag | 证明编队保持与包含性 |

### 6.2 论文里必须写清的 4 句“实现偏离”

1. **CBF 是名义化的**：`k≥1` 的 `n,h` 由上一轮解冻结（序列线性化），只有 `k=0` 严格。
2. **CBF 只作用于静态障碍**；动态障碍用形状感知 softplus 软势垒，配松弛 `δ`。
3. **跟随者仍保留一条全局路径**（兜底参考），但终端/进度项权重降到 0.2，编队项主导。
4. **恢复的时变约束用平方形式** `‖e‖²≤ρ²`（等价且对 IPOPT 更友好），`ρ` 以参数注入，状态切换不重建。

### 6.3 一组能直接复现的参数

```bash
roslaunch form_nav formation_test.launch
rostopic pub -1 /move_base_simple/goal geometry_msgs/PoseStamped \
  "header: {frame_id: 'world', stamp: now}
   pose: {position: {x: 15.0, y: 0.0, z: 1.0}, orientation: {w: 1.0}}"
```

关键值（写进论文参数表）：`N=20, dt=0.05, u_max=2.0, w_max=2.0, tau_v=0.67, tau_w=0.5, a_brake=1.5, w_form=400, follower_target_scale=0.2, gamma=0.65, cbf/safe_dist=0.10, slack_weight=1e4, T=3.0, eps_f=0.15, h_enter=0.25, h_exit=0.45, T_dwell=0.5, T_clear=0.5, e_th=0.20`。

### 6.4 提交

```bash
git add -A && git commit -m "论文仿真跑通：重心编队 + CBF + 指定时间恢复"
git push origin sim/thesis
```

---

## 7 附录

### 附录 A 参数速查（只列这三步引入的）

| 参数 | 默认 | 含义 | 调参方向 |
| --- | --- | --- | --- |
| `mpc/w_form` | 400 | follower 编队误差权重 | 跟不住 ↑；撞机/绕障差 ↓ |
| `mpc/follower_target_scale` | 0.2 | follower 终端/进度项缩放 | 更“跟队形” ↓；更“跟路径” ↑ |
| `cbf/enable` | true | CBF 开关 | 对照实验用 |
| `cbf/gamma` | 0.65 (1/s) | CBF 衰减率 | 越大越保守，也越易不可行 |
| `cbf/safe_dist` | 0.10 | CBF 安全间距 | 与 `safe_dist` 一致 |
| `cbf/slack_weight` | 1e4 | 松弛罚权重 | 太小=约束形同虚设；太大=易失败 |
| `recovery/big_t` | 3.0 s | 恢复窗口 `T` | 论文里的“指定时间” |
| `recovery/eps_f` | 0.15 m | 收敛邻域 | 太小不可行 |
| `recovery/h_enter / h_exit` | 0.25 / 0.45 | 避障进入/退出净空 | 必须 enter < exit |
| `recovery/e_th` | 0.20 m | 触发所需最小形变 | 太小会误触发 |

### 附录 B 三步之间的接口契约（新开对话必须知道）

| 接口 | 产生于 | 语义 | 下一步怎么用 |
| --- | --- | --- | --- |
| `FormationConfig::isLeader / weights` | 第 1 步 | 角色与重心权重 | 第 2 步 `barycentricTarget`、第 3 步 `e` 都靠它 |
| `Mpc::barycentricTarget(k, neighbor_params)` | 第 2 步抽出 | 第 k 步 `p*`（3×1） | CBF / 恢复都调用它，**不要各写一份** |
| `Mpc::getPredictedClearance()` | 第 2 步 | `h_k` 序列（N+1） | 第 3 步的 `h_min` |
| `Mpc::setFormationRho(rho)` | 第 3 步 | `ρ_k` 序列 | 状态机每周期调用 |

### 附录 C 故障速查

| 症状 | 先查什么 |
| --- | --- |
| 编译报 CasADi 维度错 | `Slice` 的起止；矩阵宽高；用循环赋值替代 `reshape` |
| 启动 `ROS_FATAL` 退出 | `FormationConfig::load` 的校验：权重和、leaders、neighbors 是否含 leader |
| 求解一直失败 | 先关 CBF 看是否恢复（隔离变量），再降 `safe_dist`/`γ`、加 `slack_weight` |
| follower 不动或乱飞 | 看 `use_barycentric`、`weights` 是否为空、`w_form` 是否为 0 |
| 编队误差抖 | `p*` 随领导者预测抖动 → 对 `p*` 做一阶低通（`p* = 0.7p* + 0.3p*_prev`） |

### 附录 D 时间不够时的保底方案

到第 7 天还没跑通 CBF 就这样砍：

1. **CBF 保“存在性”**：只对 `k=0` 施加 1 条约束，`γ` 给 0.4 —— 仍是“硬约束 + 软引导”，图 2 照样能做（提升幅度小些）。
2. **恢复保“机制”**：只做 `force_trigger` 实验（10 次），不做真实场景统计 —— 图 1 照样能画。
3. **编队一定要做完**：它是三个模块里最“看得见”的，也是论文核心贡献。

---

**结束**　|　每步开新对话，按 §0.2 的开场模板 + 本文档对应章节执行即可。
