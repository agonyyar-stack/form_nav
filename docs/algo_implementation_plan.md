> ⚠️ **参考文档，不必按此执行。** 本文是"理论 ↔ 代码"的完整差异分析与工程化模块方案（适合查资料、写论文时对照公式）。
> **实际执行路径请用 [docs/sim_steps.md](sim_steps.md)**：单人、10 天、3 个模块的分步骤实现说明。

# 算法落地分模块开发文档

> 从《算法框架说明》（`算法框架说明10302027.docx`，下称**理论文档**）到 `form_nav` 现有实现的可执行落地计划。
> 基线：分支 `develop/paper-algorithm` @ `d9280d6`；现有实现口径以 `docs/context.md` 与代码为准。
> 版本：v1.0（2026-11-04）　读者：算法/控制/集成/评测方向的开发者与论文作者

---

## 0. 文档说明

### 0.1 目标与范围

本文件回答两个问题：

1. **理论到落地需要改什么** —— 逐条列出理论文档中的公式、维度、术语在工程上无法直接落地的点，并给出**改法**（分"数学表述调整 / 算法结构调参 / 工程实现调整"三类）。
2. **怎么分模块做** —— 把工作拆成 10 个可**独立开发、独立验收**的模块（M0–M9），每个模块给出：前置依赖、接口契约、实现步骤、验收标准、测试方法、工作量与回退策略。

不包含：具体代码实现细节（由各模块负责人按接口契约实现）、论文正文写作。

### 0.2 一句话结论

现有代码已经完成了**单机/多机同构 MPC + 纯 A* + 软势垒避障**的主干（理论文档第 1、2 章与 3.2 节的绝大部分），
真正需要**新建**的是理论文档的四个增量贡献：**领导者/跟随者角色架构、重心（barycentric）编队、CBF 硬约束、事件触发指定时间编队恢复**；
同时有 **12 项工程缺陷**必须先修（否则新模块无法独立开发和验证），以及 **8 处文档与代码口径漂移**要一并纠正（否则论文实验不可复现）。

按本计划拆解：**新增 8 个头文件 + 8 个源文件 + 1 个离线权重脚本 + 1 套评测脚本 + 5 个 gtest 用例**，总工作量约 **78 人日**（3 人并行约 7 周）。

### 0.3 怎么用这份文档

- **接口先冻结**：§3 是接口契约（C++ 头文件签名 / YAML schema / 话题消息 / 参数键名）。模块负责人开工前按 §3 对齐，允许各模块内部自由实现，**不允许改公共接口**；确需变更走"接口变更记录"（附在 §3.7）。
- **默认关闭**：所有新功能以参数开关（默认 `false` / 权重 0）合入，主干始终可跑，便于 A/B 消融与回退。
- **独立验收**：每个模块必须能在**不启动 Gazebo** 的条件下自测（gtest 单测 + 离线回放脚本），这是"可独立开发"的硬性要求。
- **论文口径**：§2.8、附录 A/B/C 是给论文作者用的：哪些公式要改写、哪些参数要补、哪些公式缺失需补写。

### 0.5 立即开始的三件事

1. **决策**（今天是阻塞项）：确认 §2.9 的 D1–D12，其中 **D1（领导者固定）、D3（`J_prog` 口径）、D7（新队形）、D8（CBF 实施方式）** 直接决定接口，其余可边做边定。
2. **冻结接口**：§3.4 的 8 个头文件签名 + §3.5 的 YAML schema 评审通过后不再改（变更走 §3.7 记录）。
3. **并行开工**：`M0`（修缺陷/建测试）、`M1`（模型公共头）、`M4`（重心几何 + 离线脚本）三条线互不阻塞。

**第一周验收物**（P0 出口）：`MpcDiagnostics` 可用 + 3 个行为基线 bag（单机/6 机/6 机含动态障碍）+ 重心几何单测通过 + 新队形权重可用 + `scripts/run_all_tests.sh` 全绿。

### 0.4 术语对照

| 理论文档 | 代码/本文件 | 说明 |
| --- | --- | --- |
| 领导者（leader） | `role=leader` | 理论中承担全局 A* 与编队目标的 3 架机 |
| 跟随者（follower） | `role=follower` | 跟踪领导者凸组合位置的 3 架机 |
| 有效距离 `d`（扣半径） | `d_eff` | 代码统一口径：`d_eff = ‖p−o‖ − r_drone` |
| 激活距离 `D_{act}` | `detect_range` / `d_act_ca` | 势垒开始抬升的距离 |
| 最小安全距离 `D_{safe}` | `safe_dist` / `d_safe_ca` | 势垒分母的下限保护 |
| 净空 `h` | `clearance` | 距离型障碍函数的值（m），与 CBF 共用 |
| 重心坐标 `m_{i,j}` | `barycentric_weights` | follower 对 leader 的凸组合权重 |
| 编队误差 `e` | `formation_error` | `‖p_i − p*_Fi‖` |

---

## 1. 现状基线（代码事实）

### 1.1 已具备能力（可直接复用）

| 层 | 能力 | 位置 | 评价 |
| --- | --- | --- | --- |
| 模型 | 8 维状态 `[x,y,z,ψ,vx,vy,vz,ω]`、4 维控制 `[vx_c,vy_c,vz_c,ω_c]`（world 系），yaw 与线速度解耦 | `include/form_nav/mpc.h`、`src/mpc.cpp` | 与理论 §1.1 完全一致 |
| 离散化 | ZOH 解析精确离散（α/β/γ/δ 系数），非 Euler | `src/mpc.cpp`（模型段） | 与理论 §1.2 一致，系数需抽公共头 |
| 全局层 | 三维栅格纯 A*（0.1 m、26 邻域）+ LOS 去锯齿 | `src/a_star.cpp`、`src/astar_node.cpp` | 与理论 §2 主体一致，缺滚动重规划 |
| 地图 | 膨胀二值占据栅格（0.1 m、膨胀 0.4 m）+ 点云 KD-tree（0.1 m 体素、2 s 重建） | `src/grid_map.cpp`、`src/kd_tree_map.cpp` | 可复用 |
| 局部代价 | 静态/动态/机间 softplus 势垒、控制增量、逐点参考、终端参考 | `src/mpc.cpp` | 与理论 §3.2 对应，见 §2.4 差异 |
| 约束 | 逐轴输入盒、全时域制动不等式、高度盒 | `src/mpc.cpp` | 理论未写高度盒，需回填论文 |
| 通信 | 20 Hz 广播 `/uavX/predicted_path_share`（8×(N+1) world 系），按消息 age 线性插值 + yaw 连续化对齐 | `src/mpc_node.cpp` | ★ 编队/CBF/恢复三个新模块的公共基础，可直接复用 |
| 求解 | CasADi Opti + IPOPT，Opti 复用 + 上一轮解作初值，失败零速 hold | `src/mpc.cpp` | 需补计时与成功率统计 |
| 仿真 | 6 组 PX4 SITL + MAVROS 命名空间、固定场景生成器、场景编辑器、动态障碍真值轨迹 | `launch/*`、`src/obs_generator.cpp`、`scripts/scene_editor.py` | 可复用，场景口径需校正 |

### 1.2 必须先修的工程缺陷（12 项）

> 这些不是"理论差异"，而是**会阻断新模块开发或让实验结论失真**的问题。全部归入 M0 处理。

| 编号 | 缺陷 | 影响 | 位置 |
| --- | --- | --- | --- |
| B1 | KD-tree 为空时构建好的 Opti 中**静态避障项永久缺失**（构建签名不含"树已就绪"） | 静态避障可能全程不生效，实验结论不可信 | `src/mpc.cpp`（`kdtreeObstacleAvoidance`、`buildOpti` 签名） |
| B2 | MPC 求解失败 / 等待邻居分支**不发布本机预测轨迹** | 单机故障经编队传播为全编队停摆；新架构下领导者故障会直接打断跟随者 | `src/mpc_node.cpp` |
| B3 | 权重、`drone_radius` 等标量**烧进符号图**，运行期改 setter 不生效 | 无法做在线调参与参数扫描（论文需要） | `src/mpc.cpp`、`include/form_nav/mpc.h` |
| B4 | 邻居消息校验要求 `dt`/`N` 与本机**严格相等**（1e-6） | 异构时域不可行；新模块（CBF 可能需缩短时域）被锁死 | `src/mpc_node.cpp`（邻居回调） |
| B5 | 制动约束用**到终点的直线距离** | 弯道末段过早刹车，终点前速度受限 | `src/mpc.cpp` |
| B6 | 解无 NaN/可行性校验；失败时不沿用上一周期指令 | 偶发指令跳变 | `src/mpc.cpp`、`src/mpc_node.cpp` |
| B7 | A* **无滚动重规划**（仅目标回调触发一次）；启发是 3D octile 而非文档写的欧氏（`λ_h` 已由 `astar/lambda_heu` 参数给出，不是硬编码，但与文档口径不符） | 与理论 §2"滚动重规划"不符；动态新增障碍无响应 | `src/a_star.cpp`、`src/astar_node.cpp` |
| B8 | A* 只判定端点体素（不扫掠棱边）、浮点位置做哈希键导致同格重复展开、搜索 z 不受 MPC 高度盒约束、`virtual_ceil_height` 在点云路径下未生效 | 可生成穿角/高飞路径，MPC 不可跟踪 | `src/a_star.cpp`、`src/grid_map.cpp` |
| B9 | 场景不可复现：`formation_test.launch` 不透传 `scene_file`，`obs_generator.launch` 默认已是 `standard.yaml`；`fixed_scene.yaml` 实为 30 个立方体 + 2 道 20 m 长墙（文档写 30 个障碍、无墙） | 论文基准场景与文档不一致 | `launch/`、`config/scenes/`、`docs/fixed_scene_benchmark.md` |
| B10 | 编队无实际作用力：`w_form=0`；无角色；`dyaw` 通道权重为 0；现有队形里 `uav0/1/2` 三点共线，**不满足"非退化凸包"** | 理论 §4 无法直接套用 | `config/formations.yaml`、`launch/mpc.launch`、`src/mpc.cpp` |
| B11 | 文档-代码口径漂移 8 处（τ_v 0.67 vs 0.7、a_brake 1.5 vs 1.2、`w_dyn` 10 vs 文档 `m_dyn` 50 且键名未改、`AGENTS.md` 仍写 ESDF/CasADi（实际无 ESDF）、`w_subgoal`/`w_u` 过期、编队"软约束"实际关闭、`GlobalPathProgress` 无人使用） | 后续开发按文档取值会踩坑 | `docs/context.md`、`launch/mpc.launch`、`CMakeLists.txt` |
| B12 | 无单元测试、无求解耗时/成功率统计、无自动化评测脚本 | 无法量化任何改进；新模块无验收手段 | 全仓库 |

---

## 2. 理论 → 实现的差异分析与调整建议

### 2.1 差异的整体分类

理论文档与现有实现/工程可行性的差异分三类，处置方式不同：

| 类别 | 含义 | 处置 | 涉及条目 |
| --- | --- | --- | --- |
| **A. 数学表述调整** | 公式写错/写漏/量纲缺失，但算法思路可行 | 改论文公式（不改算法），必要时补定义 | T1–T3、T12–T16、T21–T24、T36–T39、T42–T43 |
| **B. 算法结构调整** | 理论写法在求解器/实时性/数据接口下无法直接实现，必须换等价或近似形式 | 改算法 + 在论文中说明近似及其代价 | T4、T17、T25–T28、T30–T32、T34 |
| **C. 工程实现调整** | 理论没写、但落地必须决定的架构/配置/通信/性能问题 | 按 §3 契约实现 | T5–T11、T18–T20、T29、T33、T35、T40–T41 |

### 2.2 运动学与离散化（理论 §1）

| 编号 | 理论表述 | 代码现状 | 差异与问题 | 落地调整 | 模块 |
| --- | --- | --- | --- | --- | --- |
| T1 | `u=[v^c, w^c]`，`v^c∈ℝ³`、`w^c∈ℝ³` | `U∈ℝ⁴=[vx_c,vy_c,vz_c,ω_c]`，`ω_c` 是标量 yaw rate | 理论维度笔误：`w^c` 应为 **ℝ¹**，或写 `u∈ℝ⁴` | 论文改为 `u=[v^{cT}, ω^c]^T∈ℝ⁴`，并明确 `ω`、`ω^c` 为标量（rad/s） | M1 |
| T2 | `x_{k+1}=Ax_k+Bu_k`，"A、B 为常数矩阵" | 实现为 α/β/γ/δ 四个系数的解析式 | 理论**没给 A、B 的具体元素**，读者无法复现；且位置方程含**指令项** `p_{k+1}=p_k+γv_k+δv^c_k`（不是 `p+dt·v`），这是"精确"的关键 | 论文附录给出显式 A、B（8×8 / 8×4）；代码抽公共头文件 `kinematics_model.h`（M1），供 MPC/CBF/参考生成三处共用，消除重复实现 | M1 |
| T3 | 状态含 ψ，未讨论角度回绕 | CasADi 符号状态内 ψ 连续累加；消息插值时用 `remainder` 连续化 | 若预测时域内 ψ 跨 ±π，终端 yaw 代价会出现跳变；初始 ψ 接近 ±π 时更明显 | ① 求解器内 ψ 用连续表示，只在**发布/代价**处做差：`Δψ=remainder(ψ−ψ_g)`；② 终端 yaw 代价改平滑形式 `2(1−cosΔψ)`（可选开关），避免非线性不可导；③ 单测覆盖 ±π 附近 | M1 |

### 2.3 全局层（理论 §2）

> **目标定位（理论 §2 首段）**：全局层**只提供方向引导**、把避障与跟踪交给局部层，从而避免局部规划陷入局部最优；因此对全局路径的要求是"**可跟踪、不穿障**"，而不是时间最优。
> **输出约定**：路径点一律为 **world 系**，经 `GlobalTrajectory` 发布（每点 `[x,y,z,vx,vy,vz]`，当前速度字段为 0 且被 MPC 忽略）；"感知范围"在实现中即 §3.4.6 的 `global/perception_radius`（默认 9 m），滚动重规划只在该窗口内展开。

| 编号 | 理论表述 | 代码现状 | 差异与问题 | 落地调整 | 模块 |
| --- | --- | --- | --- | --- | --- |
| T4 | `u_min≤u≤u_max`（盒约束） | 逐轴盒约束，三维范数可达 `√3·2≈3.46 m/s` | 若论文声称"速度上限 2 m/s"，与实现不符 | 二选一：**(a)** 论文明确"逐轴盒"；**(b)** 增加 `‖v^c‖≤v_max` 的范数约束（CasADi 支持 SOC，代价很小）。建议 (b) 作为可选项默认开启 | M1/M3 |
| T5 | `J_sm=Σ_{k=0}^{N−2}Δu_k^T W_{sm}Δu_k`，只有 Δu 软代价、无 Δu 硬约束 | 求和范围 `k=0..N−2`（与理论**完全一致**，共 N−1 个增量）；但 `W_sm` 退化为标量 `w_du=100`（yaw 通道再 /4 → 25） | 求和范围一致；差异在**权重矩阵被退化为标量**，论文需说明（或让代码支持 4×4 对角权重） | ① 论文写 `W_sm=diag(w,w,w,w/4)`；② 加可选 `Δu` 盒约束（`leader_ocp_mathtype.tex` 草稿已有，默认关闭、参数化） | M3 |
| T6 | "纯 A* **滚动重规划**，在感知范围内规划最短路径" | 只在 `/move_base_simple/goal` 回调里 `search()` 一次；定时器只重发可视化 | **核心差异**：无滚动重规划、无感知范围概念 | 新增 `GlobalPlannerScheduler`：① 周期 `T_replan`（建议 1 s）；② 触发条件（目标更新 / 实际位置偏离参考 > `d_dev` / 新增障碍格数 > `n_map`）；③ 只重规划"当前位置 + 感知半径"窗口并拼接，控制单次耗时 | M2 |
| T7 | "在感知范围内规划" | 用全局点云构建全图（50 m×50 m×3 m），一次全图 A* | 全图 A* 每周期重算不可接受；且"感知范围"在仿真里等价于全局云 | 定义 `perception_radius`（建议 8~10 m）：滚动重规划只在局部窗口内搜索；论文写成"感知域内滚动 A*" | M2 |
| T8 | 启发项 `h` 用**欧氏距离**，权重 `λ_h` | 实际是 **3D octile**；`λ_h` 由参数 `astar/lambda_heu` 给出（launch=1.0）；`getEuclHeu()` 已实现但零调用 | 与论文不符；octile 对 26 邻域是更紧的可采纳启发（搜索更省），但严格说不是欧氏 | 推荐**保留 octile 并把 `λ_h` 参数化**，论文改写为"按 26 邻域度量折算的 octile 距离"；若坚持欧氏，改 `a_star.cpp` 三处启发调用即可 | M2 |
| T9 | 通行性由**膨胀占据栅格**判定 | 膨胀 0.4 m（= r_drone 0.3 + safe 0.1，与 MPC 口径一致）；但 A* 只查端点体素 | 可穿过两个斜对障碍的"角缝"（端点空闲、棱边相撞） | 复用 `src/raycast.cpp` 做**线段扫掠检查**：`shortcutPath()` 已有 0.05 m 采样检查（`isSegmentCollisionFree`），**缺的是邻居扩展**（端点判定）→ 对每步位移做体素遍历 | M2 |
| T10 | 全局层只由**领导者**承担（§3 开头、§4.1） | 6 架机全部各自 A* + MPC，同构 | **架构差异**：这是新增的不是修的 | M5 引入角色：`leader` 才启动 `astar_node`；`follower` 不订阅 `/move_base_simple/goal`、不跑 A* | M5 |
| T11 | 未讨论飞行高度域 | 地图 z∈[−0.01,2.99]，MPC 高度盒 z∈[0.5,2.8]，终点 z 硬编码 1.0+偏移 | A* 可在 MPC 飞不到的高度规划 → 不可跟踪；天花板膨胀失效（点云路径未调用 `clearAndInflateLocalMap`） | ① A* 搜索 z 限制在 `[z_min_plan, z_max_plan]`（=MPC 高度盒内缩 0.1）；② 目标 z 改为配置项而非硬编码 1.0；③ 修复天花板膨胀 | M2 |

### 2.4 局部层代价（理论 §3.2 / §3.3）

> **这里有一个理论自相矛盾，必须先裁决（决策 D13）**：理论写"**五个代价项**"但只列出 4 个（J_oa、J_ca、J_sm、J_g）。至少有两种自洽读法：
>
> - **读法 H1**：第 5 项是未写出的"目标距离项（进度项）`J_prog`"，跟随者用 `J_form` 替换它；
> - **读法 H2**：把 `J_oa` 的静态项与动态项计作两项，加上 `J_ca`、`J_sm`、`J_g` 恰为五项；此时不存在 `J_prog`，跟随者"减少目标距离代价项"指的就是 **`J_g`**（降权或移除）。
>
> **落地策略（两种读法下都成立）**：保留代码里全部 8 个实现项并**各自独立加权**，用权重表达"增加/减少"而不是删项；论文按作者确认的读法二选一填写。因此本节的差异分析给出的是"**需要裁决的问题**"，不是结论。
>
> **实现细节（对应理论"通过查询点取上一轮求解的预测轨迹点"）**：KD-tree 的查询点 = **上一轮解的预测轨迹整体前移一列**（终端步用参考兜底），查到的最近点作为**常数参数**进入符号图；每控制周期在 `replan_max_iter` 循环里按新解重查询，索引变化才再解一次。

| 编号 | 理论表述 | 代码现状 | 差异与问题 | 落地调整 | 映射模块 |
| --- | --- | --- | --- | --- | --- |
| T12 | "五个代价项"但只列 4 项（**理论自相矛盾**） | 实际有：静态避障、动态避障、机间避碰、控制增量、输入项、中间步参考、终端参考+偏航、编队误差 | 项数与内容对不上；"跟随者减少目标距离项"到底减的是 `J_prog` 还是 `J_g`，两种读法都自洽（H1/H2，见本节引言） | **不改代码**：8 个实现项归并为 5 个理论项后**各自保留权重开关**；跟随者对"目标距离/终端"两项统一**降权**（默认 ×0.3）而非删除，两种读法都能用参数表达；论文按 D13 裁决填写 | M3 |
| T13 | `J_oa` 只含 `Σ_{k=1..m}`（下标 `k` 实际是**障碍点**编号），没有对**预测步**求和 | 实现对 `n=0..N` 每一步、每步最近 `q=1..3` 个点都求和 | 理论**漏了对预测步的求和**，按字面实现只惩罚当前时刻 → 无法前瞻避障；且符号 `k` 与预测步撞名 | 论文改用两套符号：`Σ_{n=1}^{N} Σ_{q=1}^{3}`（`n`=预测步、`q`=最近点）；代码不变 | M3 |
| T14 | `J_ca=W_cΣ_j (1/(d_j−D_{act}))softplus(k_c(D_{safe}−d_j))` | 实现为 `(Wc/d_scale)·softplus(k_ca(d_act_ca−d_eff))`，`d_scale=max(d_eff−d_safe_ca,0.01)` | 分母用 `D_act`、softplus 用 `D_safe`，与 J_oa（分母 `D_safe`、softplus `D_act`）**正好写反**；且缺对预测步的求和 | 论文统一为与 J_oa 同形：`Σ_j Σ_n (1/max(d−D_safe,ε))·softplus(k(D_act−d))`（`n`=预测步）；代码不变 | M3 |
| T15 | 动态项 `W_dΣ_iΣ_{j=1}^{i}softplus(k_d(D_act−d_ij))` | 实现：点-折线最短距离（0.5 s 窗口、0.1 s 节点）+ 形状感知表面距离 + softplus | 理论①没有 `1/d` 因子，与其他两项不一致；②`j` 的上界写成 `i`（应为常数或预测步数），无法实现；③按"第 j 个节点"逐点对齐，实际更合理的是与整条扫掠折线取最短距离 | **推荐按实现口径改写理论**：`J_dyn=Σ_n Σ_q M·softplus(k_d(d̄_d−d̃_{nq}))`，`d̃` 为第 n 步位置到第 q 个动态障碍**扫掠折线**的最短表面距离；这比逐节点对齐更稳（求解滞后也不会漏判）。论文补一段说明 | M3 |
| T16 | `J_g=(x_N−x_g)^T W_g(x_N−x_g)`；`x_g` 位置分量为目标位置、**v 与 ω 分量为 0**（终端只惩罚位置与偏航），但 `ψ_g` 的"定义为"之后**公式缺失** | 实现：终端参考取 `[x,y,z,ψ]`，权重对角 `diag(w_subgoal, w_subgoal, w_subgoal/2, w_yaw)`（**z 通道权重减半**，不是把 z 值除以 2）；`ψ_g`=指向子目标方向，距终点 0.3 m 冻结 | `ψ_g` 无定义 → 无法复现；`W_g` 的 z 通道权重减半也需说明 | 论文补三件事：① `x_g=[p_g, ψ_g, 0, 0]`（v、ω 权重为 0）；② `ψ_g=atan2(p_sub−p_N)`（与 §3.1 一致）；③ `W_g=diag(w,w,w/2,w_ψ)` 与 <0.3 m 冻结规则 | M1/M3 |
| T17 | "**目标距离代价项**"（未给公式） | 实现是"梯形速度剖面链式逐点参考"：从当前 `(s0,v_tan)` 按 加速→巡航→刹停 生成期望速度、用同一一阶模型推进 1-D 弧长得到每步目标点，中间步小权重、终端大权重 | 理论只说"到目标/子目标的距离"，代码是**时变参考点跟踪**（2026-09-04 的改动，目的：单一子目标会把预测轨迹长度卡在前瞻量内） | 二选一并写进论文：**(a) 推荐**保留链式参考，论文把 `J_prog` 定义为"对时变参考点的追踪距离代价"，给出速度剖面公式（与 §1 模型自洽）；**(b)** 退化为纯终端子目标，作为消融基线。两条都要能在参数里切换（`ref_mode=chain|subgoal`） | M3 |
| T18 | 权重 `W_s, W_d, W_c, W_{sm}, W_g, W_f` 全部无数值 | 部分键的**代码默认与 launch 相差 1.2~100 倍**（`w_oa` 100 vs 5、`k_oa` 100 vs 1、`w_ca` 200 vs 5、`k_ca` 100 vs 1、`w_subgoal` 20000 vs 1000、`tau_v` 0.8 vs 0.67） | 参数无单一来源 → 复现性差；论文表格无法填写 | 建立**唯一参数源**：launch/YAML 为准，代码默认值同步为 launch 值；M0 输出参数总表（附录 B）并加"启动时打印 + 落盘记录" | M0/M3 |
| T19 | softplus `ln(1+e^x)` | 已有 `stableSoftplus`（大/小输入分支），无数值溢出 | 实现本身健全；但 `d_scale` 下限 0.01 使近距惩罚有界而梯度饱和，需在论文注明；参数化该下限并补单测 | M3 |
| T20 | 未提"谁避让谁" | 全部机对之间是**无环竞赛图**（按优先级全序单向），每对恰一条"接收者避让发布者"的边 | 双向惩罚会互相顶牛振荡；理论没写这一层 | 论文补一节"**单向让行拓扑**（无环全序，避免双向退让）"并给出无环性论证；M5 改为**角色化优先级**（follower 让 leader > leader 之间按序） | M5 |
| T42 | `J_form=Σ_{k=1}^{N}(p_k−p*_k)^T W_f(p_k−p*_k)` | 实现是"共识误差"形式 `S_f=Σ_j[(y_j−M_j)−(p_i−M_i)]`，且 `w_form=0` 默认关闭 | 求和起点理论为 `k=1`、代码状态序列为 `k=0..N`（差异可忽略但需说明）；`W_f` 矩阵退化为标量 `w_form` | M5 改为"重心期望 + 标量权重"；论文写 `W_f=diag(w_f,w_f,w_f)`（yaw 通道可选 0），并说明 `k` 从 1 开始（`k=0` 为实测状态、不含控制） | M5 |
| T43 | `J_prog`（**理论未给公式**） | 中间步 `k=1..N−1` 用 `w_subgoal_int` 跟踪时变参考、终端 `k=N` 用 `w_subgoal` | 若 D13 裁定读法 H1，则必须给出 `J_prog` 公式；若裁定 H2，则本项与 `J_g` 合并 | 论文按 D13 写；代码侧 `ref_mode` 与两项权重独立可调，两种读法都能复现 | M3 |

### 2.5 CBF 安全约束（理论 §3.5）

> 这是全文**最需要落地改造**的一节。核心矛盾：理论声称"该约束对速度指令线性"，但 `n_{ik}`、`h_{ik}` 依赖预测位置 `p_k`（决策变量），**只有把 `n`、`h` 冻结成常数时才是线性的**。
>
> **语义复述（论文需保留）**：`n^T(v−v^o) ≥ −γh` 等价于"沿障碍法向的**接近速度不超过 γh**"，即碰撞时间 `≥1/γ`；若每个预测步都满足，则安全集 `C={h≥0}` 在**离散采样意义下前向不变**（连续时间的严格不变性要求 `Δt→0`，离散实施用衰减因子 `1−γΔt` 保证不越界）。

| 编号 | 理论表述 | 落地问题 | 落地调整 | 模块 |
| --- | --- | --- | --- | --- |
| T21 | `n^T_{ik}(u_k−v^o_{ik})+γh_{ik}≥0` 对 `u_k` 线性 | `n_{ik}=(p_k−o_{ik})/‖p_k−o_{ik}‖`、`h_{ik}=‖p_k−o_{ik}‖−R` 都是 `p_k` 的函数，`p_k` 是决策变量 → 约束**非线性（非凸）** | **分层策略（推荐）**：① `k=0`：`p_0` 是实测状态 → `n_0,h_0` 天然是**常数**，约束严格线性且是"真 CBF"，无需近似；② `k≥1`：用**上一轮解或名义参考**冻结 `n,h`（参数），约束线性——即**名义化/序列线性化 CBF**。论文必须写明这一点（否则审稿人会指出"线性"不成立），并把推导链 `h_{ik}→ḣ_{ik}=n^T(v_k−v^o_{ik})→α(h)=γh→CBF 条件` 完整写出（当前理论只给结果式），并给出"冻结策略 + 每次重规划刷新"的收敛说明（等价于 SQP 的一次线性化步）。**备选**：全部用 CasADi 非线性表达式直接交给 IPOPT（光滑可微，局部收敛良好，但非凸且耗时上升）——作为 `cbf_mode=exact|frozen` 开关，用实验对比 | M6 |
| T22 | `α(h)=γh`，`γ=0.65`（"等价于碰撞时间不小于 1/γ"、"由允许的最短制动时间确定"） | ① 理论**未标 `γ` 单位**（由 `α(h)=γh` 可推出 **1/s**）；② 离散实施应是 `h_{k+1}≥(1−γΔt)h_k`（或 `e^{−γΔt}h_k`），而 `leader_ocp_mathtype.tex` 写的是 `(1−γ_o)h`，未见 `γ_o` 与 Δt 的关系 | 明确 `γ[1/s]`；离散衰减 `β_disc=1−γΔt`（Δt=0.05、γ=0.65 → 0.9675）或指数形式；**并校验物理自洽**：`1/γ≈1.54 s` 应不小于 `v_max/a_brake=2.0/1.5≈1.33 s`（当前满足），改动 `a_brake`/`u_max` 时需同步复核 | M6 |
| T23 | 滞后补偿：`n^T_{ik}(v_{k+1}−v^o_{ik})+γh_{ik}≥0`，`v_{k+1}=e^{−Δt/τ_v}v_k+(1−e^{−Δt/τ_v})u_k` | 位置用 `p_k`、速度用 `v_{k+1}`，**半步错位**；且 `v^o_{ik}` 是障碍在"k 时刻"的速度，却与 `v_{k+1}` 相减 | 明确统一口径并写进论文：约束定义在 `[k,k+1]` 区间，用 `(p_k, v_{k+1})` 做保守化（等价于前向欧拉上界）；动态障碍速度也取 `[k,k+1]` 区间的平均/端点值，需在参数里说明 | M6 |
| T24 | `h_{ik}` 用"第 i 个障碍中心/点" | 静态障碍来自 KD-tree **最近点查询**：`o` 随 `p` 变化，`h` 的梯度在最近点切换处**不连续** | ① 每步只取**最近 1 个点**做 CBF（最紧的），避免求和保守过度；② 最近点作为**常数参数**冻结（与现有 softplus 同一机制），并在 `replan_max_iter` 循环里重查询；③ 若最近点与本轮解的距离跳变 > `d_switch`，标记该步 CBF 失效（跳过并计入违例统计），保证不产生错误约束 | M6 |
| T25 | `h=‖p−o‖−r_i−r_drone−d_safe`（单位 m；`h≥0` ⇔ 位于**膨胀障碍之外**，即安全集 `C={h≥0}`） | 静态点云 `r_i=0` ✓；动态障碍是球/圆柱，表面距离实现是分段精确的 | 论文明确 `h` 的几何语义与安全集定义（M7 的"净空"必须与之同义）；实现按两个分支给 `h`（点/球、圆柱），圆柱在"侧壁→顶/底面"切换处不光滑，需说明或用软化 | M6 |
| T26 | CBF 是**硬约束** | 密集障碍/多机狭窄通道极易**不可行** → 求解失败（现有失败=零速 hold，很危险） | ① 引入松弛 `δ_{k}≥0`（`h_{k+1}−(1−γΔt)h_k≥−δ_k`）+ 大权重线性罚（`leader_ocp_mathtype.tex` 已给出这种形式，沿用）；② **分级回退**：先放松机间 → 再放松动态 → 再放松静态 → 最后零速 hold，并把"降级"打点上报；③ 论文写明"软约束化 CBF"及其代价 | M6 |
| T27 | 软代价（softplus）与硬约束**同时**存在 | 两者会对同一障碍重复惩罚，CBF 生效时 softplus 仍持续给梯度 → 权重需要重新整定，否则轨迹过早绕行/抖动 | 给出协同配置：**CBF 为主约束**，softplus 权重降到原来的 20~40% 或只在外层（`d_eff>D_act`）保留；提供 `cbf_soft_scale` 参数与 A/B 实验 | M6 |
| T28 | 未讨论计算量 | `N=20`、每步 3 个最近点 + 动态若干 → 新增约 80~120 条约束；现有单周期预算 50 ms 且**无耗时统计** | M0 先加计时；M6 提供**降采样开关**（每 1/2/4 步施加一次 CBF、或仅 `k∈{0, N/2, N}`）；用实验确定"约束数 vs 求解耗时 vs 违例率"的折中 | M0/M6 |

### 2.6 重心编队（理论 §4）

| 编号 | 理论表述 | 现状 | 落地调整 | 模块 |
| --- | --- | --- | --- | --- |
| T29 | 跟随者期望位置 `p*_{Fi}=Σ_j m_{ij}p_{Lj}`，`m_{ij}` 由标称队形离线反解 | 现有 `offsets` 是**相对目标的固定平移**，语义完全不同；MPC 里只有 `formationError`（`a_ij≡1` 的共识误差，且默认关闭） | 新增配置模式 `formation_mode: offset \| barycentric`；`barycentric` 下由 `leader_ids` + `weights` 驱动；`offset` 保留以兼容旧实验 | M4 |
| T30 | 领导者位置构成**非退化凸包**，跟随者在**内部** | 现有 2×3 网格中 `uav0(−1,−2)/uav1(−1,0)/uav2(−1,2)` **共线** → 退化，重心坐标无解 | 必须**重新设计标称队形**：3 个领导者非共线（面积 ≥ `A_min`），3 个跟随者权重**严格为正**（在三角形内部）。附录 D 给出一个可行性算例（含权重数值），可直接用 | M4 |
| T31 | 3 领导者构成三角形（二维）——**注意：理论只说"非退化凸包"，并未限定水平面；二维三角形是本方案的工程假设** | 三维空间 | 三角形在**水平面**内定义；z 取领导者 z 的凸组合（要求领导者同巡航高度即可），或统一固定巡航高度。论文写明"水平面内凸组合 + 高度共面假设" | M4 |
| T32 | "求解由凸组合关系构成的线性方程组，并检查各权重非负" | 无实现 | 每个跟随者 3 个未知量、3 个方程（`m0+m1+m2=1` 与 xy 两式）→ 唯一解；实现为离线脚本 `scripts/formation_barycentric.py`：解方程 → 校验 `Σm=1`、`m≥−1e−9`、三角形面积、包含性 → 写回 YAML。运行时**不再求解**，只查表 | M4 |
| T33 | 未定义跟随者期望**偏航** | 现有 `dyaw` 通道权重为 0，实际不生效 | 三选一（决策 D6）：(a) 领导者 yaw 的凸组合；(b) 自身期望速度方向；(c) 领导者平均 yaw。建议 (a) 简单且与"编队随领导者整体运动"一致，论文写明 | M4/M5 |
| T34 | 未讨论领导者失效/退化 | 无 | 运行时检测：三角形面积 < `A_min`、某领导者消息超时、权重退化 → 进入 `DEGRADED` 状态：冻结权重/降级为 `offset` 模式/原地保持，并上报。论文补鲁棒性说明 | M4/M5 |
| T35 | 期望位置来自领导者**预测轨迹** | 现有消息 `PredictedTrajectory` 已含每步位姿，正好可用；时间对齐机制现成 | follower 用 `leaders 的预测轨迹在 age 对齐后的第 k 步位置` 做凸组合（复用现有插值代码）；注意领导者预测里含它自身的避障动作 → follower 的期望会跟着动，这是**特性**（保持相对形）而非 bug，但需在论文里说明"期望轨迹随领导者预测滚动更新" | M5 |

### 2.7 事件触发指定时间编队恢复（理论 §3.4）

| 编号 | 理论表述 | 落地问题 | 落地调整 | 模块 |
| --- | --- | --- | --- | --- |
| T36 | `h_min(t)=min_{i∈O(t)}min_{0≤k≤N}h_{ik}(t)`，`O(t)` 为"当前感知范围内的障碍集合" | 代码里**没有净空量**：softplus 势垒只用距离，不输出 `h` | 论文补 `O(t)` 的可实施定义：**`perception_radius` 窗口内的静态最近点集合 + 全部在跟踪的动态障碍**（与 M2 的感知半径同一参数）；新增 `ClearanceMonitor` 用与 CBF **同一个 h** 计算（静态最近点 + 动态形状表面距离），取整个预测时域最小值，使"避障模式"判据与安全约束自洽（论文里 `h` 只有一个定义） | M7 |
| T37 | 滞回 `h_enter<h_exit`、最小驻留 `T_dwell`、`T_clear` 窗口 | 无 | 状态机 `NORMAL→AVOID→RECOVER→PAUSED`；`AVOID` 退出条件 = `h_min>h_exit` 持续 `T_clear`；触发恢复还需 `T_dwell` 与 `‖e‖>e_th`。论文给默认值与整定依据 | M7 |
| T38 | `ρ(t)`、`φ(s)=1−3s²+2s³`、"四个约束（两边 C1 连续）" | 只给了 φ，**没给 ρ 的完整表达式**（图中显示 `ρ_0` 起、`ε_f` 止） | 补全：`ρ(t)=ε_f+(ρ_0−ε_f)φ(s)`，`s=(t−t_k)/T`（`s≥1` 时 `ρ=ε_f`）。**`ρ_0` 必须定义**：取触发时刻实测误差上包络 `ρ_0=max(‖e(t_k)‖·κ, e_th)`，并 clamp 到 `[ε_f, ρ_max]`——否则一旦 `‖e(t_k)‖>ρ_0` 立刻不可行 | M7 |
| T39 | `‖e(t)‖≤ρ(t), ∀t∈[t_k,t_k+T]` | `T`（秒）通常大于预测视界 `N·Δt=1 s`；且 `t_k` 跨多个 MPC 周期 | ① supervisor 用真实时钟**锁存 `t_k`**，每周期生成 `ρ_k` 序列（`k=0..N`，按 `kΔt` 取 `s`）；② 约束写平方形式 `‖e_k‖²≤ρ_k²`（`ρ_k` 为常数参数，光滑）；③ 若 `T` 小于视界，之后的步取 `ρ=ε_f`；④ 论文的 `t_k=inf{...}` 集合式中 `t_{k-1}`=上次触发时刻、`inf`=满足条件的最早时刻，与 `RecoverySupervisor` 的 `triggeredAt()`/`t_dwell`/`t_clear`/`e_th` 字段一一对应，需在附录给出映射 | M7 |
| T40 | "恢复期间若 `h_min` 再次 `<h_enter` 则暂停并清零时钟；安全事件优先" | — | 明确优先级：**CBF/避障 > 恢复**；暂停时**恢复时钟清零**（与理论"暂停恢复并将恢复时钟置零、待下一次净空恢复后重新计时"一致），重新触发时重新锁存 `t_k` 与 `ρ_0`（不沿用旧初值）；Zeno 由 `T_dwell` 保证 | M7 |
| T41 | 隐含"指定时间收敛"的定理级表述 | 实际是"滚动重规划 + 离散 + 软约束"的近似实施，无法直接给出严格定理 | 论文降级为"**仿真统计 + 可行性条件下的收敛论证**"：给出 100 次扰动实验的恢复时间分布（`≤T` 占比），并说明"若 MPC 每周期可行且约束被满足，则误差单调收敛于 `ε_f`"。避免写"保证指定时间收敛"（审稿会追问） | M9 |

### 2.8 论文表述需补写/修正的清单

> 给论文作者：以下 15 处（P1–P15）在理论文档里缺失或错误，**必须在投稿前补齐**，否则读者无法复现；另有 2 处符号/交叉引用问题记为 **P16/P17**，见附录 C 第 16–17 条。

| 编号 | 位置 | 问题 | 建议补写 |
| --- | --- | --- | --- |
| P1 | §1.1 | `w^c∈ℝ³` 维度错 | 改为标量 `ω^c∈ℝ`，`u∈ℝ⁴` |
| P2 | §1.2 | 只给 `A,B` 符号 | 附录给 8×8 / 8×4 显式矩阵（可由 α/β/γ/δ 直接写） |
| P3 | §2.1 | `λ_h`、分辨率、膨胀半径无数值；启发式描述与实现不符（octile vs 欧氏） | 补参数表；启发式改成"26 邻域 octile 度量" |
| P4 | §2 | 未写滚动重规划的触发条件与周期 | 补一节"滚动重规划策略"（周期/偏移/地图变化/目标更新） |
| P5 | §2 | 未写飞行高度域与天花板 | 补"搜索高度域受限"说明 |
| P6 | §3.2 | 写"五个代价项"却只列 4 个 | **按 D13 裁决补写**：H1→补第 5 项 `J_prog`；H2→改写五项构成说明、不新增项 |
| P7 | §3.2 | `J_oa`、`J_ca` 漏 `Σ_k` | 补对预测步的求和 |
| P8 | §3.2 | `J_ca` 分母 `D_act` 与 softplus 内 `D_safe` 写反 | 与 `J_oa` 统一为 `1/(d−D_safe)`、`softplus(k(D_act−d))` |
| P9 | §3.2 | 动态项 `Σ_iΣ_{j=1}^{i}` 上界无意义、缺 `1/d` 因子 | 改为"与扫掠折线最短表面距离"口径，与实现一致 |
| P10 | §3.2 | `ψ_g` 定义缺失（公式空白）；`x_g`/`W_g` 结构未说明 | 补 `x_g=[p_g,ψ_g,0,0]`（v、ω 权重为 0）、`ψ_g=atan2(p_sub−p_N)`、`W_g=diag(w,w,w/2,w_ψ)` 与 0.3 m 冻结规则 |
| P11 | §3.5 | "该约束对速度指令线性"不成立（`n,h` 依赖决策变量） | 写明"冻结 `n,h` 的名义化 CBF"及 `k=0` 精确、`k≥1` 冻结的分层实施 |
| P12 | §3.5 | `γ` **未标单位**（应为 1/s）、离散衰减系数未含 `Δt` | 标 `γ[1/s]`、`β_disc=1−γΔt`，并校验 `1/γ ≥ v_max/a_brake` |
| P13 | §3.4 | `ρ(t)` 只给 `φ(s)`，缺 `ρ_0`、`ε_f` 定义与量纲 | 补 `ρ(t)=ε_f+(ρ_0−ε_f)φ(s)`、`ρ_0` 的取法与 clamp |
| P14 | §4.2 | 只说"求解线性方程组并检查非负"，未给方程与算法 | 补 3×3 线性系统、包含性判据、退化阈值 `A_min` |
| P15 | 全文 | 无参数表、无单周期耗时、无求解成功率 | 补附录参数表（附录 B）+ 性能表（M9 产出） |

### 2.9 需要决策的开放问题（**开工前需确认**）

| 编号 | 决策点 | 选项 | 建议 | 影响模块 |
| --- | --- | --- | --- | --- |
| D1 | 领导者是固定 3 架还是可切换 | 固定 / 动态选举 | **固定**（论文叙事简单，凸包稳定） | M4/M5 |
| D2 | 跟随者是否保留局部规划能力 | 完全不规划 / 保留降级 A* | 先**完全不规划**（符合理论），若窄通道实测撞障再加降级路径（预留开关） | M5 |
| D3 | `J_prog` 口径 | 链式时变参考 / 纯终端子目标 | **链式参考**（保留已调好的行为），纯终端作消融 | M3 |
| D4 | 速度约束形式 | 逐轴盒 / 加范数约束 | **加 `‖v^c‖≤v_max` 可选项**（语义清晰，代价小） | M1/M3 |
| D5 | 动态障碍代价口径 | 逐节点 / 扫掠折线最短距离 | **扫掠折线**（已实现且更稳） | M3 |
| D6 | 跟随者期望 yaw | 领导者 yaw 凸组合 / 速度方向 / 平均 | **领导者 yaw 凸组合**（与编队整体运动一致） | M4/M5 |
| D7 | 标称队形尺寸与形状 | 沿用 2×3 / 新三角形+内部点 | **新队形**（附录 D 给算例） | M4 |
| D8 | CBF 实施方式 | 全非线性 / `k=0` 精确+`k≥1` 冻结 | **分层（后者）**，保留 `cbf_mode` 开关对比 | M6 |
| D9 | 恢复参数默认值 | — | `T=3 s, ε_f=0.15 m, h_enter=0.25, h_exit=0.45, T_dwell=0.5 s, T_clear=0.5 s, e_th=0.2 m`（待整定） | M7 |
| D10 | 避让拓扑 | 全序竞赛图 / 角色化（follower 让 leader） | **角色化**（更符合"领导者承担全局"的叙事，且减少 follower 之间的相互干扰） | M5 |
| D11 | 评测基线 | 仅本文算法 / 加 kinoA*、DWA | 至少加 **kinoA*+MPC**（代码已有）作对比，DWA 历史链路不可用可不做 | M9 |
| D12 | 梯形速度剖面算不算贡献 | 工程细节 / 方法贡献 | 写成"**动力学一致的目标参考生成**"作为方法细节（与 §1 模型自洽是亮点） | M3/M9 |
| D13 | **"五个代价项"的构成 + 跟随者"减少目标距离项"指哪一项**（理论自相矛盾，见 §2.4 引言） | H1：`J_prog` 存在，跟随者用 `J_form` 替换；H2：五项 = `J_oa`(静)+`J_oa`(动)+`J_ca`+`J_sm`+`J_g`，跟随者降低 `J_g` | **需理论作者确认**；在此之前按"两项都保留、跟随者统一降权（×0.3）"实现，两种读法都能跑 | M3/M5/M9 |

---

## 3. 目标架构与接口契约

### 3.1 目标系统数据流（落地后）

```
                 /move_base_simple/goal (仅 leader 订阅)
                            │
        ┌───────────────────┴────────────────────┐
        │  M2 全局层（仅 leader 进程）            │
        │  A* 滚动重规划 → 去锯齿 → 世界系路径    │
        │  发布 /uavX/global_trajectory          │
        └───────────────────┬────────────────────┘
                            │ GlobalTrajectory
        ┌───────────────────┴────────────────────┐
        │  M3 参考生成（每机）                    │
        │  弧长投影 s0 → 梯形速度剖面 → 参考序列   │
        │  leader: 自身路径; follower: 见 M4/M5   │
        └───────────────────┬────────────────────┘
                            │ ref_targets (N+1)×4
   ┌────────────────────────┴─────────────────────────┐
   │  M4 编队几何（每机，离线权重表 + 运行时凸组合）    │
   │  follower: p*_k = Σ m_ij · p_Lj,k  (leader 预测) │
   └────────────────────────┬─────────────────────────┘
                            │ p*_k (follower) / 无 (leader)
   ┌────────────────────────┴─────────────────────────┐
   │  M5 角色与代价装配（每机）                        │
   │  leader: J_oa+J_ca+J_sm+J_g+J_prog               │
   │  follower: J_oa+J_ca+J_sm+J_form                 │
   │            + 0.3*(J_g+J_prog)  # 降权，非删项     │
   └────────────────────────┬─────────────────────────┘
                            │ 符号图
   ┌────────────────────────┴─────────────────────────┐
   │  M6 CBF 约束（每机）+ δ 松弛 + 分级回退           │
   │  M7 恢复监督器（每机）→ ρ_k 时变约束              │
   │  M1 模型/离散化（公共）    M0 参数/诊断（公共）    │
   └────────────────────────┬─────────────────────────┘
                            │ /uavX/mavros/setpoint_velocity/cmd_vel
                            ▼
      20 Hz 广播 /uavX/predicted_path_share（8×(N+1)，world 系）
      ↑ 被 M4（领导者位置凸组合）、M5（J_ca）、M6（机间 CBF）共同消费
```

**关键点**：预测轨迹广播是**一个广播、三处消费**（编队凸组合 / 机间避碰 / 机间 CBF），
因此 M0 必须先把"失败时也发布预测轨迹"（B2）修掉，否则 M4/M6 的输入会周期性断裂。

### 3.2 模块地图与依赖

| 模块 | 名称 | 依赖 | 可并行组 | 主要新增/改动文件 |
| --- | --- | --- | --- | --- |
| **M0** | 工程地基与缺陷修复 | — | P0（最先） | `include/form_nav/diagnostics.h`、`msg/MpcDiagnostics.msg`、`test/`、`tools/` |
| **M1** | 模型与离散化规范化 | M0（弱） | P0 | `include/form_nav/kinematics_model.h`（新） |
| **M2** | 全局层滚动重规划 | M0、M1 | P1 | `src/a_star.cpp`、`src/astar_node.cpp`、`include/form_nav/global_planner.h`（新） |
| **M3** | 局部层代价体系重构 | M0、M1 | P1 | `include/form_nav/cost_terms.h`（新）、`src/mpc.cpp`、`include/form_nav/mpc.h` |
| **M4** | 编队几何（重心坐标） | M0（配置 schema） | **P0 即可开工（纯数学）** | `include/form_nav/formation_geometry.h`（新）、`scripts/formation_barycentric.py`（新）、`config/formations.yaml`（schema 扩展） |
| **M5** | 角色化架构与编队代价 | M2、M3、M4 | P2 | `src/mpc_node.cpp`、`src/formation_config.cpp`、`launch/formation_test.launch` |
| **M6** | CBF 安全约束 | M1、M3、M5（弱） | P2 | `include/form_nav/cbf_constraint.h`（新）、`src/cbf_constraint.cpp`（新） |
| **M7** | 事件触发编队恢复 | M4、M5、M6 | P3 | `include/form_nav/recovery_supervisor.h`（新）、`src/recovery_supervisor.cpp`（新）、`include/form_nav/clearance_monitor.h`（新） |
| **M8** | 集成、调参与实时性 | 全部 | P3 | `launch/*`、参数整定、`docs/` |
| **M9** | 评测与论文实验 | M8 | P4 | `tools/eval/*`（新）、`config/scenes/fixed_scene.yaml`、`docs/fixed_scene_benchmark.md` |

依赖 DAG（→ 表示"必须先完成"）：

```
M0 ──┬─→ M1 ──┬─→ M2 ──┐
     │        ├─→ M3 ──┼─→ M5 ──┬─→ M6 ──┬─→ M7 ──┐
     └─→ M4 ──┴────────┘        │        │        ├─→ M8 ─→ M9
                                 └────────┴────────┘
```

**并行建议**：P0 阶段 3 人 = M0 + M1/M4（纯数学，互不阻塞）；P1 = M2 + M3 + M4 收尾；P2 = M5 + M6；P3 = M7 + M8；P4 = M9。

### 3.3 新增/修改文件清单

| 类型 | 文件 | 归属 | 说明 |
| --- | --- | --- | --- |
| 新增头 | `include/form_nav/kinematics_model.h` | M1 | 模型参数、离散系数、A/B 矩阵、参考推进函数 |
| 新增头 | `include/form_nav/cost_terms.h` | M3 | 代价项注册框架（符号构建 + 参数绑定 + 开关） |
| 新增头 | `include/form_nav/formation_geometry.h` | M4 | 重心坐标、凸组合、退化检测 |
| 新增头 | `include/form_nav/cbf_constraint.h` | M6 | CBF 约束构建（静态/动态、冻结策略、松弛） |
| 新增头 | `include/form_nav/clearance_monitor.h` | M7 | 净空 `h_min` 计算 |
| 新增头 | `include/form_nav/recovery_supervisor.h` | M7 | 恢复状态机 + `ρ_k` 序列 |
| 新增头 | `include/form_nav/global_planner.h` | M2 | 滚动重规划调度器接口 |
| 新增头 | `include/form_nav/diagnostics.h` | M0 | 计时、成功率、约束违例统计 |
| 新增源 | `src/kinematics_model.cpp`、`src/cost_terms.cpp`、`src/formation_geometry.cpp`、`src/cbf_constraint.cpp`、`src/clearance_monitor.cpp`、`src/recovery_supervisor.cpp`、`src/global_planner.cpp`、`src/diagnostics.cpp` | M0/M1/M3/M4/M6/M7/M2 | 实现 |
| 新增脚本 | `scripts/formation_barycentric.py` | M4 | 离线权重求解与校验 |
| 新增脚本 | `tools/eval/run_benchmark.py`、`tools/eval/metrics.py` | M9 | 自动评测 |
| 新增消息 | `msg/MpcDiagnostics.msg` | M0 | 求解耗时/迭代/成功率/违例/状态机状态 |
| 新增测试 | `test/test_kinematics.cpp`、`test/test_formation_geometry.cpp`、`test/test_cbf.cpp`、`test/test_recovery_supervisor.cpp`、`test/test_global_planner.cpp` | 各模块 | gtest |
| 修改 | `src/mpc.cpp`、`src/mpc_node.cpp`、`src/formation_config.cpp`、`src/a_star.cpp`、`src/astar_node.cpp`、`launch/*`、`config/formations.yaml`、`CMakeLists.txt`、`package.xml` | M0–M8 | 见各模块 |

### 3.4 关键 C++ 接口草案（**冻结版，各模块按此实现**）

> 以下签名是实现契约：模块内部可自由重构，但**对外暴露的类名/函数名/语义不得变更**。使用 `Eigen` + `casadi`；不使用 `enum class` 以外的全局状态。

#### 3.4.1 M1 `kinematics_model.h`

```cpp
namespace form_nav {

struct ModelParams {
    int    n_x = 8;       // [x,y,z,psi,vx,vy,vz,omega]
    int    n_u = 4;       // [vx_c,vy_c,vz_c,omega_c]
    double dt = 0.05;     // 预测步长 [s]
    double tau_v = 0.67;  // 线速度时间常数 [s]
    double tau_w = 0.50;  // 角速度时间常数 [s]
    double k_v = 1.0;     // 速度环稳态增益 [-]
    double k_w = 1.0;     // 角速度环稳态增益 [-]
};

// ZOH 精确离散系数（与理论 §1.2 一致）
struct DiscreteCoef {
    double alpha_v, beta_v, gamma_v, delta_v;   // v_{k+1}=a v + b u ; p_{k+1}=p + g v + d u
    double alpha_w, beta_w, gamma_w, delta_w;   // omega / psi 同构
};
DiscreteCoef computeDiscreteCoef(const ModelParams& p);

// 论文附录 P2 要求的显式矩阵
Eigen::Matrix<double, 8, 8> systemMatrixA(const ModelParams& p);
Eigen::Matrix<double, 8, 4> systemMatrixB(const ModelParams& p);

// 沿 1-D 弧长模型推进一步（供参考生成与 CBF 滞后补偿共用）
struct ArcState { double s; double v; };
ArcState propagateArc(const ArcState& x, double u_cmd, const ModelParams& p);

// 角度工具：连续化差值，输出 (-pi, pi]
double wrapAngle(double a);
double angleDiff(double a, double b);

}  // namespace form_nav
```

#### 3.4.2 M4 `formation_geometry.h`

```cpp
namespace form_nav {

enum class Role { LEADER, FOLLOWER };

// 单个跟随者的重心坐标
struct FollowerWeights {
    int                 follower_id = -1;
    std::vector<int>    leader_ids;      // 长度 = 3（三角形）
    std::vector<double> weights;         // 非负，和为 1
};

struct FormationSpec {
    std::vector<int>              leader_ids;
    std::vector<FollowerWeights>  followers;
    double                        min_area = 0.25;   // 三角形最小面积 [m^2]，退化阈值
    std::string                   height_policy = "barycentric";  // barycentric | fixed
    std::string                   yaw_policy    = "barycentric";  // barycentric | velocity
};

class BarycentricFormation {
public:
    // 加载 + 校验（和=1、非负、三角形非退化、follower 严格内部）
    bool load(const FormationSpec& spec, std::string* err);

    Role roleOf(int uav_id) const;

    // 领导者集合的有效性（面积、是否所有领导者都有有效位置）
    bool isDegenerate(const std::map<int, Eigen::Vector3d>& leader_pos) const;

    // 由领导者当前位置/预测位置求 follower 期望位置；退化时返回 false
    bool desiredPosition(int follower_id,
                         const std::map<int, Eigen::Vector3d>& leader_pos,
                         Eigen::Vector3d* out) const;

    // 可选：期望 yaw（领导者 yaw 的凸组合）
    bool desiredYaw(int follower_id,
                    const std::map<int, double>& leader_yaw,
                    double* out) const;

    // 仿射不变性检查用（单测）
    Eigen::MatrixXd weightMatrix() const;   // n_follower x n_leader
};

}  // namespace form_nav
```

#### 3.4.3 M3 `cost_terms.h`

```cpp
namespace form_nav {

// 代价项开关（用于论文消融实验，默认全开）
struct CostTermFlags {
    bool static_obstacle = true;
    bool dynamic_obstacle = true;
    bool inter_uav = true;
    bool control_smooth = true;
    bool terminal = true;
    bool progress = true;
    bool formation = true;
    bool control_input = false;   // 理论未列，默认关
};

class CostTermBuilder {
public:
    explicit CostTermBuilder(const ModelParams& model);

    // 参数化权重（运行期可改，下一次 build 生效）
    void setWeights(const CostWeights& w);     // 见 §3.5.3
    void setFlags(const CostTermFlags& f);

    // 构建：X (8x(N+1)), U (4xN) 符号变量 -> 总代价 MX
    casadi::MX build(const casadi::MX& X, const casadi::MX& U,
                     const CasadiParams& params);   // 障碍点/邻居/参考/编队权重

    // 用于重建判断（M0/B3 修复：包含全部影响符号图的量）
    std::string signature() const;
};

}  // namespace form_nav
```

#### 3.4.4 M6 `cbf_constraint.h`

```cpp
namespace form_nav {

enum class CbfMode { FROZEN, EXACT };      // 名义化冻结 / 直接非线性
enum class ObstacleKind { STATIC_POINT, DYNAMIC_SPHERE, DYNAMIC_CYLINDER };

struct CbfParams {
    double   gamma = 0.65;        // [1/s]
    CbfMode  mode = CbfMode::FROZEN;
    bool     use_lag_comp = true; // 用 v_{k+1} 替代 v_k
    int      nearest_static = 1;  // 每步参与 CBF 的最近静态点数
    double   slack_weight = 1e4;  // delta 的线性罚权重
    double   d_safe = 0.1;        // [m]
    double   r_drone = 0.3;       // [m]
    bool     enable = true;
};

struct CbfObstacle {
    ObstacleKind kind = ObstacleKind::STATIC_POINT;
    Eigen::Vector3d pos;            // 第 k 步位置（world）
    Eigen::Vector3d vel;            // 第 k 步速度（world），静态置零
    double radius = 0.0;            // 球半径 / 圆柱半径
    double height = 0.0;            // 圆柱高度（中心高度在下）
};

struct CbfBuildResult {
    std::vector<casadi::MX> constraints;   // g(w) <= 0 形式
    std::vector<casadi::MX> slacks;        // 对应 delta_k >= 0
    casadi::MX              violation_cost; // sum slack_weight * delta
    int                     num_constraints = 0;
};

class CbfBuilder {
public:
    void setParams(const CbfParams& p);
    void setDroneRadius(double r);

    // X 符号变量 + 每步障碍清单（按 k 索引）
    CbfBuildResult build(const casadi::MX& X,
                         const casadi::MX& U,
                         const std::vector<std::vector<CbfObstacle>>& obstacles,
                         double dt) const;
};

}  // namespace form_nav
```

#### 3.4.5 M7 `recovery_supervisor.h` / `clearance_monitor.h`

```cpp
namespace form_nav {

enum class AvoidState { NORMAL, AVOID, RECOVER, PAUSED, DEGRADED };

struct RecoveryParams {
    double h_enter   = 0.25;  // [m] 进入避障模式
    double h_exit    = 0.45;  // [m] 允许退出（滞回，必须 > h_enter）
    double t_dwell   = 0.50;  // [s] 最小驻留
    double t_clear   = 0.50;  // [s] 净空持续满足窗口
    double big_T     = 3.00;  // [s] 恢复窗口
    double epsilon_f = 0.15;  // [m] 收敛邻域
    double rho_max   = 2.00;  // [m] 约束上界保护
    double e_th      = 0.20;  // [m] 编队误差触发阈值
    double kappa_rho0 = 1.10; // rho_0 = kappa * ||e(t_k)||
};

class RecoverySupervisor {
public:
    void setParams(const RecoveryParams& p);

    // 每控制周期调用一次；h_min 来自 ClearanceMonitor
    void update(double now, double h_min, double formation_error, bool cbf_feasible);

    AvoidState state() const;
    bool        isRecovering() const;

    // 生成本轮预测时域内的时变上界 rho_k (k=0..N)，供 MPC 施加 ||e_k|| <= rho_k
    std::vector<double> rhoSequence(double now, double dt, int N) const;

    // 诊断
    double triggeredAt() const;
    int    triggerCount() const;
};

class ClearanceMonitor {
public:
    // 传入本轮预测轨迹与障碍（静态最近距离 + 动态形状距离），求 min over horizon
    double computeMinClearance(
        const std::vector<Eigen::Vector3d>& predicted_pos,   // N+1
        const std::vector<std::vector<double>>& static_dist,  // N+1 x m
        const std::vector<std::vector<CbfObstacle>>& dynamic_obs) const;
};

}  // namespace form_nav
```

#### 3.4.6 M2 `global_planner.h`

```cpp
namespace form_nav {

struct GlobalPlannerParams {
    double resolution      = 0.1;   // [m]
    int    neighbors       = 26;    // 26 邻域
    double lambda_h        = 1.0;   // 启发权重（1.0 = 可采纳）
    double goal_tol        = 0.2;   // [m]
    double z_min_plan      = 0.6;   // [m] 与 MPC 高度盒对齐
    double z_max_plan      = 2.7;   // [m]
    double perception_radius = 9.0; // [m] 局部重规划窗口
    double replan_period   = 1.0;   // [s]
    double dev_threshold   = 0.5;   // [m] 偏离触发
    int    map_change_threshold = 200; // 新增占据体素数触发
    bool   enable_shortcut = true;
    bool   swept_check     = true;  // 棱边扫掠碰撞检查
};

struct ReplanTrigger {
    bool  replan = false;
    enum Reason { PERIOD, DEVIATION, MAP_CHANGE, GOAL, FIRST } reason = FIRST;
};

class GlobalPlannerScheduler {
public:
    void setParams(const GlobalPlannerParams& p);
    ReplanTrigger shouldReplan(double now, const Eigen::Vector3d& pos,
                               bool goal_updated, int new_occupied_voxels);
    // 在感知窗口内搜索并拼接（保持已走过段不变）
    bool replan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal,
                const GridMap& map, std::vector<Eigen::Vector3d>* path);
};

}  // namespace form_nav
```

### 3.5 配置与参数规范

#### 3.5.1 `config/formations.yaml` 新 schema（向后兼容）

```yaml
# 模式一：旧口径（默认，保持现有实验可复现）
formation_mode: offset
offsets:
  uav0: [-1.0, -2.0, 0.0, 0.0]
  uav1: [-1.0,  0.0, 0.0, 0.0]
  uav2: [-1.0,  2.0, 0.0, 0.0]
  uav3: [ 1.0, -2.0, 0.0, 0.0]
  uav4: [ 1.0,  0.0, 0.0, 0.0]
  uav5: [ 1.0,  2.0, 0.0, 0.0]

# 模式二：重心编队（M4/M5 使用）
# formation_mode: barycentric
# leaders: [uav1, uav3, uav5]          # 必须非共线
# followers: [uav0, uav2, uav4]        # 必须严格在三角形内部
# nominal:                             # 标称队形（world 系相对队形中心的偏移）
#   uav0: [-0.4, -1.0, 0.0]
#   uav1: [-1.2, -2.0, 0.0]
#   uav2: [-0.4,  1.0, 0.0]
#   uav3: [ 1.2,  0.0, 0.0]
#   uav4: [ 0.0,  0.0, 0.0]
#   uav5: [-1.2,  2.0, 0.0]
# weights:                             # 由 scripts/formation_barycentric.py 生成，勿手改
#   uav0: [0.583333, 0.333333, 0.083334]
#   uav2: [0.083334, 0.333333, 0.583333]
#   uav4: [0.25,     0.50,     0.25    ]
# height_policy: barycentric
# yaw_policy: barycentric
# min_area: 0.25

agents:
  uav0: {neighbors: [2, 3, 4, 5]}   # 当前配置值；M5 由角色化生成器覆盖
```

> 附录 D 给出上述队形的完整权重推导（可直接使用）。

#### 3.5.2 参数分组与键名规范（**统一前缀，避免再漂移**）

| 组 | 键名前缀 | 示例 | 说明 |
| --- | --- | --- | --- |
| 模型 | `model/` | `model/tau_v`、`model/dt` | 从 `mpc/` 迁出（保留旧键别名一版） |
| MPC 时域 | `mpc/` | `mpc/N`、`mpc/ipopt_max_iter` | 不变 |
| 代价权重 | `cost/` | `cost/w_oa`、`cost/k_oa`、`cost/detect_range` | 由 `mpc/` 迁出，旧键兼容 |
| 编队 | `formation/` | `formation/w_form`、`formation/min_area` | 新组 |
| CBF | `cbf/` | `cbf/enable`、`cbf/gamma`、`cbf/mode`、`cbf/slack_weight` | 新组 |
| 恢复 | `recovery/` | `recovery/h_enter`、`recovery/big_T`、`recovery/epsilon_f` | 新组 |
| 全局 | `global/` | `global/lambda_h`、`global/perception_radius`、`global/replan_period` | 新组 |
| 诊断 | `diag/` | `diag/enable`、`diag/export_csv` | 新组 |

#### 3.5.3 权重结构体（**M3 落地后的目标值**；当前代码默认与 launch 存在差异，见 T18）

```cpp
struct CostWeights {
    // 静态/动态避障（J_oa）
    double w_oa = 5.0, k_oa = 1.0, detect_range = 0.8, safe_dist = 0.1;
    double w_dyn = 10.0, k_dyn = 10.0, d_bar_dyn = 1.5;
    // 机间避碰（J_ca）
    double w_ca = 5.0, k_ca = 1.0, d_act_ca = 0.6, d_safe_ca = 0.2;
    // 控制平滑（J_sm）
    double w_du = 100.0, w_u = 0.0;
    // 终端与进度（J_g / J_prog）
    double w_subgoal = 1000.0, w_subgoal_int = 100.0, w_yaw = 100.0;
    // 编队（J_form）
    double w_form = 0.0;     // M5 打开；建议 300~800（待整定）
    // 跟随者目标项降权（D13：用权重表达"减少目标距离项"，而不是删项）
    double follower_target_scale = 0.3;
    // 参考生成模式：chain=梯形剖面链式参考（默认） | subgoal=纯终端子目标
    std::string ref_mode = "chain";
    // 质量参数
    double drone_radius = 0.3;
    int    nearest_obs_num = 3;
};
```

### 3.6 ROS 接口契约

| 接口 | 类型 | 状态 | 说明 |
| --- | --- | --- | --- |
| `/uavX/predicted_path_share` | `form_nav/PredictedTrajectory` | **复用，不改** | 8 doubles/步，`dt`、`steps`、world 系；M4/M5（编队）、M5（J_ca）、M6（机间 CBF）共用 |
| `/uavX/global_trajectory` | `form_nav/GlobalTrajectory` | **复用** | 6 doubles/点（速度字段当前被忽略；M2 可填或置零并注明） |
| `/dynamic_obstacles/trajectory` | `form_nav/DynamicObstacleTrajectory` | **复用** | 真值轨迹 → M6 动态 CBF 输入；`traj_steps` 需与 `N` 强校验（异构时域，属 B4 同类） |
| `/move_base_simple/goal` | `geometry_msgs/PoseStamped` | **语义变更** | 仅 leader 订阅；follower 不订阅 |
| `/uavX/mpc/diagnostics` | `form_nav/MpcDiagnostics`（**新增**） | 新 | 求解耗时、迭代数、成功/失败、CBF 违例数、`δ` 和、状态机状态、`h_min`、`ρ` |
| `/uavX/formation/error` | `std_msgs/Float64MultiArray`（可选） | 新 | 编队误差、恢复状态，仅用于可视化/录包 |
| `form_nav/GlobalPathProgress` | msg | **建议废弃** | 定义存在但无人收发；若 M4 需要"领导者路径进度"再启用，否则删除以免混淆 |

`MpcDiagnostics.msg` 草案：

```
std_msgs/Header header
int32  uav_id
string role                    # leader | follower
float64 solve_time_ms
int32   ipopt_iterations
bool    solve_success
int32   cbf_constraints         # 本轮施加的 CBF 约束数
int32   cbf_violations          # 补偿后仍违例的步数（松弛激活）
float64 slack_sum               # sum(delta)
string  avoid_state             # NORMAL/AVOID/RECOVER/PAUSED/DEGRADED
float64 h_min                   # 预测视域最小净空 [m]
float64 rho0                    # 恢复窗口初始上界 [m]
float64 formation_error         # ||e|| [m]
float64 replan_time_ms          # 本轮全局重规划耗时（leader）
```

### 3.7 接口变更记录

| 日期 | 变更 | 影响模块 | 批准 |
| --- | --- | --- | --- |
| 2026-11-04 | v1.0 冻结 | 全部 | — |

---

## 4. 分模块开发步骤

> 每个模块都按 **"目标 → 前置 → 交付物 → 实现步骤 → 验收标准 → 测试 → 风险与回退"** 组织，可单独派给一个开发者。
> 工作量单位为**人日**（含自测，不含联调）。所有模块的共同要求：
> ① 新功能以参数开关合入（默认关）；② 必须提供不依赖 Gazebo 的测试；③ 改动必须更新 `docs/` 对应说明。

---

### M0 工程地基与缺陷修复

**工作量**：5 人日　**依赖**：无（最先开工）　**并行度**：1 人

**目标**：让后续所有模块有一个"可信、可测、可观察"的地基。本模块不实现任何算法新功能，只修 12 项工程缺陷中的 8 项 + 建立测试/诊断设施。

**前置**：无。

**交付物**

| 文件 | 内容 |
| --- | --- |
| `include/form_nav/diagnostics.h` + `src/diagnostics.cpp`（新） | 计时、成功率、违例计数、CSV 导出 |
| `msg/MpcDiagnostics.msg`（新） | 见 §3.6 |
| `src/mpc.cpp`（改） | B1/B3 修复：符号图重建签名 + 参数化权重 |
| `src/mpc_node.cpp`（改） | B2/B4/B6 修复 + 诊断发布 |
| `test/`（新） | gtest 骨架 + 1 个冒烟测试 |
| `tools/offline_replay.py`（新） | bag → CSV → 断言（离线回放） |
| `docs/context.md`（改） | 修正 8 处口径漂移，加"以 launch 为准"声明 |

**实现步骤**

1. **参数规范化（半天）**：按 §3.5.2 加新键（`model/`、`cost/`、`global/` 等），旧键保留一版别名；节点启动时**打印全部生效参数**并写入 `/tmp/form_nav_params_<uav>.yaml`（供论文表格自动生成）。
2. **诊断设施（1 天）**：在 `Mpc::solve()` 入口/出口计时；统计每周期 `solve_time_ms`、IPOPT `iterations`、`success`；在 `mpc_node` 发布 `MpcDiagnostics`（20 Hz）并可选写 CSV（`diag/export_csv`）。
3. **修 B1/B3（1 天）**：把 `buildOpti` 的构建签名改成"版本号"机制——所有影响符号图的量（含 KD-tree 是否就绪、权重、半径、邻居数、动态障碍数）变化时 `++build_version_`，下一周期重建；KD-tree 从空变非空也触发一次重建。
4. **修 B2（0.5 天）**：在"求解失败 / 等待邻居 / 邻居超时"三个分支统一调用 `publishFallbackPrediction()`：优先发布**上一轮预测整体时移 `dt`**（保持运动外推），无历史则发布静止预测。目标：**任何情况下 20 Hz 预测轨迹不断流**。
5. **修 B4（0.5 天）**：邻居回调只校验"同一 world 系 + 数据有限 + `dt>0` + `steps≥2`"，不再要求 `dt`/`N` 相等；对齐按**本机网格**做插值（现有 `interpolateYaw` 代码可复用），越界按首/末列钳位。
6. **修 B6（0.5 天）**：求解后检查 `U(:,0)` 与 `X` 是否有限、`‖u‖` 是否在盒内；异常时沿用上一周期指令并计数；连续 3 次异常 → 零速 hold + 告警。
7. **测试地基（1 天）**：`test/` 下加 `catkin_add_gtest(test_smoke, test/test_smoke.cpp)`；`tools/offline_replay.py` 支持 `--bag x.bag --check metrics.yaml` 输出结论；`scripts/run_all_tests.sh` 一条命令跑完（可选 CI）。
8. **文档对齐（0.5 天）**：修正 τ_v=0.67、a_brake=1.5、`w_dyn`/`m_dyn` 键名、ESDF 表述（改为"KD-tree 最近点 + softplus 势垒"）、`w_subgoal`/`w_u` 当前值、"编队软约束"改为"编队项当前关闭"、删除 `GlobalPathProgress` 或标注"未使用"。

**验收标准**

- [ ] 单机 `single_test.launch` 与 6 机 `formation_test.launch` 均能跑通，行为与改动前一致（同一 bag 的 `U` 序列 RMSE < 1e-6）。
- [ ] `MpcDiagnostics` 发布频率 ≥ 19 Hz，字段有值。
- [ ] **B1 验证**：故意在点云到达前启动 MPC，日志显示"KD-tree 就绪 → 重建符号图"，且 1 个控制周期内静态避障项生效（可对比 `cost` 分解值）。
- [ ] **B2 验证**：杀掉一个邻居节点，其余各机 `/predicted_path_share` 频率仍为 20 Hz（bag 统计）。
- [ ] **B4 验证**：A 机 `N=20/dt=0.05`、B 机 `N=15/dt=0.04` 可互相消费（无"互拒"告警）。
- [ ] `scripts/run_all_tests.sh` 全绿。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| 重建符号图导致单周期耗时抖动 | 重建只在版本变化时发生，且加"重建耗时"打点；必要时重建延后到 solver 空闲周期 |
| 参数键迁移破坏现有 launch | 保留旧键别名读取（`nh.param` 顺序：新键优先，旧键兜底），并打 DEPRECATED 告警 |

---

### M1 模型与离散化规范化

**工作量**：3 人日　**依赖**：M0（弱）　**并行度**：可与 M4 同时开工

**目标**：把"8 维状态 + 4 维控制 + ZOH 精确离散"抽成**唯一公共实现**，同时满足论文附录需要（显式 A、B 矩阵），并解决 ψ 回绕问题。这是 M3/M6 的前置。

**前置**：无（可先写头文件，M0 完成后接入）。

**交付物**：`include/form_nav/kinematics_model.h`、`src/kinematics_model.cpp`、`test/test_kinematics.cpp`、`docs/algo_model.md`（推导，可直接进论文附录）。

**实现步骤**

1. 定义 `ModelParams` / `DiscreteCoef`（§3.4.1），实现 `computeDiscreteCoef`：`α=e^{−dt/τ}`、`β=K(1−α)`、`γ=τ(1−α)`、`δ=K(dt−γ)`；v/ω 两个通道同构。
2. 实现 `systemMatrixA/B`（8×8 / 8×4），并写单测：`A·x+B·u` 与 `propagate` 结果一致（1e-12）。
3. 实现 `propagateArc`（1-D 弧长模型），供 M3 参考生成与 M6 滞后补偿共用。
4. 实现 `wrapAngle`/`angleDiff`；在 MPC 里把终端 yaw 误差改为 `angleDiff`，并加可选平滑形式 `2(1−cosΔψ)`（参数 `cost/yaw_cost_mode=linear|cosine`，默认 linear 保持兼容）。
5. 让 `src/mpc.cpp` 改用本头文件（删除内联系数计算）；参考生成（`mpc.cpp` 的梯形剖面段）也改用它，保证"参考自身满足 MPC 动力学"这一性质**只有一处实现**。
6. 写 `docs/algo_model.md`：连续模型 → ZOH 推导 → A/B 显式表达式 → 系数表（`dt=0.05, τ_v=0.67, τ_w=0.5` 的一组数值）。

**验收标准**

- [ ] 单测：与"细步长数值积分（RK4，步长 dt/100）"对比，单步误差 < 1e-6（相对）。
- [ ] 单测：`A/B` 与 `propagate` 一致。
- [ ] 单测：`angleDiff(3.1, −3.1) ≈ −0.0832`（跨 ±π 正确）。
- [ ] 回归：同一 bag 的 `U` 序列与 M0 版本 RMSE < 1e-6（即"重构成公共头不改行为"）。

**风险与回退**：低。若 `τ_v` 等参数在 `mpc.launch` 与代码默认不一致，以 launch 为准并在 M0 参数打印中暴露。

---

### M2 全局层：A* 滚动重规划

**工作量**：8 人日　**依赖**：M0、M1　**并行度**：可与 M3 同时开工

**目标**：让全局层从"一次性规划"变成理论 §2 描述的"**感知域内滚动重规划**"，同时修 A* 的三类几何缺陷（穿角、重复展开、越界高度），使全局路径**可被 MPC 跟踪**。

**前置**：M0 完成（诊断 + 参数组）。

**交付物**：`include/form_nav/global_planner.h` + `src/global_planner.cpp`（新）、`src/a_star.cpp`（改）、`src/astar_node.cpp`（改）、`src/grid_map.cpp`（改：天花板/高度域）、`test/test_global_planner.cpp`、`launch/astar_grid.launch`（改）。

**实现步骤**

1. **启发与参数（0.5 天）**：`lambda_h` 参数化（默认 1.0）；`getEuclHeu()` 与 octile 都用参数 `global/heuristic=octile|euclid` 选择；论文按 D7 口径统一。
2. **扫掠碰撞检查（1.5 天）**：复用 `src/raycast.cpp` 实现 `isSegmentFree(a,b,map)`；邻居扩展时对 `(cur,next)` 做体素遍历（Bresenham 3D）；`shortcutPath()` 的 LOS 也改为真实扫掠。修复"浮点位置做哈希键"→ 统一用体素索引做键。
3. **高度域与天花板（1 天）**：A* 只在该 `[z_min_plan, z_max_plan]` 内扩展（参数由 MPC 高度盒推出，默认 0.6 / 2.7）；修复 `virtual_ceil_height` 在点云模式下未生效（在 `cloud` 回调的 inflate 前写入天花板占据）；目标 z 改为 `global/cruise_z`（默认 1.0）。
4. **滚动重规划调度器（3 天）**：实现 `GlobalPlannerScheduler`：`shouldReplan()` 按 §3.4.6 的四类触发；`replan()` 只在 `perception_radius` 窗口内搜索，把"已走过段"冻结、把"窗口出口到旧路径"拼接；窗口内不可达时退化为全图规划并告警。发布接口不变（`/uavX/global_trajectory`，latch）。
5. **路径质量（1 天）**：去锯齿后按 `arclength_resample`（默认 0.2 m）重采样，保证 MPC 的 `waypointAtArc` 有足够分辨率；输出附加"每点允许速度"（可选，用曲率推出）。
6. **性能（1 天）**：窗口内 A* 的节点上限 `max_expand`（默认 2e5）；超限则退化为"向目标直线 + 局部绕行"并告警；单次重规划耗时打点。

**验收标准**

- [ ] 固定场景：A* 路径与所有占据体素的最小距离 ≥ 膨胀半径 × 0.95（即不穿角）。
- [ ] 滚动重规划：单次耗时 < 30 ms（感知窗口 9 m、0.1 m 分辨率），P95 < 45 ms。
- [ ] 路径质量：滚动重规划拼接路径长度 ≤ 一次性全图 A* 路径长度 × 1.05。
- [ ] 高度合规：路径所有点 `z ∈ [0.5, 2.8]`，且 MPC 实际飞行高度不越盒。
- [ ] 触发正确性：注入新障碍（脚本在 `/map_generator/global_cloud` 中加一块）→ 1 个周期内触发重规划（日志可见）。
- [ ] 单测：`shouldReplan` 四类触发、`isSegmentFree` 的穿角用例（斜对障碍必须判碰）。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| 局部窗口规划导致路径不自洽（拼接处折角） | 拼接处做一次 LOS 优化；仍不自洽则回退全图规划（首次/低频） |
| 重规划频率高导致 CPU 抖动 | 触发条件加"最小间隔"（≥0.5 s）与"单周期最多一次" |
| 旧实验依赖"一次性规划" | 参数 `global/replan_enable=false` 完全回到旧行为 |

---

### M3 局部层：代价体系重构

**工作量**：8 人日　**依赖**：M0、M1　**并行度**：可与 M2 同时开工

**目标**：把理论 §3.2/§3.3 的**五个代价项**与代码实现一一对齐，把"代价项"变成**可注册、可开关、可参数化**的组件，支撑论文消融实验与在线调参；同时统一三处 softplus 势垒的口径与数值实现。

**前置**：M1（模型公共头）、M0（参数组）。

**交付物**：`include/form_nav/cost_terms.h` + `src/cost_terms.cpp`（新）、`src/mpc.cpp`（重构）、`include/form_nav/mpc.h`（瘦身）、`test/test_cost_terms.cpp`、`docs/algo_cost_terms.md`（公式与代码映射表，可直接进论文）。

**实现步骤**

1. **框架（1.5 天）**：`CostTermBuilder` 提供 `addStaticObstacle/addDynamicObstacle/addInterUav/addControlSmooth/addTerminal/addProgress/addFormation/addControlInput`；每个 term 注册：符号构建函数、参数绑定（`Opti::set_value` 或符号常量）、权重、开关。
2. **五项归并（1 天）**：静态 + 动态 → `J_oa`；机间 → `J_ca`；控制增量 → `J_sm`；终端 → `J_g`；逐点参考 → `J_prog`；编队 → `J_form`（M5 用）；控制输入项作为正则（默认关）。**保持现有数学形式不变**（只重构）。
3. **口径统一（1 天）**：按 T13–T15 统一三处势垒：`(W/max(d_eff−D_safe, ε))·softplus(k(D_act−d_eff))`；动态项按"扫掠折线最短表面距离"；所有项统一对 `k=0..N` 求和，并在文档里给出与论文公式的逐项映射表。
4. **数值稳定（1 天）**：softplus 改 `max(x,0)+log1p(exp(−|x|))`；`d_scale` 下限参数化（`cost/d_scale_floor`，默认 0.01）；加单测：`x=±50` 不溢出且梯度有限。
5. **参数化权重（1 天）**：所有权重/半径/距离阈值进入 `CostWeights` 并纳入 M0 的重建签名（或直接做成 `Opti` 的 `Parameter`，运行期可改）。
6. **参考模式开关（1.5 天）**：`cost/ref_mode=chain|subgoal`。`chain`=现有梯形剖面链式参考（默认）；`subgoal`=单一前瞻子目标 + 终端 J_g（理论字面）。两种都要能跑，用于论文消融（D3）。
7. **消融开关（1 天）**：`CostTermFlags`（§3.4.3）暴露成 ROS 参数；一组 `launch/ablation_*.launch` 方便批量跑。
8. **文档（1 天）**：`docs/algo_cost_terms.md` 写"论文符号 ↔ 代码变量 ↔ 参数键 ↔ 默认值"四列表，作为附录 A 的来源。

**验收标准**

- [ ] **行为等价**：`chain` 模式 + 全项开启时，与 M0 版本在同一 bag 输入下的轨迹偏差 < 5 cm。
- [ ] **参数生效**：`rosparam set cost/w_oa 20` 后下一周期生效（用诊断里的代价分解验证）。
- [ ] **消融矩阵可跑**：8 组开关组合（`static/dynamic/inter_uav/formation` 各关一次 + 全开）均能启动并飞行。
- [ ] **数值稳定**：单测覆盖 softplus 大/小输入、`d_eff→D_safe`（分母下限）、`d_eff=0`。
- [ ] 与论文公式的映射表逐项对照无误（由论文作者 review）。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| 重构引入数值差异导致原有调参失效 | "行为等价"是硬指标；差异来自 softplus 稳定化时，用 `cost/numerics=legacy|stable` 切换 |
| 项数归并后难以定位代价来源 | 诊断消息增加"分项代价"字段（可选 `MpcDiagnostics.cost_breakdown`），或 CSV 导出 |

---

### M4 编队几何：重心坐标与队形设计

**工作量**：6 人日　**依赖**：M0（仅配置 schema）、无算法依赖　**并行度**：★ **P0 阶段即可独立开工（纯数学 + 离线脚本）**

**目标**：把理论 §4 的"**重心坐标凸组合**"落地为一个独立的、可单测的几何库 + 离线权重求解工具 + 一套满足"非退化凸包 + 跟随者严格内部"的新标称队形。**这是 M5/M7 的前置，且完全不需要 ROS/Gazebo 就能开发与验收。**

**前置**：无（只需 §3.4.2/§3.5.1 的接口与 schema 冻结）。

**交付物**：`include/form_nav/formation_geometry.h` + `src/formation_geometry.cpp`（新）、`scripts/formation_barycentric.py`（新）、`config/formations.yaml`（schema 扩展 + 新队形）、`test/test_formation_geometry.cpp`、`docs/algo_formation.md`（推导 + 算例）。

**实现步骤**

1. **离线求解脚本（2 天）**：`scripts/formation_barycentric.py`：
   - 输入：`nominal` 标称位置 + `leaders` 列表；
   - 每个 follower 解 3×3 线性系统（`m0+m1+m2=1` 与 x、y 两式，z 用同权重复核）；
   - 校验：`Σm=1±1e−9`、`m_i ≥ −1e−9`、三角形面积 `≥ min_area`、follower 严格内部（`m_i > eps`）；
   - 输出 YAML 片段（`weights:`）+ 校验报告（含面积、最小权重、条件数）；
   - 支持 `--scale`、`--rotate-deg` 生成不同尺寸/朝向的标称队形（用于论文缩放实验）。
2. **新增标称队形（1 天）**：按 T30 设计 3 领导者非共线 + 3 跟随者内部（附录 D 的算例可直接用），给出 spawn 位置、`offsets`、`nominal`、`weights` 四者一致的配置；同步改 `multi_uav_gazebo.launch` 的 spawn（可选，先用同一批 spawn 位置起飞再由 MPC 收敛）。
3. **C++ 几何库（2 天）**：实现 `BarycentricFormation`：加载校验、`desiredPosition`、`desiredYaw`、`isDegenerate`、`weightMatrix`。全部为纯函数（不依赖 ROS），输入 `std::map<int,Eigen::Vector3d>`。
4. **退化处理（0.5 天）**：面积阈值、领导者缺失、权重非法 → 返回 `false` 并给出原因枚举（供 M5 的 `DEGRADED` 状态使用）。
5. **单测（1 天）**：
   - 包含性：1e4 组随机领导者位姿（三角形非退化），follower 期望位置**全部**在凸包内（用重心坐标非负性或射线法）；
   - 仿射不变性：对领导者施加随机平移/旋转/缩放/剪切，follower 期望位置误差 < 1e−9（相对）；
   - 退化：共线/重合/面积过小输入必须被拒绝；
   - 与 Python 脚本一致性：同一组权重，C++ 与 Python 计算的期望位置误差 < 1e−12。
6. **文档（0.5 天）**：`docs/algo_formation.md` 写 3×3 方程推导、包含性证明（凸组合的性质）、仿射不变性说明、算例数值（附录 D）。

**验收标准**

- [ ] Python 脚本对 100 组随机合法队形全部通过校验，输出权重与 C++ 结果一致。
- [ ] 1e4 随机领导者位姿包含性测试 100% 通过。
- [ ] 仿射不变性误差 < 1e−9。
- [ ] 共线/退化输入被拒绝且错误信息明确（含面积值）。
- [ ] 新队形配置能被 `FormationConfig::load` 正确解析（与 M5 联调前先做解析单测）。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| 新队形与现有 spawn/终点分配不一致，起飞就发散 | 先保持 `formation_mode=offset` 跑通，M5 再切 `barycentric`；spawn 与 nominal 对齐作为 M5 的前置检查项 |
| 三角形太扁（接近共线）导致权重极大 | 脚本对 `min_angle` 也做校验（建议每角 ≥ 20°），不满足则建议调整 nominal |

---

### M5 角色化架构与编队代价

**工作量**：10 人日　**依赖**：M2（leader 才有全局规划）、M3（代价框架）、M4（几何库）　**并行度**：2 人（架构 / MPC 代价可拆开）

**目标**：把"全对等的 6 机"改造成理论文档的"**3 领导者（规划+跟踪）+ 3 跟随者（凸组合跟踪）**"架构，并让编队项真正产生作用力。这是论文贡献"重心编队"的落地主体。

**前置**：M2 的 `GlobalPlannerScheduler`、M3 的 `CostTermBuilder`（含 `addFormation`）、M4 的 `BarycentricFormation`。

**交付物**：`src/formation_config.cpp`（改：role/leaders/weights）、`include/form_nav/mpc.h` + `src/mpc.cpp`（改：`p_star` 参数 + J_form）、`src/mpc_node.cpp`（改：角色分支、订阅拓扑、目标分配、降级状态机）、`launch/formation_test.launch`（重构：参数化 UAV 数量与角色）、`test/test_role_assembly.cpp`、`docs/algo_architecture.md`。

**实现步骤**

1. **配置扩展（1.5 天）**：`FormationConfig` 增加 `formation_mode`、`leader_ids`、`followers`、`weights`、`height_policy`、`yaw_policy`；加载时**交叉校验**（leader 非共线、weights 维数与求和、follower 严格内部、id 不重复、拓扑覆盖），失败即 `ROS_FATAL`。
2. **launch 参数化（2 天）**：把 6 份复制粘贴的 `<group ns="uavX">` 改成"参数化生成"（ROS1 用 `xacro` 或 `rosparam` + 循环脚本；推荐 `roslaunch` + `<arg name="uav_ids" ...>` + 显式 6 组但统一 `role`/`my_id` 传参）。删除 `single_test.launch` 里 `<param>`/`<arg>` 误用。新增 `uav_role` 参数（`leader|follower`）。
3. **follower 期望位置注入（2 天）**：MPC 新增参数 `P_star`（3×(N+1) 的 OX 参数，world 系）；`mpc_node` 每周期用 `BarycentricFormation` + 各 leader 的**对齐后预测位置**计算 `P_star` 并 `set_value`；`J_form=Σ_k (p_k−p*_k)^T W_f (p_k−p*_k)`（3 维位置，yaw 通道可选）。
4. **两套代价装配（1.5 天）**：`role=leader` → `J_oa+J_ca+J_sm+J_g+J_prog`；`role=follower` → `J_oa+J_ca+J_sm+J_form`，并把 `J_g`/`J_prog` **降权**（默认 ×0.3，参数 `formation/follower_target_scale`）而不是删项——这样 D13 的两种读法都能用同一份代码表达，论文按裁决结果描述。`cost/formation/w_form` 默认给 300（follower 用），leader 侧不施加。
5. **订阅拓扑（1 天）**：follower **必须**订阅全部 leader（编队框）+ 角色化避让边；leader 订阅属于它的 follower（避让）+ 其它 leader（J_ca）；拓扑由 `formation_config` 在加载时**按角色生成**并写日志，不再手写 `neighbors`（手写值作为 override 保留）。
6. **目标分配（0.5 天）**：只有 leader 订阅 `/move_base_simple/goal`，终点 = goal + leader 自身的 `nominal` 偏移（`nominal` 的质心对齐 goal）；follower 不订阅、不跑 A*。
7. **降级状态机（1 天）**：任一 leader 预测轨迹超时 > `T_lost`（0.5 s）或三角形退化 → `DEGRADED`：① 冻结 `P_star`（用最后一次有效值）；② 打告警与诊断；③ `T_lost` 超过 2 s → 退化为 `offset` 模式（用自身 A* 的降级路径，需 M2 保留可选 A* 进程）。
8. **联调与验收（0.5 天）**：更新 `formation_test.launch` 文档与 `docs/algo_architecture.md`（含角色图、拓扑图、降级流程）。

**验收标准**

- [ ] 6 机固定场景：全程无碰撞到达，`follower` 相对领导者凸包的重心坐标始终非负（录包统计 100%）。
- [ ] 稳态编队误差（`‖p_i−p*_i‖`）< 0.2 m（巡航段），到达后 < 0.15 m。
- [ ] 缩放实验：`--scale 0.8 / 1.0 / 1.2` 三组，follower 自动跟随（无需改代码），编队形状保持。
- [ ] 旋转实验：goal 方向改变 90°，编队朝向随之旋转（余量：当前队形不随航向旋转，若论文要"随航向"需在 M4 增加 `rotate_with_heading`，见风险）。
- [ ] 降级：kill 一个 leader，其余 5 机进入 `DEGRADED` 并在 2 s 内切换，无碰撞；恢复 leader 后可回到 `NORMAL`。
- [ ] leader 不再运行 A*（`rostopic list` 无 follower 的 `global_trajectory`）。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| follower 不规划，窄通道被领导者"甩"进障碍 | ① 先做开环验证（领导者路径已知）；② 若实测撞障，启用 D2 的降级 A*（follower 保留一个低频 A* 只做"避障逃逸"，不跟踪）；③ 或提高 follower 的 J_oa 权重 |
| 领导者预测轨迹含自身避障动作，follower 期望随之抖动 | 对 `P_star` 做一阶低通（`p_star_filt = a*p_star + (1−a)*p_star_prev`，`a≈0.3`）并记录；论文注明"期望轨迹滚动更新 + 滤波" |
| launch 参数化引入回归 | 保留旧 6 份 group 的 launch 作为 `formation_test_legacy.launch` 对照 |
| "编队随航向旋转"与理论 §4.2 的仿射不变性不完全等价 | 仿射不变性只保证"领导者做仿射变换时 follower 同步"，**不保证**"队形随航向自动旋转"。若论文要后者，在 nominal 里加朝向旋转（`rotate_with_heading=true`，用领导者连线方向定义局部系）——列为 M4 可选增强 |

---

### M6 CBF 安全约束（双保险的硬约束层）

**工作量**：12 人日（最难）　**依赖**：M1、M3、M5（弱，机间 CBF 需要角色拓扑）　**并行度**：1~2 人

**目标**：把理论 §3.5 的"softplus 软引导 + CBF 硬约束"双层避障结构落地，包括**执行器滞后补偿**、**松弛与分级回退**、以及与软势垒的协同整定。这是论文贡献"双层避障"的核心。

**前置**：M1（`propagate`/系数）、M3（代价框架与障碍参数装配，CBF 复用同一批障碍查询结果）。

**交付物**：`include/form_nav/cbf_constraint.h` + `src/cbf_constraint.cpp`（新）、`src/mpc.cpp`（改：装配约束、松弛、回退）、`src/mpc_node.cpp`（改：CBF 参数、违例统计、分级回退）、`launch/cbf.launch`（新）、`test/test_cbf.cpp`、`docs/algo_cbf.md`。

**实现步骤**

1. **距离型障碍函数与法向（2 天）**：实现 `h(p,o)` 与 `n(p,o)` 的三个分支：静态点云点（`r_i=0`）、球（`r_i`）、圆柱（半径 + 高度，分段精确表面距离）。注意 `h` 必须**与 M7 的 `ClearanceMonitor` 用同一个函数**（保证"避障模式判据"与"安全约束"自洽）。
2. **分层线性化策略（2 天）**：按 T21：
   - `k=0`：`p_0=x(t_k)` 为测量值 → `n_0,h_0` 是常数 → 严格线性约束（真 CBF）；
   - `k≥1`：用上一轮解（或首次用参考）计算 `n_k,h_k` 并**冻结为常数**（`Opti` 参数），在 `replan_max_iter` 循环中随新解刷新；
   - `cbf/mode=exact` 备选：直接用 `p_k` 的符号表达式（非线性），仅用于对比实验。
3. **滞后补偿（1.5 天）**：`v_{k+1}=alpha_v v_k+eta_v u_k`（`K_v=1`）代入，得到对 `u_k` 仿射的约束；离散衰减用 `β_disc=1−γΔt`（参数 `cbf/gamma`，默认 0.65 1/s；`cbf/decay=linear|exp`）。**约束统一写成** `n_k^T(v_{k+1}−v^o_k) + γ·h_k ≥ −δ_k`。
4. **松弛与分级回退（2 天）**：每条 CBF 约束配 `δ_k≥0`（`slack_weight=1e4` 线性罚）。求解失败或 `Σδ` 超阈值时按优先级降级：① 放松机间 CBF → ② 放松动态 CBF → ③ 放松静态 CBF（保持软势垒）→ ④ 零速 hold。每级打点上报 `MpcDiagnostics.avoid_state/cbf_violations`。
5. **与软势垒协同（1.5 天）**：新增 `cbf/soft_scale`（默认 0.3）：CBF 开启时把三处 softplus 权重按此系数缩放；并给出"仅软 / 软+硬 / 仅硬"三组对照参数。
6. **实时性（2 天）**：约束数控制：`cbf/stride`（每 1/2/4 步施加一次，默认 1）、`nearest_static`（默认 1）。实测单周期耗时，若不满足预算（<50 ms）则提高 stride。**必须先有 M0 的计时代码**。
7. **验证与统计（1 天）**：脚本统计：最小净空分布、CBF 违例率（`δ>0` 的步数占比）、碰撞次数、求解成功率、耗时 P50/P95；输出 CSV 供论文作图。

**验收标准**

- [ ] **安全性**：固定场景（含 2 个动态障碍）连续 100 次飞行：碰撞 0 次；最小净空 ≥ `safe_dist` 的占比 ≥ 99%；`δ>0` 的步数占比 ≤ 1%。
- [ ] **实时性**：平均单周期求解 ≤ 35 ms，P95 ≤ 50 ms（20 Hz 可运行，允许 ≤1% 丢帧）。
- [ ] **有效性对照**：与"仅 softplus"版本相比，最小净空（5% 分位）提升 ≥ 20%，且路径长度增加 ≤ 10%。
- [ ] **无副作用**：`cbf/enable=false` 时行为与 M3 完全一致（RMSE < 1e-6）。
- [ ] **单测**：`h/n` 的三个形状分支、`k=0` 线性性（对 `u` 求导为常数）、`β_disc` 数值、圆柱分段连续性。
- [ ] **故障注入**：人为让 CBF 不可行（在机头上方放障碍）→ 分级回退触发且不 crash，日志可追溯。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| 冻结 `n` 导致"名义 CBF"在高动态场景失效 | 提高 `replan_max_iter`（每周期多解 1 次）；论文如实表述为"序列线性化 CBF"，并用实验证明 100 次无碰撞 |
| 约束数过多导致求解耗时爆炸 | `stride` 降采样；或只对 `k ∈ {0, N/2, N}` 施加硬约束、其余用软势垒 |
| 与 M5 编队项冲突（编队拉拢 vs 安全推开） | 优先级：CBF > 编队；`J_form` 权重在 `AVOID` 状态下按比例降为 0.3（参数化） |
| CBF 不可行导致频繁 hold | 分级回退 + `δ` 罚；并给 `safe_dist` 与 `detect_range` 的整定建议（`safe_dist ≥ 0.1`、`D_act ≥ safe_dist + 2·v_max·Δt`） |

---

### M7 事件触发指定时间编队恢复

**工作量**：8 人日　**依赖**：M4、M5、M6（`h` 与 CBF 共用）　**并行度**：1 人

**目标**：实现理论 §3.4。**动机**（也是论文的实验依据）：避障结束后，编队误差只能按跟踪项的**固定权重渐近收敛**，恢复时间随形变量增大而增加——这正是"为什么需要事件触发的时变约束"。具体为：避障导致编队形变后，在**净空恢复事件**触发下，用**时变约束** `‖e(t)‖≤ρ(t)` 使编队误差在 `T` 内收敛到 `ε_f` 邻域，并避免 Zeno 与误触发。

**前置**：M6 的 `h` 函数（与 CBF 同一实现）、M5 的 `P_star`（编队误差 `e=p−p*` 已有）。

**交付物**：`include/form_nav/clearance_monitor.h` + `src/clearance_monitor.cpp`（新）、`include/form_nav/recovery_supervisor.h` + `src/recovery_supervisor.cpp`（新）、`src/mpc.cpp`（改：时变约束 `‖e_k‖²≤ρ_k²`）、`src/mpc_node.cpp`（改：状态机装配、诊断）、`launch/recovery.launch`（新）、`test/test_recovery_supervisor.cpp`、`docs/algo_recovery.md`。

**实现步骤**

1. **净空监测（1.5 天）**：`ClearanceMonitor::computeMinClearance`：用**上一轮预测轨迹** `p_k`，逐 `k` 查 KD-tree 最近点距离 + 动态障碍表面距离，套 `h` 定义（与 M6 同函数），取全时域最小值。注意：CBF 用的是"每步最近点"，这里要取"最小 `h` 的障碍"，两者口径一致（同一函数）。
2. **避障状态机 + 滞回（2 天）**：`NORMAL→AVOID`（`h_min<h_enter`）；`AVOID→` 满足 `h_min>h_exit` 持续 `T_clear` 且驻留 `≥T_dwell` → 触发恢复（进入 `RECOVER`）；`RECOVER` 中 `h_min<h_enter` → `PAUSED`（**时钟清零**，保守处理）；`PAUSED` 中再次满足条件 → 重新锁存 `t_k`。
3. **触发时刻锁存（1 天）**：`t_k=now`（ROS 时钟，非消息时间）；要求 `‖e(t_k)‖>e_th`（形变确实存在），否则不触发（直接 `NORMAL`）。
4. **`ρ` 序列生成（1.5 天）**：`ρ(t)=ε_f+(ρ_0−ε_f)φ(s)`，`φ(s)=1−3s²+2s³`，`s=clamp((t−t_k)/T,0,1)`；`ρ_0=max(κ·‖e(t_k)‖, e_th)` 再 clamp 到 `[ε_f, ρ_max]`（与 T38 统一）。每周期生成 `ρ_k, k=0..N`（`t=t_k+kΔt`）。
5. **时变约束装配（1 天）**：`‖e_k‖²≤ρ_k²`（`ρ_k` 为常数参数，光滑）；只在 `RECOVER` 状态施加；失配/不可行时按 M6 的回退链降级（编队约束最先放松）。
6. **参数整定与 Zeno 抑制（0.5 天）**：`T_dwell` 与 `T_clear` 保证最小触发间隔；统计 `triggerCount()` 与触发间隔分布。
7. **指标与可视化（0.5 天）**：发布 `formation_error`、`ρ(t)`、`h_min`、状态机状态（诊断消息 + RViz Marker）；CSV 记录供论文画"约束收缩曲线 + 误差曲线"。
8. **文档（0.5 天）**：`docs/algo_recovery.md` 写 `ρ` 构造、状态迁移图、参数表、以及"为什么 `t_k` 用真实时间锁存"。

**验收标准**

- [ ] **扰动恢复实验**：人为把 follower 拉开 0.5 / 1.0 / 1.5 m 三种幅度，各 20 次：恢复时间 ≤ `T`×1.1，最终 `‖e‖≤ε_f` 的比例 ≥ 95%。
- [ ] **Zeno 抑制**：单次飞行 `triggerCount ≤ 1`（除非确有第二次形变）；触发间隔 ≥ `T_dwell+T_clear`。
- [ ] **暂停逻辑**：恢复中撞上障碍 → 进入 `PAUSED`、时钟清零，5 次实验全部正确。
- [ ] **对照**：与"无恢复机制（仅 J_form 固定权重）"相比，过障后编队误差收敛时间缩短 ≥ 50%。
- [ ] **不影响安全性**：开启恢复后 M6 的安全指标不退化（最小净空、碰撞次数不变差）。
- [ ] **单测**：`ρ` 序列在 `s=0/1` 处等于 `ρ_0/ε_f`、`φ` 的 C1 性质（数值导数连续）、状态机全部迁移路径（含异常顺序）。

**风险与回退**

| 风险 | 处置 |
| --- | --- |
| `ρ_0` 取小导致立即不可行 | `ρ_0` 用 `κ=1.1` 上包络并 clamp；不可行时自动放大 `ρ_0`（记录日志） |
| `T < N·Δt` 或 `T` 远大于视界 | `T` 默认 3 s（远大于 1 s 视界）→ 每周期只覆盖窗口的前 1 s，滚动推进（这是设计意图）；`T` 参数化 |
| 恢复约束与 CBF 打架 | 优先级 CBF > 恢复；恢复约束最先被放松 |
| 论文声称"指定时间收敛"过强 | 按 T41 降级表述，用统计数据支撑 |

---

### M8 集成、调参与实时性

**工作量**：10 人日　**依赖**：M0–M7 全部　**并行度**：2 人（集成 / 调参）

**目标**：让 10 个模块在 6 机固定场景上**稳定联跑**，完成参数整定，满足实时性预算，并形成可复现的一键启动。

**交付物**：`launch/formation_test_v2.launch`（新总入口）、`config/params/*.yaml`（分场景参数集）、`docs/algo_integration.md`（启动/复现/故障码）、`docs/known_issues.md`。

**实现步骤**

1. **总入口（1.5 天）**：一个 launch 按 `formation_mode`、`cbf/enable`、`recovery/enable`、`global/replan_enable` 组合启动；`roslaunch form_nav formation_test_v2.launch` 默认 = 论文配置。
2. **分阶段整定（4 天）**：严格按"单机 leader → 3 领导者 → 3+3 编队 → 开 CBF → 开恢复"五阶段调参，每阶段冻结参数并记录；给出每阶段的参数表与调参建议顺序（权重 > 距离阈值 > 时域）。
3. **实时性（2 天）**：统计每周期各环节耗时（全局重规划 / 障碍查询 / 符号构建 / IPOPT / 发布）；瓶颈定位后决定 `N`、`cbf/stride`、`replan` 频率。目标：20 Hz 稳定，丢帧 ≤ 1%。
4. **故障矩阵（1.5 天）**：列出并逐条验证：邻居超时、leader 失联、求解失败、CBF 不可行、地图为空、目标未收到、时间戳跳变、bag 回放（时钟）。每条给"预期行为 + 观测点 + 恢复条件"。
5. **一键复现（1 天）**：`scripts/run_scenario.sh`（启动 + 发布目标 + 录包 + 自动停止）、参数快照自动落盘、bag 命名规范。

**验收标准**

- [ ] 连续 20 次固定场景飞行无人工干预（每次 ≥ 120 s）。
- [ ] 20 Hz 稳定，`solve_time` P95 ≤ 45 ms。
- [ ] 故障矩阵 8 条全部通过。
- [ ] 一键脚本可复现。

---

### M9 评测与论文实验

**工作量**：8 人日　**依赖**：M8　**并行度**：1 人

**目标**：把"能跑"变成"能出证据"：固定场景可复现、指标自动统计、消融与对比实验一键跑完、图表可直接进论文。

**交付物**：`tools/eval/run_benchmark.py`、`tools/eval/metrics.py`、`tools/eval/plot.py`（新）、`config/scenes/fixed_scene.yaml`（与文档对齐）、`docs/fixed_scene_benchmark.md`（改：场景口径修正 + 评测流程）、`results/`（CSV + 图）。

**实现步骤**

1. **场景口径修复（1 天）**：`obs_generator.launch` 默认改回 `fixed_scene.yaml`；`formation_test.launch` 透传 `scene_file`；把 `fixed_scene.yaml` 与文档对齐（要么改 YAML 回 30 个无障碍墙，要么改文档；建议**以 YAML 为准并更新文档**，同时保留一个 `paper_scene.yaml` 作为论文固定场景并冻结 hash）。
2. **指标脚本（2 天）**：从 bag/CSV 计算：到达时间、路径长度、路径平滑度、最小净空（5% 分位）、编队误差（均值/最大/稳态）、恢复时间、CBF 违例率、求解耗时 P50/P95、控制抖动。输出统一 CSV。
3. **自动跑批（2 天）**：`run_benchmark.py --config matrix.yaml` 顺序跑多组参数/开关，自动录包、解析、汇总。
4. **消融实验（1.5 天）**：至少 6 组：① 基线（仅软势垒 + 全对等）② +重心编队 ③ +CBF ④ +恢复 ⑤ 全开 ⑥ 参照（kinoA*）。每组 10 次。
5. **图表（1.5 天）**：编队误差曲线（含 `ρ` 包络）、净空分布箱线图、恢复时间分布、耗时直方图、轨迹对比图。

**验收标准**

- [ ] 同一配置两次运行的指标差异 < 5%（确定性场景下应 < 1%）。
- [ ] 6 组消融结果完整，能支撑论文的每个贡献点（每个贡献至少一个显著指标）。
- [ ] 论文所需图表全部由脚本生成，不手工改数。

---

## 5. 里程碑与排期

### 5.1 阶段划分（3 人并行，约 7 周）

| 阶段 | 周期 | 内容 | 并行分工 | Go / No-Go 判据 |
| --- | --- | --- | --- | --- |
| **P0 地基** | 第 1 周 | M0 + M1 + M4（数学部分） | A: M0　B: M1　C: M4 | 单机/6 机跑通且行为不变；M4 单测通过；诊断可用 |
| **P1 双轨** | 第 2–3 周 | M2 + M3 + M4 收尾 | A: M2　B: M3　C: M4+M5 准备 | 滚动重规划可用；代价项可开关；重心库接入配置 |
| **P2 架构** | 第 3–5 周 | M5 + M6 起步 | A: M5 架构　B: M5 代价　C: M6 CBF | 6 机角色化跑通（编队误差 < 0.2 m）；CBF 单步可用 |
| **P3 安全与恢复** | 第 5–6 周 | M6 收尾 + M7 | A: M6　B: M7　C: M8 集成 | 100 次无碰撞；恢复时间 ≤ T×1.1 |
| **P4 出证据** | 第 7 周 | M8 收尾 + M9 | 全员 | 一键复现；6 组消融完成；图表出图 |

### 5.2 关键路径

`M0 → M1 → M3 → M5 → M6 → M7 → M8 → M9`（约 6 周）；`M4` 可完全并行（P0 即开工），`M2` 与 `M3` 并行。

### 5.3 里程碑交付检查单

- [ ] **P0 出口**：`scripts/run_all_tests.sh` 通过；`MpcDiagnostics` 可用；12 项缺陷中 B1–B6、B11、B12 已关闭。
- [ ] **P1 出口**：A* 滚动重规划 + 代价开关上线；`formation_barycentric.py` 可生成权重；`ref_mode` 双模式可切。
- [ ] **P2 出口**：3 leader + 3 follower 飞通；`P_star` 注入正确；CBF 约束数/耗时满足预算。
- [ ] **P3 出口**：100 次安全性统计达标；恢复状态机全部迁移覆盖。
- [ ] **P4 出口**：论文图表齐全；文档同步（`context.md`、`fixed_scene_benchmark.md`、`algo_*.md`）。

---

## 6. 测试与验证方案

### 6.1 三层测试体系

| 层 | 手段 | 覆盖 | 频率 |
| --- | --- | --- | --- |
| 单元测试 | gtest（`catkin_add_gtest`） | M1 模型、M4 几何、M6 CBF 数学、M7 状态机、M2 调度/扫掠 | 每次提交 |
| 离线回放 | `tools/offline_replay.py`（bag → CSV → 断言） | 回归（行为等价）、指标提取 | 每次 PR |
| 全栈仿真 | Gazebo 6 机 + 固定场景 | 安全性、编队、恢复、实时性 | 每模块出口 + 每日 |

### 6.2 必须建立的"回归基线"

1. **行为基线**：M0 完成时用当前代码跑 3 个场景（单机固定场景 / 6 机固定场景 / 6 机含动态障碍）录 3 个 bag，作为后续所有重构的对照（判定标准：`U` 序列 RMSE、到达时间、最小净空）。
2. **参数快照**：每次实验自动保存生效参数到 `results/<run_id>/params.yaml`。
3. **场景哈希**：`fixed_scene.yaml` 与 `paper_scene.yaml` 计算 SHA256 写入结果目录，保证"同一场景"可核验。

### 6.3 安全性验证（CBF 相关必须做）

| 实验 | 方法 | 通过判据 |
| --- | --- | --- |
| 静态密集障碍 | 固定场景 × 50 次 | 碰撞 0；最小净空 5% 分位 ≥ 0.05 m |
| 动态障碍 | 固定场景 + 2 动态 × 50 次 | 碰撞 0；`δ>0` 占比 ≤ 1% |
| 狭窄通道压力测试 | 人为把通道缩到 0.9 m | 不碰撞；允许降级但需上报 |
| 不可行注入 | 机头正前方 0.5 m 放障碍 | 分级回退生效，不 crash |

### 6.4 编队与恢复验证

| 实验 | 方法 | 通过判据 |
| --- | --- | --- |
| 编队保持 | 巡航段 | `‖e‖` 均值 < 0.2 m，最大 < 0.4 m |
| 包含性 | 全程 | 重心坐标 min ≥ −1e−6（100% 时刻） |
| 形变恢复 | 过障后 | 恢复时间 ≤ 1.1T；末态 `‖e‖≤ε_f` 占 95% |
| 缩放/旋转 | `scale=0.8/1.0/1.2`；goal 方向 ±90° | 形状保持（边长比误差 < 10%） |

---

## 7. 风险登记表

| 编号 | 风险 | 概率 | 影响 | 缓解 | 触发后的回退 |
| --- | --- | --- | --- | --- | --- |
| R1 | CBF 导致求解频繁不可行，飞行不稳定 | 中 | 高 | δ 松弛 + 分级回退 + 降采样 | `cbf/enable=false`，退回软势垒（已验证可用） |
| R2 | follower 无规划在窄通道撞障 | 中 | 高 | 先用开环路径验证；提高 follower `J_oa` | 启用 follower 低频"逃逸 A*"（D2 备选） |
| R3 | 领导者失效导致编队框崩溃 | 中 | 中 | `DEGRADED` 状态 + 冻结 `P_star` | 退化 `offset` 模式并告警 |
| R4 | 实时性不达标（CBF+编队约束过多） | 中 | 高 | stride 降采样、缩短 N、只对关键步施加硬约束 | 降 `N` 到 15 或 CBF 只保留 `k=0` |
| R5 | 理论公式与实现口径反复漂移 | 高 | 中 | 本文件的映射表（附录 A）+ 参数快照 + 论文 review 机制 | 每模块出口强制更新映射表 |
| R6 | 场景不可复现导致结论失真 | 中 | 高 | 场景哈希 + 参数快照 + 自动评测 | 冻结 `paper_scene.yaml` |
| R7 | 冻结 `n` 的 CBF 被审稿人质疑"不是真 CBF" | 高 | 中 | 如实表述为序列线性化/名义化 CBF，补充 100 次统计 | 提供 `cbf_mode=exact` 的对比实验数据 |
| R8 | 模块间接口变更破坏并行开发 | 中 | 中 | §3 冻结 + 变更记录 | 版本号 + 兼容层 |

---

## 8. 附录

### 附录 A　理论公式 → 代码 → 参数 映射表

| 理论项（§） | 公式要点 | 代码位置 | 参数键 | 状态 |
| --- | --- | --- | --- | --- |
| §1.1 连续模型 | `ṗ=v, v̇=(K_v v^c−v)/τ_v, ψ̇=ω, ω̇=(K_ωω^c−ω)/τ_ω` | `src/mpc.cpp` 模型段 → M1 迁至 `kinematics_model` | `model/tau_v, tau_w, k_v, k_w` | 已实现 |
| §1.2 ZOH 离散 | `x_{k+1}=Ax_k+Bu_k` | 同上（α/β/γ/δ） | `model/dt` | 已实现，M1 补 A/B |
| §2.1 A* | `f=g+λ_h h`，26 邻域，r=0.1 | `src/a_star.cpp` | `astar/lambda_heu, resolution` | 部分（启发为 octile） |
| §2.1 通行性 | 膨胀占据栅格 | `src/grid_map.cpp` | `map/inflation` | 已实现 |
| §2.2 去锯齿 | LOS 直线连接 | `shortcutPath()` | `global/enable_shortcut` | 已实现（LOS 有 0.05 m 采样检查；邻居扩展缺扫掠） |
| §2 滚动重规划 | 感知域内重规划 | **无** | `global/replan_period` | **缺失（M2）** |
| §3.1 参考点 | 弧长投影 `s_0` + 前瞻子目标 + yaw + 0.3 m 冻结 | `mpc.cpp` 参考段 | `mpc/yaw_lock_dist` | 已实现（前瞻已改为链式剖面） |
| §3.2 J_oa 静态 | `Σ_nΣ_q (1/(d−D_safe))softplus(k(D_act−d))`（`n`=预测步、`q`=最近点） | `kdtreeObstacleAvoidance` | `cost/w_oa, k_oa, detect_range, safe_dist` | 已实现（论文缺对预测步的求和表述） |
| §3.2 J_oa 动态 | 动态障碍词 | `dynamicObstacleAvoidance` | `cost/w_dyn, k_dyn, d_bar_dyn` | 已实现，口径与论文不同（T15） |
| §3.2 J_ca | 机间 softplus | `collisionAvoidance` | `cost/w_ca, k_ca, d_act_ca, d_safe_ca` | 已实现（论文分母笔误 T14） |
| §3.2 J_sm | `Σ Δu^T W_sm Δu` | `buildOpti` 控制增量段（k=0..N−2，与理论一致） | `cost/w_du` | 已实现 |
| §3.2 J_g | `(x_N−x_g)^T W_g(x_N−x_g)` | 终端参考项 | `cost/w_subgoal, w_yaw` | 已实现（`ψ_g` 待定义） |
| §3.2 J_prog | "目标距离代价项" | 中间步逐点参考 | `cost/w_subgoal_int` | 已实现（口径为链式参考，T17） |
| §3.2 制动约束 | `‖v_k‖²≤2a_brake‖p_k−p_g‖` | `buildOpti` 内联制动段（平方化实现） | `mpc/a_brake` | 已实现（直线距离口径） |
| §3.3 J_form | `Σ_{n=1}^{N}(p_n−p*_n)^T W_f(p_n−p*_n)` | `formationError`（共识式） | `cost/w_form` | **未启用**，M5 改为重心式（T42） |
| §3.2 J_prog | 理论未给公式（D13 待裁决） | 中间步逐点参考 | `cost/w_subgoal_int` | 已实现（T43，按 D13 填写论文） |
| §3.4 事件触发恢复 | `h_min` 滞回 + `‖e‖≤ρ(t)` | **无** | `recovery/*` | **缺失（M7）** |
| §3.5 CBF | `n^T(v_{k+1}−v^o)+γh≥0` | **无** | `cbf/*` | **缺失（M6）** |
| §4.2 重心编队 | `p*_{Fi}=Σ_j m_{ij}p_{Lj}` | **无** | `formation/weights` | **缺失（M4/M5）** |

### 附录 B　参数总表（**唯一来源 = launch/YAML**）

> 代码默认值必须与 launch 对齐（M0）。符号 `†` 表示论文文档未给出、需要补的参数。

| 组 | 参数 | 单位 | 当前 launch | 推荐/说明 | 对应理论 |
| --- | --- | --- | --- | --- | --- |
| 模型 | `N` | 步 | 20 | 20（1 s 视界）；CBF 加重后可试 15 | † |
| 模型 | `dt` | s | 0.05 | 0.05 | † |
| 模型 | `tau_v` | s | 0.67 | 实测辨识；论文标定 | τ_v |
| 模型 | `tau_w` | s | 0.5 | — | τ_w |
| 模型 | `k_v/k_w` | — | 1.0 | — | K_v, K_ω |
| 控制 | `u_max` | m/s | 2.0 | 逐轴盒；建议另加范数约束 | † |
| 控制 | `w_max` | rad/s | 2.0 | — | † |
| 避障 | `w_oa` | — | 5.0 | CBF 开启后乘 `soft_scale=0.3` | W_s |
| 避障 | `k_oa` | 1/m | 1.0 | — | k_s |
| 避障 | `detect_range` | m | 0.8 | ≥ safe_dist+2v_maxΔt≈0.3 | D_act |
| 避障 | `safe_dist` | m | 0.1 | — | D_safe |
| 避障 | `nearest_obs_num` | 个 | 3 | — | m=3 |
| 动态 | `w_dyn` | — | 10.0 | CBF 后同样缩放 | W_d |
| 动态 | `k_dyn` | 1/m | 10.0 | — | k_d |
| 动态 | `d_bar_dyn` | m | 1.5 | — | D_act(dyn) |
| 机间 | `w_ca` | — | 5.0 | — | W_c |
| 机间 | `d_act_ca` | m | 0.6 | 全互连时收紧 | D_act(ca) |
| 机间 | `d_safe_ca` | m | 0.2 | — | D_safe(ca) |
| 平滑 | `w_du` | — | 100 | yaw 通道 /4 | W_sm |
| 终端 | `w_subgoal` | — | 1000 | — | W_g |
| 终端 | `w_yaw` | — | 100 | — | W_g(yaw) |
| 终端 | `w_subgoal_int` | — | 100 | — | W_prog |
| 编队 | `w_form` | — | **0.0** | follower 建议 300~800 | W_f |
| 编队 | `follower_target_scale` | — | — | 跟随者对 `J_g`/`J_prog` 的降权系数，默认 0.3（D13） | † |
| 参考 | `ref_mode` | — | chain | chain（链式剖面）/ subgoal（纯终端子目标） | † |
| 参考 | `yaw_lock_dist` | m | 0.3 | 距终点冻结 yaw 的距离 | † |
| 参考 | `lookahead_distance` | m | 已移除 | 仅 `ref_mode=subgoal` 时需要，M3 新增 | † |
| 制动 | `a_brake` | m/s² | 1.5 | 建议 1.0~1.5 | a_brake |
| 高度 | `z_min/z_max` | m | 0.5/2.8 | 规划域取 0.6/2.7 | † |
| 质量 | `drone_radius` | m | 0.3 | — | r_drone |
| CBF | `gamma` | 1/s | — | 0.65（需满足 `1/γ ≥ v_max/a_brake`，当前 1.54 ≥ 1.33） | γ |
| CBF | `d_safe` | m | — | 与 safe_dist 一致 | d_safe |
| CBF | `slack_weight` | — | — | 1e4 | † |
| CBF | `stride` | 步 | — | 1（先试 1，超时再 2） | † |
| 恢复 | `h_enter/h_exit` | m | — | 0.25 / 0.45 | h_enter, h_exit |
| 恢复 | `T_dwell/T_clear` | s | — | 0.5 / 0.5 | T_dwell, T_clear |
| 恢复 | `big_T` | s | — | 3.0 | T |
| 恢复 | `epsilon_f` | m | — | 0.15 | ε_f |
| 恢复 | `rho_max` | m | — | 2.0 | † |
| 恢复 | `e_th` | m | — | 0.20 | e_th |
| 全局 | `lambda_h` | — | 1.0（参数 `astar/lambda_heu`） | 1.0 | λ_h |
| 全局 | `perception_radius` | m | — | 9.0 | † |
| 全局 | `replan_period` | s | — | 1.0 | † |
| 全局 | `resolution` | m | 0.1 | 0.1 | r |
| 质量 | `respawn/spawn` | m | — | 与 nominal 对齐（M4/M5） | † |

### 附录 C　待补/待改公式清单（投稿前必须完成）

1. 控制量维度：`ω^c` 为标量（P1）。
2. `A, B` 显式 8×8 / 8×4（P2）。
3. `J_oa`/`J_ca` 补 `Σ_{k}`（P7）。
4. `J_ca` 分母与 softplus 内的距离项对调（P8）。
5. 动态障碍项改为"扫掠折线最短表面距离"口径（P9）。
6. `J_g` 的 `ψ_g` 定义（P10）。
7. 第 5 个代价项 `J_prog` 的公式（P6）。
8. CBF 的"线性"表述改为"名义化/序列线性化"，并给 `k=0` 精确、`k≥1` 冻结的说明（P11）。
9. `γ` 量纲与离散衰减系数 `β_disc=1−γΔt`（P12）。
10. `ρ(t)` 完整表达式 + `ρ_0`、`ε_f` 定义（P13）。
11. 重心矩阵求解的 3×3 线性系统、包含性判据、退化阈值（P14）。
12. 滚动重规划策略、感知半径、高度域（P4、P5）。
13. 单向让行拓扑（无环全序）的定义与无环性证明（T20）。
14. 参数总表与性能表（P15）。
15. `§2.1` 中 `λ_h`、分辨率、膨胀半径的数值，以及启发式描述与实际 octile 的一致性（P3）。
16. `§3.4` 触发条件里 `e_2(t)` 的下标与 `i` 不一致（疑为 `e_i` 笔误），需统一（**记为 P16**，与 P13 的 `ρ(t)` 无关）。
17. `§3.2` 末尾"上述约束基础上，优化问题还包含 CBF 安全约束，详见 **2.4 节**"为**错误交叉引用**（CBF 实际在 §3.5），需改正（**记为 P17**）。

### 附录 D　新标称队形与重心权重算例（可直接使用）

**设计**：3 领导者构成三角形，3 跟随者位于内部；尺寸与现有 2 m×4 m 包络相当。

| 机 | 角色 | 标称位置 (x, y) m |
| --- | --- | --- |
| uav1 | leader | (−1.20, −2.00) |
| uav3 | leader | ( 1.20,  0.00) |
| uav5 | leader | (−1.20,  2.00) |
| uav0 | follower | (−0.40, −1.00) |
| uav2 | follower | (−0.40,  1.00) |
| uav4 | follower | ( 0.00,  0.00) |

**求解**（每个 follower 解 3×3 线性系统）：

`m_1 + m_3 + m_5 = 1`，`−1.2m_1 + 1.2m_3 − 1.2m_5 = x`，`−2m_1 + 0·m_3 + 2m_5 = y`

**结果**（保留 6 位小数）：

| follower | `m_{uav1}` | `m_{uav3}` | `m_{uav5}` | 校验 |
| --- | --- | --- | --- | --- |
| uav0 | 0.583333 | 0.333333 | 0.083333 | 和为 1，全正 |
| uav2 | 0.083333 | 0.333333 | 0.583333 | 和为 1，全正 |
| uav4 | 0.250000 | 0.500000 | 0.250000 | 和为 1，全正 |

**三角形面积**：底边取 `uav1–uav5`（竖直，长 4 m），顶点 `uav3` 到该直线的距离为 2.4 m，故 `S = 0.5 × 4 × 2.4 = 4.8 m²`（`≥ min_area = 0.25 m²`，非退化）。

**验证**（代入 uav4）：`0.25·(−1.2,−2) + 0.5·(1.2,0) + 0.25·(−1.2,2) = (−0.3−0.3+0.6, −0.5+0+0.5) = (0, 0)` ✓

> 备注：该队形与现有 spawn（x=−23/−21、y=−2/0/2）不完全一致，M4/M5 联调时需同步 `multi_uav_gazebo.launch` 的 spawn 偏移，或在起飞阶段由 MPC 自然收敛（推荐前者，避免起步大机动）。

### 附录 E　审计依据（代码事实来源）

- MPC/局部层：`include/form_nav/mpc.h`、`src/mpc.cpp`、`src/mpc_node.cpp`（模型、代价、约束、邻居广播、求解器、ahold 分支）
- 全局层/地图：`src/a_star.cpp`、`src/astar_node.cpp`、`src/grid_map.cpp`、`src/kd_tree_map.cpp`、`src/sdf_map.cpp`（ESDF 未接入 MPC）
- 编队/仿真：`src/formation_config.cpp`、`config/formations.yaml`、`launch/formation_test.launch`、`launch/mpc.launch`、`src/obs_generator.cpp`
- 论文公式草稿：`docs/leader_ocp_mathtype.tex`（含离散 CBF 与松弛形式，可作为 M6 的公式参考）

---

**文档结束**　|　下一步：确认 §2.9 的 D1–D12 决策 → 冻结 §3 接口 → 启动 M0/M1/M4 并行开发。
