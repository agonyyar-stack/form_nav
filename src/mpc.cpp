#include "form_nav/mpc.h" //引入 Mpc 类的声明
#include "form_nav/kd_tree_map.h"
#include <algorithm>
#include <cmath>
#include <limits>

namespace {
// 点-线段最短距离平方（C++ 侧查询用，折线最近线段选择）
double pointSegmentDistanceSq(const Eigen::Vector3d& p,
                              const Eigen::Vector3d& a,
                              const Eigen::Vector3d& b) {
    const Eigen::Vector3d e = b - a;
    const double len2 = e.squaredNorm();
    double t = 0.0;
    if (len2 > 1e-12) {
        t = std::max(0.0, std::min(1.0, (p - a).dot(e) / len2));
    }
    const Eigen::Vector3d c = a + t * e;
    return (p - c).squaredNorm();
}
}  // namespace

Mpc::Mpc() { 
    N_ = 10; //预测步数
    dt_ = 0.05; //时间步长
    u_max_ = 6; //线速度指令上限 (m/s)
    w_max_ = 2; //偏航角速度指令上限 (rad/s)
    u_min_ = - u_max_;
    w_min_ = - w_max_;
    //权重矩阵Q,F
    Q_ = DM::diag(DM{20000, 20000, 10000, 100}); // 滚动子目标惩罚权重（可由 mpc/w_subgoal 覆盖）
    F_ = DM::diag(DM{100, 100, 100, 100});       // 控制输入惩罚
    R_d_ = DM::diag(DM{100, 100, 100, 25});      // 控制增量惩罚（抑制重规划导致的指令抖动）
    // 滚动子目标参数
    lookahead_distance_ = 2.0;                   // 子目标沿弧长前瞻距离 (m)
    yaw_ref_ = 0.0;                              // yaw 参考：由 solve() 按子目标方向更新
    yaw_lock_dist_ = 0.3;                        // 终点范围 yaw 冻结距离 (m)
    has_rolling_goal_ = false;
    // 编队误差权重：与子目标惩罚权重 Q_ 拆分为独立项，默认比 Q_ 小一个数量级
    Q_f_ = DM::diag(DM{2000, 2000, 1000, 10});
    //运动学参数（yaw 与线速度解耦的一阶系统，精确离散化见 updateDiscreteModel）
    tau_v_ = 0.8;   // 线速度一阶时间常数 (s)
    tau_w_ = 0.5;   // 偏航角速度一阶时间常数 (s)
    K_v_ = 1.0;     // 线速度指令稳态增益
    K_w_ = 1.0;     // 偏航角速度指令稳态增益（指令即真实 yaw rate，不再乘 π/180）
    updateDiscreteModel();
    // 编队偏移量由外部 setFormationOffsets() 注入，默认自身偏移为 [0,0,0,0]
    // KD-tree 对数势垒避障权重
    Wo = 50000;            // 障碍物避障权重 (标量)
    // 动态障碍物避障（已知轨迹 + 形状感知）
    W_dyn_ = 300;          // 动态障碍物势垒权重 (标量)
    d_safe_dyn_ = 0.2;     // 动态障碍物最小安全距离 (m，表面口径)
    d_act_dyn_ = 2.5;      // 动态障碍物激活范围 (m)
    // KD-tree 静态避障参数（第三步）
    kd_tree_ptr_ = nullptr;   // 障碍物 KD-tree 指针，由外部设置
    nearest_obs_num_ = 3;     // 每个预测步查询的最近障碍点数量
    drone_radius_ = 0.35;     // 无人机半径 (m)
    safe_dist_ = 0.5;         // 对数势垒最小安全距离 L (m，有效距离口径)
    detect_range_ = 2.5;      // 对数势垒探测范围 D̄ (m)
    replan_max_iter_ = 2;     // 最近点重查询重规划最大迭代次数
    ipopt_max_iter_ = 100;    // IPOPT 最大迭代次数
    ipopt_tol_ = 1e-6;        // IPOPT 收敛容差
    // Opti 复用相关
    has_last_solution_ = false;
    built_N_ = -1;
    built_dt_ = -1.0;
    built_u_max_ = -1.0;
    built_w_max_ = -1.0;
    built_tau_v_ = -1.0;
    built_tau_w_ = -1.0;
    built_K_v_ = -1.0;
    built_K_w_ = -1.0;
    built_neighbor_count_ = 0;
    built_nearest_obs_num_ = -1;
    built_detect_range_ = -1.0;
    built_safe_dist_ = -1.0;
    built_dyn_obs_count_ = 0;
    built_d_safe_ca_ = -1.0;
    built_d_act_ca_ = -1.0;
    built_ipopt_max_iter_ = -1;
    built_ipopt_tol_ = -1.0;
    //碰撞参数（中心距口径，不扣半径）
    d_safe_ca = 0.75; //编队内硬安全距离
    d_act_ca = 1.6;   //编队避碰激活范围（小于编队标称间距 2.0）
    Wc = 200;         //相互碰撞权重（对数势垒）
    //反步法控制器增益参数
    k1 = DM::diag(DM({7, 7, 7, 6}));
    k2 = DM::diag(DM({2, 2, 2, 2}));
    // 全局轨迹初始化
    has_global_traj_ = false;
}
Mpc::~Mpc() {}

// 重建精确离散化系数。连续模型（yaw 与线速度解耦）：
//   ṗ = v_e,  v̇_e = (K_v v_c - v_e)/τ_v
//   ψ̇ = ω,    ω̇ = (K_ω ω_c - ω)/τ_ω
// 一阶系统在零阶保持（每步控制恒定）下具有解析解：
//   v_{k+1} = α v_k + β v_c,   α = e^(-Δt/τ),  β = K(1-α)
//   p_{k+1} = p_k + γ v_k + δ v_c,   γ = τ(1-α),  δ = K(Δt-γ)
// 由此拼成常数矩阵：X_{k+1} = A_ X_k + B_ U_k，替代原来的 Euler 近似。
void Mpc::updateDiscreteModel() {
    const double dt = dt_;

    // 线速度通道（x/y/z 共用 τ_v、K_v）
    const double alpha_v = std::exp(-dt / tau_v_);
    const double beta_v  = K_v_ * (1.0 - alpha_v);
    const double gamma_v = tau_v_ * (1.0 - alpha_v);
    const double delta_v = K_v_ * (dt - gamma_v);

    // 偏航角速度通道（ω 的一阶系统 + ψ 积分）
    const double alpha_w = std::exp(-dt / tau_w_);
    const double beta_w  = K_w_ * (1.0 - alpha_w);
    const double gamma_w = tau_w_ * (1.0 - alpha_w);
    const double delta_w = K_w_ * (dt - gamma_w);

    // 状态 X = [x, y, z, ψ, vx, vy, vz, ω]，控制 U = [vx_c, vy_c, vz_c, ω_c]
    A_ = DM::zeros(8, 8);
    A_(0, 0) = 1.0;  A_(1, 1) = 1.0;  A_(2, 2) = 1.0;  A_(3, 3) = 1.0;
    A_(0, 4) = gamma_v;  A_(1, 5) = gamma_v;  A_(2, 6) = gamma_v;  // 位置 <- 速度
    A_(3, 7) = gamma_w;                                               // ψ <- ω
    A_(4, 4) = alpha_v;  A_(5, 5) = alpha_v;  A_(6, 6) = alpha_v;  // 速度自更新
    A_(7, 7) = alpha_w;                                               // ω 自更新

    B_ = DM::zeros(8, 4);
    B_(0, 0) = delta_v;  B_(1, 1) = delta_v;  B_(2, 2) = delta_v;  // 位置 <- 速度指令
    B_(3, 3) = delta_w;                                               // ψ <- ω_c
    B_(4, 0) = beta_v;   B_(5, 1) = beta_v;   B_(6, 2) = beta_v;   // 速度 <- 速度指令
    B_(7, 3) = beta_w;                                                // ω <- ω_c
}
// 保存按路径顺序排列的全局轨迹。路径进度用累积弧长表示，局部参考在
// 每次 solve 前进行数值采样，避免把 CasADi interpolant 放进 Opti 符号图
// 后被 expand 转为 SX。
void Mpc::setGlobalTrajectory(const std::vector<GlobalWaypoint>& traj) {
    global_traj_ = traj;
    has_global_traj_ = !traj.empty();
    if (!has_global_traj_) return;

    // 新轨迹到来：上一轮预测轨迹不再适用，障碍查询回到首轮直线采样。
    has_last_solution_ = false;
    last_pred_x_.resize(0, 0);
    last_u_.resize(0, 0);
    // 投影进度从头开始
    last_progress_idx_ = 0;
    last_progress_s_ = 0.0;

    // 预计算累积弧长表，供滚动子目标沿弧长前推
    path_arclength_.resize(global_traj_.size());
    path_arclength_[0] = 0.0;
    for (size_t i = 1; i < global_traj_.size(); ++i) {
        const Eigen::Vector3d prev(global_traj_[i - 1].x, global_traj_[i - 1].y,
                                   global_traj_[i - 1].z);
        const Eigen::Vector3d cur(global_traj_[i].x, global_traj_[i].y,
                                  global_traj_[i].z);
        path_arclength_[i] = path_arclength_[i - 1] + (cur - prev).norm();
    }

    std::cout << "[MPC] Global trajectory set with " << traj.size()
              << " waypoints, arc ∈ [0, "
              << path_arclength_.back() << "] m" << std::endl;
}

// 将物理位置投影到全局轨迹折线：在最近路径段索引附近窗口内搜索，对每条
// 线段计算最近点，返回连续弧长 s0（线段起点弧长 + 段内比例×段长）。
// 注意：不能退回"最近离散路径点"的弧长——去锯齿后的稀疏路径点间距可达数
// 米，离散投影会卡在最近路径点上不前进，导致子目标钉死、无人机原地悬停。
// 窗口命中边界时做一次全量搜索兜底；路径进度保持单调（段索引 + 弧长），
// 避免无人机在弯道或局部回头时跳回旧路径段。
double Mpc::projectToPath(const Eigen::Vector3d& query_pos) {
    if (!has_global_traj_ || global_traj_.empty() ||
        path_arclength_.size() != global_traj_.size()) {
        return 0.0;
    }

    const size_t n = global_traj_.size();
    const int win_back = 3, win_forward = 30;    // 搜索窗口: [last-3, last+30]
    const int last = static_cast<int>(std::min(last_progress_idx_, n - 1));

    int lo = std::max(0, last - win_back);
    int hi = std::min(static_cast<int>(n) - 1, last + win_forward);

    auto projectSegment = [&](int i) {
        const Eigen::Vector3d a(global_traj_[i].x, global_traj_[i].y,
                                global_traj_[i].z);
        const Eigen::Vector3d b(global_traj_[i + 1].x, global_traj_[i + 1].y,
                                global_traj_[i + 1].z);
        const Eigen::Vector3d d = b - a;
        const double seg_len2 = d.squaredNorm();
        double u = 0.0;
        if (seg_len2 > 1e-12) {
            u = (query_pos - a).dot(d) / seg_len2;
            u = std::min(std::max(u, 0.0), 1.0);
        }
        const Eigen::Vector3d proj = a + u * d;
        return std::make_pair((query_pos - proj).squaredNorm(), u);
    };

    double min_dist = 1e18;
    int best_seg = -1;
    double best_u = 0.0;
    for (int i = lo; i < hi; ++i) {
        auto [d2, u] = projectSegment(i);
        if (d2 < min_dist) {
            min_dist = d2;
            best_seg = i;
            best_u = u;
        }
    }

    // 如果最近线段落在窗口边界附近, 扩展一次全量搜索做安全重置
    if (best_seg == lo || best_seg == hi - 1) {
        for (int i = 0; i + 1 < static_cast<int>(n); ++i) {
            auto [d2, u] = projectSegment(i);
            if (d2 < min_dist) {
                min_dist = d2;
                best_seg = i;
                best_u = u;
            }
        }
    }

    if (best_seg < 0) {
        // 单点路径或窗口异常：回到路径起点
        return 0.0;
    }

    // 路径进度保持单调：段索引只前进不后退；若被钳制回 last 段，重新计算
    // 该段上的投影参数，避免与弧长计算结果不一致
    if (best_seg < last) {
        best_seg = std::min(last, static_cast<int>(n) - 2);
        auto [d2, u] = projectSegment(best_seg);
        (void)d2;
        best_u = u;
    }

    const double s0 = path_arclength_[best_seg] +
                      best_u * (path_arclength_[best_seg + 1] -
                                path_arclength_[best_seg]);
    // 弧长单调保护：无人机横向摆动时连续投影弧长可能小幅回退，
    // 钳制为不后退，避免子目标来回抖动
    const double s_final = s0 < last_progress_s_ - 1e-4 ? last_progress_s_ : s0;
    last_progress_idx_ = static_cast<size_t>(best_seg);
    last_progress_s_ = s_final;
    return s_final;
}

GlobalWaypoint Mpc::selectRollingGoal(double s0) {
    GlobalWaypoint goal;
    has_rolling_goal_ = false;
    if (!has_global_traj_ || global_traj_.empty() ||
        path_arclength_.size() != global_traj_.size()) {
        return goal;
    }

    const double total = path_arclength_.back();
    const double s_clamped = std::min(std::max(s0, 0.0), total);

    // 已到轨迹末端：子目标即终点
    if (s_clamped >= total) {
        goal = global_traj_.back();
        goal.vyaw = 0.0;
        has_rolling_goal_ = true;
        rolling_goal_ = goal;
        return goal;
    }

    // 按累积弧长二分定位 s_clamped 所在段 [i0, i0+1)
    auto upper = std::lower_bound(path_arclength_.begin(),
                                  path_arclength_.end(), s_clamped);
    size_t i0 = static_cast<size_t>(std::distance(path_arclength_.begin(), upper));
    if (i0 >= path_arclength_.size() || path_arclength_[i0] > s_clamped) {
        if (i0 > 0) --i0;
    }
    if (i0 >= global_traj_.size() - 1) {
        goal = global_traj_.back();
        goal.vyaw = 0.0;
        has_rolling_goal_ = true;
        rolling_goal_ = goal;
        return goal;
    }

    // 沿弧长前推 lookahead_distance_
    const double target_arc = s_clamped + std::max(0.0, lookahead_distance_);
    if (target_arc >= total) {
        goal = global_traj_.back();
    } else {
        auto upper2 = std::lower_bound(path_arclength_.begin(),
                                       path_arclength_.end(), target_arc);
        size_t i1 = static_cast<size_t>(std::distance(path_arclength_.begin(), upper2));
        if (i1 >= path_arclength_.size() || path_arclength_[i1] > target_arc) {
            if (i1 > 0) --i1;
        }
        if (i1 >= global_traj_.size() - 1) {
            goal = global_traj_.back();
        } else {
            const double seg_len = path_arclength_[i1 + 1] - path_arclength_[i1];
            const double ratio = seg_len > 0.0
                                     ? (target_arc - path_arclength_[i1]) / seg_len
                                     : 0.0;
            const auto lerp = [ratio](double from, double to) {
                return from + ratio * (to - from);
            };
            const GlobalWaypoint& wp0 = global_traj_[i1];
            const GlobalWaypoint& wp1 = global_traj_[i1 + 1];
            goal.x = lerp(wp0.x, wp1.x);
            goal.y = lerp(wp0.y, wp1.y);
            goal.z = lerp(wp0.z, wp1.z);
            goal.yaw = lerp(wp0.yaw, wp1.yaw);
            goal.vx = lerp(wp0.vx, wp1.vx);
            goal.vy = lerp(wp0.vy, wp1.vy);
            goal.vz = lerp(wp0.vz, wp1.vz);
        }
    }
    // 子目标只使用位置/偏航（yaw 参考恒 0），速度字段不使用
    goal.vyaw = 0.0;
    has_rolling_goal_ = true;
    rolling_goal_ = goal;
    return goal;
}

ErrorTerms Mpc::formationError(int k_step, MX reference_state, std::map<int, MX>& neighbor_params, MX current_states) {
    ErrorTerms ret;

    // 自身 UAV 状态
    MX p_i = current_states(Slice(0, 4));   // [x, y, z, psi] global
    // MX p_i_dot = mtimes(R_i, v_i);
    MX v_i = current_states(Slice(4, 8));   // [vx, vy, vz, ω] inertial
    MX w_i = p_i(3);
    // MX vw_i = v_i(3);

    // 旋转矩阵 R_i
    MX R_i = MX::zeros(4, 4);
    R_i(0,0) = cos(w_i);  R_i(0,1) = -sin(w_i);
    R_i(1,0) = sin(w_i);  R_i(1,1) =  cos(w_i);
    R_i(2,2) = 1;
    R_i(3,3) = 1;
    // MX R_i_dot = MX::zeros(4, 4);
    // R_i_dot(0,0) =-sin(w_i)*vw_i; R_i_dot(0,1) =-cos(w_i)*vw_i;
    // R_i_dot(1,0) = cos(w_i)*vw_i; R_i_dot(1,1) =-sin(w_i)*vw_i;

    MX M_i = DM(formation_offsets_[my_id_]);

    // 累加项
    MX S1_accum      = MX::zeros(4, 1);
    // MX S1_dot_accum  = MX::zeros(4, 1);
    // MX sum_term      = MX::zeros(4, 1);

    // 遍历邻居                8*11
    for (auto const& [j_id, j_param] : neighbor_params) {

        // a_ij = 1（因为仅遍历邻居，邻居即为 a_ij=1）
        MX y_j = j_param(Slice(0, 4), k_step);
        MX v_j = j_param(Slice(4, 8), k_step);
        MX w_j = y_j(3); 
        // MX vw_j = v_j(3);

        MX R_j = MX::zeros(4, 4); //**********旋转矩阵->后期封包为一个函数********************************************
        R_j(0,0) = cos(w_j);  R_j(0,1) = -sin(w_j);
        R_j(1,0) = sin(w_j);  R_j(1,1) =  cos(w_j);
        R_j(2,2) = 1;
        R_j(3,3) = 1;
        // MX R_j_dot = MX::zeros(4, 4);
        // R_j_dot(0,0) =-sin(w_j)*vw_j; R_j_dot(0,1) =-cos(w_j)*vw_j;
        // R_j_dot(1,0) = cos(w_j)*vw_j; R_j_dot(1,1) =-sin(w_j)*vw_j;

        // MX p_j_dot = mtimes(R_j, v_j);

        MX M_j = DM(formation_offsets_[j_id]);
        S1_accum += ((y_j - M_j) - (p_i - M_i));
        // S1_accum += a_ij * ((y_j - M_j) - (p_i - M_i));

        // S1_dot_accum += a_ij * ((p_j_dot - G_j_dot) - (p_i_dot - G_i_dot));

        // // 预测速度导数
        // MX v_j_dot = MX::zeros(4,1);
        // if (k_step < N_) {
        //     MX v_j_next = j_param(Slice(4, 8), k_step + 1);
        //     v_j_dot = (v_j_next - v_j) / dt_;
        // }

        // sum_term += a_ij * ( mtimes(R_j_dot, v_j) + mtimes(R_j, v_j_dot) - G_j_ddot );
    }

    // 注意：全局轨迹跟踪项 (y_d - p_i) 已从 S1 中拆出，由 buildOpti()
    // 用 Q_ 单独加权；S1 只保留邻居 containment 误差，用 Q_f_ 加权。
    // 两者不再耦合，避免“编队误差与路径跟踪打架”导致振荡。
    (void)reference_state;

    // 输出
    ret.S1       = S1_accum;
    // LMPC 暂未启用。保留接口并提供零占位量，避免将来误调用时使用
    // 未构造的 CasADi 表达式。
    ret.S1_dot   = MX::zeros(4, 1);
    ret.S2       = MX::zeros(4, 1);
    ret.sum_term = MX::zeros(4, 1);

    return ret;
}

MX Mpc::ViLmpcDot(int k_step, MX reference_state, std::map<int, MX>& neighbor_params, MX current_states, MX U_k) {
    // LMPC 稳定性约束暂时关闭。下面保留原始推导代码，待普通 MPC
    // 单机验证完成后，再单独恢复并检查辅助误差的完整定义。
    (void)k_step;
    (void)reference_state;
    (void)neighbor_params;
    (void)current_states;
    (void)U_k;
    return MX(0.0);

#if 0
    ErrorTerms errors = formationError(k_step, reference_state, neighbor_params, current_states);
    MX S1 = errors.S1;
    MX S2 = errors.S2;
    MX sum = errors.sum_term;

    MX v_i = current_states(Slice(4, 8));
    MX w_i = current_states(3);
    MX vw_i = v_i(3);

    MX R_i = MX::zeros(4, 4);
    R_i(0,0) = cos(w_i); R_i(0,1) =-sin(w_i);
    R_i(1,0) = sin(w_i); R_i(1,1) = cos(w_i);
    R_i(2,2) = 1; R_i(3,3) = 1;
    MX R_i_dot = MX::zeros(4, 4);
    R_i_dot(0,0) =-sin(w_i)*vw_i; R_i_dot(0,1) =-cos(w_i)*vw_i;
    R_i_dot(1,0) = cos(w_i)*vw_i; R_i_dot(1,1) =-sin(w_i)*vw_i;
    
    if (has_global_traj_) {
    MX y_d_dot = reference_state(Slice(4,8));
    } else {
    MX y_d_dot = MX::zeros(4,1);
    }
    MX y_d_ddot = MX::zeros(4,1);  // 无加速度信息，置零

    double b_i = 1.0;  // 分布式规划：所有机均直接跟踪自身物理参考

    double d_i = static_cast<double>(neighbor_params.size());  // 邻居数量即度
    
    MX term1 = mtimes(S1.T(), S1); 

    MX term2 = casadi::MX::mtimes({S1.T(), k1, S1}); //多于两个矩阵相乘须有花括号

    MX term3_bracket = MX::zeros(4, 1);

    // 旧模型（yaw/速度耦合）的反步法项：新解耦模型下需按
    // v̇=(K_v v_c-v)/τ_v、ω̇=(K_ω ω_c-ω)/τ_ω 重写后才可恢复。
    MX temp = MX::zeros(4, 1);

    term3_bracket += - (d_i + b_i) * temp;

    term3_bracket += sum;

    term3_bracket += mtimes(k1, (S2 - mtimes(k1, S1)));

    // MX leader_mask = MX::if_else(b_i > 0, 1.0, 0.0);
    term3_bracket += b_i * y_d_ddot;

    MX term3 = mtimes(S2.T(), term3_bracket);

    return term1 - term2 + term3;
#endif
}

MX Mpc::kdtreeObstacleAvoidance(MX current_states, MX obstacle_k) {
    MX H_oa = MX(0.0);
    if (kd_tree_ptr_ == nullptr || kd_tree_ptr_->empty() ||
        detect_range_ <= safe_dist_) {
        // 无 KD-tree 数据时返回零惩罚（与 ESDF 缺数据时行为一致）
        return H_oa;
    }

    MX pos = current_states(Slice(0, 3));       // 世界系位置

    // 对数势垒：H = max{ -M * ln( (d - L) / (D̄ - L) ), 0 }
    // d 为有效距离（扣无人机半径），L=safe_dist_ 为最小安全距离，
    // D̄=detect_range_ 为探测范围。d > D̄ 时严格为零；d -> L 时趋于无穷。
    // 钳位 d >= L + eps 避免 log 参数非正，同时限制最大惩罚值。
    for (int m = 0; m < nearest_obs_num_; ++m) {
        MX o = obstacle_k(Slice(3 * m, 3 * m + 3));
        MX d_eff = MX::norm_2(pos - o) - drone_radius_;
        MX d_c = MX::fmax(d_eff, safe_dist_ + 1e-2);
        MX term = Wo * MX::fmax(0.0, -log((d_c - safe_dist_) /
                                          (detect_range_ - safe_dist_)));
        H_oa += term;
    }
    return H_oa;
}

// 动态障碍物形状感知避障势垒。障碍物轨迹折线与形状均为参数：
//   dyn_obs_seg_param_ (6*C, N_+1)：第 j 个障碍物第 k 步存该步选中的最近线段
//     端点 [x_a,y_a,z_a,x_b,y_b,z_b]（由 queryDynamicObstacleSegments 在求解前
//     按 0.5 s 窗口、0.1 s 节点折线查询得到）；
//   dyn_obs_shape_param_ (3, C)：每列 [shape, radius, height]。
// 距离口径：对每个预测步，先算无人机位置到所选线段（避障轨迹折线的最近段）的
// 最近点 c，再用"最近点处障碍物表面"距离 d_surface（球体扣半径、圆柱按竖直柱体
// 分段精确到表面），再扣无人机半径得到有效距离 d_eff。这样避障不依赖预测步与
// 障碍物轨迹的时间对齐：即使求解滞后/卡顿导致时间步错开，也会避开障碍物该段
// 扫掠路径。对数势垒参数独立（W_dyn_/d_safe_dyn_/d_act_dyn_），不复用编队避碰
// 参数（编队为双向避让，动态障碍物为单向且可能更快）。
MX Mpc::dynamicObstacleAvoidance(MX current_states, int k_step) {
    MX H_dyn = MX(0.0);
    const int C = dyn_obs_shape_param_.size2();
    if (C <= 0 || d_act_dyn_ <= d_safe_dyn_) {
        return H_dyn;
    }

    MX pos = current_states(Slice(0, 3));
    for (int j = 0; j < C; ++j) {
        MX e_a = dyn_obs_seg_param_(Slice(6 * j, 6 * j + 3), k_step);
        MX e_b = dyn_obs_seg_param_(Slice(6 * j + 3, 6 * j + 6), k_step);
        MX shape = dyn_obs_shape_param_(0, j);
        MX R = dyn_obs_shape_param_(1, j);
        MX H = dyn_obs_shape_param_(2, j);

        // 点-线段最近点（线段端点为参数，clamp 保持分段可导）：
        //   t = clamp( dot(pos-e_a, e) / ||e||^2, 0, 1 ),  c = e_a + t*e
        MX e = e_b - e_a;
        MX denom = MX::dot(e, e) + 1e-12;  // 防零长度段除零
        MX t = MX::fmax(0.0, MX::fmin(1.0, MX::dot(pos - e_a, e) / denom));
        MX c = e_a + t * e;

        MX dx = pos(0) - c(0);
        MX dy = pos(1) - c(1);
        MX dz = pos(2) - c(2);

        // 球体：任意方向距离减半径
        MX sphere_dist = MX::norm_2(pos - c) - R;

        // 竖直圆柱：高度带内用水平距离减半径；正上/正下用竖直距离；
        // 角上用边缘圆环距离（sqrt 参数恒非负）。
        MX rho = MX::sqrt(dx * dx + dy * dy);
        MX d_horiz = rho - R;
        MX d_vert = fabs(dz) - H / 2.0;
        MX d_rim = MX::sqrt(d_horiz * d_horiz + d_vert * d_vert);
        MX in_band = fabs(dz) <= H / 2.0;
        MX in_foot = rho <= R;
        MX cyl_dist = MX::if_else(
            in_band, d_horiz, MX::if_else(in_foot, d_vert, d_rim));

        // 形状分支：0=球体，1=圆柱
        MX d_surface = MX::if_else(shape == 0.0, sphere_dist, cyl_dist);

        // 统一口径：扣无人机半径；对数势垒钳位防 log 参数非正
        MX d_eff = d_surface - drone_radius_;
        MX d_c = MX::fmax(d_eff, d_safe_dyn_ + 1e-2);
        MX term = W_dyn_ * MX::fmax(
            0.0, -log((d_c - d_safe_dyn_) / (d_act_dyn_ - d_safe_dyn_)));
        H_dyn += term;
    }
    return H_dyn;
}

std::vector<double> Mpc::queryObstacleValues(const std::vector<Eigen::Vector3d>& query_pts, std::vector<int>& out_indices) {
    const int steps = static_cast<int>(query_pts.size());
    const int total = nearest_obs_num_ * steps;
    std::vector<double> values(3 * total, 1e4);
    out_indices.assign(total, -1);
    last_obstacle_pts_.clear();
    last_obstacle_pts_.reserve(total);

    if (kd_tree_ptr_ == nullptr || kd_tree_ptr_->empty()) {
        // 无数据：全部填充远处点，代价近似为 0
        for (int i = 0; i < total; ++i) {
            last_obstacle_pts_.push_back(Eigen::Vector3d(1e4, 1e4, 1e4));
        }
        return values;
    }

    for (int k = 0; k < steps; ++k) {
        std::vector<Eigen::Vector3d> pts;
        std::vector<double> dists;
        std::vector<int> idx;
        kd_tree_ptr_->queryNearest(query_pts[k], nearest_obs_num_, pts, dists, idx);
        for (int m = 0; m < nearest_obs_num_; ++m) {
            const size_t off = (static_cast<size_t>(k) * nearest_obs_num_ + m) * 3;
            if (m < static_cast<int>(pts.size())) {
                values[off + 0] = pts[m].x();
                values[off + 1] = pts[m].y();
                values[off + 2] = pts[m].z();
                out_indices[k * nearest_obs_num_ + m] = idx[m];
                last_obstacle_pts_.push_back(pts[m]);
            } else {
                // 点数不足：填充远处点，代价近似 0
                values[off + 0] = 1e4;
                values[off + 1] = 1e4;
                values[off + 2] = 1e4;
                out_indices[k * nearest_obs_num_ + m] = -1;
                last_obstacle_pts_.push_back(Eigen::Vector3d(1e4, 1e4, 1e4));
            }
        }
    }
    return values;
}

// 动态障碍物最近线段查询。对每个预测步的查询点，在障碍物轨迹
// "0.5 s 窗口、0.1 s 节点" 采样的折线中找最近线段（即查询点到整条避障轨迹的
// 最短距离），把端点写入参数 (6*C, N_+1)，并返回线段索引供重规划判断。
// 与静态 KD-tree 的"查询→参数化"一致：优化符号图内的距离是点-线段距离
// （线段端点为参数），不再依赖预测步与障碍物轨迹的时间对齐。
std::vector<double> Mpc::queryDynamicObstacleSegments(
    const std::vector<Eigen::Vector3d>& query_pts,
    const std::vector<DynamicObstacleState>& dyn_obs,
    std::vector<int>& out_indices) {
    const int steps = static_cast<int>(query_pts.size());
    const int C = static_cast<int>(dyn_obs.size());
    std::vector<double> values(6 * C * steps, 1e4);
    out_indices.assign(C * steps, -1);

    if (C <= 0 || steps <= 0) {
        return values;
    }

    // 避障轨迹窗口与节点间隔：只考虑 0.5 s 内的轨迹，每 0.1 s 取一个节点。
    // dt_=0.05 时节点列号为 0,2,...,10，共 6 个节点、5 条线段。
    const double kDynObsWindow = 0.5;   // 避障窗口 (s)
    const double kDynObsNodeDt = 0.1;   // 节点采样间隔 (s)
    const int node_stride =
        std::max(1, static_cast<int>(std::lround(kDynObsNodeDt / dt_)));
    const int max_col =
        std::min(N_, static_cast<int>(std::lround(kDynObsWindow / dt_)));

    for (int k = 0; k < steps; ++k) {
        for (int j = 0; j < C; ++j) {
            const Eigen::MatrixXd& st = dyn_obs[j].predicted_states;
            const int cols = static_cast<int>(st.cols());
            if (st.rows() != 8 || cols < 2) {
                continue;  // 数据异常：保持远端兜底值，代价近似为 0
            }

            // 按节点间隔取折线节点（列号不超过窗口与轨迹长度）
            std::vector<int> node_cols;
            for (int col = 0; col <= max_col && col < cols; col += node_stride) {
                node_cols.push_back(col);
            }
            if (node_cols.size() < 2) {
                continue;
            }

            // 找查询点到折线的最近线段
            int best_seg = -1;
            double best_d2 = std::numeric_limits<double>::infinity();
            for (size_t s = 0; s + 1 < node_cols.size(); ++s) {
                const int ia = node_cols[s];
                const int ib = node_cols[s + 1];
                const Eigen::Vector3d a(st(0, ia), st(1, ia), st(2, ia));
                const Eigen::Vector3d b(st(0, ib), st(1, ib), st(2, ib));
                const double d2 = pointSegmentDistanceSq(query_pts[k], a, b);
                if (d2 < best_d2) {
                    best_d2 = d2;
                    best_seg = static_cast<int>(s);
                }
            }
            if (best_seg < 0) {
                continue;
            }

            // 填充端点参数：列优先布局 (6*C, N_+1)，元素 (6*j+m, k)
            const int ia = node_cols[best_seg];
            const int ib = node_cols[best_seg + 1];
            for (int m = 0; m < 3; ++m) {
                values[(6 * j + m) + 6 * C * k] = st(m, ia);
                values[(6 * j + 3 + m) + 6 * C * k] = st(m, ib);
            }
            out_indices[k * C + j] = best_seg;
        }
    }
    return values;
}

MX Mpc::collisionAvoidance(MX reference_state, int k_step, std::map<int, MX>& neighbor_params, MX current_states){
    MX pos_i = current_states(Slice(0, 3));
    MX H_ca = 0.0;

    for (auto const& [j_id, j_param] : neighbor_params) {
        // a_ij = 1（因为仅遍历邻居）
        MX y_j = j_param(Slice(0, 4), k_step);
        MX pos_j = y_j(Slice(0, 3));

        // 对数势垒（中心距口径）：
        //   d >= d_act_ca 时惩罚严格为 0；d -> d_safe_ca+ 时惩罚 -> +∞。
        // 钳位 d >= d_safe_ca + eps 避免 log 参数非正，与障碍物项处理一致。
        MX dist = MX::norm_2(pos_i - pos_j) - (2 * drone_radius_); // 有效间距
        MX d_c = MX::fmax(dist, d_safe_ca + 1e-2);
        MX h_ca = Wc * MX::fmax(
            0.0, -log((d_c - d_safe_ca) / (d_act_ca - d_safe_ca)));
        H_ca += h_ca;
    }
    return H_ca;
}

// 构建（或复用）Opti 优化问题。配置（预测时域、时间步、输入限幅、
// 邻居数量、最近障碍点数量）变化时重建；否则复用同一实例，仅更新参数值。
// 复用 + 热启动是第三步重规划循环能够在 20 Hz 控制率下运行的前提。
bool Mpc::buildOpti(const std::vector<UavState>& neighbors,
                    const std::vector<DynamicObstacleState>& dyn_obs) {
    if (opti_ &&
        built_N_ == N_ && std::abs(built_dt_ - dt_) < 1e-9 &&
        std::abs(built_u_max_ - u_max_) < 1e-9 &&
        std::abs(built_w_max_ - w_max_) < 1e-9 &&
        std::abs(built_tau_v_ - tau_v_) < 1e-9 &&
        std::abs(built_tau_w_ - tau_w_) < 1e-9 &&
        std::abs(built_K_v_ - K_v_) < 1e-9 &&
        std::abs(built_K_w_ - K_w_) < 1e-9 &&
        built_neighbor_count_ == neighbors.size() &&
        built_nearest_obs_num_ == nearest_obs_num_ &&
        std::abs(built_detect_range_ - detect_range_) < 1e-9 &&
        std::abs(built_safe_dist_ - safe_dist_) < 1e-9 &&
        built_dyn_obs_count_ == dyn_obs.size() &&
        std::abs(built_d_safe_ca_ - d_safe_ca) < 1e-9 &&
        std::abs(built_d_act_ca_ - d_act_ca) < 1e-9 &&
        built_ipopt_max_iter_ == ipopt_max_iter_ &&
        std::abs(built_ipopt_tol_ - ipopt_tol_) < 1e-12) {
        return true;
    }

    opti_ = std::make_unique<Opti>();
    Slice all; //切片工具，代表所有索引

    X = opti_->variable(8, N_ + 1); //状态序列，位姿速度
    U = opti_->variable(4, N_); //控制序列：惯性系线速度指令 + yaw rate 指令

    // 参数：滚动子目标参考、当前状态、最近障碍点（KD-tree 查询在求解器外完成）、终点
    reference_param_ = opti_->parameter(4, N_ + 1);
    X_cur_param_ = opti_->parameter(8, 1);
    obstacle_param_ = opti_->parameter(3 * nearest_obs_num_, N_ + 1);
    goal_param_ = opti_->parameter(3, 1);

    // 动态障碍物：最近线段端点参数 (6*C, N_+1)，形状参数 (3, C) [shape, radius, height]
    // 线段端点/形状均为参数，只有数量变化才重建 Opti。
    if (!dyn_obs.empty()) {
        dyn_obs_seg_param_ = opti_->parameter(
            6 * static_cast<int>(dyn_obs.size()), N_ + 1);
        dyn_obs_shape_param_ = opti_->parameter(
            3, static_cast<int>(dyn_obs.size()));
    }

    // 邻居轨迹参数
    neighbor_params_.clear();
    for (const auto& nb : neighbors) {
        neighbor_params_[nb.id] = opti_->parameter(8, N_ + 1);
    }

    // 构建代价函数
    MX cost = 0;
    for (int k = 0; k <= N_; ++k) { // N+1
        MX X_k = X(all, k);
        MX reference_k = reference_param_(all, k);

        ErrorTerms errors1 = formationError(k, reference_k, neighbor_params_, X_k);
        MX S_f = errors1.S1;  // 编队 containment 误差（仅邻居项）
        // 滚动子目标误差：预测状态（位置 + 偏航）vs 子目标参考
        MX subgoal_err = X_k(Slice(0, 4)) - reference_k(Slice(0, 4));
        MX H_ca = collisionAvoidance(reference_k, k, neighbor_params_, X_k);
        cost += H_ca;

        // KD-tree 对数势垒避障代价（障碍点是参数，代价处处可导）
        MX H_oa_kd = kdtreeObstacleAvoidance(X_k, obstacle_param_(all, k));
        cost += H_oa_kd;
        // 动态障碍物形状感知势垒（已知轨迹，位置/形状均为参数）
        cost += dynamicObstacleAvoidance(X_k, k);

        if (k < N_) {
            // 子目标惩罚只加在终端步，中间步仅编队项独立加权，互不耦合
            // cost += MX::mtimes({subgoal_err.T(), Q_, subgoal_err});
            cost += MX::mtimes({S_f.T(), Q_f_, S_f});
            
            MX U_k = U(all, k);
            cost += MX::mtimes({U_k.T(), F_, U_k}); // 控制输入惩罚
        } else {
            cost += MX::mtimes({subgoal_err.T(), Q_, subgoal_err});
            cost += MX::mtimes({S_f.T(), Q_f_, S_f});
        }
    }

    // 控制增量惩罚：抑制重规划导致的指令抖动，平滑预测轨迹。
    for (int k = 0; k < N_ - 1; ++k) {
        MX dU = U(all, k + 1) - U(all, k);
        cost += MX::mtimes({dU.T(), R_d_, dU});
    }
    opti_->minimize(cost);

    // 动力学约束（精确离散化，非 Euler 近似）：
    //   X_{k+1} = A_ X_k + B_ U_k
    // 线速度为惯性系独立状态，与 yaw 解耦；U 为惯性系线速度指令 + yaw rate 指令。
    for (int k = 0; k < N_; ++k) {
        opti_->subject_to(X(all, k + 1) ==
                          mtimes(A_, X(all, k)) + mtimes(B_, U(all, k)));
    }

    // 初始状态约束
    opti_->subject_to(X(all, 0) == X_cur_param_);

    // 制动约束：保证预测轨迹在终点前能减速停下。
    // 解耦模型下 v_c = 0 时一阶系统总停车距离精确等于 tau_v * ||v||，
    // 因此采用精确停车约束（替代原近似二次约束）：
    //   tau_v * ||v_k|| <= d_g_k
    // 注意：norm_2 在 v=0 处不可导，IPOPT 在默认零初值处求雅可比会得到
    // NaN（Invalid_Number_Detected）。两边平方化为等价且处处光滑的形式：
    //   tau_v^2 * ||v_k||^2 <= d_g_k^2
    for (int k = 0; k <= N_; ++k) {
        MX X_k = X(all, k);
        MX v_k = X_k(Slice(4, 8));
        MX d_g_vec = X_k(Slice(0, 3)) - goal_param_;
        MX v_norm_sq = MX::dot(v_k(Slice(0, 3)), v_k(Slice(0, 3)));
        MX d_g_sq = MX::dot(d_g_vec, d_g_vec);
        opti_->subject_to(tau_v_ * tau_v_ * v_norm_sq <= d_g_sq);
    }

    // 输入约束
    opti_->subject_to(u_min_ <= U(0, all) <= u_max_);
    opti_->subject_to(u_min_ <= U(1, all) <= u_max_);
    opti_->subject_to(u_min_ <= U(2, all) <= u_max_);
    opti_->subject_to(w_min_ <= U(3, all) <= w_max_);

    // 求解器设置
    casadi::Dict solver_opts; //创建求解器选项
    solver_opts["expand"] = true; //展开表达式以加快求解速度
    solver_opts["ipopt.max_iter"] = ipopt_max_iter_; //最大迭代次数
    solver_opts["ipopt.print_level"] = 0; //不打印迭代信息
    solver_opts["print_time"] = 0; //不打印时间信息
    solver_opts["ipopt.acceptable_tol"] = ipopt_tol_; //梯度容差，收敛精度设定
    solver_opts["ipopt.acceptable_obj_change_tol"] = ipopt_tol_; //目标函数变化容差

    opti_->solver("ipopt", solver_opts); //设置求解器为ipopt并应用求解器选项

    // 更新构建签名
    built_N_ = N_;
    built_dt_ = dt_;
    built_u_max_ = u_max_;
    built_w_max_ = w_max_;
    built_tau_v_ = tau_v_;
    built_tau_w_ = tau_w_;
    built_K_v_ = K_v_;
    built_K_w_ = K_w_;
    built_neighbor_count_ = neighbors.size();
    built_nearest_obs_num_ = nearest_obs_num_;
    built_detect_range_ = detect_range_;
    built_safe_dist_ = safe_dist_;
    built_dyn_obs_count_ = dyn_obs.size();
    built_d_safe_ca_ = d_safe_ca;
    built_d_act_ca_ = d_act_ca;
    built_ipopt_max_iter_ = ipopt_max_iter_;
    built_ipopt_tol_ = ipopt_tol_;
    has_last_solution_ = false;
    return true;
}

bool Mpc::solve(double s0, const std::vector<UavState>& neighbors,
                const std::vector<DynamicObstacleState>& dyn_obs,
                Eigen::VectorXd current_states) {
    // 控制节点负责等待传感器和全局轨迹；这里再做一次尺寸检查，避免
    // 未初始化状态进入 CasADi 参数并产生难以定位的维度错误。
    if (current_states.size() != 8) {
        std::cerr << "[MPC] Invalid current state dimension: "
                  << current_states.size() << std::endl;
        return false;
    }
    if (!has_global_traj_) {
        std::cerr << "[MPC] Cannot solve without a global trajectory." << std::endl;
        return false;
    }
    if (nearest_obs_num_ <= 0) {
        std::cerr << "[MPC] nearest_obs_num_ must be positive." << std::endl;
        return false;
    }

    if (!buildOpti(neighbors, dyn_obs)) {
        return false;
    }

    // 滚动子目标：沿弧长从最近投影点前推 lookahead_distance_。
    // 全局轨迹只提供方向引导，整个预测时域跟踪同一个子目标点。
    rolling_goal_ = selectRollingGoal(s0);
    if (!has_rolling_goal_) {
        std::cerr << "[MPC] Failed to select rolling goal." << std::endl;
        return false;
    }

    // yaw 参考 = 子目标方向（不新增代价项，复用终端 Q_ 的 yaw 软约束）：
    //   当前点 -> 滚动子目标 的水平方向角。与子目标距离过近（at 目标点附近）
    //   时方向无定义，保持上一周期参考；相对当前 yaw 做 ±π 连续化（unwrap），
    //   避免参考在路径折角处跳 ±π 导致 (ψ-ψ_ref)² 出现尖点/绕远路旋转。
    // 进入终点范围（距全局终点 < yaw_lock_dist_）后冻结参考：无人机已在终点
    // 附近悬停漂移时，"指向终点"的方位角会绕圈旋转，继续跟踪会产生冗余的
    // 持续转头；冻结后由终端 Q_ yaw 软约束保持进入该范围时的机头方向。
    const Eigen::Vector3d cur_pos(current_states(0), current_states(1),
                                  current_states(2));
    const Eigen::Vector3d traj_end(global_traj_.back().x,
                                   global_traj_.back().y,
                                   global_traj_.back().z);
    if ((traj_end - cur_pos).norm() > yaw_lock_dist_) {
        const Eigen::Vector3d yaw_ref_vec(rolling_goal_.x - current_states(0),
                                          rolling_goal_.y - current_states(1),
                                          0.0);
        if (yaw_ref_vec.norm() > 1e-3) {
            const double psi_cur = current_states(3);
            const double yaw_candidate =
                std::atan2(yaw_ref_vec.y(), yaw_ref_vec.x());
            yaw_ref_ = yaw_candidate +
                       2.0 * M_PI * std::round((psi_cur - yaw_candidate) /
                                               (2.0 * M_PI));
        }
    }

    std::vector<double> reference_values;
    reference_values.reserve(4 * (N_ + 1));
    for (int k = 0; k <= N_; ++k) {
        reference_values.insert(reference_values.end(),
                                {rolling_goal_.x, rolling_goal_.y,
                                 rolling_goal_.z, yaw_ref_});
    }
    opti_->set_value(reference_param_,
                     casadi::DM::reshape(DM(reference_values), 4, N_ + 1));

    // 终点位置参数（制动约束用）
    DM goal_dm = DM::zeros(3, 1);
    goal_dm(0) = global_traj_.back().x;
    goal_dm(1) = global_traj_.back().y;
    goal_dm(2) = global_traj_.back().z;
    opti_->set_value(goal_param_, goal_dm);

    // 初始状态参数
    vector<double> cur_state_vec(current_states.data(), current_states.data() + 8);
    opti_->set_value(X_cur_param_, DM(cur_state_vec));

    // 邻居轨迹参数
    for (const auto& nb : neighbors) {
        auto it = neighbor_params_.find(nb.id);
        if (it == neighbor_params_.end()) {
            std::cerr << "[MPC] Neighbor " << nb.id
                      << " not in built Opti problem." << std::endl;
            return false;
        }
        std::vector<double> nb_vec(nb.predicted_states.data(),
                                   nb.predicted_states.data() + nb.predicted_states.size());
        // Eigen 默认列优先，CasADi 也是列优先填充
        opti_->set_value(it->second,
                         casadi::DM::reshape(DM(nb_vec), 8, N_ + 1));
    }

    // 动态障碍物：形状参数 [shape, radius, height] 与查询点无关，直接填充；
    // 最近线段端点参数依赖查询点，在下方重查询循环中随查询点一起查询填充。
    if (!dyn_obs.empty()) {
        for (const auto& ob : dyn_obs) {
            if (ob.predicted_states.rows() != 8 ||
                ob.predicted_states.cols() != N_ + 1) {
                std::cerr << "[MPC] Dynamic obstacle " << ob.id
                          << " predicted states shape mismatch." << std::endl;
                return false;
            }
        }
        std::vector<double> dyn_shape_vec;
        dyn_shape_vec.reserve(3 * dyn_obs.size());
        for (const auto& ob : dyn_obs) {
            dyn_shape_vec.push_back(static_cast<double>(ob.shape));
            dyn_shape_vec.push_back(ob.radius);
            dyn_shape_vec.push_back(ob.height);
        }
        opti_->set_value(
            dyn_obs_shape_param_,
            casadi::DM::reshape(
                DM(dyn_shape_vec), 3, static_cast<int>(dyn_obs.size())));
    }

    // 查询点：
    //   非首轮：用上一轮预测轨迹整体前移一列 —— k=0..N-1 取上一轮第 k+1 列
    //           （当前状态 ≈ 上一轮第 1 列），k=N 无对应列，用滚动子目标兜底；
    //   首轮（无上一轮解/新轨迹到达）：当前状态位置 → 滚动子目标 直线采样 N+1 点。
    const Eigen::Vector3d goal_pos(rolling_goal_.x, rolling_goal_.y,
                                   rolling_goal_.z);
    auto buildQueryPoints =
        [&](const Eigen::MatrixXd* pred) -> std::vector<Eigen::Vector3d> {
        std::vector<Eigen::Vector3d> pts(N_ + 1);
        if (pred != nullptr && pred->rows() == 8 && pred->cols() == N_ + 1) {
            for (int k = 0; k < N_; ++k) {
                pts[k] = pred->block<3, 1>(0, k + 1);
            }
            pts[N_] = goal_pos;
        } else {
            for (int k = 0; k <= N_; ++k) {
                const double ratio = static_cast<double>(k) /
                                     static_cast<double>(N_);
                pts[k] = (1.0 - ratio) * cur_pos + ratio * goal_pos;
            }
        }
        return pts;
    };

    std::vector<Eigen::Vector3d> query_pts =
        buildQueryPoints(has_last_solution_ ? &last_pred_x_ : nullptr);

    // 重查询重规划循环：每轮解出后，用刚解出的预测轨迹重新生成查询点（同样
    // 前移一列 + 子目标兜底），若最近静态障碍点索引或动态障碍物最近线段索引
    // 相对本轮发生变化（地图更新或轨迹偏离），带新参数再解，否则停止。
    // replan_max_iter_=0 时每控制周期单次求解，查询引导由跨轮"上一轮预测"迭代完成。
    bool solved = false;
    std::vector<double> dyn_seg_values;
    std::vector<int> dyn_cur_indices;
    for (int iter = 0; iter <= replan_max_iter_; ++iter) {
        std::vector<int> cur_indices;
        std::vector<double> obstacle_values = queryObstacleValues(query_pts, cur_indices);
        opti_->set_value(obstacle_param_,
                         casadi::DM::reshape(DM(obstacle_values),
                                             3 * nearest_obs_num_, N_ + 1));

        // 动态障碍物：按查询点查询每个预测步对应的最近线段并填充端点参数
        if (!dyn_obs.empty()) {
            dyn_seg_values = queryDynamicObstacleSegments(
                query_pts, dyn_obs, dyn_cur_indices);
            opti_->set_value(
                dyn_obs_seg_param_,
                casadi::DM::reshape(
                    DM(dyn_seg_values),
                    6 * static_cast<int>(dyn_obs.size()), N_ + 1));
        }

        // 热启动（上一轮解）
        if (has_last_solution_ &&
            last_pred_x_.rows() == 8 && last_pred_x_.cols() == N_ + 1) {
            std::vector<double> x0_vec(last_pred_x_.data(),
                                       last_pred_x_.data() + 8 * (N_ + 1));
            opti_->set_initial(X, casadi::DM::reshape(DM(x0_vec), 8, N_ + 1));
            if (last_u_.rows() == 4 && last_u_.cols() == N_) {
                std::vector<double> u0_vec(last_u_.data(),
                                           last_u_.data() + 4 * N_);
                opti_->set_initial(U, casadi::DM::reshape(DM(u0_vec), 4, N_));
            }
        }

        try {
            solution_ = std::make_unique<casadi::OptiSol>(opti_->solve());
            solved = true;
        } catch (const std::exception& e) {
            std::cerr << "Solver failed: " << e.what() << std::endl;
            return false;
        }

        // 记录解（热启动 + 下一轮重查询）
        std::vector<double> xv = solution_->value(X).get_elements();
        last_pred_x_.resize(8, N_ + 1);
        std::copy(xv.begin(), xv.end(), last_pred_x_.data());
        std::vector<double> uv = solution_->value(U).get_elements();
        last_u_.resize(4, N_);
        std::copy(uv.begin(), uv.end(), last_u_.data());
        has_last_solution_ = true;

        if (iter >= replan_max_iter_) {
            break;
        }

        // 用刚解出的预测轨迹生成下一轮查询点，最近静态障碍点/动态线段索引变化则重解
        std::vector<int> check_indices, check_dyn_indices;
        const std::vector<Eigen::Vector3d> next_query_pts =
            buildQueryPoints(&last_pred_x_);
        std::vector<double> check_values =
            queryObstacleValues(next_query_pts, check_indices);
        (void)check_values;
        bool dyn_unchanged = true;
        if (!dyn_obs.empty()) {
            std::vector<double> check_dyn_values = queryDynamicObstacleSegments(
                next_query_pts, dyn_obs, check_dyn_indices);
            (void)check_dyn_values;
            dyn_unchanged = (check_dyn_indices == dyn_cur_indices);
        }
        if (check_indices == cur_indices && dyn_unchanged) {
            break;
        }
        query_pts = next_query_pts;
    }

    // 同步可视化用障碍点：以最后一次求解实际使用的查询点为准
    std::vector<int> final_indices;
    std::vector<double> final_values = queryObstacleValues(query_pts, final_indices);
    (void)final_values;
    (void)final_indices;

    return solved;
}

vector<double> Mpc::getFirstU() {
    vector<double> res;
    if (!solution_) return {0,0,0,0};
    auto u_val = solution_->value(U);
    //显式类型转换,将转换后的值追加到res向量末尾
    res.push_back(static_cast<double>(u_val(0, 0))); // ux
    res.push_back(static_cast<double>(u_val(1, 0))); // uy
    res.push_back(static_cast<double>(u_val(2, 0))); // uz
    res.push_back(static_cast<double>(u_val(3, 0))); // uw
    return res;
}

vector<double> Mpc::getPredictX() {
    vector<double> res;
    if (!solution_) return {};
    auto predict_x = solution_->value(X);
    for (int i = 0; i <= N_; ++i) {
        res.push_back(static_cast<double>(predict_x(0, i)));
        res.push_back(static_cast<double>(predict_x(1, i)));
        res.push_back(static_cast<double>(predict_x(2, i)));
        res.push_back(static_cast<double>(predict_x(3, i)));
        res.push_back(static_cast<double>(predict_x(4, i)));
        res.push_back(static_cast<double>(predict_x(5, i)));
        res.push_back(static_cast<double>(predict_x(6, i)));
        res.push_back(static_cast<double>(predict_x(7, i)));
    }
    return res;
}
