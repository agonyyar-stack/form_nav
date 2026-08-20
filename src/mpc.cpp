#include "form_nav/mpc.h" //引入 Mpc 类的声明
#include "form_nav/kd_tree_map.h"
#include <algorithm>
#include <cmath>
Mpc::Mpc() { 
    N_ = 10; //预测步数
    dt_ = 0.05; //时间步长
    u_max_ = 6; //线速度上限
    w_max_ = 2; //角速度上限
    u_min_ = - u_max_;
    w_min_ = - w_max_;
    //权重矩阵Q,F
    Q_ = DM::diag(DM{20000, 20000, 10000, 100}); // 全局轨迹跟踪权重（可由 mpc/w_track 覆盖）
    F_ = DM::diag(DM{100, 100, 100, 100});       // 控制输入惩罚
    R_d_ = DM::diag(DM{100, 100, 100, 25});      // 控制增量惩罚（抑制重规划导致的指令抖动）
    // 滚动子目标参数
    lookahead_distance_ = 2.0;                   // 子目标沿弧长前瞻距离 (m)
    a_brake_ = 0.8 * u_max_;                     // 制动加速度（保守最大减速度）
    has_rolling_goal_ = false;
    // 编队误差权重：与跟踪权重 Q_ 拆分为独立项，默认比 Q_ 小一个数量级
    Q_f_ = DM::diag(DM{2000, 2000, 1000, 10});
    //运动学参数
    Lx = 0.83; Kx = 1;
    Ly = 0.77; Ky = 1;
    Lz = 0.5; Kz = 1;
    Lw = 0.51; Kw = M_PI/180;
    beta = DM::diag(DM{-1/Lx, -1/Ly, -1/Lz, -1/Lw});
    gama = DM::diag(DM{Kx/Lx, Ky/Ly, Kz/Lz, Kw/Lw});
    kinematic_equation_ = setKinematicEquation(); 
    // 编队偏移量由外部 setFormationOffsets() 注入，默认自身偏移为 [0,0,0,0]
    // ESDF 障碍物避障参数
    d_safe_oa = 0.6;       // 障碍物安全距离 (m)
    d_critical_oa = 0.3;   // 临界距离 (m), 低于此距离急剧惩罚
    Wo = 50000;            // 障碍物避障权重 (标量)
    esdf_ptr_ = nullptr;   // ESDF 插值器指针，由外部设置
    // KD-tree 静态避障参数（第三步）
    kd_tree_ptr_ = nullptr;   // 障碍物 KD-tree 指针，由外部设置
    nearest_obs_num_ = 3;     // 每个预测步查询的最近障碍点数量
    drone_radius_ = 0.35;     // 无人机半径 (m)
    beta_ = 8.0;              // softplus 势垒系数（实测 8 比 32 更容易收敛）
    obs_floor_ = 0.05;        // 势垒下限（m）
    replan_max_iter_ = 2;     // 最近点重查询重规划最大迭代次数
    use_obstacle_velocity_term_ = true; // 速度方向项默认开启
    ipopt_max_iter_ = 100;    // IPOPT 最大迭代次数
    ipopt_tol_ = 1e-6;        // IPOPT 收敛容差
    // Opti 复用相关
    has_last_solution_ = false;
    built_N_ = -1;
    built_dt_ = -1.0;
    built_u_max_ = -1.0;
    built_w_max_ = -1.0;
    built_a_brake_ = -1.0;
    built_neighbor_count_ = 0;
    built_nearest_obs_num_ = -1;
    built_ipopt_max_iter_ = -1;
    built_ipopt_tol_ = -1.0;
    //碰撞参数
    d_safe_ca = 0.3; //编队内安全距离
    Wc = 1000; //相互碰撞权重
    //反步法控制器增益参数
    k1 = DM::diag(DM({7, 7, 7, 6}));
    k2 = DM::diag(DM({2, 2, 2, 2}));
    // 全局轨迹初始化
    has_global_traj_ = false;
}
Mpc::~Mpc() {}

Function Mpc::setKinematicEquation() {
    //定义符号向量
    //状态向量
    MX x = MX::sym("x");
    MX y = MX::sym("y");
    MX z = MX::sym("z");
    MX w = MX::sym("w");
    MX vx = MX::sym("vx");
    MX vy = MX::sym("vy");
    MX vz = MX::sym("vz");
    MX vw = MX::sym("vw");
    MX state_vars = MX::vertcat({x, y, z, w, vx, vy, vz, vw});
    //控制向量
    MX ux = MX::sym("ux");
    MX uy = MX::sym("uy");
    MX uz = MX::sym("uz");
    MX uw = MX::sym("uw");
    MX control_vars = MX::vertcat({ux, uy, uz, uw});

    //运动学方程
    MX rhs = MX::vertcat({
        cos(w)*vx-sin(w)*vy, 
        sin(w)*vx+cos(w)*vy, 
        vz, vw,
        -vx/Lx+Kx*ux/Lx, 
        -vy/Ly+Ky*uy/Ly, 
        -vz/Lz+Kz*uz/Lz, 
        -vw/Lw+Kw*uw/Lw});
    return Function("kinematic_equation", {state_vars, control_vars}, {rhs}); 
}
// 保存按累计时间排列的全局轨迹。局部参考在每次 solve 前进行数值采样，
// 避免把 CasADi interpolant 放进 Opti 符号图后被 expand 转为 SX。
void Mpc::setGlobalTrajectory(const std::vector<GlobalWaypoint>& traj) {
    global_traj_ = traj;
    has_global_traj_ = !traj.empty();
    if (!has_global_traj_) return;

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

    std::cout << "[MPC] Global trajectory set with " << traj.size() << " waypoints, t ∈ ["
              << traj.front().t << ", " << traj.back().t << "]" << std::endl;
}

GlobalWaypoint Mpc::interpolateGlobalTrajectory(double t) const {
    if (global_traj_.empty()) return GlobalWaypoint{};
    if (t <= global_traj_.front().t) return global_traj_.front();
    if (t >= global_traj_.back().t) return global_traj_.back();

    auto upper = std::lower_bound(
        global_traj_.begin(), global_traj_.end(), t,
        [](const GlobalWaypoint& waypoint, double query_time) {
            return waypoint.t < query_time;
        });
    const GlobalWaypoint& next = *upper;
    const GlobalWaypoint& previous = *(upper - 1);
    const double interval = next.t - previous.t;
    const double ratio = interval > 0.0 ? (t - previous.t) / interval : 0.0;
    const auto lerp = [ratio](double from, double to) {
        return from + ratio * (to - from);
    };

    GlobalWaypoint result;
    result.t = t;
    result.x = lerp(previous.x, next.x);
    result.y = lerp(previous.y, next.y);
    result.z = lerp(previous.z, next.z);
    result.yaw = lerp(previous.yaw, next.yaw);
    result.vx = lerp(previous.vx, next.vx);
    result.vy = lerp(previous.vy, next.vy);
    result.vz = lerp(previous.vz, next.vz);
    result.vyaw = lerp(previous.vyaw, next.vyaw);
    return result;
}

GlobalWaypoint Mpc::selectRollingGoal(double t0) {
    GlobalWaypoint goal;
    has_rolling_goal_ = false;
    if (!has_global_traj_ || global_traj_.empty() ||
        path_arclength_.size() != global_traj_.size()) {
        return goal;
    }

    const double t_front = global_traj_.front().t;
    const double t_end = global_traj_.back().t;
    const double t_clamped = std::min(std::max(t0, t_front), t_end);

    // 按累计时间二分找到不超过 t_clamped 的最近航点索引
    auto upper = std::lower_bound(
        global_traj_.begin(), global_traj_.end(), t_clamped,
        [](const GlobalWaypoint& wp, double query_time) {
            return wp.t < query_time;
        });
    size_t i0 = static_cast<size_t>(std::distance(global_traj_.begin(), upper));
    if (i0 > 0 && global_traj_[i0].t > t_clamped) --i0;

    // 已到轨迹末端：子目标即终点
    if (i0 >= global_traj_.size() - 1) {
        goal = global_traj_.back();
        goal.vyaw = 0.0;
        has_rolling_goal_ = true;
        rolling_goal_ = goal;
        return goal;
    }

    // 沿弧长前推 lookahead_distance_
    const double target_arc = path_arclength_[i0] + std::max(0.0, lookahead_distance_);
    size_t i1 = i0;
    while (i1 + 1 < global_traj_.size() &&
           path_arclength_[i1 + 1] < target_arc) {
        ++i1;
    }
    if (i1 >= global_traj_.size() - 1) {
        goal = global_traj_.back();
    } else {
        const double seg_len = path_arclength_[i1 + 1] - path_arclength_[i1];
        const double ratio = seg_len > 0.0
                                 ? (target_arc - path_arclength_[i1]) / seg_len
                                 : 0.0;
        const double t_g = global_traj_[i1].t +
                           ratio * (global_traj_[i1 + 1].t - global_traj_[i1].t);
        goal = interpolateGlobalTrajectory(std::min(t_g, t_end));
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
    MX v_i = current_states(Slice(4, 8));   // [vx, vy, vz, vw] body
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

    MX temp = mtimes(R_i_dot, v_i) + mtimes(R_i, mtimes(beta, v_i) + mtimes(gama, U_k));

    term3_bracket += - (d_i + b_i) * temp;

    term3_bracket += sum;

    term3_bracket += mtimes(k1, (S2 - mtimes(k1, S1)));

    // MX leader_mask = MX::if_else(b_i > 0, 1.0, 0.0);
    term3_bracket += b_i * y_d_ddot;

    MX term3 = mtimes(S2.T(), term3_bracket);

    return term1 - term2 + term3;
#endif
}

MX Mpc::esdfObstacleAvoidance(MX current_states) {
    MX pos_i = current_states(Slice(0, 3));  // [x, y, z] 世界坐标系
    MX H_oa = MX(0.0);

    if (esdf_ptr_ == nullptr || !esdf_ptr_->isInitialized()) {
        // 无 ESDF 数据时返回零惩罚
        return H_oa;
    }

    MX dist = esdf_ptr_->getDistMX(pos_i);

    // 避障惩罚函数:
    // 当 dist > d_safe_oa: 安全，惩罚 ≈ 0
    // 当 d_critical_oa < dist <= d_safe_oa: 平滑过渡区，log 型惩罚
    // 当 dist <= d_critical_oa: 临界区，指数型惩罚（更强的排斥力）
    //
    // 使用平滑近似避免 log 和 max 的不可导区域:
    //   h_oa = Wo * softplus(d_safe_oa - dist) / d_safe_oa
    // 其中 softplus(x) = log(1 + exp(k*x)) / k 是 ReLU 的平滑近似

    double k_smooth = 10.0;  // softplus 平滑系数
    MX margin = d_safe_oa - dist; 
    MX softplus_val = log(1.0 + exp(k_smooth * margin)) / k_smooth;

    // 对临界区域添加额外惩罚
    MX critical_margin = d_critical_oa - dist;
    MX critical_penalty = log(1.0 + exp(k_smooth * critical_margin)) / k_smooth;

    H_oa = Wo * (softplus_val + 5.0 * critical_penalty);

    return H_oa;
}

MX Mpc::kdtreeObstacleAvoidance(MX current_states, MX reference_k, MX obstacle_k) {
    MX H_oa = MX(0.0);
    if (kd_tree_ptr_ == nullptr || kd_tree_ptr_->empty()) {
        // 无 KD-tree 数据时返回零惩罚（与 ESDF 缺数据时行为一致）
        return H_oa;
    }

    MX pos = current_states(Slice(0, 3));       // 世界系位置
    MX yaw = current_states(3);
    MX v_body = current_states(Slice(4, 7));    // 机体系速度 [vx, vy, vz]
    // 机体速度 -> 世界速度（速度方向项需要世界系）
    MX v_world = MX::vertcat({
        cos(yaw) * v_body(0) - sin(yaw) * v_body(1),
        sin(yaw) * v_body(0) + cos(yaw) * v_body(1),
        v_body(2)});

    // 期望物理位置：分布式规划下 reference_k 已是本机物理参考点
    // （障碍查询与势垒阈值都以此为准），不再叠加编队偏移。
    MX ref_pos = reference_k(Slice(0, 3));

    // 势垒阈值：参考点在当前最近障碍点处的距离（扣除机体半径），低于下限时
    // 取下限。这样沿全局路径（参考点即期望位置）势垒为零代价零梯度，MPC 保持
    // 平滑；只有预测轨迹比参考点更贴近障碍（或参考点本身压进障碍）时才排斥。
    MX o0 = obstacle_k(Slice(0, 3));
    MX d_ref = MX::norm_2(ref_pos - o0) - drone_radius_;
    MX thr = MX::fmax(d_ref, obs_floor_);

    for (int m = 0; m < nearest_obs_num_; ++m) {
        MX o = obstacle_k(Slice(3 * m, 3 * m + 3));
        MX d_pred = MX::norm_2(pos - o) - drone_radius_;
        // 二次斜坡势垒（C1）：d_pred < thr 时以 (excess)^2 排斥，边界处值与
        // 梯度都连续，避免 fmax 线性斜坡在激活边界处把 IPOPT 步长计算搞挂；
        // 边界外（excess<=0）严格为零代价零梯度，不与跟踪代价沿路径对抗。
        MX excess = thr - d_pred;
        MX relu = MX::fmax(0.0, excess);
        MX term = Wo * relu * relu;
        if (use_obstacle_velocity_term_) {
            // 速度方向项（可选）：沿障碍方向的速度分量，用 sqrt(dot^2+eps)
            // 光滑化避免 |·| 在 0 处不可导。默认关闭（有全局路径引导时不需要）。
            MX dir = (o - pos) / (MX::norm_2(o - pos) + 1e-6);
            MX v_dot_dir = MX::dot(v_world, dir);
            MX v_toward = MX::sqrt(v_dot_dir * v_dot_dir + 1e-6);
            term *= v_toward;
        }
        H_oa += term;
    }
    return H_oa;
}

std::vector<double> Mpc::queryObstacleValues(
    const std::vector<Eigen::Vector3d>& query_pts,
    std::vector<int>& out_indices) {
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

MX Mpc::collisionAvoidance(MX reference_state, int k_step, std::map<int, MX>& neighbor_params, MX current_states){
    MX pos_i = current_states(Slice(0, 3));
    MX H_ca = 0.0;

    for (auto const& [j_id, j_param] : neighbor_params) {
        // a_ij = 1（因为仅遍历邻居）
        MX y_j = j_param(Slice(0, 4), k_step);
        MX pos_j = y_j(Slice(0, 3));

        MX diff = pos_i - pos_j;
        MX dist = MX::norm_2(diff);

        MX h_ca = Wc *(1.0 / (1.0 + exp(dist - d_safe_ca)));
        H_ca += h_ca;
    }

    return H_ca;
}

// 构建（或复用）Opti 优化问题。配置（预测时域、时间步、输入限幅、
// 邻居数量、最近障碍点数量）变化时重建；否则复用同一实例，仅更新参数值。
// 复用 + 热启动是第三步重规划循环能够在 20 Hz 控制率下运行的前提。
bool Mpc::buildOpti(const std::vector<UavState>& neighbors) {
    if (opti_ &&
        built_N_ == N_ && std::abs(built_dt_ - dt_) < 1e-9 &&
        std::abs(built_u_max_ - u_max_) < 1e-9 &&
        std::abs(built_w_max_ - w_max_) < 1e-9 &&
        std::abs(built_a_brake_ - a_brake_) < 1e-9 &&
        built_neighbor_count_ == neighbors.size() &&
        built_nearest_obs_num_ == nearest_obs_num_ &&
        built_ipopt_max_iter_ == ipopt_max_iter_ &&
        std::abs(built_ipopt_tol_ - ipopt_tol_) < 1e-12) {
        return true;
    }

    opti_ = std::make_unique<Opti>();
    Slice all; //切片工具，代表所有索引

    X = opti_->variable(8, N_ + 1); //状态序列，位姿速度
    U = opti_->variable(4, N_); //控制序列,速度指令(机体坐标系)

    // 参数：滚动子目标参考、当前状态、最近障碍点（KD-tree 查询在求解器外完成）、终点
    reference_param_ = opti_->parameter(4, N_ + 1);
    X_cur_param_ = opti_->parameter(8, 1);
    obstacle_param_ = opti_->parameter(3 * nearest_obs_num_, N_ + 1);
    goal_param_ = opti_->parameter(3, 1);

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
        // 全局轨迹跟踪误差：本机物理参考 vs 预测状态（位置 + 偏航）
        MX track_err = X_k(Slice(0, 4)) - reference_k(Slice(0, 4));
#if 0
        MX H_oa = esdfObstacleAvoidance(X_k); // 基于ESDF的障碍物避障（旧方案，已停用）
        cost += H_oa;
#endif
        MX H_ca = collisionAvoidance(reference_k, k, neighbor_params_, X_k);
        cost += H_ca;

        // KD-tree 静态避障代价（第三步）：障碍点是参数，代价处处可导
        MX H_oa_kd = kdtreeObstacleAvoidance(X_k, reference_k,
                                             obstacle_param_(all, k));
        cost += H_oa_kd;

        if (k < N_) {
            // 跟踪项与编队项独立加权，互不耦合
            // cost += MX::mtimes({track_err.T(), Q_, track_err});
            cost += MX::mtimes({S_f.T(), Q_f_, S_f});
            
            MX U_k = U(all, k);
            cost += MX::mtimes({U_k.T(), F_, U_k}); // 控制输入惩罚,已停用
        } else {
            cost += MX::mtimes({track_err.T(), Q_, track_err});
            cost += MX::mtimes({S_f.T(), Q_f_, S_f});
        }
    }

    // 控制增量惩罚：抑制重规划导致的指令抖动，平滑预测轨迹。
    for (int k = 0; k < N_ - 1; ++k) {
        MX dU = U(all, k + 1) - U(all, k);
        cost += MX::mtimes({dU.T(), R_d_, dU});
    }
    opti_->minimize(cost);

    // 动力学约束
    for (int k = 0; k < N_; ++k) {
        vector<MX> inputs = {X(all, k), U(all, k)}; //U为机体坐标系
        MX x_dot = kinematic_equation_(inputs)[0]; //运动学方程唯一输出
        MX x_next = X(all, k) + x_dot * dt_;
        opti_->subject_to(X(all, k + 1) == x_next);
    }

    // 初始状态约束
    opti_->subject_to(X(all, 0) == X_cur_param_);

    // 制动不等式约束：保证预测轨迹在终点前能减速停下。
    //   ||v_world_k||^2 <= 2 * a_brake * d_g_k
    // d_g_k 为第 k 步预测位置到终点的直线距离，a_brake 为保守最大制动加速度。
    for (int k = 0; k <= N_; ++k) {
        MX X_k = X(all, k);
        MX w_k = X_k(3);
        MX v_k = X_k(Slice(4, 8));
        MX v_world = MX::vertcat({
            cos(w_k) * v_k(0) - sin(w_k) * v_k(1),
            sin(w_k) * v_k(0) + cos(w_k) * v_k(1),
            v_k(2)});
        MX d_g = MX::norm_2(X_k(Slice(0, 3)) - goal_param_);
        MX v_norm_sq = MX::dot(v_world, v_world);
        opti_->subject_to(v_norm_sq <= 2.0 * a_brake_ * d_g);
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
    built_a_brake_ = a_brake_;
    built_neighbor_count_ = neighbors.size();
    built_nearest_obs_num_ = nearest_obs_num_;
    built_ipopt_max_iter_ = ipopt_max_iter_;
    built_ipopt_tol_ = ipopt_tol_;
    has_last_solution_ = false;
    return true;
}

bool Mpc::solve(double t, const std::vector<UavState>& neighbors, Eigen::VectorXd current_states) {
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

    if (!buildOpti(neighbors)) {
        return false;
    }

    // 滚动子目标：沿弧长从最近投影点前推 lookahead_distance_。
    // 全局轨迹只提供方向引导，整个预测时域跟踪同一个子目标点。
    rolling_goal_ = selectRollingGoal(t);
    if (!has_rolling_goal_) {
        std::cerr << "[MPC] Failed to select rolling goal." << std::endl;
        return false;
    }

    std::vector<double> reference_values;
    reference_values.reserve(4 * (N_ + 1));
    for (int k = 0; k <= N_; ++k) {
        reference_values.insert(reference_values.end(),
                                {rolling_goal_.x, rolling_goal_.y,
                                 rolling_goal_.z, 0.0});
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

    // 查询点：固定用滚动子目标，不随上一轮解变化。若用上一轮优化轨迹作为
    // 查询点，障碍物参数会随解抖动，形成"查询点→障碍参数→解→查询点"的
    // 正反馈，导致轨迹杂乱；子目标缓慢变化，障碍参数稳定，MPC 沿全局路径
    // 保持平滑，仅在预测轨迹逼近障碍时被排斥。
    std::vector<Eigen::Vector3d> query_pts(N_ + 1);
    for (int k = 0; k <= N_; ++k) {
        query_pts[k] = Eigen::Vector3d(rolling_goal_.x, rolling_goal_.y,
                                       rolling_goal_.z);
    }

    // 最近点重查询重规划循环：查询点固定为参考位置，若因地图更新导致最近点
    // 索引变化（如后续动态障碍），带新参数重解；静态地图下通常一次求解即可。
    // 热启动：每轮用上一轮解作为初值，保证 20 Hz 控制率下求解耗时可控。
    bool solved = false;
    for (int iter = 0; iter <= replan_max_iter_; ++iter) {
        std::vector<int> cur_indices;
        std::vector<double> obstacle_values = queryObstacleValues(query_pts, cur_indices);
        opti_->set_value(obstacle_param_,
                         casadi::DM::reshape(DM(obstacle_values),
                                             3 * nearest_obs_num_, N_ + 1));

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

        // 用相同的参考查询点重新查询，比较最近点索引是否变化（地图更新时
        // 会变化）；未变化则说明障碍参数与查询一致，无需再规划。
        std::vector<int> check_indices;
        std::vector<double> check_values = queryObstacleValues(query_pts, check_indices);
        (void)check_values;
        if (check_indices == cur_indices) {
            break;
        }
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
