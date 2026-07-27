#include "form_nav/mpc.h" //引入 Mpc 类的声明
#include <algorithm>
Mpc::Mpc() { 
    N_ = 10; //预测步数
    dt_ = 0.05; //时间步长
    u_max_ = 6; //线速度上限
    w_max_ = 2; //角速度上限
    u_min_ = - u_max_;
    w_min_ = - w_max_;
    //权重矩阵Q,F
    Q_ = DM::diag(DM{20000, 20000, 10000, 100}); //索引之前初始化size
    F_ = DM::diag(DM{100, 100, 100, 100});
    //运动学参数
    Lx = 0.83; Kx = 1;
    Ly = 0.77; Ky = 1;
    Lz = 0.5; Kz = 1;
    Lw = 0.51; Kw = M_PI/180;
    beta = DM::diag(DM{-1/Lx, -1/Ly, -1/Lz, -1/Lw});
    gama = DM::diag(DM{Kx/Lx, Ky/Ly, Kz/Lz, Kw/Lw});
    kinematic_equation_ = setKinematicEquation(); 
    // 默认不直接跟踪全局轨迹
    is_tracked_ = false;
    // 编队偏移量由外部 setFormationOffsets() 注入，默认自身偏移为 [0,0,0,0]
    // ESDF 障碍物避障参数
    d_safe_oa = 0.6;       // 障碍物安全距离 (m)
    d_critical_oa = 0.3;   // 临界距离 (m), 低于此距离急剧惩罚
    Wo = 50000;            // 障碍物避障权重 (标量)
    esdf_ptr_ = nullptr;   // ESDF 插值器指针，由外部设置
    //碰撞参数
    d_safe_ca = 0.3; //编队内安全距离
    Wc = 1000; //相互碰撞权重
    //反步法控制器增益参数
    k1 = DM::diag(DM({7, 7, 7, 6}));
    k2 = DM::diag(DM({2, 2, 2, 2}));
    // 全局轨迹初始化
    has_global_traj_ = false;
    // 跟踪模式: 1=仅位置 (默认)
    tracking_model_ = 1;
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

    // 虚拟领航者项
    double b_i = is_tracked_ ? 1.0 : 0.0;
    if (b_i > 0 && has_global_traj_) {
        MX y_d = reference_state(Slice(0,4)); //全局
        // MX y_d_dot = y_r(Slice(4,8)); //全局

        S1_accum += b_i * (y_d - (p_i - M_i));
        // S1_accum += b_i * (y_d - (p_i - M_i));

        // S1_dot_accum += b_i * (y_d_dot - (p_i_dot - G_i_dot));
    }

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

    double b_i = is_tracked_ ? 1.0 : 0.0;

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

    // 暂时禁用“与 tracked UAV/参考点避碰”项。单机需要跟踪该参考点，
    // 将参考点同时作为碰撞对象会在目标位置产生最大碰撞惩罚。
    // 原实现保留如下，待多机真实领航者模型接入后再恢复：
#if 0
    if (has_global_traj_) {
        MX y_d = reference_state(Slice(0, 3));
        MX diff = pos_i - y_d;
        MX dist = MX::norm_2(diff);
        MX h_ca = Wc *(1.0 / (1.0 + exp(dist - d_safe_ca)));
        H_ca += h_ca;
    }
#endif

    return H_ca;
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
    
    Opti opti = Opti(); //创建优化问题实例

    Slice all; //切片工具，代表所有索引

    X = opti.variable(8, N_ + 1); //状态序列，位姿速度
    U = opti.variable(4, N_); //控制序列,速度指令(机体坐标系)

    // 从最近路径点时间开始，对预测时域进行数值线性插值。参考轨迹只是
    // 优化参数，不参与决策，因此无需把插值器嵌入 CasADi 符号图。
    MX reference_param = opti.parameter(8, N_ + 1);
    std::vector<double> reference_values;
    reference_values.reserve(8 * (N_ + 1));
    for (int k = 0; k <= N_; ++k) {
        const GlobalWaypoint reference = interpolateGlobalTrajectory(t + k * dt_);
        reference_values.insert(reference_values.end(), {
            reference.x, reference.y, reference.z, reference.yaw,
            reference.vx, reference.vy, reference.vz, reference.vyaw});
    }
    opti.set_value(reference_param,
                   casadi::DM::reshape(DM(reference_values), 8, N_ + 1));

    // 初始状态参数
    MX X_cur_param = opti.parameter(8, 1); //当前状态
    // 先转换为DM格式因为 casadi 的 set_value 需要 DM 类型
    vector<double> cur_state_vec(current_states.data(), current_states.data() + 8); //迭代器:将矩阵中的所有元素按顺序放入
    opti.set_value(X_cur_param, DM(cur_state_vec)); 

    // 邻居轨迹参数处理
    std::map<int, MX> neighbor_params;
    for (const auto& nb : neighbors) {
        MX nb_p = opti.parameter(8, N_ + 1);
        std::vector<double> nb_vec(nb.predicted_states.data(), nb.predicted_states.data() + nb.predicted_states.size());
        // Eigen 默认列优先，CasADi 也是列优先填充
        opti.set_value(nb_p, casadi::DM::reshape(DM(nb_vec), 8, N_ + 1));
        neighbor_params[nb.id] = nb_p;
    }

    //构建代价函数
    MX cost = 0;
    for (int k = 0; k <= N_; ++k) { // N+1
        MX X_k = X(all, k);
        MX reference_k = reference_param(all, k);

        ErrorTerms errors1 = formationError(k, reference_k, neighbor_params, X_k);
        MX S_i1 = errors1.S1;
        // 第一阶段仅验证全局路径跟踪；暂时停用 ESDF 和编队碰撞代价，原实现保留。
#if 0
        MX H_oa = esdfObstacleAvoidance(X_k); // 基于ESDF的障碍物避障
        MX H_ca = collisionAvoidance(reference_k, k, neighbor_params, X_k);
#endif

        if (k < N_) {
            cost += MX::mtimes({S_i1.T(), Q_, S_i1});

            // 模式2: 位置+速度跟踪 — 增加速度误差惩罚
            if (tracking_model_ >= 2 && has_global_traj_) {
                MX v_ref = reference_k(Slice(4, 8));  // 参考速度 [vx,vy,vz,vyaw]
                MX v_i = X_k(Slice(4, 8));    // 机体速度
                MX w_i = X_k(3);
                // 机体速度 → 世界速度
                MX v_world_x = cos(w_i)*v_i(0) - sin(w_i)*v_i(1);
                MX v_world_y = sin(w_i)*v_i(0) + cos(w_i)*v_i(1);
                MX v_world_z = v_i(2);
                MX v_world_w = v_i(3);
                MX v_world = MX::vertcat({v_world_x, v_world_y, v_world_z, v_world_w});
                MX v_err = v_world - v_ref;
                cost += MX::mtimes({v_err.T(), F_, v_err});  // 复用 F_ 作为速度跟踪权重
            }

            // 障碍物/邻机避碰代价在后续阶段恢复。
#if 0
            cost += (H_oa + H_ca);
#endif

            MX U_k = U(all, k);
            cost += MX::mtimes({U_k.T(), F_, U_k});
        } else {
            cost += MX::mtimes({S_i1.T(), Q_, S_i1});

            // 终端也加速度跟踪 (模式2)
            if (tracking_model_ >= 2 && has_global_traj_) {
                MX v_ref = reference_k(Slice(4, 8));
                MX v_i = X_k(Slice(4, 8));
                MX w_i = X_k(3);
                MX v_world_x = cos(w_i)*v_i(0) - sin(w_i)*v_i(1);
                MX v_world_y = sin(w_i)*v_i(0) + cos(w_i)*v_i(1);
                MX v_world_z = v_i(2);
                MX v_world_w = v_i(3);
                MX v_world = MX::vertcat({v_world_x, v_world_y, v_world_z, v_world_w});
                MX v_err = v_world - v_ref;
                cost += MX::mtimes({v_err.T(), F_, v_err});
            }

#if 0
            cost += (H_oa + H_ca);
#endif
        }
    }
    opti.minimize(cost);
    
    // 动力学约束
    for (int k = 0; k < N_; ++k) {
        vector<MX> inputs = {X(all, k), U(all, k)}; //U为机体坐标系
        MX x_dot = kinematic_equation_(inputs)[0]; //运动学方程唯一输出
        MX x_next = X(all, k) + x_dot * dt_;
        opti.subject_to(X(all, k + 1) == x_next);
    }

    //初始状态约束
    opti.subject_to(X(all, 0) == X_cur_param);

    //输入约束
    opti.subject_to(u_min_ <= U(0, all) <= u_max_);
    opti.subject_to(u_min_ <= U(1, all) <= u_max_);
    opti.subject_to(u_min_ <= U(2, all) <= u_max_);
    opti.subject_to(w_min_ <= U(3, all) <= w_max_);

    // // 稳定性约束(仅预测起点，使用 reference_param(all, 0))
    // MX X_0 = X(all, 0);
    // MX U_k = U(all, 0);
    // MX reference_0 = reference_param(all, 0);
    // ErrorTerms errors2 = formationError(0, reference_0, neighbor_params, X_0);
    // MX S1 = errors2.S1; 
    // MX S2 = errors2.S2;

    // MX V_iLMPC_dot = ViLmpcDot(0, reference_0, neighbor_params, X_0, U_k);
    // MX V_i2_dot = - casadi::MX::mtimes({S1.T(), k1, S1}) - casadi::MX::mtimes({S2.T(), k2, S2});

    // opti.subject_to(V_iLMPC_dot <= V_i2_dot);

    //求解器设置
    casadi::Dict solver_opts; //创建求解器选项
    solver_opts["expand"] = true; //展开表达式以加快求解速度
    solver_opts["ipopt.max_iter"] = 100; //最大迭代次数
    solver_opts["ipopt.print_level"] = 0; //不打印迭代信息
    solver_opts["print_time"] = 0; //不打印时间信息
    solver_opts["ipopt.acceptable_tol"] = 1e-6; //梯度容差，收敛精度设定
    solver_opts["ipopt.acceptable_obj_change_tol"] = 1e-6; //目标函数变化容差

    opti.solver("ipopt", solver_opts); //设置求解器为ipopt并应用求解器选项
    //std::make_unique<T>(args) 动态分配 T 类型对象，返回 unique_ptr<T>
    try {
        solution_ = std::make_unique<casadi::OptiSol>(opti.solve()); 
        return true;
    } catch(const std::exception& e) {
        std::cerr << "Solver failed: " << e.what() << std::endl;
        return false;
    }
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
