#ifndef MPC_H 
#define MPC_H //头文件保护符，确保这个文件在任何编译单元中只被包含一次，防止重复定义错误

#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <iostream>
#include <chrono>
#include <map>

using namespace std;
using namespace casadi;

class KdTreeMap; // 前向声明，避免 mpc.h 引入点云/树头文件

struct ErrorTerms {
    MX S1;       // 编队误差
    MX S1_dot;   // 编队误差导数
    MX S2;       // 辅助误差
    MX sum_term; // 邻居部分的求和项
};

//邻居轨迹及id
struct UavState {
    Eigen::MatrixXd predicted_states; 
    int id;
};

// 全局轨迹路径点 (全局规划器输出, 含空间+速度)
struct GlobalWaypoint {
    double x, y, z;     // 位置 (m)
    double yaw;         // 偏航角 (rad)
    double vx, vy, vz;  // 速度 (m/s, 世界坐标系)
    double vyaw;        // 偏航角速度 (rad/s)
};

class Mpc
{
private: //存储了 MPC 算法运行时的配置和状态
    //mpc params
    int N_;  //预测步数horizon
    double dt_;  //时间步长step

    //constrains
    double u_max_, u_min_; //线加速度
    double w_max_, w_min_; //角加速度

    //权重矩阵
    DM Q_, F_, R_d_;
    // 编队误差权重（与子目标惩罚权重 Q_ 独立，避免两者耦合打架）
    DM Q_f_;

    //运动学方程参数
    double Lx, Kx;
    double Ly, Ky;
    double Lz, Kz;
    double Lw, Kw;
    DM beta;
    DM gama;

    // 自身 ID
    int my_id_;
    // 所有无人机的编队偏移量 {id → [x, y, z, yaw]}
    std::map<int, std::vector<double>> formation_offsets_;

    //平滑切换函数参数
    double h_bar;
    double T_switch;
    double delta;

    // KD-tree 对数势垒避障权重
    double Wo;             // 障碍物避障权重 (标量)

    // KD-tree 静态避障参数（第三步）
    KdTreeMap* kd_tree_ptr_;   // 指向障碍物 KD-tree 的指针 (外部管理生命周期)
    int nearest_obs_num_;      // 每个预测步查询的最近障碍点数量 M
    double drone_radius_;      // 无人机半径 (m)，有效距离 = ||p-o|| - r_drone
    double safe_dist_;         // 对数势垒最小安全距离 L (m)，有效距离口径（扣无人机半径）
    double detect_range_;      // 对数势垒探测范围 D̄ (m)，须大于 safe_dist_
    int replan_max_iter_;      // 最近点重查询重规划的最大迭代次数

    //碰撞参数
    double d_safe_ca; // 编队内安全距离 COLLISION AVOIDANCE
    double Wc;        // 相互碰撞权重

    MX X; //状态变量
    MX U; //控制变量

    // --- Opti 复用（第三步：参数化障碍物点 + 重规划循环） ---
    std::unique_ptr<casadi::Opti> opti_;        // 优化问题实例，首次 solve 构建后复用
    MX reference_param_;                        // 滚动子目标参考参数 (4, N_+1) [x,y,z,yaw]，各列相同
    MX X_cur_param_;                            // 当前状态参数 (8, 1)
    MX obstacle_param_;                         // 最近障碍点参数 (3*M, N_+1)
    MX goal_param_;                             // 终点位置参数 (3, 1)，用于制动约束
    std::map<int, MX> neighbor_params_;         // 邻居预测轨迹参数
    bool has_last_solution_;                    // 是否已有上一轮解（用于热启动）
    Eigen::MatrixXd last_pred_x_;               // 上一轮预测状态 (8, N_+1)
    Eigen::MatrixXd last_u_;                    // 上一轮控制序列 (4, N_)
    std::vector<Eigen::Vector3d> last_obstacle_pts_; // 最近一次使用的障碍点（可视化用）
    // 构建签名：配置变化时重建 Opti
    int built_N_;
    double built_dt_;
    double built_u_max_;
    double built_w_max_;
    double built_a_brake_;
    size_t built_neighbor_count_;
    int built_nearest_obs_num_;
    double built_detect_range_;
    double built_safe_dist_;
    int built_ipopt_max_iter_;
    int ipopt_max_iter_;  // IPOPT 最大迭代次数（可调，避障势垒陡峭时可加大）
    double built_ipopt_tol_;
    double ipopt_tol_;    // IPOPT 收敛容差（可调）

    // 反步法控制器增益参数
    DM k1; // ki,1
    DM k2; // ki,2
    
    Function kinematic_equation_; //运动学方程
    //OptiSol solution_; 报错没有默认构造函数
    unique_ptr<casadi::OptiSol> solution_;  //获取计算出的最优状态和控制输入序列
    
    // 全局轨迹
    std::vector<GlobalWaypoint> global_traj_;
    bool has_global_traj_;
    // 最近一次投影的路径点索引（用于窗口搜索与进度单调保护）
    size_t last_progress_idx_ = 0;
    // 滚动子目标：沿弧长前瞻选取，全局轨迹仅提供方向引导
    std::vector<double> path_arclength_;  // 与 global_traj_ 同步的累积弧长 (m)
    double lookahead_distance_;           // 子目标前瞻距离 (m)
    double a_brake_;                      // 制动加速度 (m/s^2)，默认 0.8*u_max
    GlobalWaypoint rolling_goal_;         // 最近一次求解使用的子目标
    bool has_rolling_goal_;
public: //外部程序（如 ROS 节点）与 MPC 算法交互的方式
    Mpc();
    ~Mpc();
 
    Function setKinematicEquation(); //设置运动学模型

    /// 设置自身 ID
    void setMyId(int id) { my_id_ = id; }
    /// 设置编队偏移量表
    void setFormationOffsets(const std::map<int, std::vector<double>>& offsets) { formation_offsets_ = offsets; }

    // --- 可参数化的 MPC 配置 ---
    /// 设置预测步长 (horizon)
    void setPredictionHorizon(int N) { N_ = N; }
    /// 设置时间步长 (s)
    void setTimeStep(double dt) { dt_ = dt; }
    /// 设置线速度/线加速度上限, 自动更新下限与默认制动加速度
    void setMaxVelocity(double u_max) { u_max_ = u_max; u_min_ = -u_max; a_brake_ = 0.8 * u_max_; }
    /// 设置角速度/角加速度上限, 自动更新下限
    void setMaxAngularVelocity(double w_max) { w_max_ = w_max; w_min_ = -w_max; }
    /// 设置滚动子目标前瞻距离 (m)
    void setLookaheadDistance(double d) { lookahead_distance_ = d; }
    /// 设置制动加速度 (m/s^2)，<=0 时取默认 0.8*u_max
    void setBrakeAcceleration(double a) { a_brake_ = a > 0.0 ? a : 0.8 * u_max_; }
    /// 设置 KD-tree 对数势垒避障权重
    void setObstacleWeight(double w) { Wo = w; }
    /// 设置编队内安全距离 (m)
    void setCollisionSafeDistance(double d) { d_safe_ca = d; }
    /// 设置相互碰撞惩罚权重
    void setCollisionWeight(double w) { Wc = w; }
    /// 设置控制输入惩罚权重 (对角矩阵，原 F_)
    void setControlInputWeight(const DM& w) { F_ = w; }
    /// 设置控制增量惩罚权重 (对角矩阵)
    void setControlIncrementWeight(const DM& w) { R_d_ = w; }
    /// 设置滚动子目标惩罚权重 (对角矩阵，原 Q_)
    void setSubgoalWeight(const DM& w) { Q_ = w; }
    /// 设置编队误差权重 (对角矩阵，独立于子目标惩罚权重)
    void setFormationErrorWeight(const DM& w) { Q_f_ = w; }

    /// 设置障碍物 KD-tree（第三步：静态避障）
    void setKdTree(KdTreeMap* kd_tree) { kd_tree_ptr_ = kd_tree; }
    /// 设置每个预测步查询的最近障碍点数量 M
    void setNearestObstacleNum(int num) { nearest_obs_num_ = num; }
    /// 设置无人机半径 (m)
    void setDroneRadius(double r) { drone_radius_ = r; }
    /// 设置对数避障势垒的最小安全距离 (m，有效距离口径)
    void setSafeDistance(double d) { safe_dist_ = d; }
    /// 设置对数避障势垒的探测范围 (m)，须大于安全距离
    void setDetectionRange(double d) { detect_range_ = d; }
    /// 设置最近点重查询重规划的最大迭代次数
    void setReplanMaxIter(int iter) { replan_max_iter_ = iter; }
    /// 设置 IPOPT 最大迭代次数
    void setIpoptMaxIter(int iter) { ipopt_max_iter_ = iter; }
    /// 设置 IPOPT 收敛容差
    void setIpoptTol(double tol) { ipopt_tol_ = tol; }

    /// 设置全局规划器输出的全局轨迹
    void setGlobalTrajectory(const std::vector<GlobalWaypoint>& traj);

    /// 将物理位置投影到全局轨迹，返回最近路径点的累积弧长 s0 (m)
    double projectToPath(const Eigen::Vector3d& query_pos);

    /// 沿轨迹弧长从 s0 前推 lookahead_distance_ 选取滚动子目标
    GlobalWaypoint selectRollingGoal(double s0);

    ErrorTerms formationError(int k_step, MX reference_state, std::map<int, MX>& neighbor_params, MX current_states); //编队误差

    MX ViLmpcDot(int k_step, MX reference_state, std::map<int, MX>& neighbor_params, MX current_states, MX U_k); //李雅普诺夫函数导数

    /// 基于 KD-tree 最近障碍点的对数势垒避障项（障碍点作为参数，不在符号图内查询）
    MX kdtreeObstacleAvoidance(MX current_states, MX obstacle_k);

    MX collisionAvoidance(MX reference_state, int k_step, std::map<int, MX>& neighbor_params, MX current_states); //编队内碰撞避让项

    /// 构建/复用 Opti 优化问题（配置变化时重建）
    bool buildOpti(const std::vector<UavState>& neighbors);

    /// 对查询点集合查询最近障碍点并填充 obstacle 参数值（含索引，供重规划判断）
    std::vector<double> queryObstacleValues(
        const std::vector<Eigen::Vector3d>& query_pts,
        std::vector<int>& out_indices);

    bool solve(double s0, const std::vector<UavState>& neighbors, Eigen::VectorXd current_states); //求解优化问题

    vector<double> getFirstU(); //最优控制序列中的第一个控制量
    vector<double> getPredictX(); //最优状态序列

    /// 最近一次求解实际使用的障碍点（可视化用，按步序存放）
    const std::vector<Eigen::Vector3d>& getLastObstaclePoints() const {
        return last_obstacle_pts_;
    }

    /// 最近一次求解使用的滚动子目标（可视化用）
    const GlobalWaypoint& getLastRollingGoal() const { return rolling_goal_; }
};

#endif
