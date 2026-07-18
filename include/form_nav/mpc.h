#ifndef MPC_H 
#define MPC_H //头文件保护符，确保这个文件在任何编译单元中只被包含一次，防止重复定义错误

#include <vector>
#include <Eigen/Dense>
#include <casadi/casadi.hpp>
#include <iostream>
#include <chrono>
#include <map>
#include "form_nav/esdf_casadi.h"

using namespace std;
using namespace casadi;

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

// 全局轨迹路径点 (kinodynamic A* 输出, 含时间+空间+速度)
struct GlobalWaypoint {
    double t;            // 累计时间 (s)
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

    int num; //无人机数量
    
    //权重矩阵
    DM Q_, F_; 

    //运动学方程参数
    double Lx, Kx;
    double Ly, Ky;
    double Lz, Kz;
    double Lw, Kw;
    DM beta;
    DM gama;

    // 自身 ID
    int my_id_;
    // 是否直接跟踪全局轨迹（虚拟领航者）
    bool is_tracked_;
    // 所有无人机的编队偏移量 {id → [x, y, z, yaw]}
    std::map<int, std::vector<double>> formation_offsets_;

    //平滑切换函数参数
    double h_bar;
    double T_switch;
    double delta;

    // ESDF 避障参数
    double d_safe_oa;      // 障碍物安全距离 (m)
    double d_critical_oa;  // 临界距离，低于此距离惩罚急剧增大
    double Wo;             // 障碍物避障权重 (标量)
    EsdfCasadi* esdf_ptr_; // 指向 ESDF CasADi 插值器的指针 (外部管理生命周期)

    // 跟踪模式: 1=仅位置跟踪(默认), 2=位置+速度跟踪
    int tracking_model_;

    //碰撞参数
    double d_safe_ca; // 编队内安全距离 COLLISION AVOIDANCE
    DM Wc;            // 相互碰撞权重

    MX X; //状态变量
    MX U; //控制变量

    // 反步法控制器增益参数
    DM k1; // ki,1
    DM k2; // ki,2
    
    Function kinematic_equation_; //运动学方程
    //OptiSol solution_; 报错没有默认构造函数
    unique_ptr<casadi::OptiSol> solution_;  //获取计算出的最优状态和控制输入序列
    
    // 全局轨迹
    std::vector<GlobalWaypoint> global_traj_;
    bool has_global_traj_;
    // 缓存的各维度插值器 (grid = 路径点索引 0..n-1)
    std::unique_ptr<casadi::Function> traj_interp_x_, traj_interp_y_;
    std::unique_ptr<casadi::Function> traj_interp_z_, traj_interp_yaw_;
    std::unique_ptr<casadi::Function> traj_interp_vx_, traj_interp_vy_;
    std::unique_ptr<casadi::Function> traj_interp_vz_, traj_interp_vyaw_;

public: //外部程序（如 ROS 节点）与 MPC 算法交互的方式
    Mpc();
    ~Mpc();
 
    Function setKinematicEquation(); //设置运动学模型

    /// 设置自身 ID
    void setMyId(int id) { my_id_ = id; }
    /// 设置跟踪模式: 1=仅位置, 2=位置+速度
    void setTrackingModel(int model) { tracking_model_ = model; }
    /// 设置是否直接跟踪全局轨迹
    void setTracked(bool tracked) { is_tracked_ = tracked; }
    /// 设置编队偏移量表
    void setFormationOffsets(const std::map<int, std::vector<double>>& offsets) { formation_offsets_ = offsets; }

    /// 设置 ESDF CasADi 插值器
    void setEsdf(EsdfCasadi* esdf) { esdf_ptr_ = esdf; }

    /// 设置 kinodynamic A* 输出的全局轨迹
    void setGlobalTrajectory(const std::vector<GlobalWaypoint>& traj);

    /// 获取全局参考轨迹在时刻 t 的状态
    /// t 为轨迹累计时间 (s)
    /// 返回 8x1: [x, y, z, yaw, vx, vy, vz, vyaw]
    MX globalTrajectory(MX t);

    ErrorTerms formationError(int k_step, MX t, std::map<int, MX>& neighbor_params, MX current_states); //编队误差

    MX ViLmpcDot(int k_step, MX t, std::map<int, MX>& neighbor_params, MX current_states, MX U_k); //李雅普诺夫函数导数

    /// 基于 ESDF 的障碍物避障项 (替代原来的 obstacleAvoidance)
    MX esdfObstacleAvoidance(MX current_states);

    MX collisionAvoidance(MX t, int k_step, std::map<int, MX>& neighbor_params, MX current_states); //编队内碰撞避让项

    bool solve(double t, const std::vector<UavState>& neighbors, Eigen::VectorXd current_states); //求解优化问题

    vector<double> getFirstU(); //最优控制序列中的第一个控制量
    vector<double> getPredictX(); //最优状态序列
};

#endif