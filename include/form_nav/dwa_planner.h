#ifndef FORM_NAV_DWA_PLANNER_H_ 
#define FORM_NAV_DWA_PLANNER_H_ 

#include <vector> 
#include <Eigen/Dense> 
#include <visualization_msgs/MarkerArray.h> 
#include "form_nav/sdf_map.h" 

namespace dwa_planner { 

//状态 
struct State { 
    double x, y, z; 
    double yaw; 
    double vx, vy, vz; 
    double yaw_rate; 
}; 

//控制输入 
struct Control { 
    double vx, vy, vz; 
    // double yaw_rate; 
}; 

//轨迹 
struct Trajectory { 
    std::vector<State> states; 
    double score = -1e9; 
    Control control; 
}; 

class DWAPlanner { 
public: 
    // 构造函数传入 ESDF 地图指针
    DWAPlanner(SDFMap::Ptr sdf_map); 
    
    // 修改：移除对 obstacles 参数的依赖，改用 ESDF 地图查询
    // path + nearest_idx 用于路径跟踪评分
    Trajectory plan(const State& current_state, const Eigen::Vector3d& goal,
                    const std::vector<Eigen::Vector3d>& path, int nearest_idx); 
    
    visualization_msgs::MarkerArray getTrajectoryMarkers() const; 

private: 
    // Dynamic window
    std::vector<Control> generateDynamicWindow(const State& state, const Eigen::Vector3d& goal); 

    // Trajectory generation
    Trajectory generateTrajectory(const State& initial_state, const Control& control); 

    // 修改：移除对 obstacles 参数的依赖
    // Evaluation — 增加路径信息用于路径跟踪评分
    double evaluateTrajectory(const Trajectory& traj, const Eigen::Vector3d& goal,
                              const std::vector<Eigen::Vector3d>& path, int nearest_idx); 

    // 修改：移除对 obstacles 参数的依赖
    // Collision
    bool checkCollision(const Trajectory& traj); 

private: 
    // ESDF 地图指针
    SDFMap::Ptr sdf_map_;

    // Constraints 
    double max_vel_xy_; 
    double max_vel_z_; 
    double max_yaw_rate_; 
    double max_acc_xy_; 
    double max_acc_z_; 
    double max_yaw_acc_; 

    // DWA 
    double dt_; 
    double predict_time_; 

    // Collision 
    double robot_radius_; 
    double safety_margin_; 

    // Sampling 
    double vel_resolution_; 
    double yaw_resolution_; 

    // Weights 
    double heading_weight_; 
    double clearance_weight_; 
    double velocity_weight_; 
    double goal_weight_; 
    double path_weight_;             // 路径跟踪权重（逼近路径 + 路径方向一致性）
    double dynamic_obstacle_weight_; 
    
    std::vector<Trajectory> candidate_trajectories_; 
}; 

} // namespace dwa_planner 

#endif // FORM_NAV_DWA_PLANNER_H_
