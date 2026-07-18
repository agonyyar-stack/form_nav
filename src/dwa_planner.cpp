#include "form_nav/dwa_planner.h"
#include <geometry_msgs/Point.h>
#include <cmath>
#include "form_nav/sdf_map.h"

namespace dwa_planner {

DWAPlanner::DWAPlanner(SDFMap::Ptr sdf_map): sdf_map_(sdf_map) {
    // Velocity limits                    // 成员初始化列表，等价于sdf_map_ = sdf_map；
    max_vel_xy_ = 2.0;
    max_vel_z_ = 1.0;
    // max_yaw_rate_ = 1.0;

    // Acceleration limits
    max_acc_xy_ = 1.0;
    max_acc_z_ = 0.8;
    // max_yaw_acc_ = 1.0;

    // DWA
    dt_ = 0.1;
    predict_time_ = 2.0;

    // Collision
    robot_radius_ = 0.2;
    safety_margin_ = 0.2;

    // Sampling
    vel_resolution_ = 0.1;

    // Evaluation weights
    heading_weight_ = 1.5;
    clearance_weight_ = 3.0;
    velocity_weight_ = 1.0;
    goal_weight_ = 2.0;
    path_weight_ = 2.5;             // 路径跟踪（路径逼近 + 方向一致）
    dynamic_obstacle_weight_ = 2.0;
}

Trajectory DWAPlanner::plan(const State& current_state, const Eigen::Vector3d& goal,
                            const std::vector<Eigen::Vector3d>& path, int nearest_idx) {
    candidate_trajectories_.clear();

    auto controls = generateDynamicWindow(current_state, goal);

    Trajectory best_traj;
    best_traj.score = -1e9;

    for (const auto& control : controls) {
        auto traj = generateTrajectory(current_state, control);

        // Collision check using ESDF map
        if (checkCollision(traj)) {
            continue;
        }

        // Evaluate trajectory with path tracking
        traj.score = evaluateTrajectory(traj, goal, path, nearest_idx);
        candidate_trajectories_.push_back(traj);

        // Best trajectory
        if (traj.score > best_traj.score) {
            best_traj = traj;
        }
    }

    return best_traj;
}

std::vector<Control> DWAPlanner::generateDynamicWindow(const State& state, const Eigen::Vector3d& goal) {
    std::vector<Control> controls;

    // Dynamic window
    double vx_min = std::max(-max_vel_xy_, state.vx - max_acc_xy_ * dt_);
    double vx_max = std::min(max_vel_xy_, state.vx + max_acc_xy_ * dt_);
    double vy_min = std::max(-max_vel_xy_, state.vy - max_acc_xy_ * dt_);
    double vy_max = std::min(max_vel_xy_, state.vy + max_acc_xy_ * dt_);
    double vz_min = std::max(-max_vel_z_, state.vz - max_acc_z_ * dt_);
    double vz_max = std::min(max_vel_z_, state.vz + max_acc_z_ * dt_);

    // 终点附近适当降低速度上限
    Eigen::Vector2d to_goal = goal.head<2>() - Eigen::Vector2d(state.x, state.y);
    if (to_goal.norm() < 1.0) {
        to_goal.normalize();
        vx_max = std::min(1.0, vx_max);
        vx_min = std::max(-1.0, vx_min);
        vy_max = std::min(1.0, vy_max);
        vy_min = std::max(-1.0, vy_min);
        vz_max = std::min(1.0, vz_max);
        vz_min = std::max(-1.0, vz_min);
    }

    for (double vx = vx_min; vx <= vx_max; vx += vel_resolution_) {
        for (double vy = vy_min; vy <= vy_max; vy += vel_resolution_) {
            for (double vz = vz_min; vz <= vz_max; vz += vel_resolution_) {
                // 速度过小则不考虑，避免生成过多无效轨迹
                double speed = std::sqrt(vx * vx + vy * vy + vz * vz);
                if (speed < 0.05) continue;

                // for (double yaw_rate = -max_yaw_rate_; yaw_rate <= max_yaw_rate_; yaw_rate += yaw_resolution_) {
                    Control u;
                    u.vx = vx;
                    u.vy = vy;
                    u.vz = vz;
                    // u.yaw_rate = yaw_rate;
                    controls.push_back(u);
                // }
            }
        }
    }

    return controls;
}

Trajectory DWAPlanner::generateTrajectory(const State& initial_state, const Control& control) {
    Trajectory traj;
    traj.control = control;
    State state = initial_state;
    traj.states.push_back(state);

    double time = 0.0;
    while (time <= predict_time_) {
        // Motion model
        state.x += control.vx * dt_;
        state.y += control.vy * dt_;
        state.z += control.vz * dt_;
        state.vx = control.vx;
        state.vy = control.vy;
        state.vz = control.vz;
        // state.yaw = std::atan2(control.vy, control.vx);

        traj.states.push_back(state);
        time += dt_;
    }
    return traj;
}

// 检查轨迹是否与障碍物发生碰撞
bool DWAPlanner::checkCollision(const Trajectory& traj) {
    if (!sdf_map_) {
        ROS_ERROR("ESDF map not initialized!");
        return false;
    }

    for (const auto& state : traj.states) {
        Eigen::Vector3d pos(state.x, state.y, state.z);
        
        // 检查位置是否在地图范围内
        if (!sdf_map_->isInMap(pos)) {
            continue;
        }
        
        // 使用ESDF地图查询距离
        double dist = sdf_map_->getDistance(pos);
        
        // 如果距离小于安全距离，则认为碰撞
        if (dist < robot_radius_ + safety_margin_) {
            return true;
        }
    }
    return false;
}

double DWAPlanner::evaluateTrajectory(const Trajectory& traj, const Eigen::Vector3d& goal,
                                       const std::vector<Eigen::Vector3d>& path, int nearest_idx) {
    if (traj.states.empty()) return -1e9;

    const auto& final_state = traj.states.back();
    Eigen::Vector3d final_pos(final_state.x, final_state.y, final_state.z);

    // 速度方向向量
    Eigen::Vector3d vel_vec(final_state.vx, final_state.vy, final_state.vz);
    double speed = vel_vec.norm();
    if (speed < 1e-6) {
        vel_vec = Eigen::Vector3d(0.0, 0.0, 1.0); //避免后面 dot() 时用零向量算出无意义结果，同时也给悬停轨迹一个中性的方向分
    } else {
        vel_vec.normalize();
    }

    // ============ 安全距离评价（最优先）============
    double safe_dist = 1.5 * (robot_radius_ + safety_margin_);  // 0.6m，提前预警
    double min_dist = 1e9;
    for (const auto& state : traj.states) {
        Eigen::Vector3d pos(state.x, state.y, state.z);
        if (!sdf_map_->isInMap(pos)) continue;
        double dist = sdf_map_->getDistance(pos);
        if (dist < min_dist) min_dist = dist;
    }
    // x → 0, tanh(x) ≈ x, 线性段，敏感
    // x ≈ 1, tanh(1) ≈ 0.76, 过渡段
    // x → ∞, tanh(x) → 1 , 饱和段
    double clearance_score = std::tanh(min_dist / safe_dist); 

    double safety_mod = clearance_score; // 安全调制因子：障碍物越近 → 越接近 0，用于压制路径跟踪和速度

    // ============ 目标导向评价 ============
    Eigen::Vector3d to_goal = goal - final_pos;
    double goal_dist = to_goal.norm();
    Eigen::Vector3d goal_dir = Eigen::Vector3d::UnitZ(); // 初始化为Z方向单位向量
    if (goal_dist > 1e-6) goal_dir = to_goal / goal_dist;
    double heading_score = std::max(0.0, goal_dir.dot(vel_vec));
    // 高斯衰减函数，衰减快，远距离轨迹更有区分度
    double goal_score = std::exp(-goal_dist * goal_dist / (1.5 * 1.5));

    // ============ 路径跟踪评价（由安全因子动态调节）============
    double path_score = 0.0;
    if (!path.empty()) {
        // -- a. 横向偏差 --
        double min_path_dist = 1e9;
        int closest_idx = 0;
        for (size_t i = 0; i < path.size(); ++i) { // 找到与轨迹末端最近距离及路径点
            double d = (path[i].head<2>() - final_pos.head<2>()).norm();
            if (d < min_path_dist) {
                min_path_dist = d;
                closest_idx = i;
            }
        }
        // 偏差容忍度随安全裕度动态变化：安全时紧(0.5m)，危险时松(2.0m)
        double dev_tolerance = 0.5 + (1.0 - safety_mod) * 1.5;
        double deviation_score = std::exp(-min_path_dist * min_path_dist / (dev_tolerance * dev_tolerance));

        // -- b. 路径方向一致性 --
        double direction_score = 0.5;
        if (closest_idx < (int)path.size() - 1) { 
            Eigen::Vector2d tangent = (path[closest_idx + 1].head<2>() - path[closest_idx].head<2>()); // 下一路径段方向，即路径切线
            double seg_len = tangent.norm(); 
            if (seg_len > 0.05) {  //过滤无效路径段，小于这个长度的路径段保留默认的 direction_score = 0.5（中性分），不会误导评分。
                tangent /= seg_len;
                double path_align = tangent.dot(vel_vec.head<2>()); // 正=顺路径，负=逆路径
                direction_score = std::max(0.0, path_align);    // 逆路径得0分
            }
        }

        // -- c. 进度奖励：障碍物附近不惩罚倒退 --
        double progress_score = 0.0;
        {
            int start_i = std::min(nearest_idx, closest_idx);
            int end_i   = std::max(nearest_idx, closest_idx);
            double arc_from_nearest = 0.0;
            for (int i = start_i; i < end_i && i + 1 < (int)path.size(); ++i) {
                arc_from_nearest += (path[i + 1].head<2>() - path[i].head<2>()).norm();
            }
            if (closest_idx > nearest_idx) {
                progress_score = std::min(1.0, arc_from_nearest / 1.0);
            } else if (closest_idx < nearest_idx) {
                // 安全裕度低时不惩罚倒退（允许绕障后退）
                progress_score = -0.5 * safety_mod;
            } else {
                progress_score = 0.0;
            }
        }
        path_score = 0.4 * deviation_score + 0.3 * direction_score + 0.3 * progress_score;
    }

    // ============ 速度评价（安全因子 × 靠近目标双重压制）============
    double velocity_score = 0.8 * speed / max_vel_xy_ + 0.2 * std::abs(final_state.vz) / max_vel_z_;
    // 障碍物近→降速；靠近目标→降速；两者取更严的
    double speed_mod = std::min(safety_mod, goal_dist < 1.0 ? 0.5 : 1.0);
    velocity_score *= speed_mod;

    // ============ 安全因子调节 path_weight，障碍物近时放宽路径跟踪 ============
    double effective_path_weight = path_weight_ * (0.3 + 0.7 * safety_mod);  // 危险时降至 30%

    double score = heading_weight_  * heading_score
                 + goal_weight_     * goal_score
                 + effective_path_weight * path_score
                 + velocity_weight_ * velocity_score
                 + clearance_weight_ * clearance_score;

    return score;
}

visualization_msgs::MarkerArray DWAPlanner::getTrajectoryMarkers() const {
    visualization_msgs::MarkerArray array;
    int id = 0;

    for (const auto& traj : candidate_trajectories_) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "candidate_trajectory";
        marker.id = id++;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.02;
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.25;

        for (const auto& state : traj.states) {
            geometry_msgs::Point p;
            p.x = state.x;
            p.y = state.y;
            p.z = state.z;
            marker.points.push_back(p);
        }

        array.markers.push_back(marker);
    }

    return array;
}

} // namespace dwa_planner
