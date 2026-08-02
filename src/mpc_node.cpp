#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include "form_nav/mpc.h"
#include "form_nav/formation_config.h"
#include "form_nav/esdf_casadi.h"
#include "form_nav/sdf_map.h"
#include "form_nav/GlobalPathProgress.h"
#include "form_nav/GlobalTrajectory.h"
#include "form_nav/PredictedTrajectory.h"

// --- 基本配置 ---
std::string UAV_NAME;
std::string WORLD_FRAME = "world";
std::string LOCAL_FRAME;
int MY_ID = 0;
int MPC_HORIZON = 10;
double MPC_DT = 0.05;
double NEIGHBOR_TRAJECTORY_TIMEOUT = 0.3;
double NEIGHBOR_FUTURE_TOLERANCE = 0.05;
double GLOBAL_PROGRESS_TIMEOUT = 0.3;
const double LOOP_RATE = 20.0;

std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr;

// --- ROS 通信 ---
ros::Publisher cmd_vel_pub;
ros::Publisher local_pos_pub;
ros::Publisher predicted_path_pub;
ros::Publisher share_path_pub;
ros::Publisher predicted_path_viz_pub;
ros::Publisher traveled_path_pub;
ros::Publisher global_progress_pub;

// --- 状态量 ---
// 显式初始化，避免首个传感器消息到达前把未定义数据送入求解器。
Eigen::VectorXd current_mpc_state = Eigen::VectorXd::Zero(8);

// 控制循环只有在这些数据全部到齐后才允许调用 MPC。
bool has_pose = false;
bool has_velocity = false;
bool has_yaw = false;
int last_global_idx = 0;

double vel_world_x = 0, vel_world_y = 0, vel_world_z = 0;
double vel_body_w = 0;

// --- 历史轨迹 ---
nav_msgs::Path traveled_path;  // 累积已走过的位置

// --- 邻居 + MPC ---
std::unique_ptr<Mpc> mpc_ptr;
std::vector<int> required_neighbor_ids;

struct NeighborTrajectoryCache {
    Eigen::MatrixXd predicted_states;
    ros::Time prediction_start;
    double dt = 0.0;
};

std::map<int, NeighborTrajectoryCache> neighbor_trajectories;

// --- ESDF 地图 + CasADi 插值器 ---
std::unique_ptr<SDFMap> sdf_map_ptr;
EsdfCasadi esdf_casadi;

// --- 全局轨迹 ---
std::vector<GlobalWaypoint> global_traj;
bool global_traj_received = false;
uint32_t current_trajectory_id = 0;
Eigen::Vector4d my_formation_offset = Eigen::Vector4d::Zero();

struct GlobalProgressCache {
    ros::Time stamp;
    uint32_t trajectory_id = 0;
    double path_time = 0.0;
    bool received = false;
};

GlobalProgressCache global_progress;

// // 到达终点后的悬停锁定标志：一旦置位就持续保持悬停，不再回到 MPC 求解，
// // 避免在终点阈值附近悬停/振荡反复切换。收到新的全局轨迹时解除锁定，
// // 允许重新规划后再次起飞。
// bool arrived_hold_ = false;

// --- 飞控状态 ---
// mpc_node 只有在飞控已解锁且处于 OFFBOARD 模式后才接管控制；
// 未就绪时绝不发布任何速度指令，避免中断 uav_base 的位置 setpoint 流
// （PX4 进入 OFFBOARD / 解锁的前置条件）。
mavros_msgs::State current_fc_state;
bool vehicle_ready = false;

// ========== 飞控状态回调 ==========
void fcStateCallback(const mavros_msgs::State::ConstPtr& msg) {
    current_fc_state = *msg;
    vehicle_ready = current_fc_state.armed &&
                    current_fc_state.mode == "OFFBOARD";
}

// ========== 全局轨迹回调 ==========
// 数据格式: 每点 7 个 double [t, x, y, z, vx, vy, vz]
void globalTrajCallback(const form_nav::GlobalTrajectory::ConstPtr& msg) {
    constexpr size_t stride = 7;
    if (msg->header.stamp.isZero() || msg->header.frame_id != WORLD_FRAME ||
        msg->trajectory_id == 0 || msg->steps == 0 ||
        msg->waypoints.size() != static_cast<size_t>(msg->steps) * stride ||
        !std::all_of(msg->waypoints.begin(), msg->waypoints.end(),
                     [](double value) { return std::isfinite(value); })) {
        ROS_WARN_THROTTLE(2.0, "%s: invalid global trajectory message.",
                          UAV_NAME.c_str());
        return;
    }

    std::vector<GlobalWaypoint> received_trajectory;
    received_trajectory.reserve(msg->steps);
    double previous_time = -std::numeric_limits<double>::infinity();
    for (uint32_t i = 0; i < msg->steps; ++i) {
        GlobalWaypoint wp;
        const size_t off = static_cast<size_t>(i) * stride;
        wp.t   = msg->waypoints[off + 0];
        wp.x   = msg->waypoints[off + 1];
        wp.y   = msg->waypoints[off + 2];
        wp.z   = msg->waypoints[off + 3];
        wp.vx  = msg->waypoints[off + 4];
        wp.vy  = msg->waypoints[off + 5];
        wp.vz  = msg->waypoints[off + 6];
        wp.yaw = 0;
        wp.vyaw = 0;
        if (wp.t < 0.0 || wp.t <= previous_time) {
            ROS_WARN_THROTTLE(2.0,
                              "%s: global trajectory %u has non-increasing time.",
                              UAV_NAME.c_str(), msg->trajectory_id);
            return;
        }
        previous_time = wp.t;
        received_trajectory.push_back(wp);
    }

    global_traj = std::move(received_trajectory);
    global_traj_received = true;
    current_trajectory_id = msg->trajectory_id;
    last_global_idx = 0;
    global_progress = GlobalProgressCache{};
    neighbor_trajectories.clear();
    // // 新轨迹意味着新的任务：解除到达悬停锁定，允许再次起飞。
    // arrived_hold_ = false;
    mpc_ptr->setGlobalTrajectory(global_traj);
    ROS_INFO("%s: received global trajectory %u with %zu waypoints",
             UAV_NAME.c_str(), current_trajectory_id, global_traj.size());
}

// ========== 在最近点附近窗口内搜索，返回时间 ==========
double findClosestWaypointTime(const Eigen::Vector3d& virtual_center) {
    if (global_traj.empty()) return 0.0;

    const int win_back = 3, win_forward = 30;    // 搜索窗口: [last-3, last+30]

    int lo = std::max(0, last_global_idx - win_back);
    int hi = std::min((int)global_traj.size() - 1, last_global_idx + win_forward);

    double min_dist = 1e9;
    int closest = lo;
    for (int i = lo; i <= hi; ++i) {
        double dx = virtual_center.x() - global_traj[i].x;
        double dy = virtual_center.y() - global_traj[i].y;
        double dz = virtual_center.z() - global_traj[i].z;
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < min_dist) {
            min_dist = d2;
            closest = i;
        }
    }

    // 如果最近点落在窗口边界附近, 扩展一次全量搜索做安全重置
    if (closest == hi || closest == lo) {
        for (size_t i = 0; i < global_traj.size(); ++i) {
            double dx = virtual_center.x() - global_traj[i].x;
            double dy = virtual_center.y() - global_traj[i].y;
            double dz = virtual_center.z() - global_traj[i].z;
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < min_dist) {
                min_dist = d2;
                closest = i;
            }
        }
    }

    // 路径进度保持单调，避免无人机在弯道或局部回头时跳回旧路径段。
    closest = std::max(closest, last_global_idx);
    last_global_idx = closest;
    return global_traj[closest].t;
}

void globalProgressCallback(const form_nav::GlobalPathProgress::ConstPtr& msg) {
    const ros::Time now = ros::Time::now();
    const double age = (now - msg->header.stamp).toSec();
    if (msg->header.stamp.isZero() ||
        msg->trajectory_id != current_trajectory_id ||
        !std::isfinite(msg->path_time) || global_traj.empty() ||
        msg->path_time < global_traj.front().t ||
        msg->path_time > global_traj.back().t ||
        age > GLOBAL_PROGRESS_TIMEOUT || age < -NEIGHBOR_FUTURE_TOLERANCE) {
        ROS_WARN_THROTTLE(2.0, "%s: rejected global path progress",
                          UAV_NAME.c_str());
        return;
    }
    if (global_progress.received &&
        msg->path_time < global_progress.path_time) {
        ROS_WARN_THROTTLE(2.0, "%s: rejected decreasing global path progress",
                          UAV_NAME.c_str());
        return;
    }

    global_progress.stamp = msg->header.stamp;
    global_progress.trajectory_id = msg->trajectory_id;
    global_progress.path_time = msg->path_time;
    global_progress.received = true;
}

// ========== MAVROS 回调 ==========
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    try {
        geometry_msgs::PoseStamped source = *msg;
        if (source.header.frame_id.empty()) source.header.frame_id = LOCAL_FRAME;
        const auto world_pose = tf_buffer_ptr->transform(
            source, WORLD_FRAME, ros::Duration(0.05));
        current_mpc_state(0) = world_pose.pose.position.x;
        current_mpc_state(1) = world_pose.pose.position.y;
        current_mpc_state(2) = world_pose.pose.position.z;
        current_mpc_state(3) = tf2::getYaw(world_pose.pose.orientation);
        has_pose = true;
        has_yaw = true;
    } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2.0, "%s: pose transform to %s failed: %s",
                          UAV_NAME.c_str(), WORLD_FRAME.c_str(), error.what());
    }
}

void velocityCallbackWorld(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    try {
        const std::string source_frame = msg->header.frame_id.empty()
                                             ? LOCAL_FRAME
                                             : msg->header.frame_id;
        geometry_msgs::Vector3Stamped linear_source;
        linear_source.header = msg->header;
        linear_source.header.frame_id = source_frame;
        linear_source.vector = msg->twist.linear;
        geometry_msgs::Vector3Stamped angular_source = linear_source;
        angular_source.vector = msg->twist.angular;
        const auto linear_world = tf_buffer_ptr->transform(
            linear_source, WORLD_FRAME, ros::Duration(0.05));
        const auto angular_world = tf_buffer_ptr->transform(
            angular_source, WORLD_FRAME, ros::Duration(0.05));
        vel_world_x = linear_world.vector.x;
        vel_world_y = linear_world.vector.y;
        vel_world_z = linear_world.vector.z;
        vel_body_w = angular_world.vector.z;
        has_velocity = true;
    } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2.0, "%s: velocity transform to %s failed: %s",
                          UAV_NAME.c_str(), WORLD_FRAME.c_str(), error.what());
    }
}

void neighborTrajectoryCallback(const form_nav::PredictedTrajectory::ConstPtr& msg,
                                int expected_id) {
    const ros::Time now = ros::Time::now();
    if (msg->header.stamp.isZero() || msg->header.frame_id.empty() ||
        msg->uav_id != expected_id ||
        msg->steps != static_cast<uint32_t>(MPC_HORIZON + 1) ||
        !std::isfinite(msg->dt) || msg->dt <= 0.0 ||
        std::abs(msg->dt - MPC_DT) > 1e-6 ||
        msg->states.size() != static_cast<size_t>(msg->steps) * 8) {
        ROS_WARN_THROTTLE(2.0, "%s: invalid trajectory from expected neighbor %d",
                          UAV_NAME.c_str(), expected_id);
        return;
    }

    const double message_age = (now - msg->header.stamp).toSec();
    if (message_age > NEIGHBOR_TRAJECTORY_TIMEOUT ||
        message_age < -NEIGHBOR_FUTURE_TOLERANCE) {
        ROS_WARN_THROTTLE(2.0,
                          "%s: rejected neighbor %d trajectory with age %.3f s",
                          UAV_NAME.c_str(), expected_id, message_age);
        return;
    }

    const auto existing = neighbor_trajectories.find(expected_id);
    if (existing != neighbor_trajectories.end() &&
        msg->header.stamp <= existing->second.prediction_start) {
        ROS_WARN_THROTTLE(2.0, "%s: ignored out-of-order trajectory from neighbor %d",
                          UAV_NAME.c_str(), expected_id);
        return;
    }

    if (!std::all_of(msg->states.begin(), msg->states.end(),
                     [](double value) { return std::isfinite(value); })) {
        ROS_WARN_THROTTLE(2.0, "%s: non-finite state in trajectory from neighbor %d",
                          UAV_NAME.c_str(), expected_id);
        return;
    }

    const std::string& source_frame = msg->header.frame_id;
    Eigen::MatrixXd predicted_states(8, msg->steps);
    try {
        for (uint32_t i = 0; i < msg->steps; ++i) {
            const size_t offset = static_cast<size_t>(i) * 8;
            geometry_msgs::PoseStamped source_pose;
            source_pose.header = msg->header;
            source_pose.header.frame_id = source_frame;
            source_pose.pose.position.x = msg->states[offset + 0];
            source_pose.pose.position.y = msg->states[offset + 1];
            source_pose.pose.position.z = msg->states[offset + 2];
            tf2::Quaternion source_orientation;
            source_orientation.setRPY(0.0, 0.0, msg->states[offset + 3]);
            source_pose.pose.orientation = tf2::toMsg(source_orientation);
            const auto world_pose = tf_buffer_ptr->transform(
                source_pose, WORLD_FRAME, ros::Duration(0.05));

            geometry_msgs::Vector3Stamped source_velocity;
            source_velocity.header = source_pose.header;
            source_velocity.vector.x = msg->states[offset + 4];
            source_velocity.vector.y = msg->states[offset + 5];
            source_velocity.vector.z = msg->states[offset + 6];
            const auto world_velocity = tf_buffer_ptr->transform(
                source_velocity, WORLD_FRAME, ros::Duration(0.05));

            const double yaw = tf2::getYaw(world_pose.pose.orientation);
            const double cos_yaw = std::cos(yaw);
            const double sin_yaw = std::sin(yaw);
            predicted_states(0, i) = world_pose.pose.position.x;
            predicted_states(1, i) = world_pose.pose.position.y;
            predicted_states(2, i) = world_pose.pose.position.z;
            predicted_states(3, i) = yaw;
            // Shared velocities are world-frame; the MPC model uses body-frame velocities.
            predicted_states(4, i) = cos_yaw * world_velocity.vector.x +
                                     sin_yaw * world_velocity.vector.y;
            predicted_states(5, i) = -sin_yaw * world_velocity.vector.x +
                                     cos_yaw * world_velocity.vector.y;
            predicted_states(6, i) = world_velocity.vector.z;
            predicted_states(7, i) = msg->states[offset + 7];
        }
    } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2.0, "%s: neighbor %d trajectory transform failed: %s",
                          UAV_NAME.c_str(), expected_id, error.what());
        return;
    }

    NeighborTrajectoryCache cache;
    cache.predicted_states = std::move(predicted_states);
    cache.prediction_start = msg->header.stamp;
    cache.dt = msg->dt;
    neighbor_trajectories[expected_id] = std::move(cache);
}

double interpolateYaw(double from, double to, double ratio) {
    const double delta = std::remainder(to - from, 2.0 * M_PI);
    return from + ratio * delta;
}

bool buildAlignedNeighborStates(const ros::Time& prediction_start,
                                std::vector<UavState>& neighbors,
                                std::string& failure_reason) {
    neighbors.clear();
    neighbors.reserve(required_neighbor_ids.size());

    for (int neighbor_id : required_neighbor_ids) {
        const auto cached = neighbor_trajectories.find(neighbor_id);
        if (cached == neighbor_trajectories.end()) {
            failure_reason = "missing trajectory from uav" + std::to_string(neighbor_id);
            return false;
        }

        const NeighborTrajectoryCache& trajectory = cached->second;
        const double age = (prediction_start - trajectory.prediction_start).toSec();
        if (age > NEIGHBOR_TRAJECTORY_TIMEOUT) {
            std::ostringstream stream;
            stream << "trajectory from uav" << neighbor_id << " is stale (age="
                   << age << " s)";
            failure_reason = stream.str();
            neighbor_trajectories.erase(cached);
            return false;
        }
        if (age < -NEIGHBOR_FUTURE_TOLERANCE) {
            std::ostringstream stream;
            stream << "trajectory from uav" << neighbor_id << " is in the future (skew="
                   << -age << " s)";
            failure_reason = stream.str();
            return false;
        }

        const int source_steps = trajectory.predicted_states.cols();
        if (source_steps != MPC_HORIZON + 1 || trajectory.dt <= 0.0) {
            failure_reason = "cached trajectory shape changed for uav" +
                             std::to_string(neighbor_id);
            return false;
        }

        UavState neighbor;
        neighbor.id = neighbor_id;
        neighbor.predicted_states.resize(8, MPC_HORIZON + 1);
        for (int step = 0; step <= MPC_HORIZON; ++step) {
            const double query_offset = age + step * MPC_DT;
            const double source_index = query_offset / trajectory.dt;

            if (source_index <= 0.0) {
                neighbor.predicted_states.col(step) = trajectory.predicted_states.col(0);
                continue;
            }
            if (source_index >= source_steps - 1) {
                // A received horizon is normally one communication period behind.
                // Hold its terminal prediction where no later sample is available.
                neighbor.predicted_states.col(step) =
                    trajectory.predicted_states.col(source_steps - 1);
                continue;
            }

            const int lower = static_cast<int>(std::floor(source_index));
            const int upper = lower + 1;
            const double ratio = source_index - lower;
            neighbor.predicted_states.col(step) =
                (1.0 - ratio) * trajectory.predicted_states.col(lower) +
                ratio * trajectory.predicted_states.col(upper);
            neighbor.predicted_states(3, step) = interpolateYaw(
                trajectory.predicted_states(3, lower),
                trajectory.predicted_states(3, upper), ratio);
        }
        neighbors.push_back(std::move(neighbor));
    }
    return true;
}

void publishHoldCommand() {
    geometry_msgs::TwistStamped hold_cmd;
    hold_cmd.header.stamp = ros::Time::now();
    hold_cmd.header.frame_id = WORLD_FRAME;
    cmd_vel_pub.publish(hold_cmd);
}

// // 悬停/锁定期间发布"原地静止"的预测轨迹，避免邻居因为收不到本机
// // 预测轨迹而进入 missing trajectory 等待状态，导致整个编队连锁卡死。
// void publishStationaryPrediction(const ros::Time& stamp) {
//     form_nav::PredictedTrajectory msg;
//     msg.header.stamp = stamp;
//     msg.header.frame_id = WORLD_FRAME;
//     msg.uav_id = MY_ID;
//     msg.dt = MPC_DT;
//     msg.steps = MPC_HORIZON + 1;
//     msg.states.reserve(static_cast<size_t>(msg.steps) * 8);
//     for (uint32_t i = 0; i < msg.steps; ++i) {
//         // 世界系位置/航向保持当前值，速度全 0（世界速度，符合共享轨迹约定）。
//         msg.states.insert(msg.states.end(), {
//             current_mpc_state(0), current_mpc_state(1),
//             current_mpc_state(2), current_mpc_state(3),
//             0.0, 0.0, 0.0, 0.0});
//     }
//     share_path_pub.publish(msg);
// }

// ========== 主控制循环 ==========
void controlLoop(const ros::TimerEvent&, bool is_tracked) {
    // 飞控未就绪（未解锁 / 未进入 OFFBOARD）时不发布任何指令
    if (!vehicle_ready) {
        ROS_INFO_THROTTLE(2.0,
                          "%s: waiting for FC (armed+OFFBOARD) before "
                          "taking control",
                          UAV_NAME.c_str());
        return;
    }

    // 传感器或全局参考未准备好时不求解、不发布任何指令。
    // 关键：解锁后、全局轨迹到达前若发布零速度 hold，会覆盖 uav_base 的
    // 位置 setpoint，使 PX4 保持在地面不起飞，触发
    // "Disarmed by auto preflight disarming"（解锁后 2 秒内无起飞自动上锁）。
    // 因此起飞/悬停阶段完全交给 uav_base 的位置 setpoint 兜底。
    if (!has_pose || !has_velocity || !has_yaw || !global_traj_received) {
        return;
    }

    const ros::Time solve_stamp = ros::Time::now();
    double t0 = 0.0;
    if (is_tracked) {
        const Eigen::Vector3d virtual_center(
            current_mpc_state(0) - my_formation_offset(0),
            current_mpc_state(1) - my_formation_offset(1),
            current_mpc_state(2) - my_formation_offset(2));
        t0 = findClosestWaypointTime(virtual_center);

        form_nav::GlobalPathProgress progress_msg;
        progress_msg.header.stamp = solve_stamp;
        progress_msg.header.frame_id = WORLD_FRAME;
        progress_msg.trajectory_id = current_trajectory_id;
        progress_msg.path_time = t0;
        global_progress_pub.publish(progress_msg);
    } else {
        const double progress_age =
            global_progress.received
                ? (solve_stamp - global_progress.stamp).toSec()
                : std::numeric_limits<double>::infinity();
        if (!global_progress.received ||
            global_progress.trajectory_id != current_trajectory_id ||
            progress_age > GLOBAL_PROGRESS_TIMEOUT ||
            progress_age < -NEIGHBOR_FUTURE_TOLERANCE) {
            ROS_WARN_THROTTLE(1.0,
                              "%s: waiting for trajectory %u global progress",
                              UAV_NAME.c_str(), current_trajectory_id);
            publishHoldCommand();
            return;
        }
        t0 = global_progress.path_time;
    }

    // // ========== 到达终点检测：锁定悬停，避免原地振荡 ==========
    // // 每架无人机的停靠目标是"轨迹末端 + 自身编队偏移 M_i"（与 formationError
    // // 中的 virtual center 定义一致），因此用 本机位置 - 自身偏移 去比轨迹末端。
    // // 一旦满足条件即锁定（arrived_hold_），之后持续悬停不再回 MPC 求解，
    // // 避免在阈值附近悬停/振荡反复切换；收到新轨迹时由 globalTrajCallback 解锁。
    // if (arrived_hold_) {
    //     publishHoldCommand();
    //     publishStationaryPrediction(solve_stamp);
    //     return;
    // }
    // {
    //     const Eigen::Vector3d virtual_center(
    //         current_mpc_state(0) - my_formation_offset(0),
    //         current_mpc_state(1) - my_formation_offset(1),
    //         current_mpc_state(2) - my_formation_offset(2));
    //     const Eigen::Vector3d traj_end(global_traj.back().x,
    //                                    global_traj.back().y,
    //                                    global_traj.back().z);
    //     const double end_dist = (virtual_center - traj_end).norm();
    //     const double speed = std::hypot(vel_world_x, vel_world_y);
    //     if (t0 >= global_traj.back().t - 0.5 &&
    //         end_dist < 0.3 && speed < 0.15) {
    //         arrived_hold_ = true;
    //         ROS_INFO("%s: reached trajectory end (dist=%.2f m, speed=%.2f m/s), "
    //                  "locking hold.",
    //                  UAV_NAME.c_str(), end_dist, speed);
    //         publishHoldCommand();
    //         publishStationaryPrediction(solve_stamp);
    //         return;
    //     }
    // }

    // World → Body 速度转换
    double yaw = current_mpc_state(3);
    double cos_psi = cos(yaw), sin_psi = sin(yaw);
    current_mpc_state(4) =  cos_psi * vel_world_x + sin_psi * vel_world_y;
    current_mpc_state(5) = -sin_psi * vel_world_x + cos_psi * vel_world_y;
    current_mpc_state(6) = vel_world_z;
    current_mpc_state(7) = vel_body_w;

    // Align every required neighbor prediction to this solve's time grid.
    std::vector<UavState> neighbors_vec;
    std::string neighbor_failure;
    if (!buildAlignedNeighborStates(solve_stamp, neighbors_vec, neighbor_failure)) {
        ROS_WARN_THROTTLE(1.0, "%s: waiting for synchronized neighbors: %s",
                          UAV_NAME.c_str(), neighbor_failure.c_str());
        publishHoldCommand();
        return;
    }

    // 第一阶段只验证全局路径跟踪；暂时停用 ESDF 构建/更新，原代码保留。
#if 0
    // ESDF 更新：定期重建 CasADi interpolant（每 1 秒检查一次）
    static ros::Time last_esdf_update = ros::Time::now();
    if ((ros::Time::now() - last_esdf_update).toSec() > 1.0) {
        // 确保 SDFMap 对象存在且 ESDF 数据不为空（首次初始化前 distance_buffer_all_ 为空）
        if (sdf_map_ptr && !sdf_map_ptr->getDistanceBufferPtr()->empty()) {
            // bool isInitialized() const { return interp_func_ != nullptr; }
            if (!esdf_casadi.isInitialized()) {
                // 首次初始化 — 从 SDFMap 拉取地图元信息 + 数据指针，调用 init() 构建 interpolant
                double esdf_res = sdf_map_ptr->getResolution();
                Eigen::Vector3d esdf_origin = sdf_map_ptr->getOrigin();
                Eigen::Vector3i esdf_voxel = sdf_map_ptr->getVoxelNum3i();
                esdf_casadi.init(esdf_res, esdf_origin, esdf_voxel,
                                 sdf_map_ptr->getDistanceBufferPtr());
                mpc_ptr->setEsdf(&esdf_casadi);
                ROS_INFO("%s: ESDF CasADi interpolant initialized.", UAV_NAME.c_str());
            } else {
                // 已初始化过 → 重新调用 buildInterpolant()，只重新拷贝数据到 interpolant
                esdf_casadi.updateMap();
            }
        } else {
            ROS_WARN_THROTTLE(5.0, "%s: ESDF not ready yet. Waiting for cloud + odom...", UAV_NAME.c_str());
        }
        last_esdf_update = ros::Time::now();
    }
#endif

    // 求解 MPC (t0 = 最近点时间)
    bool ok = mpc_ptr->solve(t0, neighbors_vec, current_mpc_state);
    // 可视化
    if (ok) {
        auto u = mpc_ptr->getFirstU(); // body
        // body → world
        double vx = cos_psi * u[0] - sin_psi * u[1];
        double vy = sin_psi * u[0] + cos_psi * u[1];

        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = WORLD_FRAME;
        cmd.twist.linear.x = vx;
        cmd.twist.linear.y = vy;
        cmd.twist.linear.z = u[2];
        cmd.twist.angular.z = u[3];
        cmd_vel_pub.publish(cmd);

        std::vector<double> pred = mpc_ptr->getPredictX();
        form_nav::PredictedTrajectory shared_msg;
        // State 0 is constrained to the state sampled at solve_stamp.
        shared_msg.header.stamp = solve_stamp;
        shared_msg.header.frame_id = WORLD_FRAME;
        shared_msg.uav_id = MY_ID;
        shared_msg.dt = MPC_DT;
        shared_msg.steps = pred.size() / 8;
        shared_msg.states.reserve(pred.size());
        for (uint32_t i = 0; i < shared_msg.steps; ++i) {
            const size_t offset = static_cast<size_t>(i) * 8;
            const double predicted_yaw = pred[offset + 3];
            const double predicted_cos = std::cos(predicted_yaw);
            const double predicted_sin = std::sin(predicted_yaw);
            const double predicted_vx_world = predicted_cos * pred[offset + 4] -
                                              predicted_sin * pred[offset + 5];
            const double predicted_vy_world = predicted_sin * pred[offset + 4] +
                                              predicted_cos * pred[offset + 5];
            shared_msg.states.insert(shared_msg.states.end(), {
                pred[offset + 0], pred[offset + 1], pred[offset + 2], predicted_yaw,
                predicted_vx_world, predicted_vy_world, pred[offset + 6], pred[offset + 7]});
        }
        share_path_pub.publish(shared_msg);

        std_msgs::Float64MultiArray viz_msg;
        viz_msg.data = pred;
        predicted_path_pub.publish(viz_msg);

        // --- 预测轨迹 Marker 可视化 ---
        visualization_msgs::Marker pred_marker;
        pred_marker.header.frame_id = WORLD_FRAME;
        pred_marker.header.stamp = ros::Time::now();
        pred_marker.ns = "mpc_predicted_path";
        pred_marker.id = 0;
        pred_marker.type = visualization_msgs::Marker::LINE_STRIP;
        pred_marker.action = visualization_msgs::Marker::ADD;
        pred_marker.scale.x = 0.05;  // 线宽
        pred_marker.color.a = 0.8;
        pred_marker.color.r = 0.0;
        pred_marker.color.g = 1.0;
        pred_marker.color.b = 0.0;   // 绿色
        pred_marker.pose.orientation.w = 1.0;

        int n_steps = pred.size() / 8;
        for (int i = 0; i < n_steps; ++i) {
            geometry_msgs::Point pt;
            pt.x = pred[8 * i + 0];
            pt.y = pred[8 * i + 1];
            pt.z = pred[8 * i + 2];
            pred_marker.points.push_back(pt);
        }
        predicted_path_viz_pub.publish(pred_marker);

        // --- 点状 Marker（预测步点球体）---
        visualization_msgs::Marker points_marker;
        points_marker.header.frame_id = WORLD_FRAME;
        points_marker.header.stamp = ros::Time::now();
        points_marker.ns = "mpc_predict_points";
        points_marker.id = 0;
        points_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.scale.x = 0.1;
        points_marker.scale.y = 0.1;
        points_marker.scale.z = 0.1;
        points_marker.color.a = 0.6;
        points_marker.color.r = 0.0;
        points_marker.color.g = 0.7;
        points_marker.color.b = 1.0;  // 浅蓝
        points_marker.pose.orientation.w = 1.0;
        for (int i = 0; i < n_steps; ++i) {
            geometry_msgs::Point pt;
            pt.x = pred[8 * i + 0];
            pt.y = pred[8 * i + 1];
            pt.z = pred[8 * i + 2];
            points_marker.points.push_back(pt);
        }
        predicted_path_viz_pub.publish(points_marker);

        // --- 已走过轨迹 ---
        geometry_msgs::PoseStamped traveled_pose;
        traveled_pose.header.frame_id = WORLD_FRAME;
        traveled_pose.header.stamp = ros::Time::now();
        traveled_pose.pose.position.x = current_mpc_state(0);
        traveled_pose.pose.position.y = current_mpc_state(1);
        traveled_pose.pose.position.z = current_mpc_state(2);
        traveled_pose.pose.orientation.w = 1.0;
        traveled_path.header.frame_id = WORLD_FRAME;
        traveled_path.header.stamp = ros::Time::now();
        traveled_path.poses.push_back(traveled_pose);
        traveled_path_pub.publish(traveled_path);
    } else {
        ROS_WARN_THROTTLE(1.0, "%s:MPC failed, holding.", UAV_NAME.c_str());
        publishHoldCommand();
    }
}

// ========== 主函数 ==========
int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    try {
    // --- 参数读取 ---
    int my_id          = nh_private.param<int>("my_id", 0);
    int tracking_model = nh_private.param<int>("tracking_model", 1);
    WORLD_FRAME = nh_private.param<std::string>("world_frame", "world");
    const std::string formation_config_namespace =
        nh_private.param<std::string>("formation_config_namespace", "/formation_config");
    int   N      = nh_private.param<int>("mpc/N", 10);
    double dt    = nh_private.param<double>("mpc/dt", 0.05);
    double u_max = nh_private.param<double>("mpc/u_max", 6.0);
    double w_max = nh_private.param<double>("mpc/w_max", 2.0);
    double d_safe_oa = nh_private.param<double>("mpc/d_safe_oa", 0.6);
    double d_crit_oa = nh_private.param<double>("mpc/d_critical_oa", 0.3);
    double d_safe_ca = nh_private.param<double>("mpc/d_safe_ca", 0.3);
    double neighbor_timeout = nh_private.param<double>(
        "communication/neighbor_trajectory_timeout", 0.3);
    double neighbor_future_tolerance = nh_private.param<double>(
        "communication/neighbor_future_tolerance", 0.05);
    double progress_timeout = nh_private.param<double>(
        "global_progress/timeout", 0.3);
    if (!std::isfinite(neighbor_timeout) || neighbor_timeout <= 0.0) {
        throw std::runtime_error(
            "communication/neighbor_trajectory_timeout must be positive");
    }
    if (!std::isfinite(neighbor_future_tolerance) ||
        neighbor_future_tolerance < 0.0) {
        throw std::runtime_error(
            "communication/neighbor_future_tolerance must be non-negative");
    }
    if (!std::isfinite(progress_timeout) || progress_timeout <= 0.0) {
        throw std::runtime_error("global_progress/timeout must be positive");
    }

    MY_ID = my_id;
    MPC_HORIZON = N;
    MPC_DT = dt;
    NEIGHBOR_TRAJECTORY_TIMEOUT = neighbor_timeout;
    NEIGHBOR_FUTURE_TOLERANCE = neighbor_future_tolerance;
    GLOBAL_PROGRESS_TIMEOUT = progress_timeout;
    UAV_NAME = "uav" + std::to_string(my_id);
    LOCAL_FRAME = nh_private.param<std::string>("local_frame", UAV_NAME + "/map");

    tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr);

    const FormationConfig formation = FormationConfig::load(
        nh, formation_config_namespace, my_id);
    const bool is_tracked = formation.leader_access;
    const std::vector<int>& neighbor_ids = formation.neighbor_ids;
    required_neighbor_ids = neighbor_ids;
    const auto own_offset = formation.offsets.at(my_id);
    my_formation_offset = Eigen::Vector4d(
        own_offset[0], own_offset[1], own_offset[2], own_offset[3]);
    ROS_INFO("%s: leader_access=%d, model=%d, "
             "neighbors=%zu, offsets=%zu",
             UAV_NAME.c_str(), is_tracked, tracking_model,
             neighbor_ids.size(), formation.offsets.size());

    // --- MPC 初始化 ---
    mpc_ptr.reset(new Mpc());
    mpc_ptr->setMyId(my_id);
    mpc_ptr->setTracked(is_tracked);
    mpc_ptr->setTrackingModel(tracking_model);
    mpc_ptr->setFormationOffsets(formation.offsets);

    mpc_ptr->setPredictionHorizon(N);
    mpc_ptr->setTimeStep(dt);
    mpc_ptr->setMaxVelocity(u_max);
    mpc_ptr->setMaxAngularVelocity(w_max);
    mpc_ptr->setObstacleSafeDistance(d_safe_oa);
    mpc_ptr->setObstacleCriticalDistance(d_crit_oa);
    mpc_ptr->setCollisionSafeDistance(d_safe_ca);

    ROS_INFO("%s: MPC params -- N=%d, dt=%.3f, u_max=%.2f, w_max=%.2f, "
             "d_safe_oa=%.2f, d_critical_oa=%.2f, d_safe_ca=%.2f, "
             "neighbor_timeout=%.3f, future_tolerance=%.3f",
             UAV_NAME.c_str(), N, dt, u_max, w_max,
             d_safe_oa, d_crit_oa, d_safe_ca,
             NEIGHBOR_TRAJECTORY_TIMEOUT, NEIGHBOR_FUTURE_TOLERANCE);

    // 第一阶段不使用障碍物代价，暂时停用 ESDF 地图初始化；原代码保留。
#if 0
    sdf_map_ptr.reset(new SDFMap());
    sdf_map_ptr->initMap(nh_private); // nh 是公共 NodeHandle，读取 /sdf_map/...
    mpc_ptr->setEsdf(&esdf_casadi);
#endif

    // --- 订阅（飞控状态、位置、速度、邻居预测状态） --- 
    ros::Subscriber fc_state_sub = nh.subscribe<mavros_msgs::State>(
        "/" + UAV_NAME + "/mavros/state", 10, fcStateCallback);
    ros::Subscriber pose_sub = nh.subscribe("/" + UAV_NAME + "/mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber vel_sub = nh.subscribe("/" + UAV_NAME + "/mavros/local_position/velocity_local", 10, velocityCallbackWorld);

    // 全局轨迹订阅
    ros::Subscriber global_traj_sub = nh.subscribe<form_nav::GlobalTrajectory>(
        "/kino_astar/trajectory", 1, globalTrajCallback);
    ros::Subscriber global_progress_sub;
    if (is_tracked) {
        global_progress_pub = nh.advertise<form_nav::GlobalPathProgress>(
            "/formation/global_path_progress", 1);
    } else {
        global_progress_sub = nh.subscribe<form_nav::GlobalPathProgress>(
            "/formation/global_path_progress", 1, globalProgressCallback);
    }

    // 动态订阅邻居
    std::vector<ros::Subscriber> neighbor_subs;
    for (int nid : neighbor_ids) {
        std::string topic = "/uav" + std::to_string(nid) + "/predicted_path_share";
        neighbor_subs.push_back(nh.subscribe<form_nav::PredictedTrajectory>(
            topic, 10,
            [nid](const form_nav::PredictedTrajectory::ConstPtr& message) {
                neighborTrajectoryCallback(message, nid);
            }));
    }

    // --- 发布（setpoint、速度、共享预测状态、可视化） ---
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + UAV_NAME + "/mavros/setpoint_position/local", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/" + UAV_NAME + "/mavros/setpoint_velocity/cmd_vel", 1);
    share_path_pub = nh.advertise<form_nav::PredictedTrajectory>("/" + UAV_NAME + "/predicted_path_share", 1);
    predicted_path_pub = nh.advertise<std_msgs::Float64MultiArray>("/" + UAV_NAME + "/predicted_path_viz", 1);
    predicted_path_viz_pub = nh.advertise<visualization_msgs::Marker>("/" + UAV_NAME + "/mpc_predicted_path_marker", 10);
    traveled_path_pub = nh.advertise<nav_msgs::Path>("/" + UAV_NAME + "/traveled_path", 10);

    ros::Rate rate(LOOP_RATE);

    // 第一阶段不构建 ESDF；后续恢复障碍物代价时再启用延迟初始化逻辑。
    ROS_INFO("%s: running distributed formation tracking without obstacle cost; "
             "waiting for %zu required neighbor(s).",
             UAV_NAME.c_str(), required_neighbor_ids.size());

    // 定时器
    ros::Timer timer = nh.createTimer(
        ros::Duration(1.0 / LOOP_RATE),
        [is_tracked](const ros::TimerEvent& event) {
            controlLoop(event, is_tracked);
        });

    ros::spin();
    } catch (const XmlRpc::XmlRpcException& e) {
        ROS_FATAL("XmlRpcException: %s (code=%d)", e.getMessage().c_str(), e.getCode());
        return 1;
    } catch (const std::exception& e) {
        ROS_FATAL("Exception in main: %s", e.what());
        return 1;
    } catch (...) {
        ROS_FATAL("Unknown exception in main");
        return 1;
    }
    return 0;
}
