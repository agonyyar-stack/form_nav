#include <ros/ros.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <mavros_msgs/State.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>
#include "form_nav/mpc.h"
#include "form_nav/formation_config.h"
#include "form_nav/kd_tree_map.h"
#include "form_nav/GlobalTrajectory.h"
#include "form_nav/PredictedTrajectory.h"
#include "form_nav/DynamicObstacleTrajectory.h"

// --- 基本配置 ---
std::string UAV_NAME;
std::string WORLD_FRAME = "world";
std::string LOCAL_FRAME;
int MY_ID = 0;
int MPC_HORIZON = 10;
double MPC_DT = 0.05;
double NEIGHBOR_TRAJECTORY_TIMEOUT = 0.3;
double NEIGHBOR_FUTURE_TOLERANCE = 0.05;
const double LOOP_RATE = 20.0;
double tau_v_eff = 0.8;    // 线速度一阶时间常数（与 MPC 内部一致，用于制动可行性预检）
double ARRIVAL_DISTANCE = 0.1;   // 到达锁定：到终点直线距离阈值 (m)
double ARRIVAL_VELOCITY = 0.1;   // 到达锁定：3D 速度阈值 (m/s)

std::unique_ptr<tf2_ros::Buffer> tf_buffer_ptr;
std::unique_ptr<tf2_ros::TransformListener> tf_listener_ptr;

// --- ROS 通信 ---
ros::Publisher cmd_vel_pub;
ros::Publisher local_pos_pub;
ros::Publisher predicted_path_pub;
ros::Publisher share_path_pub;
ros::Publisher predicted_path_viz_pub;
ros::Publisher traveled_path_pub;
ros::Publisher queried_obstacles_pub;  // 第三步：最近查询障碍点可视化
ros::Publisher obstacle_cloud_pub;     // 第三步：KD-tree 中的障碍点云可视化
ros::Publisher sub_goal_pub;           // 滚动子目标可视化

// --- 状态量 ---
// 显式初始化，避免首个传感器消息到达前把未定义数据送入求解器。
Eigen::VectorXd current_mpc_state = Eigen::VectorXd::Zero(8);

// 控制循环只有在这些数据全部到齐后才允许调用 MPC。
bool has_odom = false;

double vel_world_x = 0, vel_world_y = 0, vel_world_z = 0;
double vel_world_w = 0;    // 偏航角速率 (rad/s)

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

// --- 动态障碍物轨迹（已知轨迹表） ---
// 动态障碍物仅通过轨迹参与 MPC 避障惩罚；点云只用于可视化，不进入
// 静态 KD-tree / A*。轨迹超时按该障碍物缺失处理，不阻塞控制。
struct DynamicObstacleCache {
    int shape = 0;          // 0=球体, 1=圆柱
    double radius = 0.3;    // 障碍物半径 (m)
    double height = 0.0;    // 圆柱高度 (m)，球体填 0
    Eigen::MatrixXd predicted_states;
    ros::Time prediction_start;
    double dt = 0.0;
};

std::map<int, DynamicObstacleCache> dynamic_obstacles;
const double DYNAMIC_OBSTACLE_TIMEOUT = 1.0;  // 障碍物轨迹超时 (s)

// --- KD-tree 障碍物地图（第三步：静态避障） ---
std::unique_ptr<KdTreeMap> kd_tree_ptr;
bool kd_tree_ready = false;
double kd_tree_voxel_size = 0.1;  // 建树前体素降采样尺寸 (m)
double kd_tree_update_interval = 2.0;  // KD-tree 重建最小间隔 (s)
double obstacle_hold_dist = 0.15; // 当前点最近障碍距离低于该值时悬停（紧急刹车）

// ========== 障碍物点云回调（第三步） ==========
// 仿真阶段订阅 /map_generator/global_cloud（world 系、静态障碍物真值）；
// 后续接入感知模块时只需换 topic（必要时在回调里做 TF 变换到 WORLD_FRAME）。
void obstacleCloudCallback(const sensor_msgs::PointCloud2::ConstPtr& msg) {
    if (msg->header.frame_id != WORLD_FRAME) {
        ROS_WARN_THROTTLE(2.0, "%s: obstacle cloud frame %s != %s, ignored.",
                          UAV_NAME.c_str(), msg->header.frame_id.c_str(),
                          WORLD_FRAME.c_str());
        return;
    }
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg(*msg, cloud);
    if (cloud.empty()) {
        return;
    }
    // 静态点云每帧内容相同：点数未变且未到重建间隔时跳过，避免 10 Hz 下
    // 反复对十几万点重建树阻塞 MPC 求解线程。
    static ros::Time last_build = ros::Time(0);
    static size_t last_cloud_size = 0;
    const ros::Time now = ros::Time::now();
    if (cloud.size() == last_cloud_size &&
        (now - last_build).toSec() < kd_tree_update_interval) {
        return;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr(
        new pcl::PointCloud<pcl::PointXYZ>(cloud));
    kd_tree_ptr->setCloud(cloud_ptr, kd_tree_voxel_size);
    kd_tree_ready = true;
    last_build = now;
    last_cloud_size = cloud.size();

    // 障碍点云可视化（限频发布，避免刷屏）
    static ros::Time last_cloud_viz = ros::Time::now();
    if ((ros::Time::now() - last_cloud_viz).toSec() > 1.0) {
        sensor_msgs::PointCloud2 out_msg = *msg;
        out_msg.header.stamp = ros::Time::now();
        obstacle_cloud_pub.publish(out_msg);
        last_cloud_viz = ros::Time::now();
    }
    ROS_INFO_THROTTLE(5.0, "%s: KD-tree updated with %zu obstacle points.",
                      UAV_NAME.c_str(), kd_tree_ptr->size());
}

// --- 全局轨迹 ---
std::vector<GlobalWaypoint> global_traj;
bool global_traj_received = false;
uint32_t current_trajectory_id = 0;

// 到达终点后的悬停锁定标志：一旦置位就持续保持悬停，不再回到 MPC 求解，
// 避免在终点阈值附近悬停/振荡反复切换。收到新的全局轨迹时解除锁定，
// 允许重新规划后再次起飞。
bool arrived_hold_ = false;

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
// 数据格式: 每点 6 个 double [x, y, z, vx, vy, vz]
void globalTrajCallback(const form_nav::GlobalTrajectory::ConstPtr& msg) {
    constexpr size_t stride = 6;
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
    for (uint32_t i = 0; i < msg->steps; ++i) {
        GlobalWaypoint wp;
        const size_t off = static_cast<size_t>(i) * stride;
        wp.x   = msg->waypoints[off + 0];
        wp.y   = msg->waypoints[off + 1];
        wp.z   = msg->waypoints[off + 2];
        wp.vx  = msg->waypoints[off + 3];
        wp.vy  = msg->waypoints[off + 4];
        wp.vz  = msg->waypoints[off + 5];
        wp.yaw = 0;
        wp.vyaw = 0;
        received_trajectory.push_back(wp);
    }

    global_traj = std::move(received_trajectory);
    global_traj_received = true;
    current_trajectory_id = msg->trajectory_id;
    neighbor_trajectories.clear();
    // 新轨迹意味着新的任务：解除到达悬停锁定，允许再次起飞。
    arrived_hold_ = false;
    mpc_ptr->setGlobalTrajectory(global_traj);
    ROS_INFO("%s: received global trajectory %u with %zu waypoints",
             UAV_NAME.c_str(), current_trajectory_id, global_traj.size());
}

// ========== 里程计回调 ==========
// 位姿与速度统一取自里程计（仿真: /mavros/local_position/odom；
// 实机: FAST-LIO odom）。线速度在 odom 的 header 系下，角速度在机体系下。
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    try {
        const std::string source_frame = msg->header.frame_id.empty()
                                             ? LOCAL_FRAME
                                             : msg->header.frame_id;

        // 1) 位姿：从 odom 的 header 系变换到 WORLD_FRAME
        geometry_msgs::PoseStamped source_pose;
        source_pose.header = msg->header;
        source_pose.header.frame_id = source_frame;
        source_pose.pose = msg->pose.pose;
        const auto world_pose = tf_buffer_ptr->transform(
            source_pose, WORLD_FRAME, ros::Duration(0.05));
        current_mpc_state(0) = world_pose.pose.position.x;
        current_mpc_state(1) = world_pose.pose.position.y;
        current_mpc_state(2) = world_pose.pose.position.z;
        current_mpc_state(3) = tf2::getYaw(world_pose.pose.orientation);

        // 2) 线速度：MAVROS odom 的 twist.linear 定义在机体系
        //    （child_frame/base_link，遵循 nav_msgs/Odometry 约定），
        //    需用位姿四元数旋转到世界系（与 odom_to_world 的已验证逻辑一致）。
        //    注意：若实机接入 FAST-LIO，其 odom 的 twist.linear 已是
        //    camera_init（世界系），接入时需去掉这里的旋转，避免双重旋转。
        tf2::Quaternion world_orientation;
        tf2::fromMsg(world_pose.pose.orientation, world_orientation);
        const tf2::Vector3 linear_world = tf2::quatRotate(
            world_orientation,
            tf2::Vector3(msg->twist.twist.linear.x,
                         msg->twist.twist.linear.y,
                         msg->twist.twist.linear.z));
        vel_world_x = linear_world.x();
        vel_world_y = linear_world.y();
        vel_world_z = linear_world.z();

        // 3) 角速度：机体系 z 分量即偏航角速率（ENU 约定），无需 TF
        vel_world_w = msg->twist.twist.angular.z;

        has_odom = true;
    } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(2.0, "%s: odom transform to %s failed: %s",
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
            predicted_states(0, i) = world_pose.pose.position.x;
            predicted_states(1, i) = world_pose.pose.position.y;
            predicted_states(2, i) = world_pose.pose.position.z;
            predicted_states(3, i) = yaw;
            // 共享轨迹速度约定为 header.frame_id（world）系；解耦模型下
            // 线速度即惯性系独立状态，直接存储，无需再旋转到机体系。
            predicted_states(4, i) = world_velocity.vector.x;
            predicted_states(5, i) = world_velocity.vector.y;
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

// ========== 动态障碍物轨迹回调 ==========
// 轨迹为 world 系真值/预测（仿真阶段由 random_obs 用解析轨迹发布），
// 校验后按 obstacle_id 存入动态障碍物表。
void dynamicObstacleCallback(
    const form_nav::DynamicObstacleTrajectory::ConstPtr& msg) {
    if (msg->header.stamp.isZero() || msg->header.frame_id != WORLD_FRAME ||
        msg->steps == 0 || !std::isfinite(msg->dt) || msg->dt <= 0.0 ||
        msg->radius <= 0.0 ||
        msg->states.size() != static_cast<size_t>(msg->steps) * 8 ||
        (msg->shape != 0 && msg->shape != 1) ||
        (msg->shape == 1 && (!std::isfinite(msg->height) || msg->height <= 0.0))) {
        ROS_WARN_THROTTLE(2.0, "%s: invalid dynamic obstacle trajectory (id=%d)",
                          UAV_NAME.c_str(), msg->obstacle_id);
        return;
    }

    const double message_age = (ros::Time::now() - msg->header.stamp).toSec();
    if (message_age > DYNAMIC_OBSTACLE_TIMEOUT ||
        message_age < -NEIGHBOR_FUTURE_TOLERANCE) {
        ROS_WARN_THROTTLE(2.0,
                          "%s: rejected dynamic obstacle %d trajectory age %.3f s",
                          UAV_NAME.c_str(), msg->obstacle_id, message_age);
        return;
    }

    const auto existing = dynamic_obstacles.find(msg->obstacle_id);
    if (existing != dynamic_obstacles.end() &&
        msg->header.stamp <= existing->second.prediction_start) {
        return;  // 乱序旧帧直接丢弃
    }

    if (!std::all_of(msg->states.begin(), msg->states.end(),
                     [](double value) { return std::isfinite(value); })) {
        ROS_WARN_THROTTLE(2.0,
                          "%s: non-finite state in dynamic obstacle %d",
                          UAV_NAME.c_str(), msg->obstacle_id);
        return;
    }

    DynamicObstacleCache cache;
    cache.shape = msg->shape;
    cache.radius = msg->radius;
    cache.height = msg->height;
    cache.prediction_start = msg->header.stamp;
    cache.dt = msg->dt;
    cache.predicted_states.resize(8, msg->steps);
    for (uint32_t i = 0; i < msg->steps; ++i) {
        const size_t offset = static_cast<size_t>(i) * 8;
        for (int row = 0; row < 8; ++row) {
            cache.predicted_states(row, i) = msg->states[offset + row];
        }
    }
    dynamic_obstacles[msg->obstacle_id] = std::move(cache);
}

double interpolateYaw(double from, double to, double ratio);

// 把动态障碍物轨迹按求解时间网格对齐（与邻居轨迹相同的 age 采样/插值逻辑）。
// 障碍物轨迹可选：缺失或超时的障碍物直接跳过，不阻塞控制。
void buildAlignedDynamicObstacles(
    const ros::Time& prediction_start,
    std::vector<DynamicObstacleState>& out) {
    out.clear();
    for (const auto& entry : dynamic_obstacles) {
        const int id = entry.first;
        const DynamicObstacleCache& cache = entry.second;
        const double age = (prediction_start - cache.prediction_start).toSec();
        if (age > DYNAMIC_OBSTACLE_TIMEOUT ||
            age < -NEIGHBOR_FUTURE_TOLERANCE) {
            continue;
        }

        const int source_steps = static_cast<int>(cache.predicted_states.cols());
        // 障碍物轨迹至少覆盖 MPC 视界（允许更长，末端保持）
        if (source_steps < MPC_HORIZON + 1 || cache.dt <= 0.0) {
            continue;
        }

        DynamicObstacleState obstacle;
        obstacle.id = id;
        obstacle.shape = cache.shape;
        obstacle.radius = cache.radius;
        obstacle.height = cache.height;
        obstacle.predicted_states.resize(8, MPC_HORIZON + 1);
        for (int step = 0; step <= MPC_HORIZON; ++step) {
            const double query_offset = age + step * MPC_DT;
            const double source_index = query_offset / cache.dt;

            if (source_index <= 0.0) {
                obstacle.predicted_states.col(step) =
                    cache.predicted_states.col(0);
                continue;
            }
            if (source_index >= source_steps - 1) {
                obstacle.predicted_states.col(step) =
                    cache.predicted_states.col(source_steps - 1);
                continue;
            }

            const int lower = static_cast<int>(std::floor(source_index));
            const int upper = lower + 1;
            const double ratio = source_index - lower;
            obstacle.predicted_states.col(step) =
                (1.0 - ratio) * cache.predicted_states.col(lower) +
                ratio * cache.predicted_states.col(upper);
            obstacle.predicted_states(3, step) = interpolateYaw(
                cache.predicted_states(3, lower),
                cache.predicted_states(3, upper), ratio);
        }
        out.push_back(std::move(obstacle));
    }
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

// 锁定悬停期间发布终点位置 setpoint（世界系转 local 系），由 PX4 位置
// 控制器锁定位置，避免纯零速度指令下的缓慢漂移。若 TF 变换失败则退回
// 零速度 hold，保证 OFFBOARD 控制源不断流。
void publishHoldPositionSetpoint(const ros::Time& stamp) {
    geometry_msgs::PoseStamped hold_pose;
    hold_pose.header.stamp = stamp;
    hold_pose.header.frame_id = WORLD_FRAME;
    // 分布式规划：轨迹末端已是本机物理目标，不再叠加编队偏移。
    hold_pose.pose.position.x = global_traj.back().x;
    hold_pose.pose.position.y = global_traj.back().y;
    hold_pose.pose.position.z = global_traj.back().z;
    tf2::Quaternion hold_yaw;
    hold_yaw.setRPY(0.0, 0.0, current_mpc_state(3));
    hold_pose.pose.orientation = tf2::toMsg(hold_yaw);
    try {
        const geometry_msgs::PoseStamped local_pose = tf_buffer_ptr->transform(
            hold_pose, LOCAL_FRAME, ros::Duration(0.05));
        local_pos_pub.publish(local_pose);
    } catch (const tf2::TransformException& error) {
        ROS_WARN_THROTTLE(1.0, "%s: hold pose transform to %s failed: %s",
                          UAV_NAME.c_str(), LOCAL_FRAME.c_str(), error.what());
        publishHoldCommand();
    }
}

// 悬停/锁定期间发布"原地静止"的预测轨迹，避免邻居因为收不到本机
// 预测轨迹而进入 missing trajectory 等待状态，导致整个编队连锁卡死。
void publishStationaryPrediction(const ros::Time& stamp) {
    form_nav::PredictedTrajectory msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = WORLD_FRAME;
    msg.uav_id = MY_ID;
    msg.dt = MPC_DT;
    msg.steps = MPC_HORIZON + 1;
    msg.states.reserve(static_cast<size_t>(msg.steps) * 8);
    for (uint32_t i = 0; i < msg.steps; ++i) {
        // 世界系位置/航向保持当前值，速度全 0（世界速度，符合共享轨迹约定）。
        msg.states.insert(msg.states.end(), {
            current_mpc_state(0), current_mpc_state(1),
            current_mpc_state(2), current_mpc_state(3),
            0.0, 0.0, 0.0, 0.0});
    }
    share_path_pub.publish(msg);
}

// ========== 主控制循环 ==========
void controlLoop(const ros::TimerEvent&) {
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
    if (!has_odom || !global_traj_received) {
        return;
    }

    const ros::Time solve_stamp = ros::Time::now();
    // 分布式规划：每机在自身的物理轨迹上投影当前物理位置得到路径进度
    // （累积弧长 s0），供滚动子目标前瞻使用。
    const Eigen::Vector3d current_pos(
        current_mpc_state(0), current_mpc_state(1), current_mpc_state(2));
    const double s0 = mpc_ptr->projectToPath(current_pos);

    // ========== 到达终点检测：锁定悬停，避免原地振荡 ==========
    // 每架无人机的停靠目标就是自己轨迹的末端（物理坐标）。
    // 一旦满足条件即锁定（arrived_hold_），之后持续悬停不再回 MPC 求解，
    // 避免在阈值附近悬停/振荡反复切换；收到新轨迹时由 globalTrajCallback 解锁。
    // 到达判定 = 到终点直线距离 < arrival_distance 且 3D 速度 < arrival_velocity。
    const Eigen::Vector3d traj_end(global_traj.back().x,
                                   global_traj.back().y,
                                   global_traj.back().z);
    const double end_dist = (current_pos - traj_end).norm();
    const double speed = std::hypot(vel_world_x, vel_world_y, vel_world_z);

    if (arrived_hold_) {
        publishHoldPositionSetpoint(solve_stamp);
        publishStationaryPrediction(solve_stamp);
        return;
    }
    if (end_dist < ARRIVAL_DISTANCE && speed < ARRIVAL_VELOCITY) {
        arrived_hold_ = true;
        ROS_INFO("%s: reached trajectory end (dist=%.2f m, speed=%.2f m/s), "
                 "locking hold.",
                 UAV_NAME.c_str(), end_dist, speed);
        publishHoldPositionSetpoint(solve_stamp);
        publishStationaryPrediction(solve_stamp);
        return;
    }

    // 解耦模型：线速度为惯性系独立状态，与 yaw 无关，直接填入世界系速度。
    current_mpc_state(4) = vel_world_x;
    current_mpc_state(5) = vel_world_y;
    current_mpc_state(6) = vel_world_z;
    current_mpc_state(7) = vel_world_w;

    // 制动可行性预检：当前状态须满足 MPC 精确停车约束 tau_v*||v|| <= d_g，
    // 否则 k=0 约束必然不可行，直接悬停（阈值与 MPC 内部 tau_v_eff 一致）。
    const double cur_speed_sq = vel_world_x * vel_world_x +
                                vel_world_y * vel_world_y +
                                vel_world_z * vel_world_z;
    const double cur_speed = std::sqrt(cur_speed_sq);
    if (cur_speed * tau_v_eff > end_dist) {
        ROS_WARN_THROTTLE(0.5,
                          "%s: current speed exceeds braking feasibility "
                          "(v=%.2f m/s, d_g=%.2f m), holding.",
                          UAV_NAME.c_str(), cur_speed, end_dist);
        publishHoldCommand();
        publishStationaryPrediction(solve_stamp);
        return;
    }

    // 近距碰撞安全兜底 —— 当前点已进入临界距离时停止求解并悬停，
    // 避免在代价势垒梯度很大的区间内求解产生异常指令。
    if (kd_tree_ready) {
        const Eigen::Vector3d cur_pos(current_mpc_state(0),
                                      current_mpc_state(1),
                                      current_mpc_state(2));
        const double nearest_dist = kd_tree_ptr->nearestDistance(cur_pos);
        if (nearest_dist < obstacle_hold_dist) {
            ROS_WARN_THROTTLE(0.5,
                              "%s: too close to obstacle (%.2f m < %.2f m), "
                              "holding.",
                              UAV_NAME.c_str(), nearest_dist, obstacle_hold_dist);
            publishHoldCommand();
            publishStationaryPrediction(solve_stamp);
            return;
        }
    }

    // Align every required neighbor prediction to this solve's time grid.
    std::vector<UavState> neighbors_vec;
    std::string neighbor_failure;
    if (!buildAlignedNeighborStates(solve_stamp, neighbors_vec, neighbor_failure)) {
        ROS_WARN_THROTTLE(1.0, "%s: waiting for synchronized neighbors: %s",
                          UAV_NAME.c_str(), neighbor_failure.c_str());
        publishHoldCommand();
        return;
    }

    // 动态障碍物轨迹对齐（已知轨迹表，缺失/超时按无障碍物处理）
    std::vector<DynamicObstacleState> dyn_obs_vec;
    buildAlignedDynamicObstacles(solve_stamp, dyn_obs_vec);

    // 求解 MPC (t0 = 最近点时间)
    bool ok = mpc_ptr->solve(s0, neighbors_vec, dyn_obs_vec, current_mpc_state);
    // 可视化
    if (ok) {
        auto u = mpc_ptr->getFirstU(); // 惯性系线速度指令 + yaw rate 指令
        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = WORLD_FRAME;
        cmd.twist.linear.x = u[0];
        cmd.twist.linear.y = u[1];
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
            // 解耦模型下预测速度已是世界系，直接按消息约定（world 系）发布
            shared_msg.states.insert(shared_msg.states.end(), {
                pred[offset + 0], pred[offset + 1], pred[offset + 2], pred[offset + 3],
                pred[offset + 4], pred[offset + 5], pred[offset + 6], pred[offset + 7]});
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

        // 第三步：可视化本次求解实际使用的最近障碍点
        visualization_msgs::Marker obs_marker;
        obs_marker.header.frame_id = WORLD_FRAME;
        obs_marker.header.stamp = ros::Time::now();
        obs_marker.ns = "mpc_queried_obstacles";
        obs_marker.id = 0;
        obs_marker.type = visualization_msgs::Marker::SPHERE_LIST;
        obs_marker.action = visualization_msgs::Marker::ADD;
        obs_marker.scale.x = 0.15;
        obs_marker.scale.y = 0.15;
        obs_marker.scale.z = 0.15;
        obs_marker.color.a = 0.9;
        obs_marker.color.r = 1.0;
        obs_marker.color.g = 0.2;
        obs_marker.color.b = 0.0;
        obs_marker.pose.orientation.w = 1.0;
        for (const Eigen::Vector3d& pt : mpc_ptr->getLastObstaclePoints()) {
            geometry_msgs::Point p;
            p.x = pt.x();
            p.y = pt.y();
            p.z = pt.z();
            obs_marker.points.push_back(p);
        }
        queried_obstacles_pub.publish(obs_marker);

        // --- 滚动子目标可视化 ---
        visualization_msgs::Marker sub_goal_marker;
        sub_goal_marker.header.frame_id = WORLD_FRAME;
        sub_goal_marker.header.stamp = ros::Time::now();
        sub_goal_marker.ns = "mpc_sub_goal";
        sub_goal_marker.id = 0;
        sub_goal_marker.type = visualization_msgs::Marker::SPHERE;
        sub_goal_marker.action = visualization_msgs::Marker::ADD;
        sub_goal_marker.pose.orientation.w = 1.0;
        const GlobalWaypoint& sub_goal = mpc_ptr->getLastRollingGoal();
        sub_goal_marker.pose.position.x = sub_goal.x;
        sub_goal_marker.pose.position.y = sub_goal.y;
        sub_goal_marker.pose.position.z = sub_goal.z;
        sub_goal_marker.scale.x = 0.25;
        sub_goal_marker.scale.y = 0.25;
        sub_goal_marker.scale.z = 0.25;
        sub_goal_marker.color.a = 0.9;
        sub_goal_marker.color.r = 1.0;
        sub_goal_marker.color.g = 0.8;
        sub_goal_marker.color.b = 0.0;
        sub_goal_pub.publish(sub_goal_marker);
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
    WORLD_FRAME        = nh_private.param<std::string>("world_frame", "world");
    const std::string formation_config_namespace =
                         nh_private.param<std::string>("formation_config_namespace", "/formation_config");
    int   N      = nh_private.param<int>("mpc/N", 10);
    double dt    = nh_private.param<double>("mpc/dt", 0.05);
    double u_max = nh_private.param<double>("mpc/u_max", 6.0);
    double w_max = nh_private.param<double>("mpc/w_max", 2.0);
    double d_safe_ca = nh_private.param<double>("mpc/d_safe_ca", 0.75);
    double q_ca      = nh_private.param<double>("mpc/q_ca", 1.6);
    double w_ca      = nh_private.param<double>("mpc/w_ca", 200.0);
    double w_form    = nh_private.param<double>("mpc/w_form", 2000.0);
    double w_subgoal = nh_private.param<double>("mpc/w_subgoal", 20000.0);
    double w_yaw     = nh_private.param<double>("mpc/w_yaw", 1000.0);
    int nearest_obs_num = nh_private.param<int>("mpc/nearest_obs_num", 3);
    double drone_radius = nh_private.param<double>("mpc/drone_radius", 0.35);
    double safe_dist    = nh_private.param<double>("mpc/safe_dist", 0.5);
    double q_oa         = nh_private.param<double>("mpc/q_oa", 2.5);
    double w_oa         = nh_private.param<double>("mpc/w_oa", 30000.0);
    int replan_max_iter = nh_private.param<int>("mpc/replan_max_iter", 0);
    kd_tree_voxel_size  = nh_private.param<double>("mpc/kd_tree_voxel_size", 0.1);
    kd_tree_update_interval =
        nh_private.param<double>("mpc/kd_tree_update_interval", 2.0);
    int ipopt_max_iter  = nh_private.param<int>("mpc/ipopt_max_iter", 100);
    double ipopt_tol    = nh_private.param<double>("mpc/ipopt_tol", 1e-6);
    double hold_dist = nh_private.param<double>("mpc/hold_dist", 0.15);
    const std::string cloud_topic =
        nh_private.param<std::string>("cloud_topic", "/map_generator/global_cloud");
    double neighbor_timeout          = nh_private.param<double>("communication/neighbor_trajectory_timeout", 0.3);
    double neighbor_future_tolerance = nh_private.param<double>("communication/neighbor_future_tolerance", 0.05);
    double lookahead_distance        = nh_private.param<double>("mpc/lookahead_distance", 2.0);
    double yaw_lock_dist             = nh_private.param<double>("mpc/yaw_lock_dist", 0.3);
    double tau_v                     = nh_private.param<double>("mpc/tau_v", 0.8);
    double tau_w                     = nh_private.param<double>("mpc/tau_w", 0.5);
    double arrival_distance          = nh_private.param<double>("mpc/arrival_distance", 0.1);
    double arrival_velocity          = nh_private.param<double>("mpc/arrival_velocity", 0.1);
    double w_u                       = nh_private.param<double>("mpc/w_u", 100.0);
    double w_du                      = nh_private.param<double>("mpc/w_du", 100.0);
    double w_dyn                     = nh_private.param<double>("mpc/w_dyn", 300.0);
    double d_safe_dyn                = nh_private.param<double>("mpc/d_safe_dyn", 0.2);
    double d_act_dyn                 = nh_private.param<double>("mpc/d_act_dyn", 2.5);
    if (!std::isfinite(neighbor_timeout) || neighbor_timeout <= 0.0) {
        throw std::runtime_error(
            "communication/neighbor_trajectory_timeout must be positive");
    }
    if (!std::isfinite(neighbor_future_tolerance) ||
        neighbor_future_tolerance < 0.0) {
        throw std::runtime_error(
            "communication/neighbor_future_tolerance must be non-negative");
    }
    if (!std::isfinite(w_du) || w_du < 0.0) {
        throw std::runtime_error("mpc/w_du must be non-negative");
    }
    if (!std::isfinite(w_u) || w_u < 0.0) {
        throw std::runtime_error("mpc/w_u must be non-negative");
    }
    if (!std::isfinite(w_dyn) || w_dyn < 0.0) {
        throw std::runtime_error("mpc/w_dyn must be non-negative");
    }
    if (!std::isfinite(d_safe_dyn) || d_safe_dyn < 0.0) {
        throw std::runtime_error("mpc/d_safe_dyn must be non-negative");
    }
    if (!std::isfinite(d_act_dyn) || d_act_dyn <= d_safe_dyn) {
        throw std::runtime_error(
            "mpc/d_act_dyn must be finite and greater than mpc/d_safe_dyn");
    }
    if (!std::isfinite(d_safe_ca) || d_safe_ca < 0.0) {
        throw std::runtime_error("mpc/d_safe_ca must be non-negative");
    }
    if (!std::isfinite(q_ca) || q_ca <= d_safe_ca) {
        throw std::runtime_error(
            "mpc/q_ca must be finite and greater than mpc/d_safe_ca");
    }
    if (!std::isfinite(w_ca) || w_ca < 0.0) {
        throw std::runtime_error("mpc/w_ca must be non-negative");
    }
    if (!std::isfinite(w_form) || w_form < 0.0) {
        throw std::runtime_error("mpc/w_form must be non-negative");
    }
    if (!std::isfinite(w_subgoal) || w_subgoal < 0.0) {
        throw std::runtime_error("mpc/w_subgoal must be non-negative");
    }
    if (!std::isfinite(w_yaw) || w_yaw < 0.0) {
        throw std::runtime_error("mpc/w_yaw must be non-negative");
    }
    if (!std::isfinite(lookahead_distance) || lookahead_distance <= 0.0) {
        throw std::runtime_error("mpc/lookahead_distance must be positive");
    }
    if (!std::isfinite(yaw_lock_dist) || yaw_lock_dist < 0.0) {
        throw std::runtime_error("mpc/yaw_lock_dist must be non-negative");
    }
    if (!std::isfinite(arrival_distance) || arrival_distance <= 0.0) {
        throw std::runtime_error("mpc/arrival_distance must be positive");
    }
    if (!std::isfinite(arrival_velocity) || arrival_velocity <= 0.0) {
        throw std::runtime_error("mpc/arrival_velocity must be positive");
    }
    if (!std::isfinite(tau_v) || tau_v <= 0.0) {
        throw std::runtime_error("mpc/tau_v must be positive");
    }
    if (!std::isfinite(tau_w) || tau_w <= 0.0) {
        throw std::runtime_error("mpc/tau_w must be positive");
    }
    if (nearest_obs_num <= 0) {
        throw std::runtime_error("mpc/nearest_obs_num must be positive");
    }
    if (!std::isfinite(drone_radius) || drone_radius <= 0.0) {
        throw std::runtime_error("mpc/drone_radius must be positive");
    }
    if (!std::isfinite(safe_dist) || safe_dist <= 0.0) {
        throw std::runtime_error("mpc/safe_dist must be positive");
    }
    if (!std::isfinite(q_oa) || q_oa <= safe_dist) {
        throw std::runtime_error(
            "mpc/q_oa must be greater than mpc/safe_dist");
    }
    if (!std::isfinite(w_oa) || w_oa < 0.0) {
        throw std::runtime_error("mpc/w_oa must be non-negative");
    }
    if (replan_max_iter < 0) {
        throw std::runtime_error("mpc/replan_max_iter must be non-negative");
    }
    if (!std::isfinite(kd_tree_update_interval) || kd_tree_update_interval <= 0.0) {
        throw std::runtime_error("mpc/kd_tree_update_interval must be positive");
    }
    if (!std::isfinite(hold_dist) || hold_dist < 0.0) {
        throw std::runtime_error("mpc/hold_dist must be non-negative");
    }
    if (ipopt_max_iter <= 0) {
        throw std::runtime_error("mpc/ipopt_max_iter must be positive");
    }
    if (!std::isfinite(ipopt_tol) || ipopt_tol <= 0.0) {
        throw std::runtime_error("mpc/ipopt_tol must be positive");
    }
    MY_ID = my_id;
    MPC_HORIZON = N;
    MPC_DT = dt;
    obstacle_hold_dist = hold_dist;
    ARRIVAL_DISTANCE = arrival_distance;
    ARRIVAL_VELOCITY = arrival_velocity;
    NEIGHBOR_TRAJECTORY_TIMEOUT = neighbor_timeout;
    NEIGHBOR_FUTURE_TOLERANCE = neighbor_future_tolerance;
    UAV_NAME = "uav" + std::to_string(my_id);
    LOCAL_FRAME = nh_private.param<std::string>("local_frame", UAV_NAME + "/map");

    tf_buffer_ptr = std::make_unique<tf2_ros::Buffer>();
    tf_listener_ptr = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_ptr);

    const FormationConfig formation = FormationConfig::load(
        nh, formation_config_namespace, my_id);
    const std::vector<int>& neighbor_ids = formation.neighbor_ids;
    required_neighbor_ids = neighbor_ids;
    ROS_INFO("%s: neighbors=%zu, offsets=%zu",
             UAV_NAME.c_str(),
             neighbor_ids.size(), formation.offsets.size());

    // --- MPC 初始化 ---
    mpc_ptr.reset(new Mpc());
    mpc_ptr->setMyId(my_id);
    mpc_ptr->setFormationOffsets(formation.offsets);

    mpc_ptr->setPredictionHorizon(N);
    mpc_ptr->setTimeStep(dt);
    mpc_ptr->setMaxVelocity(u_max);
    mpc_ptr->setMaxAngularVelocity(w_max);
    mpc_ptr->setObstacleWeight(w_oa);
    mpc_ptr->setDynamicObstacleWeight(w_dyn);
    mpc_ptr->setDynamicObstacleSafeDistance(d_safe_dyn);
    mpc_ptr->setDynamicObstacleActivationRange(d_act_dyn);
    mpc_ptr->setNearestObstacleNum(nearest_obs_num);
    mpc_ptr->setDroneRadius(drone_radius);
    mpc_ptr->setSafeDistance(safe_dist);
    mpc_ptr->setObstacleBarrierQ(q_oa);
    mpc_ptr->setReplanMaxIter(replan_max_iter);
    mpc_ptr->setIpoptMaxIter(ipopt_max_iter);
    mpc_ptr->setIpoptTol(ipopt_tol);
    mpc_ptr->setCollisionSafeDistance(d_safe_ca);
    mpc_ptr->setCollisionBarrierQ(q_ca);
    mpc_ptr->setCollisionWeight(w_ca);
    mpc_ptr->setLookaheadDistance(lookahead_distance);
    mpc_ptr->setYawLockDistance(yaw_lock_dist);
    mpc_ptr->setVelocityTimeConstant(tau_v);
    mpc_ptr->setYawTimeConstant(tau_w);
    tau_v_eff = tau_v;
    mpc_ptr->setControlInputWeight(
        DM::diag(DM{w_u, w_u, w_u, w_u}));
    mpc_ptr->setControlIncrementWeight(
        DM::diag(DM{w_du, w_du, w_du, w_du / 4.0}));
    // 编队误差权重：独立于子目标惩罚权重（默认小一个数量级），
    // yaw 通道恒为 0 —— yaw 只跟踪各自速度/子目标方向，编队误差不再拉齐机头，
    // 避免与 yaw 参考（子目标方向）打架。
    mpc_ptr->setFormationErrorWeight(
        DM::diag(DM{w_form, w_form, w_form / 2.0, 0.0}));
    // 滚动子目标惩罚权重（原 Q_）：位置 x:y:z = 1:1:0.5；
    // yaw 通道用独立参数 w_yaw（默认 1000，约为位置权重的 1/20），
    // 复用现有终端软约束跟踪子目标方向，不新增代价项。
    mpc_ptr->setSubgoalWeight(
        DM::diag(DM{w_subgoal, w_subgoal, w_subgoal / 2.0, w_yaw}));

    ROS_INFO("%s: MPC params -- N=%d, dt=%.3f, u_max=%.2f, w_max=%.2f, "
             "d_safe_ca=%.2f, q_ca=%.2f, w_ca=%.2f, "
             "w_form=%.2f, w_subgoal=%.2f, w_yaw=%.2f, nearest_obs_num=%d, drone_radius=%.2f, w_oa=%.2f, "
             "safe_dist=%.2f, q_oa=%.2f, "
             "replan_max_iter=%d, ipopt_max_iter=%d, ipopt_tol=%.1e, "
             "neighbor_timeout=%.3f, future_tolerance=%.3f, "
             "lookahead=%.2f, yaw_lock_dist=%.2f, tau_v=%.2f, tau_w=%.2f, "
             "arrival_dist=%.2f, arrival_vel=%.2f, "
             "w_dyn=%.2f, d_safe_dyn=%.2f, d_act_dyn=%.2f",
             UAV_NAME.c_str(), N, dt, u_max, w_max,
             d_safe_ca, q_ca, w_ca, w_form, w_subgoal, w_yaw,
             nearest_obs_num, drone_radius, w_oa,
             safe_dist, q_oa, replan_max_iter,
             ipopt_max_iter, ipopt_tol,
             NEIGHBOR_TRAJECTORY_TIMEOUT, NEIGHBOR_FUTURE_TOLERANCE,
             lookahead_distance, yaw_lock_dist, tau_v, tau_w,
             ARRIVAL_DISTANCE, ARRIVAL_VELOCITY,
             w_dyn, d_safe_dyn, d_act_dyn);

    // 第三步：初始化 KD-tree 障碍物地图（静态避障）
    kd_tree_ptr.reset(new KdTreeMap());
    mpc_ptr->setKdTree(kd_tree_ptr.get());

    // --- 订阅（飞控状态、位置、速度、邻居预测状态） --- 
    ros::Subscriber fc_state_sub = nh.subscribe<mavros_msgs::State>(
        "/" + UAV_NAME + "/mavros/state", 10, fcStateCallback);
    // 位姿及位姿变化率统一来自里程计（仿真: mavros local_position/odom；实机: FAST-LIO odom）
    ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>(
        "/" + UAV_NAME + "/mavros/local_position/odom", 10, odomCallback);

    // 全局轨迹订阅
    ros::Subscriber global_traj_sub = nh.subscribe<form_nav::GlobalTrajectory>(
        "/" + UAV_NAME + "/global_trajectory", 1, globalTrajCallback);

    // 第三步：障碍物点云（仿真阶段为全局真值点云，后续可换感知 local cloud）
    ros::Subscriber obstacle_cloud_sub = nh.subscribe<sensor_msgs::PointCloud2>(
        cloud_topic, 10, obstacleCloudCallback);

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

    // 动态障碍物轨迹（已知轨迹表；topic 可在 launch 中手动 remap）
    ros::Subscriber dyn_obs_sub =
        nh.subscribe<form_nav::DynamicObstacleTrajectory>(
            "/dynamic_obstacles/trajectory", 10, dynamicObstacleCallback);

    // --- 发布（setpoint、速度、共享预测状态、可视化） ---
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + UAV_NAME + "/mavros/setpoint_position/local", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::TwistStamped>("/" + UAV_NAME + "/mavros/setpoint_velocity/cmd_vel", 1);
    share_path_pub = nh.advertise<form_nav::PredictedTrajectory>("/" + UAV_NAME + "/predicted_path_share", 1);
    predicted_path_pub = nh.advertise<std_msgs::Float64MultiArray>("/" + UAV_NAME + "/predicted_path_viz", 1);
    predicted_path_viz_pub = nh.advertise<visualization_msgs::Marker>("/" + UAV_NAME + "/mpc_predicted_path_marker", 10);
    traveled_path_pub = nh.advertise<nav_msgs::Path>("/" + UAV_NAME + "/traveled_path", 10);
    queried_obstacles_pub = nh.advertise<visualization_msgs::Marker>(
        "/" + UAV_NAME + "/mpc_queried_obstacles", 10);
    obstacle_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(
        "/" + UAV_NAME + "/obstacle_cloud", 10);
    sub_goal_pub = nh.advertise<visualization_msgs::Marker>(
        "/" + UAV_NAME + "/mpc_sub_goal_marker", 10);

    ros::Rate rate(LOOP_RATE);

    // 第一阶段不构建 ESDF；后续恢复障碍物代价时再启用延迟初始化逻辑。
    ROS_INFO("%s: running distributed formation tracking with KD-tree static "
             "obstacle avoidance; waiting for %zu required neighbor(s).",
             UAV_NAME.c_str(), required_neighbor_ids.size());

    // 定时器
    ros::Timer timer = nh.createTimer(
        ros::Duration(1.0 / LOOP_RATE),
        [](const ros::TimerEvent& event) {
            controlLoop(event);
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
