#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Float64MultiArray.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2/utils.h>
#include <Eigen/Dense>
#include <cmath>
#include "form_nav/mpc.h"
#include "form_nav/esdf_casadi.h"
#include "form_nav/sdf_map.h"

// --- 基本配置 ---
std::string UAV_NAME;
const double LOOP_RATE = 20.0;

// --- ROS 通信 ---
ros::Publisher cmd_vel_pub;
ros::Publisher local_pos_pub;
ros::Publisher predicted_path_pub;
ros::Publisher share_path_pub;
ros::Publisher predicted_path_viz_pub;
ros::Publisher traveled_path_pub;

// --- 状态量 ---
Eigen::VectorXd current_mpc_state(8);

double vel_world_x = 0, vel_world_y = 0, vel_world_z = 0;
double vel_body_w = 0;

// --- 历史轨迹 ---
nav_msgs::Path traveled_path;  // 累积已走过的位置

// --- 邻居 + MPC ---
std::unique_ptr<Mpc> mpc_ptr;
std::map<int, UavState> neighbors_map;

// --- ESDF 地图 + CasADi 插值器 ---
std::unique_ptr<SDFMap> sdf_map_ptr;
EsdfCasadi esdf_casadi;

// --- 全局轨迹 ---
std::vector<GlobalWaypoint> global_traj;
bool global_traj_received = false;

// ========== 全局轨迹回调 ==========
// 数据格式: 每点 7 个 double [t, x, y, z, vx, vy, vz]
void globalTrajCallback(const std_msgs::Float64MultiArray::ConstPtr& msg) {
    if (msg->layout.dim.empty()) return;
    int stride = msg->layout.dim[0].stride;  // = 7
    int n = msg->data.size() / stride;
    if (n == 0) return;

    global_traj.clear();
    global_traj.reserve(n);
    for (int i = 0; i < n; ++i) {
        GlobalWaypoint wp;
        int off = i * stride;
        wp.t   = msg->data[off + 0];
        wp.x   = msg->data[off + 1];
        wp.y   = msg->data[off + 2];
        wp.z   = msg->data[off + 3];
        wp.vx  = msg->data[off + 4];
        wp.vy  = msg->data[off + 5];
        wp.vz  = msg->data[off + 6];
        wp.yaw = 0;
        wp.vyaw = 0;
        global_traj.push_back(wp);
    }

    if (!global_traj.empty()) {
        global_traj_received = true;
        mpc_ptr->setGlobalTrajectory(global_traj);
        ROS_INFO("%s: Received global trajectory with %zu waypoints", 
                 UAV_NAME.c_str(), global_traj.size());
    }
}

// ========== 在最近点附近窗口内搜索，返回时间 ==========
double findClosestWaypointTime(const Eigen::VectorXd& state) {
    if (global_traj.empty()) return 0.0;

    static int last_idx = 0;                     // 上一时刻最近点索引
    const int win_back = 3, win_forward = 30;    // 搜索窗口: [last-3, last+30]

    int lo = std::max(0, last_idx - win_back);
    int hi = std::min((int)global_traj.size() - 1, last_idx + win_forward);

    double min_dist = 1e9;
    int closest = lo;
    for (int i = lo; i <= hi; ++i) {
        double dx = state(0) - global_traj[i].x;
        double dy = state(1) - global_traj[i].y;
        double dz = state(2) - global_traj[i].z;
        double d2 = dx*dx + dy*dy + dz*dz;
        if (d2 < min_dist) {
            min_dist = d2;
            closest = i;
        }
    }

    // 如果最近点落在窗口边界附近, 扩展一次全量搜索做安全重置
    if (closest == hi || closest == lo) {
        for (size_t i = 0; i < global_traj.size(); ++i) {
            double dx = state(0) - global_traj[i].x;
            double dy = state(1) - global_traj[i].y;
            double dz = state(2) - global_traj[i].z;
            double d2 = dx*dx + dy*dy + dz*dz;
            if (d2 < min_dist) {
                min_dist = d2;
                closest = i;
            }
        }
    }

    last_idx = closest;
    return global_traj[closest].t;
}

// ========== MAVROS 回调 ==========
void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
    current_mpc_state(0) = msg->pose.position.x - 2; //******** 后面也参数化*******************************************
    current_mpc_state(1) = msg->pose.position.y;
    current_mpc_state(2) = msg->pose.position.z;
}

void velocityCallbackWorld(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    vel_world_x = msg->twist.linear.x;
    vel_world_y = msg->twist.linear.y;
    vel_world_z = msg->twist.linear.z;
    vel_body_w = msg->twist.angular.z;
}

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    tf2::Quaternion q(
        msg->orientation.x, msg->orientation.y,
        msg->orientation.z, msg->orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_mpc_state(3) = yaw;
}

// ========== 主控制循环 ==========
void controlLoop(const ros::TimerEvent&) {
    // 寻找当前最近路径点的时间
    double t0 = findClosestWaypointTime(current_mpc_state);

    // World → Body 速度转换
    double yaw = current_mpc_state(3);
    double cos_psi = cos(yaw), sin_psi = sin(yaw);
    current_mpc_state(4) =  cos_psi * vel_world_x + sin_psi * vel_world_y;
    current_mpc_state(5) = -sin_psi * vel_world_x + cos_psi * vel_world_y;
    current_mpc_state(6) = vel_world_z;
    current_mpc_state(7) = vel_body_w;

    // 邻居
    std::vector<UavState> neighbors_vec;
    for (auto& p : neighbors_map) neighbors_vec.push_back(p.second); // 末尾追加

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

    // 求解 MPC (t0 = 最近点时间)
    bool ok = mpc_ptr->solve(t0, neighbors_vec, current_mpc_state);
    // 可视化
    if (ok) {
        auto u = mpc_ptr->getFirstU(); // body
        // body → world
        double vx = cos_psi * u[0] - sin_psi * u[1];
        double vy = sin_psi * u[0] + cos_psi * u[1];

        geometry_msgs::Twist cmd;
        cmd.linear.x = vx;
        cmd.linear.y = vy;
        cmd.linear.z = u[2];
        cmd.angular.z = u[3];
        cmd_vel_pub.publish(cmd);

        std::vector<double> pred = mpc_ptr->getPredictX();
        std_msgs::Float64MultiArray msg;
        msg.data = pred;
        share_path_pub.publish(msg);
        predicted_path_pub.publish(msg);

        // --- 预测轨迹 Marker 可视化 ---
        visualization_msgs::Marker pred_marker;
        pred_marker.header.frame_id = "world";
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
        points_marker.header.frame_id = "world";
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
        traveled_pose.header.frame_id = "world";
        traveled_pose.header.stamp = ros::Time::now();
        traveled_pose.pose.position.x = current_mpc_state(0);
        traveled_pose.pose.position.y = current_mpc_state(1);
        traveled_pose.pose.position.z = current_mpc_state(2);
        traveled_pose.pose.orientation.w = 1.0;
        traveled_path.header.frame_id = "world";
        traveled_path.header.stamp = ros::Time::now();
        traveled_path.poses.push_back(traveled_pose);
        traveled_path_pub.publish(traveled_path);
    } else {
        ROS_WARN_THROTTLE(1.0, "%s:MPC failed, holding.", UAV_NAME.c_str());
    }
}

// ========== 主函数 ==========
int main(int argc, char **argv) {
    ros::init(argc, argv, "mpc_node");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // --- 参数读取 ---
    int my_id = nh_private.param<int>("my_id", 0);
    UAV_NAME = "uav" + std::to_string(my_id);
    bool is_tracked = nh_private.param<bool>("is_tracked", false);
    int tracking_model = nh_private.param<int>("tracking_model", 1);
    std::vector<int> neighbor_ids = nh_private.param<std::vector<int>>("neighbor_ids", std::vector<int>());

    // 编队偏移量表 {id → [x, y, z, yaw]}
    std::map<int, std::vector<double>> formation_offsets;
    // XmlRpc::XmlRpcValue 是 ROS 参数服务器的通用容器类型，可以容纳 int/double/string/array/struct 等任意 YAML 结构
    XmlRpc::XmlRpcValue offsets_param;
    if (nh_private.getParam("formation_offsets", offsets_param)) {
        for (auto it = offsets_param.begin(); it != offsets_param.end(); ++it) {
            // it->first 是键，std::stoi 转为 int
            int id = std::stoi(it->first);
            // 初始化 4 个元素为 0 的偏移向量 [0,0,0,0]
            std::vector<double> offset(4, 0.0);
            // it->second 是值
            for (int i = 0; i < it->second.size() && i < 4; ++i)
                offset[i] = static_cast<double>(it->second[i]);
            formation_offsets[id] = offset;
        }
    }
    // 如果 formation_offsets 里没有自己的 ID，补一个 [0,0,0,0]
    if (formation_offsets.find(my_id) == formation_offsets.end())
        formation_offsets[my_id] = {0.0, 0.0, 0.0, 0.0};

    ROS_INFO("%s: my_id=%d, is_tracked=%d, model=%d, neighbors=%zu, offsets=%zu",
             UAV_NAME.c_str(), my_id, is_tracked, tracking_model, neighbor_ids.size(), formation_offsets.size());

    // --- MPC 初始化 ---
    mpc_ptr.reset(new Mpc());
    mpc_ptr->setMyId(my_id);
    mpc_ptr->setTracked(is_tracked);
    mpc_ptr->setTrackingModel(tracking_model);
    mpc_ptr->setFormationOffsets(formation_offsets);

    // --- ESDF 地图初始化 ---
    sdf_map_ptr.reset(new SDFMap());
    sdf_map_ptr->initMap(nh_private); // nh 是公共 NodeHandle，读取 /sdf_map/...
    mpc_ptr->setEsdf(&esdf_casadi);

    // --- 订阅（位置、速度、姿态、邻居预测状态） --- 
    ros::Subscriber pose_sub = nh.subscribe("/" + UAV_NAME + "/mavros/local_position/pose", 10, poseCallback);
    ros::Subscriber vel_sub = nh.subscribe("/" + UAV_NAME + "/mavros/local_position/velocity_local", 10, velocityCallbackWorld);
    ros::Subscriber imu_sub = nh.subscribe("/" + UAV_NAME + "/mavros/imu/data", 10, imuCallback);

    // 全局轨迹订阅
    ros::Subscriber global_traj_sub = nh.subscribe("/kino_astar/trajectory", 1, globalTrajCallback);

    // 动态订阅邻居
    std::vector<ros::Subscriber> neighbor_subs;
    for (int nid : neighbor_ids) {
        std::string topic = "/uav" + std::to_string(nid) + "/predicted_path_share";
        neighbor_subs.push_back(nh.subscribe<std_msgs::Float64MultiArray>(topic, 10,  //这里可以不显示说明数据类型吗
            [nid](const std_msgs::Float64MultiArray::ConstPtr& msg) {
                int n_steps = msg->data.size() / 8;
                if (n_steps == 0) return;
                Eigen::MatrixXd pred_states(8, n_steps);
                int idx = 0;
                for (int i = 0; i < n_steps; ++i)
                    for (int j = 0; j < 8; ++j) // 这里前后置完全一样，通常优先用 ++j，因为对迭代器等复杂类型能避免一次不必要的临时对象拷贝
                        pred_states(j, i) = msg->data[idx++];
                UavState neighbor;
                neighbor.id = nid;
                neighbor.predicted_states = pred_states;
                neighbors_map[nid] = neighbor;
            }));
    }

    // --- 发布（setpoint、速度、共享预测状态、可视化） ---
    local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + UAV_NAME + "/mavros/setpoint_position/local", 10);
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/" + UAV_NAME + "/mavros/setpoint_velocity/cmd_vel_unstamped", 1);
    share_path_pub = nh.advertise<std_msgs::Float64MultiArray>("/" + UAV_NAME + "/predicted_path_share", 1);
    predicted_path_pub = nh.advertise<std_msgs::Float64MultiArray>("/" + UAV_NAME + "/predicted_path_viz", 1);
    predicted_path_viz_pub = nh.advertise<visualization_msgs::Marker>("/" + UAV_NAME + "/mpc_predicted_path_marker", 10);
    traveled_path_pub = nh.advertise<nav_msgs::Path>("/" + UAV_NAME + "/traveled_path", 10);

    ros::Rate rate(LOOP_RATE);

    // --- 初始化 ESDF CasADi 插值器 (延迟初始化，由 controlLoop 完成) ---
    // 不在此处阻塞等待，因为 odom/cloud 到达顺序不确定可能导致死等
    ROS_INFO("%s: ESDF will be lazily initialized in control loop.", UAV_NAME.c_str());

    // 定时器
    ros::Timer timer = nh.createTimer(ros::Duration(1.0/LOOP_RATE), controlLoop);

    ros::spin();
    return 0;
}