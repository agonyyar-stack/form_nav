#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/tf.h>
#include <Eigen/Dense>
#include "form_nav/sdf_map.h"
#include "form_nav/dwa_planner.h"

namespace dwa_planner {

class DWANode {
public:
        DWANode() : nh_("~") {    //~ 句柄读“节点私有参数”，<node> <param name="..."> 默认是放在节点私有命名空间里的
        // 1. 初始化 ESDF 地图
        sdf_map_.reset(new SDFMap());
        sdf_map_->initMap(nh_); // 需要传入nodehandle是因为SDFmap需要收发消息

        // 2. 初始化 DWA 规划器，传入 ESDF 地图指针
        dwa_planner_.reset(new DWAPlanner(sdf_map_));

        // 3. 订阅与发布初始化
        odom_sub_ = nh_.subscribe("/mavros/local_position/odom", 1, &DWANode::odomCallback, this);
        // 订阅 A* 全局路径，替代原来的手动目标点
        global_path_sub_ = nh_.subscribe("/global_path", 1, &DWANode::globalPathCallback, this);
        
        cmd_pub_ = nh_.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);
        // best_path_pub_ = nh_.advertise<nav_msgs::Path>("/dwa/best_path", 1);
        candidate_traj_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dwa/candidate_trajectories", 1);
        best_traj_pub_ = nh_.advertise<visualization_msgs::Marker>("/dwa/best_trajectory", 1);
        goal_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/dwa/goal_marker", 1);

        planner_timer_ = nh_.createTimer(ros::Duration(0.05), &DWANode::plannerLoop, this);

        // State
        has_goal_ = false;

        ROS_INFO("[PlannerNode] Initialized with ESDF Map, subscribing to A* global path.");
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber odom_sub_;
    ros::Subscriber global_path_sub_; // 订阅 A* 全局路径
    ros::Publisher cmd_pub_;
    ros::Publisher best_path_pub_;
    ros::Publisher candidate_traj_pub_;
    ros::Publisher best_traj_pub_;
    ros::Publisher goal_marker_pub_;

    ros::Timer planner_timer_;

    // Planner
    SDFMap::Ptr sdf_map_;
    std::unique_ptr<DWAPlanner> dwa_planner_;

    // Current state
    State current_state_;
    Eigen::Vector3d goal_; // 当前 DWA 目标点
    std::vector<Eigen::Vector3d> global_path_; // A* 全局路径点
    bool has_goal_;

private:
    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        current_state_.x = msg->pose.pose.position.x;
        current_state_.y = msg->pose.pose.position.y;
        current_state_.z = msg->pose.pose.position.z;

        current_state_.vx = msg->twist.twist.linear.x;
        current_state_.vy = msg->twist.twist.linear.y;
        current_state_.vz = msg->twist.twist.linear.z;

        tf::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w);
        tf::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        current_state_.yaw = yaw;
        current_state_.yaw_rate = msg->twist.twist.angular.z;
    }

    void globalPathCallback(const nav_msgs::PathConstPtr& msg) {
        global_path_.clear();
        for (const auto& pose : msg->poses) {
            global_path_.emplace_back(pose.pose.position.x,
                                      pose.pose.position.y,
                                      pose.pose.position.z);
        }
        if (!global_path_.empty()) {
            has_goal_ = true;
            ROS_INFO("[PlannerNode] Global path received: %zu waypoints, end=(%.2f, %.2f, %.2f)",
                     global_path_.size(), global_path_.back()[0], global_path_.back()[1], global_path_.back()[2]);
        }
    }

    // 滚动预瞄点
    bool LookaheadTarget(Eigen::Vector3d& target, int& nearest_idx) {
        if (global_path_.empty()) return false;

        const Eigen::Vector2d current_pos(current_state_.x, current_state_.y);
        const Eigen::Vector3d global_end = global_path_.back();
        
        // 0. 检查是否已到达全局终点
        double dist_to_global_end = (global_end.head<2>() - current_pos).norm();
        if (dist_to_global_end < 0.3) {
            return false;  // 已到达终点
        }

        // 1. 计算当前位姿与路径上每个点的距离，找最近点
        double min_dist = 1e9;
        nearest_idx = 0;
        for (size_t i = 0; i < global_path_.size(); ++i) {
            double d = (global_path_[i].head<2>() - current_pos).norm();
            if (d < min_dist) {
                min_dist = d;
                nearest_idx = i;
            }
        }
        double deviation_dist = min_dist;  // 当前偏离路径的距离

        // 2. 根据偏离程度动态调整前瞻距离
        double target_lookahead_dist = 1.0;
        if (deviation_dist > 1.0) target_lookahead_dist = 2.0;

        // 3. 从最近点开始沿路径累积距离，找到预瞄点
        double temp_dist = 0.0;
        target = global_path_.back();  // 默认预瞄点是全局终点
        // size_t为无符号整数类型，索引从0开始
        for (size_t i = nearest_idx; i < global_path_.size() - 1; ++i) {
            temp_dist += (global_path_[i + 1].head<2>() - global_path_[i].head<2>()).norm();
            if (temp_dist > target_lookahead_dist) {
                target = global_path_[i + 1];
                break;
            }
        }
        
        return true; // 成功获取预瞄点
    }

    void plannerLoop(const ros::TimerEvent&) {
        // 确保有全局路径
        if (!has_goal_ || global_path_.empty()) {return;}

        // 等待ESDF Map
        if (!sdf_map_->odomValid()) {
            ROS_WARN_THROTTLE(1.0, "[PlannerNode] Waiting for valid odometry for ESDF map...");
            return;
        }

        // 从全局路径中计算滚动预瞄点
        Eigen::Vector3d lookahead_target;
        int nearest_idx = 0;
        if (!LookaheadTarget(lookahead_target, nearest_idx)) {
            // 已到达全局终点，停止。THROTTLE表示节流。
            ROS_INFO_THROTTLE(1.0, "[PlannerNode] Reached global end point, stopping.");
            publishStopCommand();
            return;
        }
        goal_ = lookahead_target;  // 更新当前目标为预瞄点

        //运行 DWA — 传入全局路径和最近点索引，用于路径跟踪评价
        auto best_traj = dwa_planner_->plan(current_state_, goal_, global_path_, nearest_idx);

        // 到达当前预瞄点附近时继续滚动，不停止（除非已到全局终点）
        // LookaheadTarget 已处理全局终点判断

        // 无有效轨迹
        if (best_traj.states.empty()) {
            ROS_WARN_THROTTLE(1.0, "[PlannerNode] No valid trajectory.");
            publishStopCommand();
            return;
        }

        // 发布命令
        publishVelocity(best_traj);

        // 可视化
        publishBestPath(best_traj);
        publishBestTrajectoryMarker(best_traj);
        publishGoalMarker();

        // candidate_traj_pub_.publish(dwa_planner_->getTrajectoryMarkers());

        // Debug
        ROS_INFO_THROTTLE(1.0, "[DWANode] lookahead=(%.2f,%.2f,%.2f) cmd vx=%.2f vy=%.2f vz=%.2f score=%.2f", 
                          goal_.x(), goal_.y(), goal_.z(),
                          best_traj.control.vx, best_traj.control.vy, best_traj.control.vz, best_traj.score);
    }

    void publishVelocity(const Trajectory& traj) {
        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        cmd.twist.linear.x = traj.control.vx;
        cmd.twist.linear.y = traj.control.vy;
        cmd.twist.linear.z = traj.control.vz;
        cmd_pub_.publish(cmd);
    }

    void publishStopCommand() {
        geometry_msgs::TwistStamped cmd;
        cmd.header.stamp = ros::Time::now();
        cmd.header.frame_id = "map";
        cmd.twist.linear.x = 0.0;
        cmd.twist.linear.y = 0.0;
        cmd.twist.linear.z = 0.0;
        cmd_pub_.publish(cmd);
    }

    void publishBestPath(const Trajectory& traj) {
        nav_msgs::Path path;
        path.header.frame_id = "map";
        path.header.stamp = ros::Time::now();

        for (const auto& state : traj.states) {
            geometry_msgs::PoseStamped pose;
            pose.header = path.header;
            pose.pose.position.x = state.x;
            pose.pose.position.y = state.y;
            pose.pose.position.z = state.z;

            tf::Quaternion q;
            q.setRPY(0.0, 0.0, state.yaw);
            pose.pose.orientation.x = q.x();
            pose.pose.orientation.y = q.y();
            pose.pose.orientation.z = q.z();
            pose.pose.orientation.w = q.w();

            path.poses.push_back(pose);
        }
        best_path_pub_.publish(path);
    }

    void publishBestTrajectoryMarker(const Trajectory& traj) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "best_trajectory";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::LINE_STRIP;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;

        // Green best trajectory
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        for (const auto& state : traj.states) {
            geometry_msgs::Point p;
            p.x = state.x;
            p.y = state.y;
            p.z = state.z;
            marker.points.push_back(p);
        }
        best_traj_pub_.publish(marker);
    }

    void publishGoalMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = ros::Time::now();
        marker.ns = "goal";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = goal_.x();
        marker.pose.position.y = goal_.y();
        marker.pose.position.z = goal_.z();
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.5;
        marker.scale.y = 0.5;
        marker.scale.z = 0.5;

        // Red goal
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        goal_marker_pub_.publish(marker);
    }
};

} // namespace dwa_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "dwa_node");
    dwa_planner::DWANode node;
    ros::spin(); // 一直阻塞当前线程，不断检查 ROS 是否有新的消息、服务请求或定时器事件，一旦收到，就调用对应的回调函数
    return 0;    // 没有它，节点会执行完 main() 就退出。
}
