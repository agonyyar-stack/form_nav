#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <form_nav/grid_map.h>
#include <form_nav/a_star.h>
#include <form_nav/formation_config.h>
#include <form_nav/GlobalTrajectory.h>

path_searching::Astar::Ptr astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_marker_pub;   // 可视化路径 Marker
ros::Publisher global_path_pub_;  // nav_msgs::Path 供 DWA 订阅
ros::Publisher path_traj_pub;     // form_nav::GlobalTrajectory 供 MPC 订阅
ros::Publisher goal_marker_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
bool has_path_ = false;          // 是否有有效路径
std::string world_frame = "world";
Eigen::Vector3d planning_uav_formation_offset = Eigen::Vector3d::Zero();
std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_listener;
uint32_t next_trajectory_id = 1;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = odom;
}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!odom_)
  {
    ROS_WARN("Goal received but no odometry yet, ignoring.");
    return;
  }

  geometry_msgs::PoseStamped goal_world;
  try
  {
    geometry_msgs::PoseStamped goal = *msg;
    if (goal.header.frame_id.empty()) goal.header.frame_id = world_frame;
    goal_world = tf_buffer->transform(goal, world_frame, ros::Duration(0.05));
  }
  catch (const tf2::TransformException& error)
  {
    ROS_WARN("Cannot transform goal to %s: %s", world_frame.c_str(), error.what());
    return;
  }

  // 分布式规划：每架无人机直接用自身位置作为起点，终点 = 用户目标 + 本机编队偏移。
  // 与 kino_astar_node 的终点定义保持一致（z 取规划高度 1.0 + 偏移）。
  Eigen::Vector3d end_pt(goal_world.pose.position.x + planning_uav_formation_offset(0),
                         goal_world.pose.position.y + planning_uav_formation_offset(1),
                         1.0 + planning_uav_formation_offset(2));
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;

  // --- goal visualization ---
  {
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = world_frame;
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "astar/goal";
    goal_marker.id = 0;
    goal_marker.type = visualization_msgs::Marker::SPHERE;
    goal_marker.action = visualization_msgs::Marker::ADD;
    goal_marker.pose.position.x = end_pt(0);
    goal_marker.pose.position.y = end_pt(1);
    goal_marker.pose.position.z = end_pt(2);
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.3;
    goal_marker.scale.y = 0.3;
    goal_marker.scale.z = 0.3;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 0.0;
    goal_marker_pub.publish(goal_marker);
  }

  // 每次新目标重新搜索前复位节点池，避免 use_node_num_ 跨目标累积
  astar_->reset();
  path.clear();
  int success = astar_->search(start_pt, end_pt, path);

  if (success == 1)
  {
    // --- 可视化路径 Marker ---
    {
      visualization_msgs::Marker path_marker;
      path_marker.header.frame_id = world_frame;
      path_marker.header.stamp = ros::Time::now();
      path_marker.ns = "astar/path";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = 0.04;
      path_marker.scale.y = 0.08;
      path_marker.scale.z = 0.08;
      path_marker.color.a = 0.6;
      path_marker.color.r = 0.6;
      path_marker.color.g = 0.6;
      path_marker.color.b = 0.6;

      for (size_t i = 0; i < path.size(); i++)
      {
        geometry_msgs::Point pt;
        pt.x = path[i][0];
        pt.y = path[i][1];
        pt.z = path[i][2];
        path_marker.points.push_back(pt);
      }
      path_marker_pub.publish(path_marker);
    }

    // --- 发布 nav_msgs::Path 供 DWA 节点订阅（保留旧功能） ---
    {
      nav_msgs::Path nav_path;
      nav_path.header.frame_id = world_frame;
      nav_path.header.stamp = ros::Time::now();
      for (size_t i = 0; i < path.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header = nav_path.header;
        pose.pose.position.x = path[i][0];
        pose.pose.position.y = path[i][1];
        pose.pose.position.z = path[i][2];
        pose.pose.orientation.w = 1.0;
        nav_path.poses.push_back(pose);
      }
      global_path_pub_.publish(nav_path);
      has_path_ = true;
    }

    // --- 带编号的全局轨迹: 每点 [x, y, z, vx, vy, vz] ---
    // MPC 局部层只使用位置（弧长投影/滚动子目标/制动/到达判定），速度字段填 0。
    {
      form_nav::GlobalTrajectory traj_msg;
      traj_msg.header.stamp = ros::Time::now();
      traj_msg.header.frame_id = world_frame;
      traj_msg.trajectory_id = next_trajectory_id++;
      if (next_trajectory_id == 0) next_trajectory_id = 1;
      traj_msg.steps = path.size();
      traj_msg.waypoints.reserve(path.size() * 6);
      for (size_t i = 0; i < path.size(); i++)
      {
        traj_msg.waypoints.push_back(path[i][0]);
        traj_msg.waypoints.push_back(path[i][1]);
        traj_msg.waypoints.push_back(path[i][2]);
        traj_msg.waypoints.push_back(0.0);
        traj_msg.waypoints.push_back(0.0);
        traj_msg.waypoints.push_back(0.0);
      }
      path_traj_pub.publish(traj_msg);
      ROS_INFO("Published global trajectory %u with %zu waypoints",
               traj_msg.trajectory_id, path.size());
    }
  }
  else
  {
    std::cout << "Path not found!" << std::endl;
    has_path_ = false;
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "astar_node");
  ros::NodeHandle nh("~"); // 私有命名空间 → /astar_node/

  nh.param<std::string>("world_frame", world_frame, "world");
  const int my_id = nh.param<int>("my_id", 0);
  const std::string formation_config_namespace =
      nh.param<std::string>("formation_config_namespace", "/formation_config");
  // 分布式规划：本机编队偏移直接从 formations.yaml / single_uav.yaml 读取
  try {
    const FormationConfig formation =
        FormationConfig::load(nh, formation_config_namespace, my_id);
    const auto& own_offset = formation.offsets.at(my_id);
    planning_uav_formation_offset =
        Eigen::Vector3d(own_offset[0], own_offset[1], own_offset[2]);
  } catch (const std::exception& e) {
    ROS_FATAL("Failed to load formation config: %s", e.what());
    return 1;
  }
  tf_buffer = std::make_unique<tf2_ros::Buffer>();
  tf_listener = std::make_unique<tf2_ros::TransformListener>(*tf_buffer);

  goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalCallback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/astar/odom", 10, &OdomCallback);

  path_marker_pub = nh.advertise<visualization_msgs::Marker>("/astar/path_marker", 10);
  global_path_pub_ = nh.advertise<nav_msgs::Path>("/astar/global_path", 10);
  // Latching guarantees that MPC nodes started later receive the active path.
  path_traj_pub = nh.advertise<form_nav::GlobalTrajectory>("/astar/trajectory", 1, true);
  goal_marker_pub = nh.advertise<visualization_msgs::Marker>("/astar/goal_marker", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  astar_ = std::make_shared<path_searching::Astar>();
  astar_->setParam(nh);
  astar_->setGridMap(grid_map);
  astar_->init();

  // 定时器：持续重发布 nav_msgs::Path，确保 DWA 节点在任何时刻订阅都能收到
  ros::Timer path_repub_timer = nh.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent&) {
    if (has_path_ && !path.empty()) {
      nav_msgs::Path nav_path;
      nav_path.header.frame_id = world_frame;
      nav_path.header.stamp = ros::Time::now();
      for (const auto& pt : path) {
        geometry_msgs::PoseStamped pose;
        pose.header = nav_path.header;
        pose.pose.position.x = pt[0];
        pose.pose.position.y = pt[1];
        pose.pose.position.z = pt[2];
        pose.pose.orientation.w = 1.0;
        nav_path.poses.push_back(pose);
      }
      global_path_pub_.publish(nav_path);
    }
  });

  ros::spin();

  return 0;
}
