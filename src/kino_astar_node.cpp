#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <form_nav/grid_map.h>
#include <form_nav/kino_astar.h>
#include <form_nav/formation_config.h>
#include <form_nav/GlobalTrajectory.h>

path_searching::KinoAstar::Ptr kino_astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_marker_pub;
ros::Publisher path_traj_pub;
ros::Publisher goal_marker_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
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
  Eigen::Vector3d end_pt(goal_world.pose.position.x + planning_uav_formation_offset(0),
                         goal_world.pose.position.y + planning_uav_formation_offset(1),
                         1.0 + planning_uav_formation_offset(2));
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x,
                           odom_->pose.pose.position.y,
                           odom_->pose.pose.position.z);
  Eigen::Vector3d start_vel(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y, odom_->twist.twist.linear.z);
  Eigen::Vector3d end_vel(0, 0, 0);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;

  // --- goal visualization ---
  {
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = world_frame;
    goal_marker.header.stamp = ros::Time::now();
    goal_marker.ns = "kino_astar/goal";
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

  int success = kino_astar_->search(start_pt, start_vel, end_pt, end_vel, path);

  if (success == 1)
  {
    // --- path Marker ---
    {
      visualization_msgs::Marker path_marker;
      path_marker.header.frame_id = world_frame;
      path_marker.header.stamp = ros::Time::now();
      path_marker.ns = "kino_astar/path";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::Marker::LINE_STRIP; //把点按顺序连成一条折线，还有SPHERE、CUBE、ARROW等
      path_marker.action = visualization_msgs::Marker::ADD; //操作类型，ADD 表示创建/更新
      path_marker.pose.orientation.w = 1.0; //单位四元数（无旋转），LINE_STRIP 的点坐标是绝对坐标，所以 pose 的平移/旋转对这条线没有实际影响，但必须给一个合法四元数
      path_marker.scale.x = 0.04;
      path_marker.scale.y = 0.04;
      path_marker.scale.z = 0.04;
      path_marker.color.a = 1.0; //不透明度
      path_marker.color.r = 1.0;
      path_marker.color.g = 0.5;
      path_marker.color.b = 0.0;

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

    // --- 带编号的全局轨迹: 每点 [t,x,y,z,vx,vy,vz] ---
    {
      const auto& velocities = kino_astar_->getPathVelocities();
      const auto& times = kino_astar_->getPathTimes();
      size_t n = std::min({path.size(), velocities.size(), times.size()});
      if (n == 0)
      {
        ROS_WARN("Kinodynamic A* returned an empty sampled trajectory");
        path.clear();
        kino_astar_->reset();
        return;
      }
      form_nav::GlobalTrajectory traj_msg;
      traj_msg.header.stamp = ros::Time::now();
      traj_msg.header.frame_id = world_frame;
      traj_msg.trajectory_id = next_trajectory_id++;
      if (next_trajectory_id == 0) next_trajectory_id = 1;
      traj_msg.steps = n;
      traj_msg.waypoints.reserve(n * 7);
      for (size_t i = 0; i < n; i++)
      {
        traj_msg.waypoints.push_back(times[i]);
        traj_msg.waypoints.push_back(path[i][0]);
        traj_msg.waypoints.push_back(path[i][1]);
        traj_msg.waypoints.push_back(path[i][2]);
        traj_msg.waypoints.push_back(velocities[i][0]);
        traj_msg.waypoints.push_back(velocities[i][1]);
        traj_msg.waypoints.push_back(velocities[i][2]);
      }
      path_traj_pub.publish(traj_msg);
      ROS_INFO("Published global trajectory %u with %zu waypoints",
               traj_msg.trajectory_id, n);
    }
  }
  else
  {
    std::cout << "Path not found!" << std::endl;
  }
  path.clear();
  kino_astar_->reset();
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kino_astar_node");
  ros::NodeHandle nh("~");

  nh.param<std::string>("world_frame", world_frame, "world");
  const int my_id = nh.param<int>("my_id", 4);
  const std::string formation_config_namespace =
      nh.param<std::string>("formation_config_namespace", "/formation_config");
  // 分布式规划：本机编队偏移直接从 formations.yaml 读取，不再需要
  // planning_uav_formation_offset_x/y/z 参数。
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
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/kino_astar/odom", 10, &OdomCallback);

  path_marker_pub = nh.advertise<visualization_msgs::Marker>("/kino_astar/path_marker", 10);
  // Latching guarantees that MPC nodes started later receive the active path.
  path_traj_pub = nh.advertise<form_nav::GlobalTrajectory>( "/kino_astar/trajectory", 1, true);
  goal_marker_pub = nh.advertise<visualization_msgs::Marker>("/kino_astar/goal_marker", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  kino_astar_ = std::make_shared<path_searching::KinoAstar>();

  kino_astar_->setParam(nh);
  kino_astar_->setGridMap(grid_map);
  kino_astar_->init();

  ros::spin();
  return 0;
}
