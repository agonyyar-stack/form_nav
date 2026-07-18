#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <form_nav/grid_map.h>
#include <form_nav/a_star.h>

path_searching::Astar::Ptr astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_pub;         // 可视化路径
ros::Publisher global_path_pub_; // nav_msgs::Path 供 DWA 订阅

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;
bool has_path_ = false;          // 是否有有效路径

void OdomCallback(const nav_msgs::Odometry::ConstPtr& odom)
{
  odom_ = odom;
}

void GoalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  if (!odom_)
  {
    ROS_WARN("No odometry received yet, ignoring goal.");
    return;
  }
  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y, 1.0);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;
  path.clear();
  int success = astar_->search(start_pt, end_pt, path);

  if (success == 1)
  {
    // 发布 nav_msgs::Path 供 DWA 节点订阅
    nav_msgs::Path nav_path;
    nav_path.header.frame_id = "world";
    nav_path.header.stamp = ros::Time::now();
    for (int i = 0; i < path.size(); i++)
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

    // 发布可视化 Marker
    visualization_msgs::Marker path_marker;
    path_marker.header.frame_id = "world";
    path_marker.header.stamp = ros::Time::now();

    path_marker.ns = "astar/path";
    path_marker.id = 0;

    path_marker.type = visualization_msgs::Marker::LINE_STRIP;
    path_marker.action = visualization_msgs::Marker::ADD;

    path_marker.pose.orientation.w = 1.0;

    path_marker.scale.x = 0.1;
    path_marker.scale.y = 0.1;
    path_marker.scale.z = 0.1;

    path_marker.color.a = 1.0;
    path_marker.color.r = 1.0;
    path_marker.color.g = 0.0;
    path_marker.color.b = 0.0;

    for (int i = 0; i < path.size(); i++)
    {
      geometry_msgs::Point pt;
      pt.x = path[i][0];
      pt.y = path[i][1];
      pt.z = path[i][2];
      path_marker.points.push_back(pt);
    }

    path_pub.publish(path_marker);
  }
  else
  {
    std::cout << "Path not found!" << std::endl;
    has_path_ = false;
  }
  // 不再清空 path，保留供 DWA 使用（新 goal 到来时会重新搜索覆盖）
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "astar_node");
  ros::NodeHandle nh("~"); // 私有命名空间 → /astar_node/

  ros::Subscriber goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalCallback);
  ros::Subscriber odom_sub = nh.subscribe<nav_msgs::Odometry>("odom", 10, &OdomCallback);

  path_pub = nh.advertise<visualization_msgs::Marker>("path", 10);
  global_path_pub_ = nh.advertise<nav_msgs::Path>("/astar/global_path", 10);

  GridMap::Ptr grid_map = std::make_shared<GridMap>();
  grid_map->initMap(nh);

  astar_ = std::make_shared<path_searching::Astar>();

  astar_->setParam(nh);
  astar_->setGridMap(grid_map);
  astar_->init();

  // 定时器：持续重发布路径，确保 DWA 节点在任何时刻订阅都能收到
  ros::Timer path_repub_timer = nh.createTimer(ros::Duration(0.5), [&](const ros::TimerEvent&) {
    if (has_path_ && !path.empty()) {
      nav_msgs::Path nav_path;
      nav_path.header.frame_id = "world";
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