#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <form_nav/grid_map.h>
#include <form_nav/kino_astar.h>

path_searching::KinoAstar::Ptr kino_astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_pub;         // nav_msgs::Path
ros::Publisher goal_marker_pub;

nav_msgs::Odometry::ConstPtr odom_;
std::vector<Eigen::Vector3d> path;

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

  Eigen::Vector3d end_pt(msg->pose.position.x, msg->pose.position.y, 1.0);
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
  Eigen::Vector3d start_vel(odom_->twist.twist.linear.x, odom_->twist.twist.linear.y, odom_->twist.twist.linear.z);
  Eigen::Vector3d end_vel(0, 0, 0);
  std::cout << "Start point: " << start_pt.transpose() << std::endl;
  std::cout << "End point: " << end_pt.transpose() << std::endl;

  // --- goal visualization ---
  {
    visualization_msgs::Marker goal_marker;
    goal_marker.header.frame_id = "world";
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
    // --- nav_msgs::Path ---
    {
      nav_msgs::Path path_msg;
      path_msg.header.frame_id = "world";
      path_msg.header.stamp = ros::Time::now();

      const auto& times = kino_astar_->getPathTimes();
      for (size_t i = 0; i < path.size(); i++)
      {
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "world";
        pose.header.stamp = path_msg.header.stamp;
        pose.pose.position.x = path[i][0];
        pose.pose.position.y = path[i][1];
        pose.pose.position.z = path[i][2];
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
      }
      path_pub.publish(path_msg);
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
  ros::init(argc, argv, "kino_astar_path_node");
  ros::NodeHandle nh("~");

  goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalCallback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/kino_astar/odom", 10, &OdomCallback);

  path_pub = nh.advertise<nav_msgs::Path>("/kino_astar/path", 10);
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
