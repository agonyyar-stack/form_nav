#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <form_nav/grid_map.h>
#include <form_nav/kino_astar.h>

path_searching::KinoAstar::Ptr kino_astar_;

ros::Subscriber goal_sub;
ros::Subscriber odom_sub;

ros::Publisher path_marker_pub;
ros::Publisher path_traj_pub;  // Float64MultiArray: 带速度的轨迹
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
  Eigen::Vector3d start_pt(odom_->pose.pose.position.x - 23, odom_->pose.pose.position.y, odom_->pose.pose.position.z);
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
    // --- path Marker ---
    {
      visualization_msgs::Marker path_marker;
      path_marker.header.frame_id = "world";
      path_marker.header.stamp = ros::Time::now();
      path_marker.ns = "kino_astar/path";
      path_marker.id = 0;
      path_marker.type = visualization_msgs::Marker::LINE_STRIP;
      path_marker.action = visualization_msgs::Marker::ADD;
      path_marker.pose.orientation.w = 1.0;
      path_marker.scale.x = 0.08;
      path_marker.scale.y = 0.08;
      path_marker.scale.z = 0.08;
      path_marker.color.a = 1.0;
      path_marker.color.r = 1.0;
      path_marker.color.g = 0.0;
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

    // --- Float64MultiArray 轨迹: 每点 7 double [t,x,y,z,vx,vy,vz] ---
    {
      const auto& velocities = kino_astar_->getPathVelocities();
      const auto& times = kino_astar_->getPathTimes();
      std_msgs::Float64MultiArray traj_msg;
      traj_msg.layout.dim.resize(1);
      traj_msg.layout.dim[0].label = "waypoints";
      traj_msg.layout.dim[0].size = path.size();
      traj_msg.layout.dim[0].stride = 7;
      size_t n = std::min({path.size(), velocities.size(), times.size()});
      traj_msg.data.reserve(n * 7);
      for (size_t i = 0; i < n; i++)
      {
        traj_msg.data.push_back(times[i]);
        traj_msg.data.push_back(path[i][0]);
        traj_msg.data.push_back(path[i][1]);
        traj_msg.data.push_back(path[i][2]);
        traj_msg.data.push_back(velocities[i][0]);
        traj_msg.data.push_back(velocities[i][1]);
        traj_msg.data.push_back(velocities[i][2]);
      }
      path_traj_pub.publish(traj_msg);
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

  goal_sub = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &GoalCallback);
  odom_sub = nh.subscribe<nav_msgs::Odometry>("/kino_astar/odom", 10, &OdomCallback);

  path_marker_pub = nh.advertise<visualization_msgs::Marker>("/kino_astar/path_marker", 10);
  path_traj_pub = nh.advertise<std_msgs::Float64MultiArray>("/kino_astar/trajectory", 10);
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
