#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <string>

// 订阅无人机局部位姿，发布 MESH_RESOURCE Marker 将里程计显示为无人机模型。
// 参考 egoswarm 的 odom_visualization 实现，仅保留 mesh 部分。
static std::string mesh_resource;
static double color_r, color_g, color_b, color_a;
static double robot_scale;
static double yaw_offset;  // 弧度，修正模型朝向（对应参考实现的 cross_config）

ros::Publisher mesh_pub;

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  visualization_msgs::Marker marker;
  marker.header = msg->header;  // frame_id = uavX/map，RViz 用 TF 转到 world
  marker.ns = "mesh";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::MESH_RESOURCE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = msg->pose;

  if (yaw_offset != 0.0) {
    tf2::Quaternion q, dyaw;
    tf2::fromMsg(msg->pose.orientation, q);
    dyaw.setRPY(0.0, 0.0, yaw_offset);
    marker.pose.orientation = tf2::toMsg(q * dyaw);
  }

  marker.scale.x = robot_scale;
  marker.scale.y = robot_scale;
  marker.scale.z = robot_scale;
  marker.color.a = color_a;
  marker.color.r = color_r;
  marker.color.g = color_g;
  marker.color.b = color_b;
  marker.mesh_resource = mesh_resource;
  mesh_pub.publish(marker);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle nh("~");

  nh.param<std::string>("mesh_resource", mesh_resource,
                        "package://form_nav/meshes/fake_drone.dae");
  nh.param("color/r", color_r, 1.0);
  nh.param("color/g", color_g, 0.0);
  nh.param("color/b", color_b, 0.0);
  nh.param("color/a", color_a, 1.0);
  nh.param("robot_scale", robot_scale, 0.5);
  nh.param("yaw_offset", yaw_offset, 0.0);

  ros::Subscriber pose_sub = nh.subscribe("pose_in", 10, poseCallback);
  mesh_pub = nh.advertise<visualization_msgs::Marker>("robot", 10);
  ros::spin();
  return 0;
}
