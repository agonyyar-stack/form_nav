#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <visualization_msgs/Marker.h>

#include <string>

// 订阅无人机局部位姿，发布 MESH_RESOURCE Marker 将里程计显示为无人机模型。
// 参考 egoswarm 的 odom_visualization 实现。模型拆分为三个部分：
//   前两个旋翼（fake_drone_rotors_a/b，随无人机自定义颜色）
//   后两个旋翼 + 中央连接件（固定灰色）
// 通过三个 Marker（同一命名空间 mesh，id 0/1/2）组合显示。
static std::string front_mesh_resource;  // 前旋翼对（自定义颜色）
static std::string rear_mesh_resource;   // 后旋翼对（固定灰色）
static std::string body_mesh_resource;   // 中央连接件（固定灰色）
static double color_r, color_g, color_b, color_a;
static double gray_r, gray_g, gray_b;
static double robot_scale;
static double yaw_offset;  // 弧度，修正模型朝向（对应参考实现的 cross_config）

ros::Publisher mesh_pub;

visualization_msgs::Marker makeMeshMarker(
    const geometry_msgs::PoseStamped::ConstPtr& msg, int id,
    const std::string& resource, double r, double g, double b, double a) {
  visualization_msgs::Marker marker;
  marker.header = msg->header;  // frame_id = uavX/map，RViz 用 TF 转到 world
  marker.ns = "mesh";
  marker.id = id;
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
  marker.color.a = a;
  marker.color.r = r;
  marker.color.g = g;
  marker.color.b = b;
  marker.mesh_resource = resource;
  return marker;
}

void poseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
  // 前旋翼：跟随无人机自定义颜色
  mesh_pub.publish(makeMeshMarker(msg, 0, front_mesh_resource,
                                  color_r, color_g, color_b, color_a));
  // 后旋翼与中央连接件：固定灰色
  mesh_pub.publish(makeMeshMarker(msg, 1, rear_mesh_resource,
                                  gray_r, gray_g, gray_b, color_a));
  mesh_pub.publish(makeMeshMarker(msg, 2, body_mesh_resource,
                                  gray_r, gray_g, gray_b, color_a));
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_visualization");
  ros::NodeHandle nh("~");

  nh.param<std::string>("front_mesh_resource", front_mesh_resource,
                        "package://form_nav/meshes/fake_drone_rotors_a.dae");
  nh.param<std::string>("rear_mesh_resource", rear_mesh_resource,
                        "package://form_nav/meshes/fake_drone_rotors_b.dae");
  nh.param<std::string>("body_mesh_resource", body_mesh_resource,
                        "package://form_nav/meshes/fake_drone_body.dae");
  nh.param("color/r", color_r, 1.0);
  nh.param("color/g", color_g, 0.0);
  nh.param("color/b", color_b, 0.0);
  nh.param("color/a", color_a, 1.0);
  nh.param("gray/r", gray_r, 0.5);
  nh.param("gray/g", gray_g, 0.5);
  nh.param("gray/b", gray_b, 0.5);
  nh.param("robot_scale", robot_scale, 0.5);
  nh.param("yaw_offset", yaw_offset, 0.0);

  ros::Subscriber pose_sub = nh.subscribe("pose_in", 10, poseCallback);
  mesh_pub = nh.advertise<visualization_msgs::Marker>("robot", 10);
  ros::spin();
  return 0;
}
