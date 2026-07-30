#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <xmlrpcpp/XmlRpcException.h>
#include <xmlrpcpp/XmlRpcValue.h>

#include <string>
#include <vector>

namespace {

double numericValue(const XmlRpc::XmlRpcValue& value) {
  if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) {
    return static_cast<int>(value);
  }
  if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
    return static_cast<double>(value);
  }
  throw XmlRpc::XmlRpcException("Expected a numeric value");
}

}  // namespace

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_uav_static_tf");
  ros::NodeHandle private_nh("~");

  std::string world_frame;
  private_nh.param<std::string>("world_frame", world_frame, "world");

  XmlRpc::XmlRpcValue frames;
  if (!private_nh.getParam("frames", frames) ||
      frames.getType() != XmlRpc::XmlRpcValue::TypeStruct) {
    ROS_FATAL("~frames must be a non-empty struct");
    return 1;
  }

  std::vector<geometry_msgs::TransformStamped> transforms;
  try {
    for (auto it = frames.begin(); it != frames.end(); ++it) {
      const XmlRpc::XmlRpcValue& config = it->second;
      if (!config.hasMember("child_frame") || !config.hasMember("origin") ||
          !config.hasMember("yaw")) {
        throw XmlRpc::XmlRpcException("Each frame needs child_frame, origin and yaw");
      }

      const XmlRpc::XmlRpcValue& origin = config["origin"];
      if (origin.getType() != XmlRpc::XmlRpcValue::TypeArray || origin.size() != 3) {
        throw XmlRpc::XmlRpcException("origin must contain [x, y, z]");
      }

      geometry_msgs::TransformStamped transform;
      transform.header.stamp = ros::Time::now();
      transform.header.frame_id = world_frame;
      transform.child_frame_id = static_cast<std::string>(config["child_frame"]);
      transform.transform.translation.x = numericValue(origin[0]);
      transform.transform.translation.y = numericValue(origin[1]);
      transform.transform.translation.z = numericValue(origin[2]);

      tf2::Quaternion orientation;
      orientation.setRPY(0.0, 0.0, numericValue(config["yaw"]));
      transform.transform.rotation.x = orientation.x();
      transform.transform.rotation.y = orientation.y();
      transform.transform.rotation.z = orientation.z();
      transform.transform.rotation.w = orientation.w();
      transforms.push_back(transform);

      ROS_INFO("Static TF %s -> %s: origin=[%.3f, %.3f, %.3f]",
               world_frame.c_str(), transform.child_frame_id.c_str(),
               transform.transform.translation.x, transform.transform.translation.y,
               transform.transform.translation.z);
    }
  } catch (const XmlRpc::XmlRpcException& error) {
    ROS_FATAL("Invalid static TF configuration: %s", error.getMessage().c_str());
    return 1;
  }

  tf2_ros::StaticTransformBroadcaster broadcaster;
  broadcaster.sendTransform(transforms);
  ros::spin();
  return 0;
}
