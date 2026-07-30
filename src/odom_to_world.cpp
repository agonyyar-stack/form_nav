#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <string>

class OdomToWorld {
 public:
  OdomToWorld() : private_nh_("~"), tf_listener_(tf_buffer_) {
    private_nh_.param<std::string>("world_frame", world_frame_, "world");
    private_nh_.param<std::string>("input_topic", input_topic_, "odom");
    private_nh_.param<std::string>("output_topic", output_topic_, "odom_world");
    private_nh_.param<std::string>("default_local_frame", default_local_frame_, "");
    publisher_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
    subscriber_ = nh_.subscribe(input_topic_, 20, &OdomToWorld::callback, this);
  }

 private:
  void callback(const nav_msgs::Odometry::ConstPtr& message) {
    const std::string source_frame = message->header.frame_id.empty()
                                         ? default_local_frame_
                                         : message->header.frame_id;
    if (source_frame.empty()) {
      ROS_WARN_THROTTLE(2.0, "Odometry has no source frame");
      return;
    }

    try {
      geometry_msgs::PoseStamped source_pose;
      source_pose.header = message->header;
      source_pose.header.frame_id = source_frame;
      source_pose.pose = message->pose.pose;
      const geometry_msgs::PoseStamped world_pose =
          tf_buffer_.transform(source_pose, world_frame_, ros::Duration(0.05));

      // nav_msgs/Odometry defines twist in child_frame_id. The transformed
      // pose orientation already expresses that child frame in world.
      tf2::Quaternion world_orientation;
      tf2::fromMsg(world_pose.pose.orientation, world_orientation);
      const tf2::Vector3 linear_world = tf2::quatRotate(
          world_orientation,
          tf2::Vector3(message->twist.twist.linear.x,
                       message->twist.twist.linear.y,
                       message->twist.twist.linear.z));
      const tf2::Vector3 angular_world = tf2::quatRotate(
          world_orientation,
          tf2::Vector3(message->twist.twist.angular.x,
                       message->twist.twist.angular.y,
                       message->twist.twist.angular.z));

      nav_msgs::Odometry output = *message;
      output.header.frame_id = world_frame_;
      output.child_frame_id.clear();
      output.pose.pose = world_pose.pose;
      output.twist.twist.linear.x = linear_world.x();
      output.twist.twist.linear.y = linear_world.y();
      output.twist.twist.linear.z = linear_world.z();
      output.twist.twist.angular.x = angular_world.x();
      output.twist.twist.angular.y = angular_world.y();
      output.twist.twist.angular.z = angular_world.z();
      publisher_.publish(output);
    } catch (const tf2::TransformException& error) {
      ROS_WARN_THROTTLE(2.0, "Cannot transform odometry to %s: %s",
                        world_frame_.c_str(), error.what());
    }
  }

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber subscriber_;
  ros::Publisher publisher_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
  std::string world_frame_;
  std::string input_topic_;
  std::string output_topic_;
  std::string default_local_frame_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_world");
  OdomToWorld node;
  ros::spin();
  return 0;
}
