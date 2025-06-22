#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

class OdomToTf {
public:
  OdomToTf(ros::NodeHandle& nh) {
    sub_ = nh.subscribe("/odometry", 10, &OdomToTf::odomCallback, this);
  }

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    geometry_msgs::TransformStamped tf_msg;

    tf_msg.header.stamp = msg->header.stamp;
    tf_msg.header.frame_id = "odom";
    tf_msg.child_frame_id = "base_link";

    tf_msg.transform.translation.x = msg->pose.pose.position.x;
    tf_msg.transform.translation.y = msg->pose.pose.position.y;
    tf_msg.transform.translation.z = msg->pose.pose.position.z;

    tf_msg.transform.rotation = msg->pose.pose.orientation;

    br_.sendTransform(tf_msg);
  }

private:
  ros::Subscriber sub_;
  tf2_ros::TransformBroadcaster br_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_to_tf_node");
  ros::NodeHandle nh;

  OdomToTf odom_to_tf(nh);

  ros::spin();

  return 0;
}