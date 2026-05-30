#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>

class OdomTfBridge
{
public:
  OdomTfBridge()
  {
    sub_ = nh_.subscribe("odom", 50, &OdomTfBridge::odomCallback, this);
  }

private:
  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
  {
    static tf::TransformBroadcaster br;

    tf::Transform transform;
    transform.setOrigin(tf::Vector3(
        msg->pose.pose.position.x,
        msg->pose.pose.position.y,
        msg->pose.pose.position.z));

    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    transform.setRotation(q);

    // IMPORTANT: use the timestamp from /odom
    br.sendTransform(tf::StampedTransform(
        transform,
        msg->header.stamp,      // correct, non-zero time
        msg->header.frame_id,   // "odom"
        msg->child_frame_id));  // "base_link"
  }

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "odom_tf_bridge");
  OdomTfBridge bridge;
  ros::spin();
  return 0;
}
