#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Quaternion.h>
#include <cmath>

// Pose state (from speed node)
double x_pos   = 0.0;
double y_pos   = 0.0;
double theta   = 0.0;

// Velocities (in base frame)
double vx      = 0.0;   // forward linear velocity (m/s)
double vth     = 0.0;   // angular velocity (rad/s)

// For derivative
bool have_prev_pose         = false;
double prev_x_pos           = 0.0;
double prev_y_pos           = 0.0;
double prev_theta           = 0.0;
ros::Time prev_pose_time;

// Normalize angle to [-pi, pi]
inline double normalize_angle(double angle) {
  while (angle > M_PI)  angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

// Callback: now receives pose (x, y, theta) instead of wheel speeds
void handle_speed(const geometry_msgs::Vector3Stamped& msg) {
  // Get timestamp – prefer message stamp if set
  ros::Time t = msg.header.stamp.isZero() ? ros::Time::now() : msg.header.stamp;

  double new_x     = msg.vector.x;   // x position [m]
  double new_y     = msg.vector.y;   // y position [m]
  double new_theta = msg.vector.z;   // heading [rad]

  if (!have_prev_pose) {
    // First reading: just store pose, zero velocity
    x_pos = new_x;
    y_pos = new_y;
    theta = new_theta;

    prev_x_pos     = new_x;
    prev_y_pos     = new_y;
    prev_theta     = new_theta;
    prev_pose_time = t;

    vx   = 0.0;
    vth  = 0.0;
    have_prev_pose = true;

    ROS_DEBUG("First pose: x=%f y=%f th=%f", x_pos, y_pos, theta);
    return;
  }

  double dt = (t - prev_pose_time).toSec();
  if (dt <= 0.0) {
    // Bad timestamp, keep pose but don’t update velocity
    x_pos = new_x;
    y_pos = new_y;
    theta = new_theta;
    ROS_WARN_THROTTLE(1.0, "Non-positive dt in handle_speed, skipping velocity update");
    return;
  }

  double dx = new_x - prev_x_pos;
  double dy = new_y - prev_y_pos;
  double dtheta = normalize_angle(new_theta - prev_theta);

  // Velocity in odom frame
  double vx_odom = dx / dt;
  double vy_odom = dy / dt;

  // Project velocity along robot heading to get forward speed in base_link
  // base_link x-axis is robot forward, so:
  double heading_x = std::cos(new_theta);
  double heading_y = std::sin(new_theta);
  vx = heading_x * vx_odom + heading_y * vy_odom;

  // We ignore lateral speed (vy in base frame) for diff-drive:
  vth = dtheta / dt;

  // Update pose state
  x_pos = new_x;
  y_pos = new_y;
  theta = normalize_angle(new_theta);

  // Store for next iteration
  prev_x_pos     = new_x;
  prev_y_pos     = new_y;
  prev_theta     = new_theta;
  prev_pose_time = t;

  ROS_DEBUG("Pose update: x=%f y=%f th=%f | vx=%f vth=%f dt=%f",
            x_pos, y_pos, theta, vx, vth, dt);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "nox_controller");

  ros::NodeHandle n;
  ros::NodeHandle nh_private("~");

  ros::Subscriber sub = n.subscribe("speed", 50, handle_speed);
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster broadcaster;

  double rate = 10.0;
  bool publish_tf = true;

  nh_private.getParam("publish_rate", rate);
  nh_private.getParam("publish_tf", publish_tf);

  const char base_link[] = "base_link";
  const char odom[]      = "odom";

  ros::Rate r(rate);

  while (n.ok()) {
    ros::spinOnce();
    ros::Time now = ros::Time::now();

    // Orientation quaternion from theta
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);

    if (publish_tf) {
      geometry_msgs::TransformStamped t;
      t.header.stamp = now;
      t.header.frame_id = odom;
      t.child_frame_id  = base_link;
      t.transform.translation.x = x_pos;
      t.transform.translation.y = y_pos;
      t.transform.translation.z = 0.0;
      t.transform.rotation = odom_quat;

      broadcaster.sendTransform(t);
    }

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom;
    odom_msg.child_frame_id  = base_link;

    // Pose from Arduino / speed node
    odom_msg.pose.pose.position.x = x_pos;
    odom_msg.pose.pose.position.y = y_pos;
    odom_msg.pose.pose.position.z = 0.0;
    odom_msg.pose.pose.orientation = odom_quat;

    // Simple covariance model: stationary vs moving
    bool moving = std::fabs(vx) > 1e-4 || std::fabs(vth) > 1e-4;

    // Initialize covariances
    for (int i = 0; i < 36; ++i) {
      odom_msg.pose.covariance[i]  = 0.0;
      odom_msg.twist.covariance[i] = 0.0;
    }

    if (!moving) {
      // Stationary: high confidence
      odom_msg.pose.covariance[0]  = 1e-9;  // x
      odom_msg.pose.covariance[7]  = 1e-9;  // y
      odom_msg.pose.covariance[35] = 1e-9;  // yaw

      odom_msg.twist.covariance[0]  = 1e-9; // vx
      odom_msg.twist.covariance[7]  = 1e-9; // vy
      odom_msg.twist.covariance[35] = 1e-9; // vth
    } else {
      // Moving: more uncertainty
      odom_msg.pose.covariance[0]  = 1e-3;
      odom_msg.pose.covariance[7]  = 1e-3;
      odom_msg.pose.covariance[35] = 1e-2;

      odom_msg.twist.covariance[0]  = 1e-3;
      odom_msg.twist.covariance[7]  = 1e-3;
      odom_msg.twist.covariance[35] = 1e-2;
    }

    // Twist in base_link
    odom_msg.twist.twist.linear.x  = vx;
    odom_msg.twist.twist.linear.y  = 0.0;
    odom_msg.twist.twist.linear.z  = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = vth;

    odom_pub.publish(odom_msg);

    r.sleep();
  }

  return 0;
}

